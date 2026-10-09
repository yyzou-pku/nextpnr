#ifndef SPARTAN3E_LUTRAM_H
#define SPARTAN3E_LUTRAM_H

#include <array>
#include <functional>
#include <string>
#include <vector>
#include "ff.h"
#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN

inline void disconnect_inactive_ff_sr(Context *ctx, CellInfo *ff, const std::function<bool(IdString)> &is_lut)
{
    IdString control;
    if (ff->type == ctx->id("FDRE"))
        control = ctx->id("R");
    else if (ff->type == ctx->id("FDSE"))
        control = ctx->id("S");
    else if (ff->type.in(ctx->id("FDCE"), ctx->id("FDC")))
        control = ctx->id("CLR");
    else if (ff->type.in(ctx->id("FDPE"), ctx->id("FDP")))
        control = ctx->id("PRE");
    else
        return;
    auto inverted = ff->params.find(ctx->idf("IS_%s_INVERTED", control.c_str(ctx)));
    if (inverted != ff->params.end() && inverted->second.as_int64() != 0)
        return;
    NetInfo *net = ff->getPort(control);
    if (net == nullptr || net->driver.cell == nullptr)
        return;
    CellInfo *source = net->driver.cell;
    auto init = source->params.find(ctx->id("INIT"));
    if (is_lut(source->type) && init != source->params.end() && init->second.as_int64() == 0)
        ff->disconnectPort(control);
}

// A RAM32 consumes both memories and F5. FF paths must share the actual
// remaining BX/BY or local F5 data path and never borrow the RAM write SR.
struct Ram32Resources
{
    BelId composite;
    std::vector<BelId> members;
    std::array<BelId, 2> ffs;
};

// Initial dual-port scope owns the slice. F5 is not a computational member,
// but mixed wide/carry/FF occupancy remains closed until independently tested.
struct Ram16DualResources
{
    BelId composite;
    std::vector<BelId> excluded;
};

struct Ram64Resources
{
    BelId composite;
    std::array<int, 2> sites; // Low address bank, high address bank.
    std::vector<BelId> excluded;
};

inline void remap_ram64(Context *ctx, CellInfo *cell)
{
    for (int i = 0; i < 4; ++i) {
        std::vector<IdString> pins;
        for (const std::string half : {"L", "H"})
            for (const std::string bank : {"F", "G"}) {
                pins.push_back(ctx->id(half + bank + "A" + std::to_string(i)));
                pins.push_back(ctx->id(half + bank + "W" + std::to_string(i)));
            }
        cell->bel_pins[ctx->idf("A%d", i)] = pins;
    }
    cell->bel_pins[ctx->id("A4")] = {ctx->id("LA4"), ctx->id("LWA4"), ctx->id("HA4"), ctx->id("HWA4")};
    cell->bel_pins[ctx->id("A5")] = {ctx->id("A5"), ctx->id("LWA5"), ctx->id("HWA5")};
    cell->bel_pins[ctx->id("D")] = {ctx->id("LFD"), ctx->id("LGD"), ctx->id("HFD"), ctx->id("HGD")};
    cell->bel_pins[ctx->id("WE")] = {ctx->id("LWE"), ctx->id("HWE")};
    cell->bel_pins[ctx->id("WCLK")] = {ctx->id("LCLK"), ctx->id("HCLK")};
    for (const std::string half : {"L", "H"}) {
        IdString output = ctx->id("$RAM64_F5_" + half), input = ctx->id("$RAM64_F6_" + half);
        cell->addOutput(output);
        cell->addInput(input);
        cell->bel_pins[output] = {output};
        cell->bel_pins[input] = {input};
        NetInfo *net = ctx->createNet(ctx->id(cell->name.str(ctx) + "/$RAM64_READ_" + half));
        cell->connectPort(output, net);
        cell->connectPort(input, net);
    }
}

inline bool ram16_dual_resources_valid(Context *ctx, const Ram16DualResources &site)
{
    if (ctx->getBoundBelCell(site.composite) == nullptr)
        return true;
    for (BelId member : site.excluded)
        if (ctx->getBoundBelCell(member) != nullptr)
            return false;
    return true;
}

inline void connect_ram16_dual(Context *ctx, BelId bel,
                               const std::function<WireId(const std::string &, const std::string &)> &endpoint)
{
    for (int i = 0; i < 4; ++i) {
        ctx->addBelInput(bel, ctx->idf("A%d", i), endpoint("G", "A" + std::to_string(i + 1)));
        ctx->addBelInput(bel, ctx->idf("DPRA%d", i), endpoint("F", "A" + std::to_string(i + 1)));
        for (const std::string bank : {"F", "G"})
            ctx->addBelInput(bel, ctx->idf("W%s%d", bank.c_str(), i),
                             endpoint(bank, "W" + bank + std::to_string(i + 1)));
    }
    ctx->addBelInput(bel, ctx->id("D"), endpoint("G", "DI"));
    ctx->addBelInput(bel, ctx->id("DF"), endpoint("F", "DI"));
    ctx->addBelInput(bel, ctx->id("WE"), endpoint("WSGEN", "WE"));
    ctx->addBelInput(bel, ctx->id("WCLK"), endpoint("WSGEN", "CK"));
    ctx->addBelOutput(bel, ctx->id("SPO"), endpoint("G", "D"));
    ctx->addBelOutput(bel, ctx->id("DPO"), endpoint("F", "D"));
}

inline void remap_ram16_dual(Context *ctx, CellInfo *cell)
{
    for (int i = 0; i < 4; ++i)
        cell->bel_pins[ctx->idf("A%d", i)] = {ctx->idf("A%d", i), ctx->idf("WF%d", i), ctx->idf("WG%d", i)};
    cell->bel_pins[ctx->id("D")] = {ctx->id("D"), ctx->id("DF")};
}

inline bool ram32_resources_valid(Context *ctx, const Ram32Resources &site)
{
    CellInfo *ram = ctx->getBoundBelCell(site.composite);
    if (ram == nullptr)
        return true;
    for (BelId member : site.members)
        if (ctx->getBoundBelCell(member) != nullptr)
            return false;
    for (int half = 0; half < 2; ++half) {
        CellInfo *ff = ctx->getBoundBelCell(site.ffs[half]);
        if (ff == nullptr)
            continue;
        if (ff->getPort(ctx->id("C")) != ram->getPort(ctx->id("WCLK")) || ff_clock_inverted(ctx, ff))
            return false;
        for (const std::string control : {"R", "S", "CLR", "PRE"})
            if (ff->getPort(ctx->id(control)) != nullptr)
                return false;
        NetInfo *data = ff->getPort(ctx->id("D"));
        if (half == 0) {
            if (data != ram->getPort(ctx->id("O")) && data != ram->getPort(ctx->id("A4")))
                return false;
        } else if (data != ram->getPort(ctx->id("D")))
            return false;
    }
    return true;
}

inline void connect_ram32(Context *ctx, BelId bel,
                          const std::function<WireId(const std::string &, const std::string &)> &endpoint)
{
    for (int i = 0; i < 4; ++i) {
        ctx->addBelInput(bel, ctx->idf("A%d", i), endpoint("F", "A" + std::to_string(i + 1)));
        ctx->addBelInput(bel, ctx->idf("AG%d", i), endpoint("G", "A" + std::to_string(i + 1)));
        for (const std::string bank : {"F", "G"})
            ctx->addBelInput(bel, ctx->idf("W%s%d", bank.c_str(), i),
                             endpoint(bank, "W" + bank + std::to_string(i + 1)));
    }
    ctx->addBelInput(bel, ctx->id("A4"), endpoint("F5MUX", "S0"));
    ctx->addBelInput(bel, ctx->id("WA4"), endpoint("WSGEN", "WE0"));
    ctx->addBelInput(bel, ctx->id("D"), endpoint("F", "DI"));
    ctx->addBelInput(bel, ctx->id("DG"), endpoint("G", "DI"));
    ctx->addBelInput(bel, ctx->id("WE"), endpoint("WSGEN", "WE"));
    ctx->addBelInput(bel, ctx->id("WCLK"), endpoint("WSGEN", "CK"));
    ctx->addBelOutput(bel, ctx->id("O"), endpoint("F5MUX", "OUT"));
}

inline void remap_ram32(Context *ctx, CellInfo *cell)
{
    for (int i = 0; i < 4; ++i)
        cell->bel_pins[ctx->idf("A%d", i)] = {ctx->idf("A%d", i), ctx->idf("AG%d", i), ctx->idf("WF%d", i),
                                              ctx->idf("WG%d", i)};
    cell->bel_pins[ctx->id("A4")] = {ctx->id("A4"), ctx->id("WA4")};
    cell->bel_pins[ctx->id("D")] = {ctx->id("D"), ctx->id("DG")};
}

NEXTPNR_NAMESPACE_END
#endif
