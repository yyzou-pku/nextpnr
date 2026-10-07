#include "spartan3e_ise_carry.h"

#include <algorithm>
#include <optional>
#include <unordered_set>

#include "log.h"

#define VIADUCT_CONSTIDS "viaduct/spartan3e/constids.inc"
#include "viaduct_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {

bool is_lut(IdString type) { return type.in(id_LUT1, id_LUT2, id_LUT3, id_LUT4); }

int lut_width(IdString type)
{
    if (type == id_LUT1)
        return 1;
    if (type == id_LUT2)
        return 2;
    if (type == id_LUT3)
        return 3;
    return 4;
}

std::optional<bool> constant_value(NetInfo *net)
{
    if (net == nullptr || net->driver.cell == nullptr || !is_lut(net->driver.cell->type))
        return std::nullopt;
    auto *cell = net->driver.cell;
    unsigned mask = (1u << (1 << lut_width(cell->type))) - 1;
    unsigned init = cell->params.at(id_INIT).as_int64() & mask;
    if (init == 0 || init == mask)
        return init != 0;
    return std::nullopt;
}

CellInfo *private_lut(Context *ctx, CellInfo *owner, NetInfo *propagate, NetInfo *di, CellInfo *product = nullptr)
{
    if (propagate == nullptr)
        log_error("Spartan-3E ISE: carry %s has no propagate input\n", owner->name.c_str(ctx));
    CellInfo *source = propagate->driver.cell;
    std::vector<NetInfo *> old_inputs;
    unsigned old_init = 2; // Buffer for a propagate signal not driven by a LUT.
    bool copied_source = source != nullptr && is_lut(source->type);
    if (source != nullptr && is_lut(source->type)) {
        for (int i = 0; i < lut_width(source->type); i++)
            old_inputs.push_back(source->getPort(ctx->idf("I%d", i)));
        old_init = source->params.at(id_INIT).as_int64();
    } else {
        old_inputs.push_back(propagate);
    }

    // CY0F/G can use F1/G1. Keep the DI net on LUT input zero and permute
    // INIT, so a variable DI does not compete with the root's BX carry-in.
    std::vector<NetInfo *> forced;
    if (product != nullptr) {
        for (int i = 0; i < 2; ++i) {
            auto *net = product->getPort(ctx->idf("I%d", i));
            if (net == nullptr) log_error("Spartan-3E ISE: missing MULT_AND input\n");
            if (std::find(forced.begin(), forced.end(), net) == forced.end()) forced.push_back(net);
        }
    } else if (di != nullptr) forced.push_back(di);
    std::vector<NetInfo *> combined = forced;
    for (auto *input : old_inputs)
        if (std::find(combined.begin(), combined.end(), input) == combined.end()) combined.push_back(input);
    size_t product_alias_slots = product != nullptr ? 2 - forced.size() : 0;
    if (combined.size() + product_alias_slots > 4) {
        old_inputs = {propagate};
        old_init = 2;
        copied_source = false;
    }
    std::vector<NetInfo *> inputs = forced;
    // Keep the two product pins at physical input zero/one even when aliased.
    if (product != nullptr) inputs = {product->getPort(ctx->id("I0")), product->getPort(ctx->id("I1"))};
    for (auto *input : old_inputs)
        if (std::find(inputs.begin(), inputs.end(), input) == inputs.end())
            inputs.push_back(input);
    NPNR_ASSERT(inputs.size() <= 4);
    unsigned init = 0;
    for (unsigned value = 0; value < 16; value++) {
        unsigned old_value = 0;
        for (size_t i = 0; i < old_inputs.size(); i++) {
            size_t j = std::find(inputs.begin(), inputs.end(), old_inputs[i]) - inputs.begin();
            // Missing logical LUT inputs are tied low.
            if (old_inputs[i] != nullptr && (value & (1u << j)))
                old_value |= 1u << i;
        }
        if (old_init & (1u << old_value))
            init |= 1u << value;
    }
    CellInfo *lut = ctx->createCell(ctx->idf("%s$carry_lut", owner->name.c_str(ctx)), id_LUT4);
    lut->params[id_INIT] = Property(init, 16);
    if (copied_source)
        lut->attrs[ctx->id("S3E_CARRY_ORIGIN")] = Property(source->name.str(ctx));
    for (size_t i = 0; i < inputs.size(); i++) {
        IdString pin = ctx->idf("I%zu", i);
        lut->addInput(pin);
        lut->connectPort(pin, inputs[i]);
    }
    NetInfo *out = ctx->createNet(ctx->idf("%s$carry_s", owner->name.c_str(ctx)));
    lut->addOutput(id_O);
    lut->connectPort(id_O, out);
    return lut;
}

CellInfo *find_xor(NetInfo *ci, NetInfo *li, const std::unordered_set<CellInfo *> &used)
{
    if (ci == nullptr)
        return nullptr;
    for (const auto &user : ci->users)
        if (user.cell->type == id_XORCY && user.port == id_CI && !used.count(user.cell) &&
            (li == nullptr || user.cell->getPort(id_LI) == li))
            return user.cell;
    return nullptr;
}

IseCarryBit prepare_bit(Context *ctx, CellInfo *mux, CellInfo *xorcy)
{
    IseCarryBit bit;
    bit.mux = mux;
    bit.xorcy = xorcy;
    NetInfo *propagate = mux != nullptr ? mux->getPort(id_S) : xorcy->getPort(id_LI);
    NetInfo *di = mux != nullptr ? mux->getPort(id_DI) : nullptr;
    if (di != nullptr && di->driver.cell != nullptr && di->driver.cell->type == ctx->id("MULT_AND")) {
        bit.andcell = di->driver.cell;
        bit.di = IseCarryDi::Product;
        di = nullptr;
    } else if (mux != nullptr) {
        auto constant = constant_value(di);
        if (constant.has_value()) {
            bit.di = *constant ? IseCarryDi::One : IseCarryDi::Zero;
            mux->disconnectPort(id_DI);
            di = nullptr;
        } else {
            if (di == nullptr)
                log_error("Spartan-3E ISE: carry %s has no DI input\n", mux->name.c_str(ctx));
            bit.di = IseCarryDi::LutInput;
        }
    }
    bit.lut = private_lut(ctx, mux != nullptr ? mux : xorcy, propagate, di, bit.andcell);
    NetInfo *s = bit.lut->getPort(id_O);
    if (mux != nullptr) {
        mux->disconnectPort(id_S);
        mux->connectPort(id_S, s);
    }
    if (xorcy != nullptr) {
        xorcy->disconnectPort(id_LI);
        xorcy->connectPort(id_LI, s);
        NetInfo *sum = xorcy->getPort(id_O);
        if (sum != nullptr)
            for (const auto &user : sum->users)
                if (user.port == id_D && user.cell->cluster == ClusterId() &&
                    user.cell->type.in(id_FD, id_FDCE, id_FDC, id_FDPE, id_FDP, id_FDRE, id_FDSE)) {
                    bit.ff = user.cell;
                    break;
                }
    }
    return bit;
}

void cluster_chain(IseCarryChain &chain)
{
    for (size_t i = 0; i < chain.bits.size(); i++) {
        const auto &bit = chain.bits[i];
        for (auto *cell : {bit.mux, bit.xorcy, bit.lut, bit.ff, bit.andcell}) {
            if (cell == nullptr)
                continue;
            NPNR_ASSERT(cell->cluster == ClusterId());
            cell->cluster = chain.root->name;
            // Approximate placement bounds; actual slots follow chipdb connectivity.
            cell->constr_x = 0;
            cell->constr_y = -int(i / 4);
            cell->constr_abs_z = false;
            if (cell != chain.root)
                chain.root->constr_children.push_back(cell);
        }
    }
}

} // namespace

std::vector<IseCarryChain> pack_ise_carries(Context *ctx)
{
    // ISE MAP permits MULT_AND only as a single MUXCY.DI source. XST can
    // rewrite other shapes first, but the raw primitive graph must obey MAP.
    std::vector<CellInfo *> products;
    for (auto &entry : ctx->cells)
        if (entry.second->type == ctx->id("MULT_AND")) products.push_back(entry.second.get());
    std::sort(products.begin(), products.end(), [](CellInfo *a, CellInfo *b) { return a->name < b->name; });
    for (auto *product : products) {
        auto *output = product->getPort(ctx->id("LO"));
        if (output == nullptr) log_error("Spartan-3E ISE: disconnected MULT_AND %s\n", ctx->nameOf(product));
        if (output->users.entries() != 1 || (*output->users.begin()).cell->type != id_MUXCY ||
            (*output->users.begin()).port != id_DI)
            log_error("Spartan-3E ISE: MULT_AND %s must drive exactly one MUXCY.DI (ISE MAP policy)\n", ctx->nameOf(product));
    }
    std::vector<CellInfo *> muxes, xors;
    for (auto &entry : ctx->cells) {
        if (entry.second->type == id_MUXCY)
            muxes.push_back(entry.second.get());
        else if (entry.second->type == id_XORCY)
            xors.push_back(entry.second.get());
    }
    auto ordered = [](CellInfo *a, CellInfo *b) { return a->name < b->name; };
    std::sort(muxes.begin(), muxes.end(), ordered);
    std::sort(xors.begin(), xors.end(), ordered);
    // A carry output has one dedicated successor. Other branches retain the
    // same logical net and start independent chains through a fabric/BX entry,
    // as ISE MAP does. Pick before rewriting any propagate inputs so neither
    // iteration order nor private LUT creation changes the decomposition.
    dict<IdString, CellInfo *> successor;
    std::unordered_set<CellInfo *> dedicated_children;
    for (auto *mux : muxes) {
        std::vector<CellInfo *> children;
        if (auto *co = mux->getPort(id_O))
            for (const auto &user : co->users)
                if (user.cell->type == id_MUXCY && user.port == id_CI)
                    children.push_back(user.cell);
        std::sort(children.begin(), children.end(), ordered);
        if (!children.empty()) {
            successor[mux->name] = children.front();
            dedicated_children.insert(children.front());
        }
    }
    std::unordered_set<CellInfo *> used;
    std::vector<IseCarryChain> chains;
    for (auto *root : muxes) {
        if (dedicated_children.count(root))
            continue;
        IseCarryChain chain;
        chain.root = root;
        CellInfo *mux = root;
        while (mux != nullptr) {
            if (!used.insert(mux).second)
                log_error("Spartan-3E ISE: carry chain revisits %s\n", mux->name.c_str(ctx));
            CellInfo *xorcy = find_xor(mux->getPort(id_CI), mux->getPort(id_S), used);
            if (xorcy != nullptr)
                used.insert(xorcy);
            chain.bits.push_back(prepare_bit(ctx, mux, xorcy));
            NetInfo *co = mux->getPort(id_O);
            CellInfo *next = successor.count(mux->name) ? successor.at(mux->name) : nullptr;
            if (next == nullptr) {
                auto *terminal = find_xor(co, nullptr, used);
                if (terminal != nullptr) {
                    used.insert(terminal);
                    chain.bits.push_back(prepare_bit(ctx, nullptr, terminal));
                }
            }
            mux = next;
        }
        cluster_chain(chain);
        chains.push_back(std::move(chain));
    }
    for (auto *mux : muxes)
        if (!used.count(mux))
            log_error("Spartan-3E ISE: carry chain has no root: %s\n", mux->name.c_str(ctx));
    for (auto *xorcy : xors) {
        if (used.count(xorcy))
            continue;
        NetInfo *ci = xorcy->getPort(id_CI);
        if (ci != nullptr && ci->driver.cell != nullptr && ci->driver.cell->type == id_MUXCY)
            log_error("Spartan-3E ISE: unpaired carry-driven XORCY %s\n", xorcy->name.c_str(ctx));
        IseCarryChain chain;
        chain.root = xorcy;
        chain.bits.push_back(prepare_bit(ctx, nullptr, xorcy));
        cluster_chain(chain);
        chains.push_back(std::move(chain));
    }

    // Remove propagate LUTs made redundant by the private copies. Iterate to
    // also remove an upstream LUT after its last buffer consumer disappears.
    bool removed;
    do {
        removed = false;
        std::vector<IdString> dead;
        for (auto &entry : ctx->cells) {
            auto *cell = entry.second.get();
            NetInfo *out = cell->getPort(id_O);
            if (is_lut(cell->type) && cell->cluster == ClusterId() && out != nullptr && out->users.entries() == 0)
                dead.push_back(cell->name);
        }
        for (auto name : dead) {
            auto *cell = ctx->cells.at(name).get();
            for (auto &port : cell->ports)
                cell->disconnectPort(port.first);
            ctx->cells.erase(name);
            removed = true;
        }
    } while (removed);
    log_info("Spartan-3E ISE: packed %zu MUXCY and %zu XORCY into %zu carry chains\n", muxes.size(), xors.size(),
             chains.size());
    return chains;
}

NEXTPNR_NAMESPACE_END
