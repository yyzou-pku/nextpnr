#ifndef SPARTAN3E_FF_H
#define SPARTAN3E_FF_H

#include <optional>
#include <string>
#include <utility>
#include "log.h"
#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN

struct FfShape
{
    bool negative = false;
    bool synchronous = false;
    bool latch = false;
    const char *sr = nullptr;
    const char *rev = nullptr;
    bool sr_high = false;
};

inline std::optional<FfShape> ff_shape(const std::string &type)
{
    FfShape shape;
    std::string base = type;
    if (base.size() > 2 && base.compare(base.size() - 2, 2, "_1") == 0) {
        shape.negative = true;
        base.resize(base.size() - 2);
    }
    if (base == "FD" || base == "FDE")
        return shape;
    if (base == "FDC" || base == "FDCE")
        shape.sr = "CLR";
    else if (base == "FDP" || base == "FDPE") {
        shape.sr = "PRE";
        shape.sr_high = true;
    } else if (base == "FDR" || base == "FDRE") {
        shape.sr = "R";
        shape.synchronous = true;
    } else if (base == "FDS" || base == "FDSE") {
        shape.sr = "S";
        shape.synchronous = true;
        shape.sr_high = true;
    } else if (base == "FDCP" || base == "FDCPE") {
        shape.sr = "CLR";
        shape.rev = "PRE";
    } else if (base == "FDRS" || base == "FDRSE") {
        shape.sr = "R";
        shape.rev = "S";
        shape.synchronous = true;
    } else if (base == "LD" || base == "LDE")
        shape.latch = true;
    else if (base == "LDC" || base == "LDCE") {
        shape.latch = true;
        shape.sr = "CLR";
    } else if (base == "LDP" || base == "LDPE") {
        shape.latch = true;
        shape.sr = "PRE";
        shape.sr_high = true;
    } else if (base == "LDCP" || base == "LDCPE") {
        shape.latch = true;
        shape.sr = "CLR";
        shape.rev = "PRE";
    } else
        return std::nullopt;
    return shape;
}

inline bool ff_inverted(Context *ctx, const CellInfo *cell, const std::string &port)
{
    auto parameter = cell->params.find(ctx->id("IS_" + port + "_INVERTED"));
    return parameter != cell->params.end() && parameter->second.as_int64() != 0;
}

inline bool ff_clock_inverted(Context *ctx, const CellInfo *cell)
{
    auto shape = ff_shape(cell->type.str(ctx));
    // The CLB latch is transparent for CK low; a positive logical G must
    // therefore select CLK_B. Edge-triggered FFs use the opposite convention.
    return shape.has_value() && ((shape->negative != ff_inverted(ctx, cell, shape->latch ? "G" : "C")) != shape->latch);
}

inline bool legacy_iob_ff(Context *ctx, const CellInfo *cell)
{
    if (!cell->type.in(ctx->id("FD"), ctx->id("FDCE"), ctx->id("FDC"), ctx->id("FDPE"), ctx->id("FDP"), ctx->id("FDRE"),
                       ctx->id("FDSE")))
        return false;
    auto shape = ff_shape(cell->type.str(ctx));
    if (!shape || shape->latch || shape->negative || shape->rev != nullptr || ff_clock_inverted(ctx, cell))
        return false;
    for (const auto &parameter : cell->params)
        if (parameter.first.str(ctx) != "INIT" && parameter.second.as_int64() != 0)
            return false;
    return true;
}

inline void validate_ff(Context *ctx, CellInfo *cell)
{
    auto shape = ff_shape(cell->type.str(ctx));
    if (!shape)
        return;
    if (!cell->params.count(ctx->id("INIT")))
        cell->params[ctx->id("INIT")] = Property(shape->sr_high ? 1 : 0, 1);
    for (const auto &parameter : cell->params) {
        const std::string key = parameter.first.str(ctx);
        if (key == "INIT") {
            if (parameter.second.is_string || parameter.second.size() > 1)
                log_error("spartan3e: invalid FF INIT on %s\n", ctx->nameOf(cell));
            continue;
        }
        bool accepted = key == (shape->latch ? "IS_G_INVERTED" : "IS_C_INVERTED") ||
                        key == (shape->latch ? "IS_GE_INVERTED" : "IS_CE_INVERTED") ||
                        (shape->sr != nullptr && key == std::string("IS_") + shape->sr + "_INVERTED") ||
                        (shape->rev != nullptr && key == std::string("IS_") + shape->rev + "_INVERTED");
        if (!accepted || parameter.second.is_string || parameter.second.size() > 1)
            log_error("spartan3e: unsupported FF parameter %s on %s\n", key.c_str(), ctx->nameOf(cell));
    }
}

inline bool ff_pair_valid(Context *ctx, CellInfo *a, CellInfo *b)
{
    if (a == nullptr || b == nullptr)
        return true;
    auto as = ff_shape(a->type.str(ctx)), bs = ff_shape(b->type.str(ctx));
    if (!as || !bs)
        return false;
    if (a->getPort(ctx->id(as->latch ? "G" : "C")) != b->getPort(ctx->id(bs->latch ? "G" : "C")) ||
        ff_clock_inverted(ctx, a) != ff_clock_inverted(ctx, b))
        return false;
    const char *ace = as->latch ? "GE" : "CE", *bce = bs->latch ? "GE" : "CE";
    if (a->getPort(ctx->id(ace)) != b->getPort(ctx->id(bce)))
        return false;
    if (a->getPort(ctx->id(ace)) != nullptr && ff_inverted(ctx, a, ace) != ff_inverted(ctx, b, bce))
        return false;
    auto control = [&](CellInfo *cell, const char *port) {
        return port == nullptr ? nullptr : cell->getPort(ctx->id(port));
    };
    for (auto ports : {std::make_pair(as->sr, bs->sr), std::make_pair(as->rev, bs->rev)}) {
        NetInfo *an = control(a, ports.first), *bn = control(b, ports.second);
        if (an != bn)
            return false;
        if (an != nullptr && ff_inverted(ctx, a, ports.first) != ff_inverted(ctx, b, ports.second))
            return false;
    }
    if ((control(a, as->sr) != nullptr || control(a, as->rev) != nullptr) && as->synchronous != bs->synchronous)
        return false;
    return true;
}

NEXTPNR_NAMESPACE_END
#endif
