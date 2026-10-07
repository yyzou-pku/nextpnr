#ifndef SPARTAN3E_ISE_SRL_H
#define SPARTAN3E_ISE_SRL_H

#include <array>
#include <map>
#include <set>
#include <vector>
#include "log.h"
#include "nextpnr.h"
#include "spartan3e_ise_ff.h"

NEXTPNR_NAMESPACE_BEGIN

inline bool is_ise_srl(Context *ctx, IdString type)
{
    return type.in(ctx->id("SRL16"), ctx->id("SRL16E"), ctx->id("SRLC16"), ctx->id("SRLC16E"));
}

inline bool ise_srl_cascade(Context *ctx, CellInfo *cell)
{
    return cell->type.in(ctx->id("SRLC16"), ctx->id("SRLC16E"));
}

struct IseSrlStage
{
    CellInfo *cell = nullptr;
    CellInfo *ff = nullptr;
};

struct IseSrlChain
{
    CellInfo *root = nullptr;
    std::vector<IseSrlStage> stages;
};

inline std::vector<IseSrlChain> pack_ise_srl_chains(Context *ctx)
{
    std::map<CellInfo *, CellInfo *> next, previous;
    std::vector<CellInfo *> cells;
    for (auto &entry : ctx->cells) {
        CellInfo *cell = entry.second.get();
        if (!is_ise_srl(ctx, cell->type)) continue;
        cells.push_back(cell);
        if (!ise_srl_cascade(ctx, cell)) continue;
        NetInfo *tail = cell->getPort(ctx->id("Q15"));
        if (tail == nullptr) continue;
        for (const PortRef &user : tail->users) {
            if (user.port != ctx->id("D") || !is_ise_srl(ctx, user.cell->type)) continue;
            if (next.count(cell)) log_error("spartan3e-ise: SRL Q15 has multiple dedicated successors\n");
            next[cell] = user.cell;
            if (!previous.emplace(user.cell, cell).second)
                log_error("spartan3e-ise: ambiguous SRL predecessor\n");
        }
    }
    std::set<CellInfo *> used;
    std::vector<IseSrlChain> result;
    for (CellInfo *root : cells) {
        if (previous.count(root) || !next.count(root)) continue;
        IseSrlChain chain{root, {}};
        for (CellInfo *cell = root; cell != nullptr; cell = next.count(cell) ? next.at(cell) : nullptr) {
            if (!used.insert(cell).second) log_error("spartan3e-ise: SRL cascade cycle\n");
            if (cell->cluster != ClusterId()) log_error("spartan3e-ise: SRL already belongs to another cluster\n");
            if (chain.stages.size() % 2 == 1) {
                CellInfo *peer = chain.stages.back().cell;
                if (cell->getPort(ctx->id("CLK")) != peer->getPort(ctx->id("CLK")) ||
                    cell->getPort(ctx->id("CE")) != peer->getPort(ctx->id("CE")))
                    log_error("spartan3e-ise: dedicated SRL pair requires common CLK and CE\n");
            }
            CellInfo *ff = nullptr;
            NetInfo *tap = cell->getPort(ctx->id("Q"));
            if (tap != nullptr)
                for (const PortRef &user : tap->users) {
                    CellInfo *candidate = user.cell;
                    if (user.port != ctx->id("D") || candidate->cluster != ClusterId() ||
                        !candidate->type.in(ctx->id("FD"), ctx->id("FDRE"), ctx->id("FDSE"),
                            ctx->id("FDCE"), ctx->id("FDPE"), ctx->id("FDC"), ctx->id("FDP")) ||
                        candidate->getPort(ctx->id("C")) != cell->getPort(ctx->id("CLK"))) continue;
                    bool reset = false;
                    for (const char *port : {"R", "S", "CLR", "PRE"}) reset |= candidate->getPort(ctx->id(port)) != nullptr;
                    if (!reset) { ff = candidate; break; }
                }
            chain.stages.push_back({cell, ff});
            for (CellInfo *member : {cell, ff}) {
                if (member == nullptr) continue;
                member->cluster = root->name;
                member->constr_x = 0;
                member->constr_y = 0;
                member->constr_abs_z = false;
                if (member != root) root->constr_children.push_back(member);
            }
        }
        result.push_back(std::move(chain));
    }
    for (const auto &entry : next)
        if (!used.count(entry.first)) log_error("spartan3e-ise: SRL cascade has no acyclic root\n");
    return result;
}

inline void validate_ise_srl(Context *ctx, CellInfo *cell)
{
    for (const auto &param : cell->params) {
        const std::string key = param.first.str(ctx);
        if (key == "INIT") {
            if (param.second.is_string || param.second.size() > 16)
                log_error("spartan3e-ise: invalid SRL INIT on %s\n", ctx->nameOf(cell));
        } else if (key != "IS_CLK_INVERTED" || param.second.as_int64() != 0) {
            log_error("spartan3e-ise: unsupported SRL parameter %s on %s\n", key.c_str(), ctx->nameOf(cell));
        }
    }
}

inline bool ise_srl_resources_valid(Context *ctx, const std::array<CellInfo *, 2> &logic,
                                    const std::array<CellInfo *, 2> &ffs,
                                    const std::vector<CellInfo *> &excluded)
{
    for (int half = 0; half < 2; ++half) {
        CellInfo *srl = logic[half];
        if (srl == nullptr || !is_ise_srl(ctx, srl->type)) continue;
        for (CellInfo *cell : excluded) if (cell != nullptr) return false;
        CellInfo *peer = logic[1 - half];
        // ISE requires F SHIFT_REG to have G SHIFT_REG or RAM. Support the
        // proven two-SRL form only; a standalone shift register must use G.
        if (half == 0 && (peer == nullptr || !is_ise_srl(ctx, peer->type))) return false;
        if (peer != nullptr) {
            if (peer->type == ctx->id("RAM16X1S")) return false;
            if (is_ise_srl(ctx, peer->type) &&
                (peer->getPort(ctx->id("CLK")) != srl->getPort(ctx->id("CLK")) ||
                 peer->getPort(ctx->id("CE")) != srl->getPort(ctx->id("CE")))) return false;
        }
        for (int other = 0; other < 2; ++other) {
            CellInfo *ff = ffs[other];
            if (ff == nullptr) continue;
            if (ff->getPort(ctx->id("C")) != srl->getPort(ctx->id("CLK")) || ise_ff_clock_inverted(ctx, ff)) return false;
            for (const char *port : {"R", "S", "CLR", "PRE"})
                if (ff->getPort(ctx->id(port)) != nullptr) return false;
            if (half == other && ff->getPort(ctx->id("D")) != srl->getPort(ctx->id("Q")) &&
                ff->getPort(ctx->id("D")) != srl->getPort(ctx->id("D"))) return false;
        }
    }
    return true;
}

NEXTPNR_NAMESPACE_END

#endif
