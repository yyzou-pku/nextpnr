#ifndef SPARTAN3E_MULT_H
#define SPARTAN3E_MULT_H

#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>
#include "log.h"
#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN

struct MultShape
{
    bool wrapper = false;
    bool areg = false;
    bool breg = false;
    bool preg = false;
    bool cascade = false;
    bool clocked() const { return areg || breg || preg; }
};

struct MultChain
{
    CellInfo *root = nullptr;
    std::vector<CellInfo *> cells;
};

inline std::optional<MultShape> mult_shape(Context *ctx, const CellInfo *cell);

inline std::vector<MultChain> pack_mult_chains(Context *ctx)
{
    std::map<CellInfo *, CellInfo *> next, previous;
    for (const auto &entry : ctx->cells) {
        CellInfo *cell = entry.second.get();
        auto shape = mult_shape(ctx, cell);
        if (!shape || !shape->cascade)
            continue;
        CellInfo *source = nullptr;
        for (int bit = 0; bit < 18; ++bit) {
            NetInfo *input = cell->getPort(ctx->idf("BCIN[%d]", bit));
            if (input == nullptr || input->driver.cell == nullptr ||
                input->driver.cell->type != ctx->id("MULT18X18SIO") || input->driver.port != ctx->idf("BCOUT[%d]", bit))
                log_error("spartan3e: MULT cascade requires all eighteen matching BCOUT bits\n");
            if (source != nullptr && source != input->driver.cell)
                log_error("spartan3e: MULT cascade bits have different predecessors\n");
            source = input->driver.cell;
        }
        if (source == cell || !next.emplace(source, cell).second || !previous.emplace(cell, source).second)
            log_error("spartan3e: ambiguous or branched dedicated MULT cascade\n");
    }
    std::set<CellInfo *> used;
    std::vector<MultChain> result;
    for (const auto &edge : next) {
        CellInfo *root = edge.first;
        if (previous.count(root))
            continue;
        MultChain chain{root, {}};
        for (CellInfo *cell = root; cell != nullptr; cell = next.count(cell) ? next.at(cell) : nullptr) {
            if (!used.insert(cell).second || cell->cluster != ClusterId())
                log_error("spartan3e: MULT cascade cycle or cluster conflict\n");
            chain.cells.push_back(cell);
            cell->cluster = root->name;
            cell->constr_x = 0;
            cell->constr_y = 0;
            cell->constr_abs_z = false;
            if (cell != root)
                root->constr_children.push_back(cell);
        }
        result.push_back(std::move(chain));
    }
    for (const auto &edge : next)
        if (!used.count(edge.first))
            log_error("spartan3e: MULT cascade has no acyclic root\n");
    return result;
}

inline std::optional<MultShape> mult_shape(Context *ctx, const CellInfo *cell)
{
    if (cell->type == ctx->id("MULT18X18"))
        return MultShape{true, false, false, false, false};
    if (cell->type == ctx->id("MULT18X18S"))
        return MultShape{true, false, false, true, false};
    if (cell->type != ctx->id("MULT18X18SIO"))
        return std::nullopt;
    auto enabled = [&](const char *parameter) {
        auto found = cell->params.find(ctx->id(parameter));
        return found == cell->params.end() || found->second.as_int64() == 1;
    };
    auto mode = cell->params.find(ctx->id("B_INPUT"));
    return MultShape{false, enabled("AREG"), enabled("BREG"), enabled("PREG"),
                     mode != cell->params.end() && mode->second.as_string() == "CASCADE"};
}

inline bool mult_type(Context *ctx, IdString type)
{
    return type.in(ctx->id("MULT18X18"), ctx->id("MULT18X18S"), ctx->id("MULT18X18SIO"));
}

inline bool mult_inverted(Context *ctx, CellInfo *cell, const std::string &port)
{
    auto parameter = cell->params.find(ctx->id("IS_" + port + "_INVERTED"));
    return parameter != cell->params.end() && parameter->second.as_int64() != 0;
}

inline bool mult_bus_port(const std::string &port, const std::string &bus, int width)
{
    for (int bit = 0; bit < width; ++bit)
        if (port == bus + "[" + std::to_string(bit) + "]")
            return true;
    return false;
}

inline bool mult_inactive(const MultShape &shape, const std::string &port)
{
    if (shape.wrapper)
        return false;
    if (mult_bus_port(port, "BCIN", 18))
        return !shape.cascade;
    if (mult_bus_port(port, "B", 18))
        return shape.cascade;
    if (port == "CLK")
        return !shape.clocked();
    if (port == "CEA" || port == "RSTA")
        return !shape.areg;
    if (port == "CEB" || port == "RSTB")
        return !shape.breg;
    if (port == "CEP" || port == "RSTP")
        return !shape.preg;
    return false;
}

inline void remap_mult(Context *ctx, CellInfo *cell)
{
    auto shape = *mult_shape(ctx, cell);
    for (const auto &parameter : cell->params) {
        std::string name = parameter.first.str(ctx);
        if (!shape.wrapper && name == "B_INPUT") {
            std::string mode = parameter.second.as_string();
            if (mode != "DIRECT" && mode != "CASCADE")
                log_error("spartan3e: unsupported MULT B_INPUT\n");
            continue;
        }
        bool accepted =
                !shape.wrapper && (name == "AREG" || name == "BREG" || name == "PREG" || name == "PREG_CLKINVERSION");
        for (const std::string port : {"CLK", "CEA", "CEB", "CEP", "RSTA", "RSTB", "RSTP"})
            accepted |= !shape.wrapper && name == "IS_" + port + "_INVERTED";
        for (const std::string port : {"C", "CE", "R"})
            accepted |= shape.wrapper && shape.preg && name == "IS_" + port + "_INVERTED";
        if (!accepted || parameter.second.is_string || parameter.second.as_int64() < 0 ||
            parameter.second.as_int64() > 1)
            log_error("spartan3e: unsupported MULT parameter %s\n", name.c_str());
        if (name == "PREG_CLKINVERSION" && parameter.second.as_int64() != 0)
            log_error("spartan3e: inverted MULT PREG clock is not supported\n");
    }
    std::vector<IdString> inactive;
    for (const auto &entry : cell->ports) {
        const std::string logical = entry.first.str(ctx);
        bool accepted =
                mult_bus_port(logical, "A", 18) || mult_bus_port(logical, "B", 18) || mult_bus_port(logical, "P", 36);
        if (!shape.wrapper) {
            accepted |= mult_bus_port(logical, "BCIN", 18) || mult_bus_port(logical, "BCOUT", 18);
            for (const std::string control : {"CLK", "CEA", "CEB", "CEP", "RSTA", "RSTB", "RSTP"})
                accepted |= logical == control;
        } else if (shape.preg) {
            accepted |= logical == "C" || logical == "CE" || logical == "R";
        }
        if (!accepted)
            log_error("spartan3e: unsupported MULT port %s\n", logical.c_str());
        if (mult_inactive(shape, logical)) {
            inactive.push_back(entry.first);
            continue;
        }
        std::string physical = logical;
        auto open = physical.find('[');
        if (open != std::string::npos && physical.back() == ']') {
            physical.erase(physical.size() - 1);
            physical.erase(open, 1);
        }
        if (shape.wrapper && logical == "C")
            physical = "CLK";
        if (shape.wrapper && logical == "CE")
            physical = "CEP";
        if (shape.wrapper && logical == "R")
            physical = "RSTP";
        cell->bel_pins[entry.first] = {ctx->id(physical)};
    }
    for (IdString port : inactive)
        cell->disconnectPort(port);
}

NEXTPNR_NAMESPACE_END
#endif
