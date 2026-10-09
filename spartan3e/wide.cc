/* Native wide-LUT clustering. Physical placement is resolved from chipdb links. */
#include "wide.h"
#include <algorithm>
#include <map>
#include <set>
#include "log.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
int level(Context *ctx, CellInfo *cell)
{
    const auto type = cell->type.str(ctx);
    if (type == "MUXF5")
        return 5;
    if (type == "MUXF6")
        return 6;
    if (type == "MUXF7")
        return 7;
    if (type == "MUXF8")
        return 8;
    if (type == "LUT1" || type == "LUT2" || type == "LUT3" || type == "LUT4")
        return 4;
    return -1;
}
} // namespace

std::vector<WideTree> pack_wide(Context *ctx)
{
    std::vector<CellInfo *> muxes;
    for (auto &entry : ctx->cells)
        if (level(ctx, entry.second.get()) >= 5)
            muxes.push_back(entry.second.get());
    auto ordered = [&](CellInfo *a, CellInfo *b) {
        int al = level(ctx, a), bl = level(ctx, b);
        return al != bl ? al > bl : a->name.str(ctx) < b->name.str(ctx);
    };
    std::sort(muxes.begin(), muxes.end(), ordered);
    std::map<CellInfo *, std::array<CellInfo *, 2>> children;
    std::map<CellInfo *, std::vector<CellInfo *>> neighbors;
    std::map<CellInfo *, CellInfo *> f7_parent;
    for (CellInfo *mux : muxes) {
        for (int input = 0; input < 2; ++input) {
            NetInfo *net = mux->getPort(ctx->idf("I%d", input));
            if (net == nullptr || net->driver.cell == nullptr || net->driver.port != ctx->id("O") ||
                level(ctx, net->driver.cell) != level(ctx, mux) - 1)
                log_error("spartan3e: MUXF%d %s needs two dedicated level-%d sources\n", level(ctx, mux),
                          ctx->nameOf(mux), level(ctx, mux) - 1);
            CellInfo *source = net->driver.cell;
            if (level(ctx, source) == 7) {
                auto parent = f7_parent.emplace(source, mux);
                if (!parent.second && parent.first->second != mux)
                    log_error("spartan3e: MUXF7 %s may drive at most one MUXF8 dedicated parent\n",
                              ctx->nameOf(source));
            }
            children[mux][input] = source;
            neighbors[mux].push_back(source);
            neighbors[source].push_back(mux);
        }
    }
    std::vector<WideTree> components;
    std::set<CellInfo *> used;
    for (CellInfo *anchor : muxes) {
        if (used.count(anchor))
            continue;
        std::vector<CellInfo *> members{anchor};
        used.insert(anchor);
        for (size_t head = 0; head < members.size(); ++head)
            for (CellInfo *neighbor : neighbors.at(members[head]))
                if (used.insert(neighbor).second)
                    members.push_back(neighbor);
        std::sort(members.begin(), members.end(), ordered);
        WideTree component;
        component.root = members.front();
        std::map<CellInfo *, int> indices;
        for (CellInfo *cell : members) {
            if (cell->cluster != ClusterId())
                log_error("spartan3e: already clustered wide-LUT resource %s\n", ctx->nameOf(cell));
            indices[cell] = int(component.nodes.size());
            component.nodes.push_back(WideNode{cell, {{-1, -1}}});
        }
        for (size_t i = 0; i < members.size(); ++i) {
            CellInfo *cell = members[i];
            auto inputs = children.find(cell);
            if (inputs != children.end())
                for (int input = 0; input < 2; ++input)
                    component.nodes[i].children[input] = indices.at(inputs->second[input]);
            cell->cluster = component.root->name;
            cell->constr_x = 0;
            cell->constr_y = 0;
            cell->constr_abs_z = false;
            if (cell != component.root)
                component.root->constr_children.push_back(cell);
        }
        components.push_back(std::move(component));
    }
    log_info("Spartan-3E: packed %zu dedicated wide-LUT DAG components\n", components.size());
    return components;
}

NEXTPNR_NAMESPACE_END
