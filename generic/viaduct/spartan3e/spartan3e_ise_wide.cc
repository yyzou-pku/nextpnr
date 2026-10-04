/* Native wide-LUT clustering. Physical placement is resolved from chipdb links. */
#include "spartan3e_ise_wide.h"
#include <functional>
#include <set>
#include "log.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
int level(Context *ctx, CellInfo *cell)
{
    const auto type = cell->type.str(ctx);
    if (type == "MUXF5") return 5;
    if (type == "MUXF6") return 6;
    if (type == "MUXF7") return 7;
    if (type == "MUXF8") return 8;
    if (type == "LUT1" || type == "LUT2" || type == "LUT3" || type == "LUT4") return 4;
    return -1;
}
}

std::vector<IseWideTree> pack_ise_wide(Context *ctx)
{
    std::vector<IseWideTree> trees;
    std::set<CellInfo *> used;
    for (auto &entry : ctx->cells) {
        CellInfo *root = entry.second.get();
        if (level(ctx, root) < 5) continue;
        auto *out = root->getPort(ctx->id("O"));
        bool child = false;
        if (out != nullptr)
            for (const auto &user : out->users)
                child |= level(ctx, user.cell) > 5 && user.port.in(ctx->id("I0"), ctx->id("I1"));
        if (child) continue;
        IseWideTree tree;
        tree.root = root;
        std::function<int(CellInfo *)> visit = [&](CellInfo *cell) {
            if (!used.insert(cell).second || cell->cluster != ClusterId())
                log_error("spartan3e-ise: shared/cyclic or already clustered wide-LUT resource %s\n", ctx->nameOf(cell));
            int index = int(tree.nodes.size());
            tree.nodes.push_back(IseWideNode{cell, {{-1, -1}}});
            cell->cluster = root->name;
            cell->constr_x = 0;
            cell->constr_y = 0;
            cell->constr_abs_z = false;
            if (cell != root) root->constr_children.push_back(cell);
            int depth = level(ctx, cell);
            if (depth > 4) {
                for (int i = 0; i < 2; ++i) {
                    NetInfo *net = cell->getPort(ctx->idf("I%d", i));
                    if (net == nullptr || net->driver.cell == nullptr || net->driver.port != ctx->id("O") ||
                        level(ctx, net->driver.cell) != depth - 1)
                        log_error("spartan3e-ise: MUXF%d %s needs two dedicated level-%d sources\n",
                                  depth, ctx->nameOf(cell), depth - 1);
                    if (net->users.entries() != 1)
                        log_error("spartan3e-ise: wide cascade source %s has unsupported fanout\n", ctx->nameOf(net));
                    int source = visit(net->driver.cell);
                    tree.nodes[index].children[i] = source;
                }
            }
            return index;
        };
        visit(root);
        trees.push_back(std::move(tree));
    }
    for (auto &entry : ctx->cells)
        if (level(ctx, entry.second.get()) >= 5 && !used.count(entry.second.get()))
            log_error("spartan3e-ise: wide cascade has no root: %s\n", ctx->nameOf(entry.second.get()));
    log_info("Spartan-3E ISE: packed %zu dedicated wide-LUT trees\n", trees.size());
    return trees;
}

NEXTPNR_NAMESPACE_END
