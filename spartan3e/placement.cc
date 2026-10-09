#include "implementation.h"

#include "constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace spartan3e_detail {

bool Spartan3eImpl::place_mult_chain(const MultChain &chain, BelId root,
                                     std::vector<std::pair<CellInfo *, BelId>> &placement) const
{
    auto found = mult_site_by_bel.find(root.index);
    if (found == mult_site_by_bel.end())
        return false;
    int site = found->second;
    placement.clear();
    std::set<int32_t> used;
    for (CellInfo *cell : chain.cells) {
        if (site < 0)
            return false;
        const auto &target = mult_sites[site];
        if (!used.insert(target.bel.index).second)
            return false;
        auto fixed = cell->attrs.find(ctx->id("BEL"));
        if (fixed != cell->attrs.end() && fixed->second.as_string() != ctx->getBelName(target.bel).str(ctx))
            return false;
        auto loc = cell->attrs.find(ctx->id("LOC"));
        if (loc != cell->attrs.end() && loc->second.as_string() != db.tiles[target.site.tile].sites[target.site.site])
            return false;
        CellInfo *occupied = ctx->getBoundBelCell(target.bel);
        if (occupied != nullptr && occupied != cell && occupied->cluster != chain.root->name)
            return false;
        placement.emplace_back(cell, target.bel);
        site = target.next;
    }
    return true;
}

bool Spartan3eImpl::place_srl_chain(const SrlChain &chain, BelId root_bel,
                                    std::vector<std::pair<CellInfo *, BelId>> &placement) const
{
    auto root = carry_bel_slot.find(root_bel.index);
    if (root == carry_bel_slot.end() || root->second.second != 1 || carry_sites[root->second.first].lut[1] != root_bel)
        return false;
    placement.clear();
    int site = root->second.first;
    std::set<int32_t> used;
    for (size_t i = 0; i < chain.stages.size(); ++i) {
        if (site < 0)
            return false;
        const auto &ref = carry_sites[site].site;
        if (db.variants[db.tiles[ref.tile].variant].sites[ref.site].type != "SLICEM")
            return false;
        int half = i % 2 == 0 ? 1 : 0;
        for (auto target : {std::make_pair(chain.stages[i].cell, carry_sites[site].lut[half]),
                            std::make_pair(chain.stages[i].ff, carry_sites[site].ff[half])}) {
            if (target.first == nullptr)
                continue;
            if (!used.insert(target.second.index).second)
                return false;
            auto fixed = target.first->attrs.find(ctx->id("BEL"));
            if (fixed != target.first->attrs.end() &&
                fixed->second.as_string() != ctx->getBelName(target.second).str(ctx))
                return false;
            auto loc = target.first->attrs.find(ctx->id("LOC"));
            if (loc != target.first->attrs.end() && loc->second.as_string() != db.tiles[ref.tile].sites[ref.site])
                return false;
            CellInfo *occupied = ctx->getBoundBelCell(target.second);
            if (occupied != nullptr && occupied != target.first && occupied->cluster != chain.root->name)
                return false;
            placement.push_back(target);
        }
        if (half == 0 && i + 1 < chain.stages.size())
            site = srl_next_site[site];
    }
    return true;
}

bool Spartan3eImpl::place_wide_tree(const WideTree &tree, BelId root_bel,
                                    std::vector<std::pair<CellInfo *, BelId>> &placement) const
{
    auto root = wide_bel_slot.find(root_bel.index);
    if (root == wide_bel_slot.end() || (tree.root->type == id_MUXF5 ? 0 : 1) != root->second.second)
        return false;
    using Assignment = std::vector<int32_t>;
    std::function<bool(int, int, int, Assignment &)> assign = [&](int node, int site, int half, Assignment &state) {
        if (site < 0)
            return false;
        const auto &entry = tree.nodes[node];
        bool leaf = is_lut_cell(entry.cell->type);
        int kind = entry.cell->type == id_MUXF5 ? 0 : 1;
        BelId bel = leaf ? carry_sites[site].lut[half] : wide_bels[site][kind];
        if (state[node] >= 0)
            return state[node] == bel.index;
        for (size_t i = 0; i < state.size(); ++i)
            if (int(i) != node && state[i] == bel.index)
                return false;
        auto fixed = entry.cell->attrs.find(ctx->id("BEL"));
        if (fixed != entry.cell->attrs.end() && fixed->second.as_string() != ctx->getBelName(bel).str(ctx))
            return false;
        auto loc = entry.cell->attrs.find(ctx->id("LOC"));
        const auto &ref = carry_sites[site].site;
        if (loc != entry.cell->attrs.end() && loc->second.as_string() != db.tiles[ref.tile].sites[ref.site])
            return false;
        state[node] = bel.index;
        if (leaf)
            return true;
        for (int input = 0; input < 2; ++input) {
            int source_kind = entry.cell->type == id_MUXF6 ? 0 : 1;
            int source_site = kind == 0 ? site : wide_sources[site][source_kind][input];
            if (!assign(entry.children[input], source_site, 1 - input, state))
                return false;
        }
        return true;
    };
    Assignment state(tree.nodes.size(), -1);
    if (!assign(0, root->second.first, 0, state))
        return false;
    std::function<bool(Assignment &)> complete = [&](Assignment &current) {
        int pending = -1;
        for (size_t i = 0; i < tree.nodes.size(); ++i) {
            if (current[i] >= 0)
                continue;
            for (int child : tree.nodes[i].children)
                if (child >= 0 && current[child] >= 0) {
                    pending = int(i);
                    break;
                }
            if (pending >= 0)
                break;
        }
        if (pending < 0)
            return std::all_of(current.begin(), current.end(), [](int32_t bel) { return bel >= 0; });
        // Another root must agree with an already placed shared child.
        // Candidate relations come from indexed directed chipdb links.
        for (size_t site = 0; site < carry_sites.size(); ++site) {
            Assignment candidate = current;
            if (!assign(pending, int(site), 0, candidate))
                continue;
            if (complete(candidate)) {
                current = std::move(candidate);
                return true;
            }
        }
        return false;
    };
    if (!complete(state))
        return false;
    placement.clear();
    for (size_t i = 0; i < tree.nodes.size(); ++i)
        placement.emplace_back(tree.nodes[i].cell, BelId{state[i]});
    return true;
}

std::optional<bool> Spartan3eImpl::getClusterPlacement(ClusterId cluster, BelId root_bel,
                                                       std::vector<std::pair<CellInfo *, BelId>> &placement) const
{
    auto clock = clock_model->get_cluster_placement(cluster, root_bel, placement);
    if (clock.has_value())
        return clock;
    auto io = iob_off_clusters.find(cluster);
    auto input_io = iob_if_clusters.find(cluster);
    auto tristate_io = iob_tff_clusters.find(cluster);
    if (io != iob_off_clusters.end() || input_io != iob_if_clusters.end() || tristate_io != iob_tff_clusters.end()) {
        placement = {{ctx->cells.at(cluster).get(), root_bel}};
    }
    if (io != iob_off_clusters.end()) {
        auto off = iob_off_bels.find(root_bel.index);
        if (off == iob_off_bels.end())
            return false;
        placement.emplace_back(io->second, off->second);
    }
    if (input_io != iob_if_clusters.end()) {
        auto ff = iob_if_bels.find(root_bel.index);
        if (ff == iob_if_bels.end())
            return false;
        placement.emplace_back(input_io->second, ff->second);
    }
    if (tristate_io != iob_tff_clusters.end()) {
        auto ff = iob_tff_bels.find(root_bel.index);
        if (ff == iob_tff_bels.end())
            return false;
        placement.emplace_back(tristate_io->second, ff->second);
    }
    if (io != iob_off_clusters.end() || input_io != iob_if_clusters.end() || tristate_io != iob_tff_clusters.end())
        return true;
    auto mult = mult_cluster.find(cluster);
    if (mult != mult_cluster.end())
        return place_mult_chain(mult_chains[mult->second], root_bel, placement);
    auto srl = srl_cluster.find(cluster);
    if (srl != srl_cluster.end())
        return place_srl_chain(srl_chains[srl->second], root_bel, placement);
    auto wide = wide_cluster.find(cluster);
    if (wide != wide_cluster.end())
        return place_wide_tree(wide_trees[wide->second], root_bel, placement);
    auto chain_it = carry_cluster.find(cluster);
    if (chain_it == carry_cluster.end())
        return std::nullopt;
    placement.clear();
    auto slot = carry_bel_slot.find(root_bel.index);
    if (slot == carry_bel_slot.end() || slot->second.second != 0)
        return false;
    const auto &chain = carry_chains[chain_it->second];
    if (ctx->getBelType(root_bel) != chain.root->type)
        return false;
    int site = slot->second.first;
    for (size_t i = 0; i < chain.bits.size(); i++) {
        if (site < 0)
            return false;
        int half = i % 2;
        const auto &carry = carry_sites[site];
        const auto &bit = chain.bits[i];
        for (auto target : {std::make_pair(bit.mux, carry.mux[half]), std::make_pair(bit.xorcy, carry.xorcy[half]),
                            std::make_pair(bit.lut, carry.lut[half]), std::make_pair(bit.ff, carry.ff[half]),
                            std::make_pair(bit.andcell, carry.andcell[half])})
            if (target.first != nullptr)
                placement.push_back(target);
        if (half == 1)
            site = carry.next;
    }
    return true;
}

bool Spartan3eImpl::wide_resources_valid(int index) const
{
    const auto &site = carry_sites[index];
    for (int half = 0; half < 2; ++half) {
        CellInfo *wide = ctx->getBoundBelCell(wide_bels[index][half]);
        if (wide == nullptr || !wide->type.in(id_MUXF5, id_MUXF6, id_MUXF7, id_MUXF8))
            continue;
        CellInfo *lut = ctx->getBoundBelCell(site.lut[half]);
        CellInfo *sum = ctx->getBoundBelCell(site.xorcy[half]);
        int outputs = int(needs_slice_output(wide));
        outputs += int(lut != nullptr && is_lut_cell(lut->type) && needs_slice_output(lut));
        outputs += int(sum != nullptr && needs_slice_output(sum));
        if (outputs > 1)
            return false;
        NetInfo *select = wide->getPort(id_S);
        CellInfo *ff = ctx->getBoundBelCell(site.ff[half]);
        if (ff != nullptr && !local_ff_data(ff, site.site, half) && ff->getPort(id_D) != select)
            return false; // External FF data and the mux select share BX/BY.
        if (half == 0) {
            for (BelId carry : {site.mux[0], site.xorcy[0]}) {
                CellInfo *cell = ctx->getBoundBelCell(carry);
                if (cell == nullptr)
                    continue;
                NetInfo *ci = cell->getPort(id_CI);
                bool dedicated = dedicated_carry_input(cell);
                if (!dedicated && ci != select)
                    return false; // CYINIT=BX.
            }
        }
    }
    return true;
}

IdString Spartan3eImpl::getBelBucketForCellType(IdString cell_type) const
{
    if (is_xilinx_io_cell(cell_type))
        return id_IOB;
    if (is_xilinx_ff_cell(cell_type))
        return id_DFF;
    if (is_lut_cell(cell_type))
        return id_LUT4;
    if (cell_type == ctx->id("RAM16X1S") || is_srl(ctx, cell_type))
        return id_LUT4;
    if (bram_shape(cell_type.str(ctx)))
        return ctx->id("RAMB16");
    if (mult_type(ctx, cell_type))
        return ctx->id("MULT18X18SIO");
    if (cell_type.in(id_MUXF6, id_MUXF7, id_MUXF8))
        return id_MUXF6;
    if (cell_type.in(id_BUFG, ctx->id("BUFGMUX_1")))
        return id_BUFGMUX;
    if (cell_type == ctx->id("STARTUP_SPARTAN3E"))
        return id_STARTUP;
    if (cell_type == ctx->id("CAPTURE_SPARTAN3"))
        return id_CAPTURE;
    if (cell_type == ctx->id("BSCAN_SPARTAN3"))
        return id_BSCAN;
    return cell_type;
}

BelBucketId Spartan3eImpl::getBelBucketForBel(BelId bel) const
{
    return ctx->getBelType(bel).in(ctx->id("IOB_OFF1"), ctx->id("IOB_IFF1"), ctx->id("IOB_TFF1"))
                   ? id_DFF
                   : ctx->getBelType(bel);
}

bool Spartan3eImpl::isValidBelForCellType(IdString cell_type, BelId bel) const
{
    IdString bel_type = ctx->getBelType(bel);
    if (bel_type == ctx->id("IOB_OFF1"))
        return cell_type.in(id_FD, id_FDCE, id_FDC, id_FDPE, id_FDP, id_FDRE, id_FDSE);
    if (bel_type == ctx->id("IOB_TFF1"))
        return cell_type.in(id_FD, id_FDCE, id_FDC, id_FDPE, id_FDP, id_FDRE, id_FDSE);
    if (bel_type == ctx->id("IOB_IFF1"))
        return cell_type.in(id_FDCE, id_FDPE);
    if (bel_type == id_IOB) {
        const auto &ref = bel_site_info.at(bel.index).site;
        const auto &tile = db.tiles[ref.tile];
        const auto &type = db.variants[tile.variant].sites[ref.site].type;
        return cell_type == id_IBUF ||
               (cell_type.in(id_OBUF, id_OBUFT, id_IOBUF) && type != "IBUF" && type != "DIFFMI" && type != "DIFFSI");
    }
    if (bel_type == id_DFF)
        return is_xilinx_ff_cell(cell_type);
    if (bel_type == id_LUT4) {
        if (cell_type == ctx->id("RAM16X1S") || is_srl(ctx, cell_type)) {
            const auto &ref = bel_site_info.at(bel.index).site;
            return db.variants[db.tiles[ref.tile].variant].sites[ref.site].type == "SLICEM";
        }
        return is_lut_cell(cell_type);
    }
    if (bel_type == id_MUXF6)
        return cell_type.in(id_MUXF6, id_MUXF7, id_MUXF8);
    if (bel_type == id_BUFGMUX)
        return cell_type.in(id_BUFG, id_BUFGMUX, ctx->id("BUFGMUX_1"));
    if (bel_type == id_STARTUP)
        return cell_type == ctx->id("STARTUP_SPARTAN3E");
    if (bel_type == id_CAPTURE)
        return cell_type == ctx->id("CAPTURE_SPARTAN3");
    if (bel_type == id_BSCAN)
        return cell_type == ctx->id("BSCAN_SPARTAN3");
    if (bel_type == ctx->id("RAMB16"))
        return bram_shape(cell_type.str(ctx)).has_value();
    if (bel_type == ctx->id("MULT18X18SIO"))
        return mult_type(ctx, cell_type);
    return bel_type == cell_type;
}

bool Spartan3eImpl::ff_rev_resources_valid(int index) const
{
    const auto &site = carry_sites[index];
    for (int half = 0; half < 2; ++half) {
        CellInfo *ff = ctx->getBoundBelCell(site.ff[half]);
        if (ff == nullptr)
            continue;
        auto shape = ff_shape(ff->type.str(ctx));
        if (!shape || shape->rev == nullptr)
            continue;
        NetInfo *rev = ff->getPort(ctx->id(shape->rev));
        if (rev == nullptr)
            continue;
        // REV takes the shared BY input. Keep unverified memory/carry/FX
        // mixtures closed; ordinary LUT data paths do not consume BY.
        if (ctx->getBoundBelCell(wide_bels[index][1]) != nullptr)
            return false;
        for (int other = 0; other < 2; ++other) {
            CellInfo *logic = ctx->getBoundBelCell(site.lut[other]);
            if (logic != nullptr && !is_lut_cell(logic->type))
                return false;
            if (ctx->getBoundBelCell(site.mux[other]) != nullptr || ctx->getBoundBelCell(site.xorcy[other]) != nullptr)
                return false;
        }
        CellInfo *y = ctx->getBoundBelCell(site.ff[1]);
        if (y != nullptr && !local_ff_data(y, site.site, 1) &&
            (y->getPort(id_D) != rev || ff_inverted(ctx, ff, shape->rev)))
            return false;
    }
    return true;
}

bool Spartan3eImpl::isBelLocationValid(BelId bel, bool) const
{
    auto wide_slot = wide_bel_slot.find(bel.index);
    auto carry_slot = carry_bel_slot.find(bel.index);
    if (wide_slot != wide_bel_slot.end() || carry_slot != carry_bel_slot.end()) {
        int index = carry_slot != carry_bel_slot.end() ? carry_slot->second.first : wide_slot->second.first;
        if (!ff_rev_resources_valid(index))
            return false;
        const auto &site = carry_sites[index];
        std::array<CellInfo *, 2> logic, ffs;
        std::vector<CellInfo *> excluded;
        for (int half = 0; half < 2; ++half) {
            logic[half] = ctx->getBoundBelCell(site.lut[half]);
            ffs[half] = ctx->getBoundBelCell(site.ff[half]);
            excluded.push_back(ctx->getBoundBelCell(site.mux[half]));
            excluded.push_back(ctx->getBoundBelCell(site.xorcy[half]));
            excluded.push_back(ctx->getBoundBelCell(site.andcell[half]));
            excluded.push_back(ctx->getBoundBelCell(wide_bels[index][half]));
        }
        // Cluster/fixed BELs are checked one at a time. Permit the actual
        // not-yet-bound G member, never an inferred dummy memory.
        // Final route policy still requires an actual occupied G SRL.
        if (logic[0] != nullptr && is_srl(ctx, logic[0]->type) && logic[1] == nullptr) {
            const std::string fixed_g = ctx->getBelName(site.lut[1]).str(ctx);
            CellInfo *pending = nullptr;
            auto cluster = srl_cluster.find(logic[0]->cluster);
            if (cluster != srl_cluster.end()) {
                const auto &stages = srl_chains[cluster->second].stages;
                for (size_t i = 1; i < stages.size(); i += 2)
                    if (stages[i].cell == logic[0]) {
                        CellInfo *peer = stages[i - 1].cell;
                        auto fixed = peer->attrs.find(ctx->id("BEL"));
                        if (peer->bel == BelId() &&
                            (fixed == peer->attrs.end() || fixed->second.as_string() == fixed_g))
                            pending = peer;
                    }
            }
            for (const auto &entry : ctx->cells) {
                CellInfo *candidate = entry.second.get();
                auto fixed = candidate->attrs.find(ctx->id("BEL"));
                if (!is_srl(ctx, candidate->type) || candidate->bel != BelId() || fixed == candidate->attrs.end() ||
                    fixed->second.as_string() != fixed_g)
                    continue;
                if (pending != nullptr && pending != candidate)
                    return false;
                pending = candidate;
            }
            logic[1] = pending;
        }
        if (!srl_resources_valid(ctx, logic, ffs, excluded))
            return false;
    }
    if (wide_slot != wide_bel_slot.end() && !wide_resources_valid(wide_slot->second.first))
        return false;
    if (carry_slot != carry_bel_slot.end() && !wide_resources_valid(carry_slot->second.first))
        return false;
    for (const auto &memory : ram64_resources) {
        if (bel != memory.composite &&
            std::find(memory.excluded.begin(), memory.excluded.end(), bel) == memory.excluded.end())
            continue;
        if (ctx->getBoundBelCell(memory.composite) == nullptr)
            continue;
        for (BelId excluded : memory.excluded)
            if (ctx->getBoundBelCell(excluded) != nullptr)
                return false;
    }
    auto dual = ram16_dual_resource_by_bel.find(bel.index);
    if (dual != ram16_dual_resource_by_bel.end() &&
        !ram16_dual_resources_valid(ctx, ram16_dual_resources[dual->second]))
        return false;
    auto ram32 = ram32_resource_by_bel.find(bel.index);
    if (ram32 != ram32_resource_by_bel.end() && !ram32_resources_valid(ctx, ram32_resources[ram32->second]))
        return false;
    CellInfo *bound = ctx->getBoundBelCell(bel);
    if (bound != nullptr && ff_shape(bound->type.str(ctx)).has_value()) {
        auto loc = bound->attrs.find(ctx->id("LOC"));
        if (loc != bound->attrs.end()) {
            const auto &ref = bel_site_info.at(bel.index).site;
            if (loc->second.as_string() != db.tiles[ref.tile].sites[ref.site])
                return false;
        }
    }
    if (bound != nullptr && ctx->getBelType(bel) == ctx->id("IOB_OFF1") && !iob_off_clusters.count(bound->cluster))
        return false;
    if (bound != nullptr && ctx->getBelType(bel) == ctx->id("IOB_TFF1") && !iob_tff_clusters.count(bound->cluster))
        return false;
    if (bound != nullptr && ctx->getBelType(bel) == ctx->id("IOB_IFF1") && !iob_if_clusters.count(bound->cluster))
        return false;
    if (bound != nullptr && !clock_model->region_valid(bound, bel))
        return false;
    auto slot = carry_bel_slot.find(bel.index);
    if (slot != carry_bel_slot.end()) {
        const auto &site = carry_sites[slot->second.first];
        std::array<CellInfo *, 2> logic{{ctx->getBoundBelCell(site.lut[0]), ctx->getBoundBelCell(site.lut[1])}};
        auto is_ram = [&](CellInfo *cell) { return cell != nullptr && cell->type == ctx->id("RAM16X1S"); };
        for (int half = 0; half < 2; ++half) {
            CellInfo *ram = logic[half];
            if (!is_ram(ram))
                continue;
            for (int other = 0; other < 2; ++other) {
                CellInfo *peer = logic[other];
                if (is_ram(peer) && (peer->getPort(ctx->id("WCLK")) != ram->getPort(ctx->id("WCLK")) ||
                                     peer->getPort(ctx->id("WE")) != ram->getPort(ctx->id("WE"))))
                    return false;
                CellInfo *ff = ctx->getBoundBelCell(site.ff[other]);
                if (ff == nullptr)
                    continue;
                if (ff->getPort(ctx->id("C")) != ram->getPort(ctx->id("WCLK")) || ff_clock_inverted(ctx, ff))
                    return false;
                for (IdString pin : {ctx->id("R"), id_S, ctx->id("CLR"), ctx->id("PRE")})
                    // Slice SR cannot serve the write generator and a register
                    // simultaneously, including aliases of the same logical net.
                    if (ff->getPort(pin) != nullptr)
                        return false;
                if (half == other) {
                    NetInfo *d = ff->getPort(id_D);
                    bool local = d != nullptr && d->driver.cell == ram;
                    if (!local && d != ram->getPort(id_D))
                        return false;
                }
            }
            // F RAM's write decoder uses the G address pins, even in
            // single-port mode. A co-resident G resource must agree.
            if (half == 0 && logic[1] != nullptr)
                for (int i = 0; i < 4; ++i) {
                    NetInfo *address = logic[1]->getPort(ctx->idf(is_ram(logic[1]) ? "A%d" : "I%d", i));
                    if (address != nullptr && address != ram->getPort(ctx->idf("A%d", i)))
                        return false;
                }
        }
    }
    auto bs_it = bel_site_info.find(bel.index);
    if (bs_it == bel_site_info.end())
        return true;
    const std::string &element = bs_it->second.element;
    if (element != "FFX" && element != "FFY")
        return true;
    IdStringList name = ctx->getBelName(bel);
    if (name.size() < 4)
        return true;
    const std::string other_element = (element == "FFX") ? "FFY" : "FFX";
    std::array<IdString, 3> prefix{name[0], name[1], name[2]};
    BelId other = ctx->getBelByName(IdStringList::concat(IdStringList(prefix), ctx->id(other_element)));
    if (other == BelId())
        return true;
    CellInfo *a = ctx->getBoundBelCell(bel);
    CellInfo *b = ctx->getBoundBelCell(other);
    if (a == nullptr || b == nullptr)
        return true;
    return ff_pair_valid(ctx, a, b);
}

} // namespace spartan3e_detail

NEXTPNR_NAMESPACE_END
