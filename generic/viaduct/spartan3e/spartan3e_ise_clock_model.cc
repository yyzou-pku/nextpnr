/* Native clock controller: clustering, region legality, ties and reserved entry routes. */
#include "spartan3e_ise_clock.h"
#include "spartan3e_ise_bram.h"
#include "spartan3e_ise_srl.h"
#include "spartan3e_ise_ff.h"
#include "spartan3e_ise_mult.h"
#include <algorithm>
#include <set>
#include "log.h"
#define VIADUCT_CONSTIDS "viaduct/spartan3e/constids.inc"
#include "viaduct_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
bool is_xilinx_ff_cell(Context *ctx, IdString type)
{
    return ise_ff_shape(type.str(ctx)).has_value();
}

std::vector<IdString> clock_ports(Context *ctx, CellInfo *cell)
{
    if (is_xilinx_ff_cell(ctx, cell->type)) return {ctx->id(ise_ff_shape(cell->type.str(ctx))->latch ? "G" : "C")};
    if (is_ise_srl(ctx, cell->type)) return {ctx->id("CLK")};
    if (cell->type.in(ctx->id("STARTUP_SPARTAN3E"), ctx->id("CAPTURE_SPARTAN3"))) return {ctx->id("CLK")};
    if (auto shape = ise_mult_shape(ctx, cell))
        return shape->clocked() ? std::vector<IdString>{ctx->id(shape->wrapper ? "C" : "CLK")} : std::vector<IdString>{};
    if (cell->type.in(ctx->id("RAM16X1S"), ctx->id("RAM32X1S"), ctx->id("RAM64X1S"), ctx->id("RAM16X1D"))) return {ctx->id("WCLK")};
    if (auto shape = ise_bram_shape(cell->type.str(ctx)))
        return shape->dual ? std::vector<IdString>{ctx->id("CLKA"), ctx->id("CLKB")} : std::vector<IdString>{ctx->id("CLK")};
    return {};
}
}

IseClockModel::IseClockModel(Context *context, const IseChipDb &database, std::vector<IseClockSite> sites,
    std::unordered_map<int, BelId> power_bels, IseClockPins clock_pins)
    : ctx(context), db(database), clock_sites(std::move(sites)), clock_power_bels(std::move(power_bels)),
      physical_clock_pins(std::move(clock_pins))
{
    for (size_t i = 0; i < clock_sites.size(); ++i) clock_bel_index[clock_sites[i].bel.index] = i;
}

bool IseClockModel::pip_available(PipId pip, const NetInfo *net) const
{
    if (net->driver.cell != nullptr && net->driver.cell->type.in(id_BUFG, id_BUFGMUX, ctx->id("BUFGMUX_1"))) {
        const auto &clock = clock_sites.at(clock_bel_index.at(net->driver.cell->bel.index));
        return clock.global_pips.count(pip.index) != 0;
    }
    return true;
}

void IseClockModel::pack()
{
    for (auto &entry : ctx->cells) {
        CellInfo *cell = entry.second.get();
        if (!cell->type.in(id_BUFG, id_BUFGMUX, ctx->id("BUFGMUX_1"))) continue;
        if (cell->type != id_BUFG)
            for (const auto &parameter : cell->params)
                if (parameter.first != ctx->id("CLK_SEL_TYPE") || !parameter.second.is_string ||
                    parameter.second.as_string() != "SYNC")
                    log_error("spartan3e-ise: BUFGMUX/BUFGMUX_1 currently requires CLK_SEL_TYPE=SYNC\n");
        ClockCluster cluster;
        cluster.root = cell;
        auto loc = cell->attrs.find(ctx->id("LOC"));
        if (loc != cell->attrs.end()) {
            BelId constrained;
            for (const auto &clock : clock_sites) {
                const auto &ref = clock.site;
                if (db.tiles[ref.tile].sites[ref.site] == loc->second.as_string()) constrained = clock.bel;
            }
            if (constrained == BelId()) log_error("spartan3e-ise: invalid global-clock LOC\n");
            auto bel = cell->attrs.find(ctx->id("BEL"));
            if (bel != cell->attrs.end() && ctx->getBelByNameStr(bel->second.as_string()) != constrained)
                log_error("spartan3e-ise: conflicting global-clock LOC/BEL\n");
            cell->attrs[ctx->id("BEL")] = Property(ctx->getBelName(constrained).str(ctx));
        }
        for (int input = 0; input < (cell->type == id_BUFG ? 1 : 2); ++input) {
            NetInfo *net = cell->getPort(cell->type == id_BUFG ? id_I : (input == 0 ? id_I0 : id_I1));
            if (net == nullptr || net->driver.cell == nullptr || net->driver.cell->type != id_IBUF)
                log_error("spartan3e-ise: clock %s needs an IBUF-driven dedicated input\n", ctx->nameOf(cell));
            cluster.pads[input] = net->driver.cell;
        }
        bool feasible = false;
        for (const auto &clock : clock_sites) {
            auto fixed_root = cell->attrs.find(ctx->id("BEL"));
            if (fixed_root != cell->attrs.end() && ctx->getBelByNameStr(fixed_root->second.as_string()) != clock.bel) continue;
            bool valid = true;
            for (int input = 0; input < 2; ++input) {
                auto *pad = cluster.pads[input];
                if (pad == nullptr) continue;
                auto fixed = pad->attrs.find(ctx->id("BEL"));
                valid &= !clock.pad_paths[input].empty() && (fixed == pad->attrs.end() ||
                          clock.pad_paths[input].count(ctx->getBelByNameStr(fixed->second.as_string()).index));
            }
            feasible |= valid;
        }
        if (!feasible) log_error("spartan3e-ise: no dedicated clock-pad path satisfies input/BUF constraints\n");
        if (auto *out = cell->getPort(id_O))
            for (const auto &user : out->users) {
                    auto ports = clock_ports(ctx, user.cell);
                    if (std::find(ports.begin(), ports.end(), user.port) == ports.end())
                        log_error("spartan3e-ise: global clock %s has an unsupported clock sink\n", ctx->nameOf(cell));
            }
        clock_cluster[cell->name] = cluster;
    }
    // A pad is a single physical cell even when it feeds several global
    // buffers. Cluster the connected component, not each buffer separately.
    std::vector<CellInfo *> buffers;
    std::map<CellInfo *, std::vector<CellInfo *>> pad_users;
    for (const auto &entry : clock_cluster) {
        buffers.push_back(entry.second.root);
        for (CellInfo *pad : entry.second.pads)
            if (pad != nullptr) pad_users[pad].push_back(entry.second.root);
    }
    std::sort(buffers.begin(), buffers.end(), [&](CellInfo *a, CellInfo *b) {
        return a->name.str(ctx) < b->name.str(ctx);
    });
    std::set<CellInfo *> seen;
    for (CellInfo *root : buffers) {
        if (!seen.insert(root).second) continue;
        std::vector<CellInfo *> group{root};
        for (size_t head = 0; head < group.size(); ++head)
            for (CellInfo *pad : clock_cluster.at(group[head]->name).pads)
                if (pad != nullptr)
                    for (CellInfo *buffer : pad_users.at(pad))
                        if (seen.insert(buffer).second) group.push_back(buffer);
        std::sort(group.begin() + 1, group.end(), [&](CellInfo *a, CellInfo *b) {
            return a->name.str(ctx) < b->name.str(ctx);
        });
        std::set<CellInfo *> members;
        auto add = [&](CellInfo *member) {
            if (member == nullptr || !members.insert(member).second) return;
            if (member->cluster != ClusterId())
                log_error("spartan3e-ise: clock member already belongs to a different cluster: %s\n", ctx->nameOf(member));
            member->cluster = root->name;
            member->constr_x = 0;
            member->constr_y = 0;
            member->constr_abs_z = false;
            if (member != root) root->constr_children.push_back(member);
        };
        for (CellInfo *buffer : group) {
            add(buffer);
            for (CellInfo *pad : clock_cluster.at(buffer->name).pads) add(pad);
        }
        clock_groups[root->name] = group;
        bool feasible = false;
        std::vector<std::pair<CellInfo *, BelId>> placement;
        for (const auto &site : clock_sites)
            if (get_cluster_placement(root->name, site.bel, placement).value_or(false)) {
                feasible = true;
                break;
            }
        if (!feasible)
            log_error("spartan3e-ise: shared clock cluster has no joint dedicated pad/BUF placement: %s\n", ctx->nameOf(root));
        // HeAP skips a cluster whose root is already placed by a BEL/LOC
        // constraint. Freeze its complete graph-derived assignment as well;
        // leaving the shared pads/partner buffers movable loses the cluster.
        if (root->attrs.count(ctx->id("BEL")))
            for (const auto &member : placement)
                member.first->attrs[ctx->id("BEL")] = Property(ctx->getBelName(member.second).str(ctx));
    }
}

void IseClockModel::add_ties()
{
    std::map<int, std::vector<CellInfo *>> groups;
    for (auto &entry : ctx->cells)
        if (entry.second->type == id_BUFG)
            groups[clock_sites.at(clock_bel_index.at(entry.second->bel.index)).site.tile].push_back(entry.second.get());
    for (auto &group : groups) {
        auto power = clock_power_bels.find(group.first);
        if (power == clock_power_bels.end())
            log_error("spartan3e-ise: clock tile has no native VCC site\n");
        CellInfo *cell = ctx->createCell(ctx->idf("$ISE_CLOCK_VCC_%d", group.first), ctx->id("VCC"));
        cell->addOutput(ctx->id("P"));
        cell->bel_pins[ctx->id("P")] = {ctx->id("P")};
        NetInfo *net = ctx->createNet(ctx->idf("$ISE_CLOCK_VCC_NET_%d", group.first));
        cell->connectPort(ctx->id("P"), net);
        ctx->bindBel(power->second, cell, STRENGTH_LOCKED);
        for (CellInfo *buffer : group.second) {
            buffer->addInput(id_S);
            buffer->bel_pins[id_S] = {id_S};
            buffer->connectPort(id_S, net);
        }
    }
    if (!groups.empty()) ctx->assignArchInfo();
}

bool IseClockModel::region_valid(CellInfo *cell, BelId bel) const
{
    auto clk_pin = [&](BelId target, IdString port) { return physical_clock_pins.at(target.index).at(port); };
    if (cell->type.in(id_BUFG, id_BUFGMUX, ctx->id("BUFGMUX_1"))) {
        const auto &clock = clock_sites.at(clock_bel_index.at(bel.index));
        if (auto *out = cell->getPort(id_O))
            for (const auto &user : out->users)
                if (user.cell->bel != BelId())
                    for (WireId pin : clk_pin(user.cell->bel, user.port))
                        if (!clock.global_wires.count(pin.index)) return false;
    } else for (IdString port : clock_ports(ctx, cell)) {
        NetInfo *net = cell->getPort(port);
        if (net != nullptr && net->driver.cell != nullptr && net->driver.cell->type.in(id_BUFG, id_BUFGMUX, ctx->id("BUFGMUX_1")) &&
            net->driver.cell->bel != BelId()) {
            const auto &clock = clock_sites.at(clock_bel_index.at(net->driver.cell->bel.index));
            for (WireId pin : clk_pin(bel, port))
                if (!clock.global_wires.count(pin.index)) return false;
        }
    }
    return true;
}

std::vector<PipId> IseClockModel::internal_path(WireId source, WireId target, const std::function<bool(PipId)> &is_internal, const std::function<bool(PipId)> &available) const
{
    std::vector<WireId> queue{source};
    std::unordered_map<int32_t, PipId> previous;
    std::unordered_set<int32_t> seen{source.index};
    for (size_t head = 0; head < queue.size() && !seen.count(target.index); ++head)
        for (PipId pip : ctx->getPipsDownhill(queue[head])) {
            if (!is_internal(pip) || !available(pip)) continue;
            WireId to = ctx->getPipDstWire(pip);
            if (!seen.insert(to.index).second) continue;
            previous[to.index] = pip;
            queue.push_back(to);
        }
    if (!seen.count(target.index))
        log_error("spartan3e-ise: selected clock site path is disconnected\n");
    std::vector<PipId> path;
    for (WireId wire = target; wire != source;) {
        PipId pip = previous.at(wire.index);
        path.push_back(pip);
        wire = ctx->getPipSrcWire(pip);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void IseClockModel::pre_route(const std::function<bool(PipId)> &is_internal, const std::function<bool(PipId)> &available)
{
    for (const auto &entry : clock_cluster) {
        const auto &cluster = entry.second;
        const auto &clock = clock_sites.at(clock_bel_index.at(cluster.root->bel.index));
        for (int input = 0; input < 2; ++input) {
            CellInfo *pad = cluster.pads[input];
            if (pad == nullptr) continue;
            IdString port = cluster.root->type == id_BUFG ? id_I : input == 0 ? id_I0 : id_I1;
            NetInfo *net = cluster.root->getPort(port);
            const auto &fabric = clock.pad_paths[input].at(pad->bel.index);
            if (fabric.empty()) log_error("spartan3e-ise: empty dedicated clock pad-entry path\n");
            WireId source = ctx->getNetinfoSourceWire(net);
            WireId target = ctx->getBelPinWire(cluster.root->bel, port);
            auto path = internal_path(source, ctx->getPipSrcWire(fabric.front()), is_internal, available);
            path.insert(path.end(), fabric.begin(), fabric.end());
            auto tail = internal_path(ctx->getPipDstWire(fabric.back()), target, is_internal, available);
            path.insert(path.end(), tail.begin(), tail.end());
            if (ctx->getBoundWireNet(source) == nullptr) ctx->bindWire(source, net, STRENGTH_LOCKED);
            for (PipId pip : path) {
                if (ctx->getBoundPipNet(pip) == net) continue;
                if (ctx->getBoundPipNet(pip) != nullptr ||
                    (ctx->getBoundWireNet(ctx->getPipDstWire(pip)) != nullptr &&
                     ctx->getBoundWireNet(ctx->getPipDstWire(pip)) != net))
                    log_error("spartan3e-ise: dedicated clock-entry resource conflict\n");
                ctx->bindPip(pip, net, STRENGTH_LOCKED);
            }
        }
    }
    route_global_outputs(is_internal, available);
}

void IseClockModel::route_global_outputs(const std::function<bool(PipId)> &is_internal,
                                        const std::function<bool(PipId)> &available)
{
    std::vector<CellInfo *> buffers;
    for (const auto &entry : clock_cluster) buffers.push_back(entry.second.root);
    std::sort(buffers.begin(), buffers.end(), [&](CellInfo *a, CellInfo *b) {
        return a->name.str(ctx) < b->name.str(ctx);
    });
    size_t sinks = 0, locked = 0;
    for (CellInfo *buffer : buffers) {
        NetInfo *net = buffer->getPort(id_O);
        if (net == nullptr) log_error("spartan3e-ise: global buffer has no output net\n");
        const auto &clock = clock_sites.at(clock_bel_index.at(buffer->bel.index));
        WireId source = ctx->getNetinfoSourceWire(net);
        std::vector<WireId> targets;
        for (const auto &user : net->users)
            for (WireId wire : ctx->getNetinfoSinkWires(net, user)) targets.push_back(wire);
        std::vector<WireId> queue{source};
        std::unordered_set<int32_t> seen{source.index};
        std::unordered_map<int32_t, PipId> previous;
        for (size_t head = 0; head < queue.size(); ++head) {
            for (PipId pip : ctx->getPipsDownhill(queue[head])) {
                if ((!is_internal(pip) && !clock.global_pips.count(pip.index)) || !available(pip)) continue;
                WireId to = ctx->getPipDstWire(pip);
                auto *pip_owner = ctx->getBoundPipNet(pip);
                auto *wire_owner = ctx->getBoundWireNet(to);
                if ((pip_owner != nullptr && pip_owner != net) || (wire_owner != nullptr && wire_owner != net)) continue;
                if (!seen.insert(to.index).second) continue;
                previous[to.index] = pip;
                queue.push_back(to);
            }
        }
        std::vector<PipId> tree;
        std::unordered_set<int32_t> included;
        for (WireId target : targets) {
            if (!seen.count(target.index))
                log_error("spartan3e-ise: dedicated clock routing cannot reach sink on %s (resource conflict or region); no fabric fallback\n", ctx->nameOf(net));
            for (WireId wire = target; wire != source;) {
                PipId pip = previous.at(wire.index);
                if (!included.insert(pip.index).second) break;
                wire = ctx->getPipSrcWire(pip);
            }
        }
        auto *owner = ctx->getBoundWireNet(source);
        if (owner != nullptr && owner != net) log_error("spartan3e-ise: dedicated clock source is occupied\n");
        if (owner == nullptr) ctx->bindWire(source, net, STRENGTH_LOCKED);
        // Each target path follows one BFS parent tree. Bind root-to-leaf so
        // the generic router receives a complete, immutable clock network.
        for (WireId wire : queue) {
            if (wire == source) continue;
            PipId pip = previous.at(wire.index);
            if (included.count(pip.index)) tree.push_back(pip);
        }
        for (PipId pip : tree) {
            if (ctx->getBoundPipNet(pip) == net) continue;
            ctx->bindPip(pip, net, STRENGTH_LOCKED);
            ++locked;
        }
        sinks += targets.size();
    }
    log_info("Spartan-3E ISE: dedicated clock routing locked %zu pips for %zu sinks in %zu networks before fabric routing\n", locked, sinks, buffers.size());
}

std::optional<bool> IseClockModel::get_cluster_placement(ClusterId cluster, BelId root_bel,
    std::vector<std::pair<CellInfo *, BelId>> &placement) const
{
    auto group = clock_groups.find(cluster);
    if (group == clock_groups.end()) return std::nullopt;
    placement.clear();
    std::map<CellInfo *, BelId> assigned;
    std::set<int32_t> used;
    auto fixed_matches = [&](CellInfo *cell, BelId bel) {
        auto fixed = cell->attrs.find(ctx->id("BEL"));
        return fixed == cell->attrs.end() || ctx->getBelByNameStr(fixed->second.as_string()) == bel;
    };
    // Backtrack over the small dedicated clock/pad graph. Shared pads must
    // satisfy every buffer's input path; no coordinate or BEL-name inference.
    std::function<bool(size_t)> assign_buffer;
    assign_buffer = [&](size_t index) {
        if (index == group->second.size()) return true;
        CellInfo *buffer = group->second[index];
        const auto &info = clock_cluster.at(buffer->name);
        for (const auto &site : clock_sites) {
            if ((index == 0 && site.bel != root_bel) || used.count(site.bel.index) ||
                !fixed_matches(buffer, site.bel) || !region_valid(buffer, site.bel)) continue;
            assigned[buffer] = site.bel;
            used.insert(site.bel.index);
            std::function<bool(int)> assign_pad = [&](int input) {
                if (input == 2) return assign_buffer(index + 1);
                CellInfo *pad = info.pads[input];
                if (pad == nullptr) return assign_pad(input + 1);
                auto existing = assigned.find(pad);
                if (existing != assigned.end())
                    return site.pad_paths[input].count(existing->second.index) && assign_pad(input + 1);
                std::vector<int32_t> candidates;
                for (const auto &path : site.pad_paths[input]) candidates.push_back(path.first);
                std::sort(candidates.begin(), candidates.end());
                for (int32_t candidate : candidates) {
                    BelId bel{candidate};
                    if (used.count(candidate) || !fixed_matches(pad, bel)) continue;
                    assigned[pad] = bel;
                    used.insert(candidate);
                    if (assign_pad(input + 1)) return true;
                    used.erase(candidate);
                    assigned.erase(pad);
                }
                return false;
            };
            if (assign_pad(0)) return true;
            used.erase(site.bel.index);
            assigned.erase(buffer);
        }
        return false;
    };
    if (!assign_buffer(0)) return false;
    std::set<CellInfo *> emitted;
    for (CellInfo *buffer : group->second) {
        placement.emplace_back(buffer, assigned.at(buffer));
        for (CellInfo *pad : clock_cluster.at(buffer->name).pads)
            if (pad != nullptr && emitted.insert(pad).second) placement.emplace_back(pad, assigned.at(pad));
    }
    return true;
}

NEXTPNR_NAMESPACE_END
