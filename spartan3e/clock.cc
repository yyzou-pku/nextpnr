/* Native clock topology: bounded pad entry and directed global distribution. */
#include "clock.h"
#include <deque>
#include "log.h"

#include <algorithm>
#include <set>
#include "bram.h"
#include "constids.h"
#include "ff.h"
#include "mult.h"
#include "srl.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
bool numbered(const std::string &name, const char *prefix, int count, const char *suffix = "")
{
    const std::string head(prefix), tail(suffix);
    return name.size() == head.size() + 1 + tail.size() && name.compare(0, head.size(), head) == 0 &&
           name[head.size()] >= '0' && name[head.size()] < '0' + count &&
           name.compare(head.size() + 1, tail.size(), tail) == 0;
}

bool distribution_wire(const std::string &name)
{
    // Match local clock-wire names, not the electrical-node representative: E/W trunks
    // share nodes with IOIS fabric aliases which must not classify pips.
    for (const char *prefix : {"CLKT_GCLK_PINWIRE", "CLKB_GCLK_PINWIRE", "CLKT_GCLK_MAIN", "CLKB_GCLK_MAIN",
                               "CLKC_GCLK_MAIN_T", "CLKC_GCLK_MAIN_B"})
        if (numbered(name, prefix, 4))
            return true;
    for (const char *prefix :
         {"CLKL_GCLK_PWO",    "CLKR_GCLK_PWO",        "CLKL_GCLKH_MAIN",         "CLKR_GCLKH_MAIN",
          "CLKC_GCLK",        "GCLKC_GCLK",           "GCLKC_GCLK_OUT_L",        "GCLKC_GCLK_OUT_R",
          "GCLKH_GCLK",       "GCLKH_GCLK_UP",        "GCLKH_GCLK_DN",           "GCLKVML_GCLKLR",
          "GCLKVML_GCLKCORE", "GCLKVMR_GCLKLR",       "GCLKVMR_GCLKCORE",        "GCLKVMLR_GCLK_UP",
          "GCLKVMLR_GCLK_DN", "BRAMSITE2_GCLKH_GCLK", "BRAMSITE2_GCLKH_GCLK_UP", "BRAMSITE2_GCLKH_GCLK_DN",
          "BRAMSITE2H_GCLK",  "BRAM2_FEEDTHRU_GCLK",  "BRAM2_FEEDTHRUH_GCLK",    "GCLK",
          "IOIS_CLK"})
        if (numbered(name, prefix, 8))
            return true;
    if (numbered(name, "GCLK", 8, "_BRK") || numbered(name, "CLK", 4) || numbered(name, "BRAM_CLK", 4))
        return true;
    for (const char *suffix : {"_INT0", "_INT1", "_INT2", "_INT3"})
        if (numbered(name, "BRAM_CLK", 4, suffix))
            return true;
    return name == "CNR_CLK0" || name == "CNR_CLK2" || name == "DCM_CLKIN" || name == "DCM_CLKFB" ||
           name == "DCM_PSCLK";
}

bool sink_wire(const std::string &name)
{
    return numbered(name, "ICLK1", 3) || numbered(name, "OTCLK1", 3) || name == "RAMB16_CLKA" ||
           name == "RAMB16_CLKB" || name == "MULT18X18SIO_CLK" || name == "LR_STARTUP_CLK" ||
           name == "LR_CAPTURE_CLK" || name == "DCM_CLKIN_PINWIRE" || name == "DCM_CLKFB_PINWIRE" ||
           name == "DCM_PSCLK_PINWIRE";
}
} // namespace

bool clock_distribution_pip(const std::string &from, const std::string &to)
{
    return distribution_wire(from) && (distribution_wire(to) || sink_wire(to));
}

void index_clock_paths(Context *ctx, std::vector<ClockSite> &clocks,
                       const std::unordered_map<int32_t, BelId> &pad_wires,
                       const std::unordered_set<int32_t> &clock_pins, int fabric_wires, int fabric_pips,
                       const std::function<bool(PipId)> &is_distribution)
{
    std::vector<std::vector<PipId>> uphill(fabric_wires), downhill(fabric_wires);
    for (int pi = 0; pi < fabric_pips; ++pi) {
        PipId pip{pi};
        uphill.at(ctx->getPipDstWire(pip).index).push_back(pip);
        downhill.at(ctx->getPipSrcWire(pip).index).push_back(pip);
    }
    size_t entries = 0, regions = 0;
    for (auto &clock : clocks) {
        for (int input = 0; input < 2; ++input) {
            std::deque<std::pair<WireId, int>> queue;
            std::unordered_map<int32_t, PipId> next;
            std::unordered_set<int32_t> seen;
            queue.emplace_back(clock.inputs[input], 0);
            seen.insert(clock.inputs[input].index);
            while (!queue.empty()) {
                auto item = queue.front();
                queue.pop_front();
                auto pad = pad_wires.find(item.first.index);
                if (pad != pad_wires.end()) {
                    std::vector<PipId> path;
                    WireId wire = item.first;
                    while (wire != clock.inputs[input]) {
                        PipId pip = next.at(wire.index);
                        path.push_back(pip);
                        wire = ctx->getPipDstWire(pip);
                    }
                    clock.pad_paths[input][pad->second.index] = std::move(path);
                    ++entries;
                    continue;
                }
                if (item.second == 3)
                    continue;
                for (PipId pip : uphill.at(item.first.index)) {
                    WireId from = ctx->getPipSrcWire(pip);
                    if (!seen.insert(from.index).second)
                        continue;
                    next[from.index] = pip;
                    queue.emplace_back(from, item.second + 1);
                }
            }
        }
        std::deque<WireId> queue{clock.output};
        clock.global_wires.insert(clock.output.index);
        while (!queue.empty()) {
            WireId wire = queue.front();
            queue.pop_front();
            if (clock_pins.count(wire.index)) {
                ++regions;
                continue;
            }
            for (PipId pip : downhill.at(wire.index)) {
                if (!is_distribution(pip))
                    continue;
                clock.global_pips.insert(pip.index);
                WireId to = ctx->getPipDstWire(pip);
                if (clock.global_wires.insert(to.index).second)
                    queue.push_back(to);
            }
        }
        if (clock.global_pips.empty())
            log_error("spartan3e: empty dedicated global-clock distribution\n");
    }
    log_info("Spartan-3E: indexed %zu clock-pad paths and %zu global CLK endpoints\n", entries, regions);
}

namespace {
bool is_xilinx_ff_cell(Context *ctx, IdString type) { return ff_shape(type.str(ctx)).has_value(); }

std::vector<IdString> clock_ports(Context *ctx, CellInfo *cell)
{
    if (is_xilinx_ff_cell(ctx, cell->type))
        return {ctx->id(ff_shape(cell->type.str(ctx))->latch ? "G" : "C")};
    if (is_srl(ctx, cell->type))
        return {ctx->id("CLK")};
    if (cell->type.in(ctx->id("STARTUP_SPARTAN3E"), ctx->id("CAPTURE_SPARTAN3")))
        return {ctx->id("CLK")};
    if (auto shape = mult_shape(ctx, cell))
        return shape->clocked() ? std::vector<IdString>{ctx->id(shape->wrapper ? "C" : "CLK")}
                                : std::vector<IdString>{};
    if (cell->type.in(ctx->id("RAM16X1S"), ctx->id("RAM32X1S"), ctx->id("RAM64X1S"), ctx->id("RAM16X1D")))
        return {ctx->id("WCLK")};
    if (auto shape = bram_shape(cell->type.str(ctx)))
        return shape->dual ? std::vector<IdString>{ctx->id("CLKA"), ctx->id("CLKB")}
                           : std::vector<IdString>{ctx->id("CLK")};
    return {};
}
} // namespace

ClockModel::ClockModel(Context *context, const ChipDb &database, std::vector<ClockSite> sites,
                       std::unordered_map<int, BelId> power_bels, ClockPins clock_pins)
        : ctx(context), db(database), clock_sites(std::move(sites)), clock_power_bels(std::move(power_bels)),
          physical_clock_pins(std::move(clock_pins))
{
    for (size_t i = 0; i < clock_sites.size(); ++i)
        clock_bel_index[clock_sites[i].bel.index] = i;
}

bool ClockModel::pip_available(PipId pip, const NetInfo *net) const
{
    if (net->driver.cell != nullptr && net->driver.cell->type.in(id_BUFG, id_BUFGMUX, ctx->id("BUFGMUX_1"))) {
        const auto &clock = clock_sites.at(clock_bel_index.at(net->driver.cell->bel.index));
        return clock.global_pips.count(pip.index) != 0;
    }
    return true;
}

void ClockModel::pack()
{
    for (auto &entry : ctx->cells) {
        CellInfo *cell = entry.second.get();
        if (!cell->type.in(id_BUFG, id_BUFGMUX, ctx->id("BUFGMUX_1")))
            continue;
        if (cell->type != id_BUFG)
            for (const auto &parameter : cell->params)
                if (parameter.first != ctx->id("CLK_SEL_TYPE") || !parameter.second.is_string ||
                    parameter.second.as_string() != "SYNC")
                    log_error("spartan3e: BUFGMUX/BUFGMUX_1 currently requires CLK_SEL_TYPE=SYNC\n");
        ClockCluster cluster;
        cluster.root = cell;
        auto loc = cell->attrs.find(ctx->id("LOC"));
        if (loc != cell->attrs.end()) {
            BelId constrained;
            for (const auto &clock : clock_sites) {
                const auto &ref = clock.site;
                if (db.tiles[ref.tile].sites[ref.site] == loc->second.as_string())
                    constrained = clock.bel;
            }
            if (constrained == BelId())
                log_error("spartan3e: invalid global-clock LOC\n");
            auto bel = cell->attrs.find(ctx->id("BEL"));
            if (bel != cell->attrs.end() && ctx->getBelByNameStr(bel->second.as_string()) != constrained)
                log_error("spartan3e: conflicting global-clock LOC/BEL\n");
            cell->attrs[ctx->id("BEL")] = Property(ctx->getBelName(constrained).str(ctx));
        }
        for (int input = 0; input < (cell->type == id_BUFG ? 1 : 2); ++input) {
            NetInfo *net = cell->getPort(cell->type == id_BUFG ? id_I : (input == 0 ? id_I0 : id_I1));
            if (net == nullptr || net->driver.cell == nullptr || net->driver.cell->type != id_IBUF)
                log_error("spartan3e: clock %s needs an IBUF-driven dedicated input\n", ctx->nameOf(cell));
            cluster.pads[input] = net->driver.cell;
        }
        bool feasible = false;
        for (const auto &clock : clock_sites) {
            auto fixed_root = cell->attrs.find(ctx->id("BEL"));
            if (fixed_root != cell->attrs.end() && ctx->getBelByNameStr(fixed_root->second.as_string()) != clock.bel)
                continue;
            bool valid = true;
            for (int input = 0; input < 2; ++input) {
                auto *pad = cluster.pads[input];
                if (pad == nullptr)
                    continue;
                auto fixed = pad->attrs.find(ctx->id("BEL"));
                valid &= !clock.pad_paths[input].empty() &&
                         (fixed == pad->attrs.end() ||
                          clock.pad_paths[input].count(ctx->getBelByNameStr(fixed->second.as_string()).index));
            }
            feasible |= valid;
        }
        if (!feasible)
            log_error("spartan3e: no dedicated clock-pad path satisfies input/BUF constraints\n");
        if (auto *out = cell->getPort(id_O))
            for (const auto &user : out->users) {
                auto ports = clock_ports(ctx, user.cell);
                if (std::find(ports.begin(), ports.end(), user.port) == ports.end())
                    log_error("spartan3e: global clock %s has an unsupported clock sink\n", ctx->nameOf(cell));
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
            if (pad != nullptr)
                pad_users[pad].push_back(entry.second.root);
    }
    std::sort(buffers.begin(), buffers.end(),
              [&](CellInfo *a, CellInfo *b) { return a->name.str(ctx) < b->name.str(ctx); });
    std::set<CellInfo *> seen;
    for (CellInfo *root : buffers) {
        if (!seen.insert(root).second)
            continue;
        std::vector<CellInfo *> group{root};
        for (size_t head = 0; head < group.size(); ++head)
            for (CellInfo *pad : clock_cluster.at(group[head]->name).pads)
                if (pad != nullptr)
                    for (CellInfo *buffer : pad_users.at(pad))
                        if (seen.insert(buffer).second)
                            group.push_back(buffer);
        std::sort(group.begin() + 1, group.end(),
                  [&](CellInfo *a, CellInfo *b) { return a->name.str(ctx) < b->name.str(ctx); });
        std::set<CellInfo *> members;
        auto add = [&](CellInfo *member) {
            if (member == nullptr || !members.insert(member).second)
                return;
            if (member->cluster != ClusterId())
                log_error("spartan3e: clock member already belongs to a different cluster: %s\n", ctx->nameOf(member));
            member->cluster = root->name;
            member->constr_x = 0;
            member->constr_y = 0;
            member->constr_abs_z = false;
            if (member != root)
                root->constr_children.push_back(member);
        };
        for (CellInfo *buffer : group) {
            add(buffer);
            for (CellInfo *pad : clock_cluster.at(buffer->name).pads)
                add(pad);
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
            log_error("spartan3e: shared clock cluster has no joint dedicated pad/BUF placement: %s\n",
                      ctx->nameOf(root));
        // HeAP skips a cluster whose root is already placed by a BEL/LOC
        // constraint. Freeze its complete graph-derived assignment as well;
        // leaving the shared pads/partner buffers movable loses the cluster.
        if (root->attrs.count(ctx->id("BEL")))
            for (const auto &member : placement)
                member.first->attrs[ctx->id("BEL")] = Property(ctx->getBelName(member.second).str(ctx));
    }
}

void ClockModel::add_ties()
{
    std::map<int, std::vector<CellInfo *>> groups;
    for (auto &entry : ctx->cells)
        if (entry.second->type == id_BUFG)
            groups[clock_sites.at(clock_bel_index.at(entry.second->bel.index)).site.tile].push_back(entry.second.get());
    for (auto &group : groups) {
        auto power = clock_power_bels.find(group.first);
        if (power == clock_power_bels.end())
            log_error("spartan3e: clock tile has no native VCC site\n");
        CellInfo *cell = ctx->createCell(ctx->idf("$CLOCK_VCC_%d", group.first), ctx->id("VCC"));
        cell->addOutput(ctx->id("P"));
        cell->bel_pins[ctx->id("P")] = {ctx->id("P")};
        NetInfo *net = ctx->createNet(ctx->idf("$CLOCK_VCC_NET_%d", group.first));
        cell->connectPort(ctx->id("P"), net);
        ctx->bindBel(power->second, cell, STRENGTH_LOCKED);
        for (CellInfo *buffer : group.second) {
            buffer->addInput(id_S);
            buffer->bel_pins[id_S] = {id_S};
            buffer->connectPort(id_S, net);
        }
    }
    if (!groups.empty())
        ctx->assignArchInfo();
}

bool ClockModel::region_valid(CellInfo *cell, BelId bel) const
{
    auto clk_pin = [&](BelId target, IdString port) { return physical_clock_pins.at(target.index).at(port); };
    if (cell->type.in(id_BUFG, id_BUFGMUX, ctx->id("BUFGMUX_1"))) {
        const auto &clock = clock_sites.at(clock_bel_index.at(bel.index));
        if (auto *out = cell->getPort(id_O))
            for (const auto &user : out->users)
                if (user.cell->bel != BelId())
                    for (WireId pin : clk_pin(user.cell->bel, user.port))
                        if (!clock.global_wires.count(pin.index))
                            return false;
    } else
        for (IdString port : clock_ports(ctx, cell)) {
            NetInfo *net = cell->getPort(port);
            if (net != nullptr && net->driver.cell != nullptr &&
                net->driver.cell->type.in(id_BUFG, id_BUFGMUX, ctx->id("BUFGMUX_1")) &&
                net->driver.cell->bel != BelId()) {
                const auto &clock = clock_sites.at(clock_bel_index.at(net->driver.cell->bel.index));
                for (WireId pin : clk_pin(bel, port))
                    if (!clock.global_wires.count(pin.index))
                        return false;
            }
        }
    return true;
}

std::vector<PipId> ClockModel::internal_path(WireId source, WireId target,
                                             const std::function<bool(PipId)> &is_internal,
                                             const std::function<bool(PipId)> &available) const
{
    std::vector<WireId> queue{source};
    std::unordered_map<int32_t, PipId> previous;
    std::unordered_set<int32_t> seen{source.index};
    for (size_t head = 0; head < queue.size() && !seen.count(target.index); ++head)
        for (PipId pip : ctx->getPipsDownhill(queue[head])) {
            if (!is_internal(pip) || !available(pip))
                continue;
            WireId to = ctx->getPipDstWire(pip);
            if (!seen.insert(to.index).second)
                continue;
            previous[to.index] = pip;
            queue.push_back(to);
        }
    if (!seen.count(target.index))
        log_error("spartan3e: selected clock site path is disconnected\n");
    std::vector<PipId> path;
    for (WireId wire = target; wire != source;) {
        PipId pip = previous.at(wire.index);
        path.push_back(pip);
        wire = ctx->getPipSrcWire(pip);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void ClockModel::pre_route(const std::function<bool(PipId)> &is_internal, const std::function<bool(PipId)> &available)
{
    for (const auto &entry : clock_cluster) {
        const auto &cluster = entry.second;
        const auto &clock = clock_sites.at(clock_bel_index.at(cluster.root->bel.index));
        for (int input = 0; input < 2; ++input) {
            CellInfo *pad = cluster.pads[input];
            if (pad == nullptr)
                continue;
            IdString port = cluster.root->type == id_BUFG ? id_I : input == 0 ? id_I0 : id_I1;
            NetInfo *net = cluster.root->getPort(port);
            const auto &fabric = clock.pad_paths[input].at(pad->bel.index);
            if (fabric.empty())
                log_error("spartan3e: empty dedicated clock pad-entry path\n");
            WireId source = ctx->getNetinfoSourceWire(net);
            WireId target = ctx->getBelPinWire(cluster.root->bel, port);
            auto path = internal_path(source, ctx->getPipSrcWire(fabric.front()), is_internal, available);
            path.insert(path.end(), fabric.begin(), fabric.end());
            auto tail = internal_path(ctx->getPipDstWire(fabric.back()), target, is_internal, available);
            path.insert(path.end(), tail.begin(), tail.end());
            if (ctx->getBoundWireNet(source) == nullptr)
                ctx->bindWire(source, net, STRENGTH_LOCKED);
            for (PipId pip : path) {
                if (ctx->getBoundPipNet(pip) == net)
                    continue;
                if (ctx->getBoundPipNet(pip) != nullptr || (ctx->getBoundWireNet(ctx->getPipDstWire(pip)) != nullptr &&
                                                            ctx->getBoundWireNet(ctx->getPipDstWire(pip)) != net))
                    log_error("spartan3e: dedicated clock-entry resource conflict\n");
                ctx->bindPip(pip, net, STRENGTH_LOCKED);
            }
        }
    }
    route_global_outputs(is_internal, available);
}

void ClockModel::route_global_outputs(const std::function<bool(PipId)> &is_internal,
                                      const std::function<bool(PipId)> &available)
{
    std::vector<CellInfo *> buffers;
    for (const auto &entry : clock_cluster)
        buffers.push_back(entry.second.root);
    std::sort(buffers.begin(), buffers.end(),
              [&](CellInfo *a, CellInfo *b) { return a->name.str(ctx) < b->name.str(ctx); });
    size_t sinks = 0, locked = 0;
    for (CellInfo *buffer : buffers) {
        NetInfo *net = buffer->getPort(id_O);
        if (net == nullptr)
            log_error("spartan3e: global buffer has no output net\n");
        const auto &clock = clock_sites.at(clock_bel_index.at(buffer->bel.index));
        WireId source = ctx->getNetinfoSourceWire(net);
        std::vector<WireId> targets;
        for (const auto &user : net->users)
            for (WireId wire : ctx->getNetinfoSinkWires(net, user))
                targets.push_back(wire);
        std::vector<WireId> queue{source};
        std::unordered_set<int32_t> seen{source.index};
        std::unordered_map<int32_t, PipId> previous;
        for (size_t head = 0; head < queue.size(); ++head) {
            for (PipId pip : ctx->getPipsDownhill(queue[head])) {
                if ((!is_internal(pip) && !clock.global_pips.count(pip.index)) || !available(pip))
                    continue;
                WireId to = ctx->getPipDstWire(pip);
                auto *pip_owner = ctx->getBoundPipNet(pip);
                auto *wire_owner = ctx->getBoundWireNet(to);
                if ((pip_owner != nullptr && pip_owner != net) || (wire_owner != nullptr && wire_owner != net))
                    continue;
                if (!seen.insert(to.index).second)
                    continue;
                previous[to.index] = pip;
                queue.push_back(to);
            }
        }
        std::vector<PipId> tree;
        std::unordered_set<int32_t> included;
        for (WireId target : targets) {
            if (!seen.count(target.index))
                log_error("spartan3e: dedicated clock routing cannot reach sink on %s (resource conflict or region); "
                          "no fabric fallback\n",
                          ctx->nameOf(net));
            for (WireId wire = target; wire != source;) {
                PipId pip = previous.at(wire.index);
                if (!included.insert(pip.index).second)
                    break;
                wire = ctx->getPipSrcWire(pip);
            }
        }
        auto *owner = ctx->getBoundWireNet(source);
        if (owner != nullptr && owner != net)
            log_error("spartan3e: dedicated clock source is occupied\n");
        if (owner == nullptr)
            ctx->bindWire(source, net, STRENGTH_LOCKED);
        // Each target path follows one BFS parent tree. Bind root-to-leaf so
        // the generic router receives a complete, immutable clock network.
        for (WireId wire : queue) {
            if (wire == source)
                continue;
            PipId pip = previous.at(wire.index);
            if (included.count(pip.index))
                tree.push_back(pip);
        }
        for (PipId pip : tree) {
            if (ctx->getBoundPipNet(pip) == net)
                continue;
            ctx->bindPip(pip, net, STRENGTH_LOCKED);
            ++locked;
        }
        sinks += targets.size();
    }
    log_info(
            "Spartan-3E: dedicated clock routing locked %zu pips for %zu sinks in %zu networks before fabric routing\n",
            locked, sinks, buffers.size());
}

std::optional<bool> ClockModel::get_cluster_placement(ClusterId cluster, BelId root_bel,
                                                      std::vector<std::pair<CellInfo *, BelId>> &placement) const
{
    auto group = clock_groups.find(cluster);
    if (group == clock_groups.end())
        return std::nullopt;
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
        if (index == group->second.size())
            return true;
        CellInfo *buffer = group->second[index];
        const auto &info = clock_cluster.at(buffer->name);
        for (const auto &site : clock_sites) {
            if ((index == 0 && site.bel != root_bel) || used.count(site.bel.index) ||
                !fixed_matches(buffer, site.bel) || !region_valid(buffer, site.bel))
                continue;
            assigned[buffer] = site.bel;
            used.insert(site.bel.index);
            std::function<bool(int)> assign_pad = [&](int input) {
                if (input == 2)
                    return assign_buffer(index + 1);
                CellInfo *pad = info.pads[input];
                if (pad == nullptr)
                    return assign_pad(input + 1);
                auto existing = assigned.find(pad);
                if (existing != assigned.end())
                    return site.pad_paths[input].count(existing->second.index) && assign_pad(input + 1);
                std::vector<int32_t> candidates;
                for (const auto &path : site.pad_paths[input])
                    candidates.push_back(path.first);
                std::sort(candidates.begin(), candidates.end());
                for (int32_t candidate : candidates) {
                    BelId bel{candidate};
                    if (used.count(candidate) || !fixed_matches(pad, bel))
                        continue;
                    assigned[pad] = bel;
                    used.insert(candidate);
                    if (assign_pad(input + 1))
                        return true;
                    used.erase(candidate);
                    assigned.erase(pad);
                }
                return false;
            };
            if (assign_pad(0))
                return true;
            used.erase(site.bel.index);
            assigned.erase(buffer);
        }
        return false;
    };
    if (!assign_buffer(0))
        return false;
    std::set<CellInfo *> emitted;
    for (CellInfo *buffer : group->second) {
        placement.emplace_back(buffer, assigned.at(buffer));
        for (CellInfo *pad : clock_cluster.at(buffer->name).pads)
            if (pad != nullptr && emitted.insert(pad).second)
                placement.emplace_back(pad, assigned.at(pad));
    }
    return true;
}

NEXTPNR_NAMESPACE_END
