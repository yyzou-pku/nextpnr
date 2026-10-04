/* Native clock controller: clustering, region legality, ties and reserved entry routes. */
#include "spartan3e_ise_clock.h"
#include "spartan3e_ise_bram.h"
#include <algorithm>
#include <set>
#include "log.h"
#define VIADUCT_CONSTIDS "viaduct/spartan3e/constids.inc"
#include "viaduct_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
bool is_xilinx_ff_cell(IdString type)
{
    return type.in(id_FD, id_FDCE, id_FDC, id_FDPE, id_FDP, id_FDRE, id_FDSE, id_FDRSE);
}

std::vector<IdString> clock_ports(Context *ctx, CellInfo *cell)
{
    if (is_xilinx_ff_cell(cell->type)) return {ctx->id("C")};
    if (cell->type == ctx->id("RAM16X1S")) return {ctx->id("WCLK")};
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
    if (net->driver.cell != nullptr && net->driver.cell->type.in(id_BUFG, id_BUFGMUX)) {
        const auto &clock = clock_sites.at(clock_bel_index.at(net->driver.cell->bel.index));
        return clock.global_pips.count(pip.index) != 0;
    }
    return true;
}

void IseClockModel::pack()
{
    for (auto &entry : ctx->cells) {
        CellInfo *cell = entry.second.get();
        if (!cell->type.in(id_BUFG, id_BUFGMUX)) continue;
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
        for (auto *member : {cell, cluster.pads[0], cluster.pads[1]}) {
            if (member == nullptr) continue;
            if (member->cluster != ClusterId())
                log_error("spartan3e-ise: shared clock input needs a multi-buffer cluster: %s\n", ctx->nameOf(member));
            member->cluster = cell->name;
            member->constr_x = 0;
            member->constr_y = 0;
            member->constr_abs_z = false;
            if (member != cell) cell->constr_children.push_back(member);
        }
        clock_cluster[cell->name] = cluster;
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
    if (cell->type.in(id_BUFG, id_BUFGMUX)) {
        const auto &clock = clock_sites.at(clock_bel_index.at(bel.index));
        if (auto *out = cell->getPort(id_O))
            for (const auto &user : out->users)
                if (user.cell->bel != BelId() && !clock.global_wires.count(clk_pin(user.cell->bel, user.port).index))
                    return false;
    } else for (IdString port : clock_ports(ctx, cell)) {
        NetInfo *net = cell->getPort(port);
        if (net != nullptr && net->driver.cell != nullptr && net->driver.cell->type.in(id_BUFG, id_BUFGMUX) &&
            net->driver.cell->bel != BelId()) {
            const auto &clock = clock_sites.at(clock_bel_index.at(net->driver.cell->bel.index));
            if (!clock.global_wires.count(clk_pin(bel, port).index)) return false;
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
}

std::optional<bool> IseClockModel::get_cluster_placement(ClusterId cluster, BelId root_bel,
    std::vector<std::pair<CellInfo *, BelId>> &placement) const
{
    auto clock = clock_cluster.find(cluster);
    if (clock != clock_cluster.end()) {
        auto candidate = clock_bel_index.find(root_bel.index);
        if (candidate == clock_bel_index.end()) return false;
        const auto &site = clock_sites[candidate->second];
        if (!region_valid(clock->second.root, root_bel)) return false;
        placement.clear();
        placement.emplace_back(clock->second.root, root_bel);
        std::set<int32_t> used;
        for (int input = 0; input < 2; ++input) {
            CellInfo *pad = clock->second.pads[input];
            if (pad == nullptr) continue;
            BelId chosen;
            auto fixed = pad->attrs.find(ctx->id("BEL"));
            for (const auto &path : site.pad_paths[input]) {
                BelId bel{path.first};
                if (used.count(bel.index)) continue;
                if (fixed != pad->attrs.end() && ctx->getBelByNameStr(fixed->second.as_string()) != bel) continue;
                if (chosen == BelId() || bel.index < chosen.index) chosen = bel;
            }
            if (chosen == BelId()) return false;
            used.insert(chosen.index);
            placement.emplace_back(pad, chosen);
        }
        return true;
    }
    return std::nullopt;
}

NEXTPNR_NAMESPACE_END
