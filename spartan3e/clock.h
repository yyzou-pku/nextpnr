#ifndef SPARTAN3E_CLOCK_H
#define SPARTAN3E_CLOCK_H

#include <array>
#include <functional>
#include <map>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "device.h"
#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN

struct ClockSite
{
    SiteRef site;
    BelId bel;
    std::array<WireId, 2> inputs;
    WireId output;
    // Each path is actual XDLRC pips, not a wire-name template.
    std::array<std::unordered_map<int32_t, std::vector<PipId>>, 2> pad_paths;
    std::unordered_set<int32_t> global_wires;
    std::unordered_set<int32_t> global_pips;
};

// Indexed only over fabric nodes/pips. The initial pad contract admits paths
// of at most three pips (the native clock-pad entry), never a long fabric detour.
void index_clock_paths(Context *ctx, std::vector<ClockSite> &clocks,
                       const std::unordered_map<int32_t, BelId> &pad_wires,
                       const std::unordered_set<int32_t> &clock_pins, int fabric_wires, int fabric_pips,
                       const std::function<bool(PipId)> &is_distribution);

bool clock_distribution_pip(const std::string &from, const std::string &to);

// Mutable clock packing/routing state is separate from the read-only topology.
using ClockPins = std::unordered_map<int32_t, dict<IdString, std::vector<WireId>>>;

class ClockModel
{
  public:
    ClockModel(Context *context, const ChipDb &database, std::vector<ClockSite> sites,
               std::unordered_map<int, BelId> power_bels, ClockPins clock_pins);
    void pack();
    void add_ties();
    bool region_valid(CellInfo *cell, BelId bel) const;
    bool pip_available(PipId pip, const NetInfo *net) const;
    std::optional<bool> get_cluster_placement(ClusterId cluster, BelId root_bel,
                                              std::vector<std::pair<CellInfo *, BelId>> &placement) const;
    void pre_route(const std::function<bool(PipId)> &is_internal, const std::function<bool(PipId)> &available);

  private:
    struct ClockCluster
    {
        CellInfo *root = nullptr;
        std::array<CellInfo *, 2> pads{{nullptr, nullptr}};
    };
    Context *ctx;
    const ChipDb &db;
    std::vector<ClockSite> clock_sites;
    std::unordered_map<int32_t, size_t> clock_bel_index;
    std::unordered_map<int, BelId> clock_power_bels;
    ClockPins physical_clock_pins;
    dict<ClusterId, ClockCluster> clock_cluster;
    dict<ClusterId, std::vector<CellInfo *>> clock_groups;
    std::vector<PipId> internal_path(WireId source, WireId target, const std::function<bool(PipId)> &is_internal,
                                     const std::function<bool(PipId)> &available) const;
    void route_global_outputs(const std::function<bool(PipId)> &is_internal,
                              const std::function<bool(PipId)> &available);
};

NEXTPNR_NAMESPACE_END
#endif
