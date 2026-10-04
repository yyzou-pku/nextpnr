#ifndef SPARTAN3E_ISE_CLOCK_H
#define SPARTAN3E_ISE_CLOCK_H

#include <array>
#include <functional>
#include <map>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "nextpnr.h"
#include "spartan3e_ise_db.h"

NEXTPNR_NAMESPACE_BEGIN

struct IseClockSite
{
    IseSiteRef site;
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
void index_ise_clock_paths(Context *ctx, std::vector<IseClockSite> &clocks,
                           const std::unordered_map<int32_t, BelId> &pad_wires,
                           const std::unordered_set<int32_t> &clock_pins, int fabric_wires, int fabric_pips);

// Mutable clock packing/routing state is separate from the read-only topology.
using IseClockPins = std::unordered_map<int32_t, dict<IdString, WireId>>;

class IseClockModel
{
  public:
    IseClockModel(Context *context, const IseChipDb &database, std::vector<IseClockSite> sites,
                  std::unordered_map<int, BelId> power_bels, IseClockPins clock_pins);
    void pack();
    void add_ties();
    bool region_valid(CellInfo *cell, BelId bel) const;
    bool pip_available(PipId pip, const NetInfo *net) const;
    std::optional<bool> get_cluster_placement(ClusterId cluster, BelId root_bel,
                                            std::vector<std::pair<CellInfo *, BelId>> &placement) const;
    void pre_route(const std::function<bool(PipId)> &is_internal, const std::function<bool(PipId)> &available);

  private:
    struct ClockCluster {
        CellInfo *root = nullptr;
        std::array<CellInfo *, 2> pads{{nullptr, nullptr}};
    };
    Context *ctx;
    const IseChipDb &db;
    std::vector<IseClockSite> clock_sites;
    std::unordered_map<int32_t, size_t> clock_bel_index;
    std::unordered_map<int, BelId> clock_power_bels;
    IseClockPins physical_clock_pins;
    dict<ClusterId, ClockCluster> clock_cluster;
    std::vector<PipId> internal_path(WireId source, WireId target,
        const std::function<bool(PipId)> &is_internal, const std::function<bool(PipId)> &available) const;
};

NEXTPNR_NAMESPACE_END
#endif
