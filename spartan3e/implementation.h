#ifndef SPARTAN3E_IMPLEMENTATION_H
#define SPARTAN3E_IMPLEMENTATION_H

// Shared device state for graph construction, packing, placement and serialization.

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "backend.h"
#include "bram.h"
#include "carry.h"
#include "clock.h"
#include "device.h"
#include "ff.h"
#include "json11.hpp"
#include "log.h"
#include "lutram.h"
#include "mult.h"
#include "nextpnr.h"
#include "pack_helpers.h"
#include "srl.h"
#include "wide.h"

NEXTPNR_NAMESPACE_BEGIN

namespace spartan3e_detail {

// Side table entry for one nextpnr pip.  `twin` is the reverse pip of a `=-`
// pair, or -1. Route-through pips are not instantiated.
struct PipMeta
{
    int tile = -1;
    int from = -1;
    int to = -1;
    int twin = -1;
    int rt = -1;
};

struct SiteElemKey
{
    int tile = -1;
    int site = -1;
    std::string element, pin;

    bool operator<(const SiteElemKey &other) const
    {
        return std::tie(tile, site, element, pin) < std::tie(other.tile, other.site, other.element, other.pin);
    }
};

struct SitePipKey
{
    int tile = -1;
    int site = -1;
    std::string element, cfg;

    bool operator<(const SitePipKey &other) const
    {
        return std::tie(tile, site, element, cfg) < std::tie(other.tile, other.site, other.element, other.cfg);
    }
};

struct SiteConnKey
{
    std::string src_el, src_pin, dst_el, dst_pin;

    bool operator<(const SiteConnKey &other) const
    {
        return std::tie(src_el, src_pin, dst_el, dst_pin) <
               std::tie(other.src_el, other.src_pin, other.dst_el, other.dst_pin);
    }
};

struct BelSiteInfo
{
    SiteRef site;
    std::string element;
};

struct SiteRouteMeta
{
    SiteRef site;
    SiteConnKey connection;
    std::optional<SitePipKey> selection;
};

struct CarrySite
{
    SiteRef site;
    std::array<BelId, 2> lut, ff, mux, xorcy, andcell;
    int next = -1;
};

struct MultSite
{
    SiteRef site;
    BelId bel;
    int next = -1;
};

struct Dsu
{
    std::vector<uint32_t> parent;
    std::vector<uint8_t> rank;

    explicit Dsu(uint32_t n) : parent(n), rank(n, 0) { std::iota(parent.begin(), parent.end(), 0); }

    uint32_t find(uint32_t x)
    {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }

    void unite(uint32_t a, uint32_t b)
    {
        uint32_t ra = find(a), rb = find(b);
        if (ra == rb)
            return;
        if (rank[ra] < rank[rb])
            std::swap(ra, rb);
        parent[rb] = ra;
        if (rank[ra] == rank[rb])
            rank[ra]++;
    }
};

struct Spartan3eImpl : DeviceBackend
{
    PackHelpers h;
    std::string db_path;
    std::string physical_path;
    ChipDb db;
    std::vector<uint32_t> tile_base;
    std::vector<WireId> member_wire;
    std::vector<PipMeta> pip_meta;
    std::unordered_map<std::string, SiteRef> site_by_name;
    std::unordered_map<std::string, std::unordered_map<std::string, size_t>> site_elem_index;
    std::map<SiteElemKey, WireId> site_wire;
    // Configuration mux pips are enabled only by the placed site's selections.
    std::map<SitePipKey, PipId> site_pip;
    std::unordered_map<int32_t, SitePipKey> site_pip_by_pip;
    std::unordered_set<int32_t> selected_site_pips;
    std::unordered_map<int32_t, BelSiteInfo> bel_site_info;
    std::unordered_map<std::string, BelId> package_bels;
    std::unordered_map<int32_t, std::string> bel_package_pin;
    std::unordered_map<int32_t, BelId> iob_off_bels;
    dict<ClusterId, CellInfo *> iob_off_clusters;
    std::unordered_map<int32_t, BelId> iob_tff_bels;
    dict<ClusterId, CellInfo *> iob_tff_clusters;
    std::unordered_map<int32_t, BelId> iob_if_bels;
    dict<ClusterId, CellInfo *> iob_if_clusters;
    // Diagnostic route provenance; no parsing of displayed pip names.
    std::unordered_map<int32_t, SiteRouteMeta> site_route_meta;
    std::vector<CarrySite> carry_sites;
    std::vector<Ram32Resources> ram32_resources;
    std::unordered_map<int32_t, size_t> ram32_resource_by_bel;
    std::vector<Ram16DualResources> ram16_dual_resources;
    std::unordered_map<int32_t, size_t> ram16_dual_resource_by_bel;
    std::vector<Ram64Resources> ram64_resources;
    std::unordered_map<int32_t, size_t> ram64_by_bel;
    std::unordered_map<int32_t, std::pair<int, int>> carry_bel_slot;
    std::vector<CarryChain> carry_chains;
    dict<ClusterId, size_t> carry_cluster;
    dict<IdString, CarryDi> carry_di;
    std::vector<std::array<BelId, 2>> wide_bels;
    std::unordered_map<int32_t, std::pair<int, int>> wide_bel_slot;
    // [destination site][source kind F5/FX][input 0/1] -> source site.
    std::vector<std::array<std::array<int, 2>, 2>> wide_sources;
    std::vector<WideTree> wide_trees;
    dict<ClusterId, size_t> wide_cluster;
    std::vector<int> srl_next_site;
    std::vector<SrlChain> srl_chains;
    dict<ClusterId, size_t> srl_cluster;
    std::vector<MultSite> mult_sites;
    std::unordered_map<int32_t, int> mult_site_by_bel;
    std::vector<MultChain> mult_chains;
    dict<ClusterId, size_t> mult_cluster;
    std::vector<ClockSite> clock_sites;
    std::unordered_map<int, BelId> clock_power_bels;
    std::unique_ptr<ClockModel> clock_model;
    std::set<std::pair<int, int>> active_sites;
    bool site_graph_built = false;
    unsigned site_pip_seq = 0;

    Spartan3eImpl(std::string path, std::string physical_design);
    void init(Context *ctx) override;
    void index_sites();
    WireId site_pin_wire(const Tile &tile, size_t ti, const SiteDef &site, const std::string &pin) const;
    WireId site_elem_wire(size_t ti, size_t si, const std::string &element, const std::string &pin);
    void build_site_graph();
    std::array<BelId, 4> add_carry_bels(const Tile &tile, size_t ti, size_t si, const std::string &site_name, int z);
    BelId add_lut4_bel(const Tile &tile, size_t ti, size_t si, const std::string &site_name, const std::string &element,
                       int z);
    BelId add_ff_bel(const Tile &tile, size_t ti, size_t si, const std::string &site_name, const std::string &element,
                     int z);
    BelId add_iob_bel(const Tile &tile, size_t ti, size_t si, const SiteDef &site, const std::string &site_name, int z);
    void add_bram_bel(const Tile &tile, int ti, int si, const std::string &name, int z);
    void init_bels();
    void index_carry_topology();
    void index_clock_topology();
    void index_wide_topology();
    void init_ram64_bels();
    void index_mult_topology();
    bool place_mult_chain(const MultChain &chain, BelId root,
                          std::vector<std::pair<CellInfo *, BelId>> &placement) const;
    void index_srl_topology();
    bool place_srl_chain(const SrlChain &chain, BelId root_bel,
                         std::vector<std::pair<CellInfo *, BelId>> &placement) const;
    bool place_wide_tree(const WideTree &tree, BelId root_bel,
                         std::vector<std::pair<CellInfo *, BelId>> &placement) const;
    std::optional<bool> getClusterPlacement(ClusterId cluster, BelId root_bel,
                                            std::vector<std::pair<CellInfo *, BelId>> &placement) const override;
    bool select_site_pip(const SiteRef &site, const std::string &element, const std::string &cfg);
    bool needs_slice_output(CellInfo *cell) const;
    bool local_ff_data(CellInfo *ff, const SiteRef &site, int half) const;
    bool dedicated_carry_input(CellInfo *cell) const;
    bool wide_resources_valid(int index) const;
    void assign_site_info();
    bool checkPipAvail(PipId pip) const override;
    bool checkPipAvailForNet(PipId pip, const NetInfo *net) const override;
    static bool is_io_site_type(const std::string &type);
    static bool is_lut_cell(IdString t);
    bool is_xilinx_ff_cell(IdString t) const;
    static bool is_xilinx_io_cell(IdString t);
    void pack_io_constraints();
    json11::Json diagnostic_io_route(CellInfo *ci, IdString port);
    void inherit_iob_register_attrs(CellInfo *ff);
    void pack_iob_off();
    void postRoute() override;
    std::set<int32_t> route_subtree(const NetInfo *net, WireId root) const;
    json11::Json::object physical_route_fragment(const NetInfo *net, WireId root, const std::set<int32_t> &wires,
                                                 const std::function<json11::Json(WireId)> &endpoint) const;
    void write_physical_design();
    void remove_transparent_buffers();
    void normalize_ibufg();
    void pack() override;
    void constrain_ff_locations();
    void fix_cluster_placement(const std::vector<std::pair<CellInfo *, BelId>> &placement);
    void postPlace() override;
    void preRoute() override;
    IdString getBelBucketForCellType(IdString cell_type) const override;
    BelBucketId getBelBucketForBel(BelId bel) const override;
    bool isValidBelForCellType(IdString cell_type, BelId bel) const override;
    bool ff_rev_resources_valid(int index) const;
    bool isBelLocationValid(BelId bel, bool) const override;
    void build_wires();
    void build_pips();
};

} // namespace spartan3e_detail

NEXTPNR_NAMESPACE_END

#endif
