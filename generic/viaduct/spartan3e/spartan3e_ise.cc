/*
 * nextpnr -- Spartan-3E ISE-native routing, BELs and carry placement (T40 stage 3).
 *
 * Step 2 builds the routing graph: one nextpnr wire per ISE electrical node,
 * one nextpnr pip per non-route-through XDLRC pip row, and a pip side table.
 * The g3b XDL replay hook is available through `--vopt check_xdl=<file>`.
 * Occupied CLB sites receive a lazy site graph after placement.
 */

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

#include "log.h"
#include "json11.hpp"
#include "nextpnr.h"
#include "spartan3e_ise.h"
#include "spartan3e_ise_carry.h"
#include "spartan3e_ise_wide.h"
#include "spartan3e_ise_clock.h"
#include "spartan3e_ise_bram.h"
#include "spartan3e_ise_lutram.h"
#include "spartan3e_ise_srl.h"
#include "spartan3e_ise_ff.h"
#include "spartan3e_ise_mult.h"
#include "spartan3e_ise_db.h"
#include "spartan3e_xdl.h"
#include "viaduct_api.h"
#include "viaduct_helpers.h"

#define GEN_INIT_CONSTIDS
#define VIADUCT_CONSTIDS "viaduct/spartan3e/constids.inc"
#include "viaduct_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {

// Side table entry for one nextpnr pip.  `twin` is the reverse pip of a `=-`
// pair, or -1.  Route-through pips are not instantiated in v1, so `rt` is
// currently always -1 but kept in the model for the later step.
struct IsePipMeta
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
    IseSiteRef site;
    std::string element;
};

struct IseSiteRouteMeta
{
    IseSiteRef site;
    SiteConnKey connection;
    std::optional<SitePipKey> selection;
};

struct CarrySite
{
    IseSiteRef site;
    std::array<BelId, 2> lut, ff, mux, xorcy, andcell;
    int next = -1;
};

struct MultSite
{
    IseSiteRef site;
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

struct Spartan3eIseImpl : ViaductAPI
{
    ViaductHelpers h;
    std::string db_path;
    std::string check_xdl_path;
    std::string dump_path;
    IseChipDb db;
    std::vector<uint32_t> tile_base;
    std::vector<WireId> member_wire;
    std::vector<IsePipMeta> pip_meta;
    std::unordered_map<std::string, IseSiteRef> site_by_name;
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
    std::unordered_map<int32_t, IseSiteRouteMeta> site_route_meta;
    std::vector<CarrySite> carry_sites;
    std::vector<IseRam32Resources> ram32_resources;
    std::unordered_map<int32_t, size_t> ram32_resource_by_bel;
    std::vector<IseRam16DualResources> ram16_dual_resources;
    std::unordered_map<int32_t, size_t> ram16_dual_resource_by_bel;
    std::vector<IseRam64Resources> ram64_resources;
    std::unordered_map<int32_t, size_t> ram64_by_bel;
    std::unordered_map<int32_t, std::pair<int, int>> carry_bel_slot;
    std::vector<IseCarryChain> carry_chains;
    dict<ClusterId, size_t> carry_cluster;
    dict<IdString, IseCarryDi> carry_di;
    std::vector<std::array<BelId, 2>> wide_bels;
    std::unordered_map<int32_t, std::pair<int, int>> wide_bel_slot;
    // [destination site][source kind F5/FX][input 0/1] -> source site.
    std::vector<std::array<std::array<int, 2>, 2>> wide_sources;
    std::vector<IseWideTree> wide_trees;
    dict<ClusterId, size_t> wide_cluster;
    std::vector<int> srl_next_site;
    std::vector<IseSrlChain> srl_chains;
    dict<ClusterId, size_t> srl_cluster;
    std::vector<MultSite> mult_sites;
    std::unordered_map<int32_t, int> mult_site_by_bel;
    std::vector<IseMultChain> mult_chains;
    dict<ClusterId, size_t> mult_cluster;
    std::vector<IseClockSite> clock_sites;
    std::unordered_map<int, BelId> clock_power_bels;
    std::unique_ptr<IseClockModel> clock_model;
    std::set<std::pair<int, int>> active_sites;
    bool site_graph_built = false;
    unsigned site_pip_seq = 0;

    Spartan3eIseImpl(std::string path, std::string check_path, std::string dump)
            : db_path(std::move(path)), check_xdl_path(std::move(check_path)), dump_path(std::move(dump))
    {
    }

    void init(Context *ctx) override
    {
        init_uarch_constids(ctx);
        ViaductAPI::init(ctx);
        h.init(ctx);
        db = parse_ise_chipdb(db_path);
        index_sites();
        build_wires();
        build_pips();
        log_info("Spartan-3E ISE: loaded '%s': tiles=%zu nodes=%d multi_nodes=%zu wires=%d pips=%d members=%zu\n",
                 db_path.c_str(), db.tiles.size(), ctx->getWires().size, db.nodes.size(), ctx->getWires().size,
                 ctx->getPips().size, member_wire.size());
        init_bels();
        index_carry_topology();
        index_wide_topology();
        index_srl_topology();
        index_mult_topology();
        init_ram64_bels();
        index_clock_topology();
        if (!check_xdl_path.empty())
            check_xdl_routing(ctx, db, tile_base, member_wire, site_by_name, check_xdl_path);
        site_by_name.clear();
    }

    void index_sites()
    {
        site_by_name.reserve(db.tiles.size() * 2); // ~2 sites per tile on average
        for (size_t ti = 0; ti < db.tiles.size(); ti++) {
            const auto &tile = db.tiles[ti];
            if (tile.variant < 0 || tile.variant >= (int)db.variants.size())
                log_error("spartan3e-ise: tile %s has bad variant %d\n", tile.name.c_str(), tile.variant);
            if (tile.sites.size() > db.variants[tile.variant].sites.size())
                log_error("spartan3e-ise: tile %s has too many sites\n", tile.name.c_str());
            for (size_t si = 0; si < tile.sites.size(); si++) {
                if (!site_by_name.emplace(tile.sites[si], IseSiteRef{(int)ti, (int)si}).second)
                    log_error("spartan3e-ise: duplicate site name %s\n", tile.sites[si].c_str());
            }
        }
        for (const auto &kv : db.site_defs) {
            auto &index = site_elem_index[kv.first];
            for (size_t ei = 0; ei < kv.second.elements.size(); ei++)
                index[kv.second.elements[ei].name] = ei;
        }
    }

    WireId site_pin_wire(const IseTile &tile, size_t ti, const IseSiteDef &site, const std::string &pin) const
    {
        auto it = site.pins.find(pin);
        if (it == site.pins.end() || it->second < 0)
            return WireId();
        uint32_t member = tile_base[ti] + (uint32_t)it->second;
        if (member >= member_wire.size())
            return WireId();
        return member_wire[member];
    }

    WireId site_elem_wire(size_t ti, size_t si, const std::string &element, const std::string &pin)
    {
        SiteElemKey key{(int)ti, (int)si, element, pin};
        auto it = site_wire.find(key);
        if (it != site_wire.end())
            return it->second;
        const auto &tile = db.tiles[ti];
        IdStringList name = h.xy_id(tile.col, tile.row,
                                    ctx->idf("S_%s_%s_%s", tile.sites[si].c_str(), element.c_str(), pin.c_str()));
        WireId wire = ctx->addWire(name, ctx->id("WIRE"), tile.col, tile.row);
        site_wire.emplace(key, wire);
        return wire;
    }

    void build_site_graph()
    {
        log_info("Spartan-3E ISE: creating occupied site graphs...\n");
        static const std::set<std::string> clb_selected = {
                "F",      "G",      "FFX",      "FFY",    "FXMUX",  "GYMUX", "DXMUX",  "DYMUX",
                "XUSED",  "YUSED",  "CLKINV",   "CEINV",  "SRINV",  "BXINV", "BYINV",  "XORF",
                "XORG",   "FAND",   "GAND",     "CYINIT", "CY0F",   "CY0G",  "CYSELF", "CYSELG",
                "CYMUXF", "CYMUXG", "COUTUSED", "XBUSED", "YBUSED", "XBMUX", "YBMUX", "SRFFMUX", "REVUSED",
                "F5MUX", "F6MUX", "F5USED", "FXUSED",
                "DIF_MUX", "DIG_MUX", "WSGEN", "SLICEWE0USED", "SLICEWE1USED",
                "BYOUTUSED", "BYINVOUTUSED", "DIGUSED", "SHIFTIN", "SHIFTOUT", "SHIFTOUTUSED",
                "FMC15_BLACKBOX", "GMC15_BLACKBOX",
                "WF1USED", "WF2USED", "WF3USED", "WF4USED", "WG1USED", "WG2USED", "WG3USED", "WG4USED",
        };
        static const std::set<std::string> io_selected = {"INBUF", "IDELMUX", "IMUX", "O1INV", "OMUX", "OUTBUF", "T1INV", "TMUX", "TFF1", "TCEINV", "TSR_USED", "OFF1", "O1_DDRMUX", "OTCLK1INV", "OCEINV", "PCICE_MUX", "SRINV", "OSR_USED", "IFF1", "IFFDELMUX", "IFFDMUX", "IDDRIN_MUX", "ICLK1INV", "ICEINV", "ISR_USED"};
        static const std::set<std::string> clock_selected = {"I0_USED", "I1_USED", "SINV", "GCLKMUX", "GCLK_BUFFER"};
        static const std::set<std::string> power_selected = {"POWER"};
        static const std::set<std::string> bram_selected = {"RAMB16", "RAMB16A", "RAMB16B", "CLKAINV", "CLKBINV",
                "ENAINV", "ENBINV", "WEAINV", "WEBINV", "SSRAINV", "SSRBINV"};
        static const std::set<std::string> mult_selected = {"MULT18X18SIO", "CLKINV", "CEAINV", "CEBINV",
                "CEPINV", "RSTAINV", "RSTBINV", "RSTPINV"};
        static const std::set<std::string> startup_selected = {"STARTUP", "CLKINV", "GTSINV", "GSRINV", "CLK", "GSR", "GTS", "MBT"};
        static const std::set<std::string> capture_selected = {"CAPTURE_BLACKBOX", "CLKINV", "CAPINV", "ONESHOT_ATTR", "CLK", "CAP"};
        static const std::set<std::string> bscan_selected = {"BSCAN_BLACKBOX", "TDO1", "TDO2", "SEL1", "SEL2", "TDI",
                "SHIFT", "UPDATE", "DRCK1", "DRCK2", "RESET", "CAPTURE"};
        // Logic BELs compute their outputs; they are not routing multiplexers.
        static const std::set<std::string> no_internal_pips = {"F",      "G",    "FFX",  "FFY",  "CYMUXF",
                                                               "CYMUXG", "XORF", "XORG", "FAND", "GAND", "F5MUX", "F6MUX",
                                                               "GCLKMUX", "GCLK_BUFFER", "POWER", "WSGEN",
                                                               "FMC15_BLACKBOX", "GMC15_BLACKBOX",
                                                               "RAMB16", "RAMB16A", "RAMB16B", "MULT18X18SIO",
                                                               "STARTUP", "CAPTURE_BLACKBOX", "BSCAN_BLACKBOX"};
        if (site_graph_built)
            return;
        site_graph_built = true;
        size_t wires = 0, pips = 0;
        for (const auto &site_key : active_sites) {
            size_t ti = (size_t)site_key.first;
            size_t si = (size_t)site_key.second;
            if (ti >= db.tiles.size())
                continue;
            const auto &tile = db.tiles[ti];
            const auto &variant = db.variants[tile.variant];
            if (si >= tile.sites.size() || si >= variant.sites.size())
                continue;
            {
                const auto &site = variant.sites[si];
                bool io = is_io_site_type(site.type);
                if (site.type != "SLICEL" && site.type != "SLICEM" && !io && site.type != "BUFGMUX" && site.type != "VCC" &&
                    site.type != "RAMB16" && site.type != "MULT18X18SIO" && site.type != "STARTUP" && site.type != "CAPTURE" && site.type != "BSCAN")
                    continue;
                const auto &selected = io ? io_selected : site.type == "BUFGMUX" ? clock_selected :
                                       site.type == "VCC" ? power_selected : site.type == "RAMB16" ? bram_selected :
                                       site.type == "MULT18X18SIO" ? mult_selected :
                                       site.type == "STARTUP" ? startup_selected : site.type == "CAPTURE" ? capture_selected :
                                       site.type == "BSCAN" ? bscan_selected : clb_selected;
                auto st_it = db.site_defs.find(site.type);
                if (st_it == db.site_defs.end())
                    log_error("spartan3e-ise: missing site_defs for %s\n", site.type.c_str());
                const auto &site_type = st_it->second;
                const auto &elem_index = site_elem_index.at(site.type);

                for (const auto &ename : selected) {
                    auto eit = elem_index.find(ename);
                    if (eit == elem_index.end())
                        continue;
                    const auto &elem = site_type.elements[eit->second];
                    for (auto &pin : elem.pins) {
                        site_elem_wire(ti, si, elem.name, pin.first);
                        wires++;
                    }
                }

                std::set<SiteConnKey> seen;
                for (const auto &ename : selected) {
                    auto eit = elem_index.find(ename);
                    if (eit == elem_index.end())
                        continue;
                    const auto &elem = site_type.elements[eit->second];
                    for (auto &conn : elem.conns) {
                        std::string src_el, src_pin, dst_el, dst_pin;
                        if (conn.op == "==>") {
                            src_el = conn.from;
                            src_pin = conn.from_pin;
                            dst_el = conn.to;
                            dst_pin = conn.to_pin;
                        } else if (conn.op == "<==") {
                            src_el = conn.to;
                            src_pin = conn.to_pin;
                            dst_el = conn.from;
                            dst_pin = conn.from_pin;
                        } else {
                            continue;
                        }
                        SiteConnKey key{src_el, src_pin, dst_el, dst_pin};
                        if (!seen.insert(key).second)
                            continue;
                        auto resolve = [&](const std::string &el, const std::string &pin) -> WireId {
                            if (site.pins.count(el))
                                return site_pin_wire(tile, ti, site, el);
                            if (!selected.count(el))
                                return WireId();
                            if (elem_index.find(el) == elem_index.end())
                                return WireId();
                            return site_elem_wire(ti, si, el, pin);
                        };
                        WireId sw = resolve(src_el, src_pin);
                        WireId dw = resolve(dst_el, dst_pin);
                        if (sw == WireId() || dw == WireId() || sw == dw)
                            continue;
                        IdStringList name = h.xy_id(tile.col, tile.row,
                                                    ctx->idf("SP_%s_%u", tile.sites[si].c_str(), site_pip_seq++));
                        PipId pip = ctx->addPip(name, id_PIP, sw, dw, 0, Loc(tile.col, tile.row, 0));
                        site_route_meta[pip.index] = IseSiteRouteMeta{IseSiteRef{int(ti), int(si)}, key, std::nullopt};
                        pips++;
                    }
                }

                for (const auto &ename : selected) {
                    auto eit = elem_index.find(ename);
                    if (eit == elem_index.end())
                        continue;
                    const auto &elem = site_type.elements[eit->second];
                    if (no_internal_pips.count(elem.name) || (io && elem.bel) || elem.pins.empty())
                        continue;
                    std::string out_pin;
                    for (auto &pin : elem.pins)
                        if (pin.second == "output") {
                            out_pin = pin.first;
                            break;
                        }
                    if (out_pin.empty())
                        continue;
                    for (auto &pin : elem.pins) {
                        if (pin.second != "input")
                            continue;
                        WireId sw = site_elem_wire(ti, si, elem.name, pin.first);
                        WireId dw = site_elem_wire(ti, si, elem.name, out_pin);
                        if (sw == WireId() || dw == WireId() || sw == dw)
                            continue;
                        IdStringList name = h.xy_id(tile.col, tile.row,
                                                    ctx->idf("SP_%s_%u", tile.sites[si].c_str(), site_pip_seq++));
                        PipId pip = ctx->addPip(name, id_PIP, sw, dw, 0, Loc(tile.col, tile.row, 0));
                        site_route_meta[pip.index] = IseSiteRouteMeta{
                            IseSiteRef{int(ti), int(si)}, SiteConnKey{elem.name, pin.first, elem.name, out_pin},
                            SitePipKey{int(ti), int(si), elem.name, pin.first}};
                        if (!elem.cfg.empty()) {
                            SitePipKey key{(int)ti, (int)si, elem.name, pin.first};
                            site_pip[key] = pip;
                            site_pip_by_pip[pip.index] = key;
                        }
                        pips++;
                    }
                }
            }
        }
        log_info("Spartan-3E ISE: occupied site graphs: %zu wires, %zu pips\n", wires, pips);
    }

    std::array<BelId, 4> add_carry_bels(const IseTile &tile, size_t ti, size_t si, const std::string &site_name, int z)
    {
        std::array<BelId, 4> result;
        // MUXCY + XORCY bels per slice half (F or G).  These connect to the
        // carry chain element wires already built by build_site_graph().
        // Pin names match the chipdb site_defs: CYMUXF/G has pins 0,1,S0,OUT;
        // XORF/G has pins 0,1,O; CYSELF/G has pins F/G,1,OUT.
        for (int half = 0; half < 2; half++) {
            const char *cy0 = half ? "CY0G" : "CY0F";
            const char *cysel = half ? "CYSELG" : "CYSELF";
            const char *cymux = half ? "CYMUXG" : "CYMUXF";
            const char *xor_el = half ? "XORG" : "XORF";

            // MUXCY bel: S ← CYSELF/G.OUT, DI ← CY0F/G.OUT,
            //            CI ← CYMUXF/G.1 (carry chain), O → CYMUXF/G.OUT
            IdStringList mux_name =
                    h.xy_id(tile.col, tile.row, ctx->idf("%s_MUXCY%s", site_name.c_str(), half ? "G" : "F"));
            BelId muxbel = ctx->addBel(mux_name, id_MUXCY, Loc(tile.col, tile.row, z), false, false);
            result[2 * half] = muxbel;
            bel_site_info[muxbel.index] = BelSiteInfo{IseSiteRef{(int)ti, (int)si}, cymux};
            z++;
            WireId s_w = site_elem_wire(ti, si, cysel, "OUT");
            if (s_w != WireId())
                ctx->addBelInput(muxbel, id_S, s_w);
            WireId di_w = site_elem_wire(ti, si, cy0, "OUT");
            if (di_w != WireId())
                ctx->addBelInput(muxbel, id_DI, di_w);
            WireId ci_w = site_elem_wire(ti, si, cymux, "1");
            if (ci_w != WireId())
                ctx->addBelInput(muxbel, id_CI, ci_w);
            WireId co_w = site_elem_wire(ti, si, cymux, "OUT");
            if (co_w != WireId())
                ctx->addBelOutput(muxbel, id_O, co_w);

            // XORCY bel: LI ← XORF/G.0 (LUT output), CI ← XORF/G.1 (carry),
            //            O → XORF/G.O (sum)
            IdStringList xor_name =
                    h.xy_id(tile.col, tile.row, ctx->idf("%s_XORCY%s", site_name.c_str(), half ? "G" : "F"));
            BelId xorbel = ctx->addBel(xor_name, id_XORCY, Loc(tile.col, tile.row, z), false, false);
            result[2 * half + 1] = xorbel;
            bel_site_info[xorbel.index] = BelSiteInfo{IseSiteRef{(int)ti, (int)si}, xor_el};
            z++;
            WireId li_w = site_elem_wire(ti, si, xor_el, "0");
            if (li_w != WireId())
                ctx->addBelInput(xorbel, id_LI, li_w);
            WireId xci_w = site_elem_wire(ti, si, xor_el, "1");
            if (xci_w != WireId())
                ctx->addBelInput(xorbel, id_CI, xci_w);
            WireId xout_w = site_elem_wire(ti, si, xor_el, "O");
            if (xout_w != WireId())
                ctx->addBelOutput(xorbel, id_O, xout_w);
        }
        return result;
    }

    BelId add_lut4_bel(const IseTile &tile, size_t ti, size_t si, const std::string &site_name,
                       const std::string &element, int z)
    {
        IdStringList name = h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id(element)));
        BelId bel = ctx->addBel(name, id_LUT4, Loc(tile.col, tile.row, z), false, false);
        bel_site_info[bel.index] = BelSiteInfo{IseSiteRef{(int)ti, (int)si}, element};
        for (int k = 0; k < 4; k++) {
            WireId w = site_elem_wire(ti, si, element, "A" + std::to_string(k + 1));
            if (w != WireId())
                ctx->addBelInput(bel, ctx->idf("I%d", k), w);
            if (db.variants[tile.variant].sites[si].type == "SLICEM") {
                ctx->addBelInput(bel, ctx->idf("A%d", k), w);
                ctx->addBelInput(bel, ctx->idf("WA%d", k),
                    site_elem_wire(ti, si, element, (element == "F" ? "WF" : "WG") + std::to_string(k + 1)));
            }
        }
        if (db.variants[tile.variant].sites[si].type == "SLICEM") {
            ctx->addBelInput(bel, id_D, site_elem_wire(ti, si, element, "DI"));
            ctx->addBelInput(bel, ctx->id("WCLK"), site_elem_wire(ti, si, "WSGEN", "CK"));
            ctx->addBelInput(bel, ctx->id("WE"), site_elem_wire(ti, si, "WSGEN", "WE"));
            ctx->addBelInput(bel, ctx->id("CLK"), site_elem_wire(ti, si, "WSGEN", "CK"));
            ctx->addBelInput(bel, id_CE, site_elem_wire(ti, si, "WSGEN", "WE"));
            ctx->addBelOutput(bel, id_Q, site_elem_wire(ti, si, element, "D"));
            ctx->addBelOutput(bel, ctx->id("Q15"), site_elem_wire(ti, si, element + "MC15_BLACKBOX", "MC15"));
        }
        WireId out = site_elem_wire(ti, si, element, "D");
        if (out != WireId())
            ctx->addBelOutput(bel, ctx->id("O"), out);
        return bel;
    }

    BelId add_ff_bel(const IseTile &tile, size_t ti, size_t si, const std::string &site_name,
                     const std::string &element, int z)
    {
        IdStringList name = h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id(element)));
        BelId bel = ctx->addBel(name, element == "OFF1" ? ctx->id("IOB_OFF1") : element == "IFF1" ? ctx->id("IOB_IFF1") : element == "TFF1" ? ctx->id("IOB_TFF1") : id_DFF, Loc(tile.col, tile.row, z), false, false);
        bel_site_info[bel.index] = BelSiteInfo{IseSiteRef{(int)ti, (int)si}, element};
        WireId d = site_elem_wire(ti, si, element, "D");
        if (d != WireId())
            ctx->addBelInput(bel, id_D, d);
        WireId clk = site_elem_wire(ti, si, element, "CK");
        if (clk != WireId())
            ctx->addBelInput(bel, ctx->id("C"), clk);
        WireId ce = site_elem_wire(ti, si, element, "CE");
        if (ce != WireId())
            ctx->addBelInput(bel, id_CE, ce);
        WireId sr = site_elem_wire(ti, si, element, "SR");
        if (sr != WireId()) {
            ctx->addBelInput(bel, ctx->id("R"), sr);
            ctx->addBelInput(bel, id_S, sr);
            ctx->addBelInput(bel, ctx->id("CLR"), sr);
            ctx->addBelInput(bel, ctx->id("PRE"), sr);
        }
        if (element == "FFX" || element == "FFY") {
            ctx->addBelInput(bel, ctx->id("REV_R"), site_elem_wire(ti, si, element, "REV"));
            ctx->addBelInput(bel, ctx->id("REV_S"), site_elem_wire(ti, si, element, "REV"));
        }
        WireId q = site_elem_wire(ti, si, element, "Q");
        if (q != WireId())
            ctx->addBelOutput(bel, id_Q, q);
        return bel;
    }

    BelId add_iob_bel(const IseTile &tile, size_t ti, size_t si, const IseSiteDef &site, const std::string &site_name,
                     int z)
    {
        IdStringList name = h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id("IOB")));
        BelId bel = ctx->addBel(name, id_IOB, Loc(tile.col, tile.row, z), false, false);
        bel_site_info[bel.index] = BelSiteInfo{IseSiteRef{(int)ti, (int)si}, "IOB"};
        WireId out = site_elem_wire(ti, si, "INBUF", "OUT");
        if (out != WireId())
            ctx->addBelOutput(bel, id_O, out);
        bool output_capable = site_elem_index.at(site.type).count("OUTBUF") != 0;
        WireId in = output_capable ? site_elem_wire(ti, si, "OUTBUF", "IN") : WireId();
        if (output_capable)
            ctx->addBelInput(bel, id_I, in);
        if (output_capable)
            ctx->addBelInput(bel, ctx->id("T"), site_elem_wire(ti, si, "OUTBUF", "TRI"));
        IdStringList pad_name = h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id("PAD")));
        WireId pad = ctx->addWire(pad_name, ctx->id("IO"), tile.col, tile.row);
        ctx->addBelInout(bel, ctx->id("IO"), pad);
        // XDLRC for a packaged part already names bonded sites by package pin.
        // Other exporters may retain the partgen pad name; both require table evidence.
        for (const auto &pin : db.package_pads) {
            if (site_name != pin.first && site_name != pin.second)
                continue;
            if (!package_bels.emplace(pin.first, bel).second || bel_package_pin.count(bel.index))
                log_error("spartan3e-ise: ambiguous package binding for %s\n", site_name.c_str());
            bel_package_pin[bel.index] = pin.first;
        }
        if (!bel_package_pin.count(bel.index))
            log_error("spartan3e-ise: bonded I/O site %s missing from package table\n", site_name.c_str());
        return bel;
    }

    void add_bram_bel(const IseTile &tile, int ti, int si, const std::string &name, int z)
    {
        BelId bel = ctx->addBel(h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(name), ctx->id("RAMB16"))),
                               ctx->id("RAMB16"), Loc(tile.col, tile.row, z), false, false);
        bel_site_info[bel.index] = BelSiteInfo{IseSiteRef{ti, si}, "RAMB16"};
        // One composite occupancy resource owns the array and both computational
        // port wrappers. No input-to-output bypass is introduced between them.
        for (const std::string &port : {std::string("A"), std::string("B")}) {
            std::string wrapper = "RAMB16" + port;
            for (int i = 0; i < 14; ++i)
                ctx->addBelInput(bel, ctx->idf("ADDR%s%d", port.c_str(), i),
                                 site_elem_wire(ti, si, wrapper, "ADDR" + port + std::to_string(i)));
            for (const std::string &base : {std::string("DI"), std::string("DIP"), std::string("DO"), std::string("DOP")})
                for (int i = 0; i < (base == "DI" || base == "DO" ? 32 : 4); ++i) {
                    std::string pin = base + port + std::to_string(i);
                    WireId wire = site_elem_wire(ti, si, wrapper, pin);
                    if (base == "DI" || base == "DIP") ctx->addBelInput(bel, ctx->id(pin), wire);
                    else ctx->addBelOutput(bel, ctx->id(pin), wire);
                }
            for (const std::string &control : {std::string("CLK"), std::string("EN"), std::string("WE"), std::string("SSR")})
                ctx->addBelInput(bel, ctx->id(control + port), site_elem_wire(ti, si, wrapper, control + port));
        }
    }

    void init_bels()
    {
        log_info("Spartan-3E ISE: creating bels...\n");
        std::map<std::pair<int, int>, int> zpos;
        size_t nlut = 0, nff = 0, nio = 0;
        for (size_t ti = 0; ti < db.tiles.size(); ti++) {
            const auto &tile = db.tiles[ti];
            const auto &variant = db.variants[tile.variant];
            int &z = zpos[{tile.col, tile.row}];
            for (size_t si = 0; si < tile.sites.size() && si < variant.sites.size(); si++) {
                const auto &site = variant.sites[si];
                const std::string &site_name = tile.sites[si];
                if (site.type == "SLICEL" || site.type == "SLICEM") {
                    CarrySite carry;
                    carry.site = IseSiteRef{int(ti), int(si)};
                    carry.lut[0] = add_lut4_bel(tile, ti, si, site_name, "F", z++);
                    carry.ff[0] = add_ff_bel(tile, ti, si, site_name, "FFX", z++);
                    carry.lut[1] = add_lut4_bel(tile, ti, si, site_name, "G", z++);
                    carry.ff[1] = add_ff_bel(tile, ti, si, site_name, "FFY", z++);
                    auto bels = add_carry_bels(tile, ti, si, site_name, z);
                    std::array<BelId, 2> wide;
                    for (int kind = 0; kind < 2; ++kind) {
                        const std::string element = kind == 0 ? "F5MUX" : "F6MUX";
                        BelId bel = ctx->addBel(h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id(element))),
                                               kind == 0 ? id_MUXF5 : id_MUXF6, Loc(tile.col, tile.row, z + 4 + kind), false, false);
                        wide[kind] = bel;
                        bel_site_info[bel.index] = BelSiteInfo{IseSiteRef{int(ti), int(si)}, element};
                        wide_bel_slot[bel.index] = {int(carry_sites.size()), kind};
                        ctx->addBelInput(bel, id_I0, site_elem_wire(ti, si, element, kind == 0 ? "G" : "0"));
                        ctx->addBelInput(bel, id_I1, site_elem_wire(ti, si, element, kind == 0 ? "F" : "1"));
                        ctx->addBelInput(bel, id_S, site_elem_wire(ti, si, element, "S0"));
                        ctx->addBelOutput(bel, id_O, site_elem_wire(ti, si, element, "OUT"));
                    }
                    wide_bels.push_back(wide);
                    for (int half = 0; half < 2; ++half) {
                        const std::string element = half == 0 ? "FAND" : "GAND";
                        BelId bel = ctx->addBel(h.xy_id(tile.col, tile.row,
                            IdStringList::concat(ctx->id(site_name), ctx->id(element))), ctx->id("MULT_AND"),
                            Loc(tile.col, tile.row, z + (site.type == "SLICEM" ? 8 : 6) + half), false, false);
                        carry.andcell[half] = bel;
                        bel_site_info[bel.index] = BelSiteInfo{IseSiteRef{int(ti), int(si)}, element};
                        ctx->addBelInput(bel, id_I0, site_elem_wire(ti, si, element, "0"));
                        ctx->addBelInput(bel, id_I1, site_elem_wire(ti, si, element, "1"));
                        ctx->addBelOutput(bel, ctx->id("LO"), site_elem_wire(ti, si, element, "O"));
                    }
                    if (site.type == "SLICEM") {
                        BelId composite = ctx->addBel(h.xy_id(tile.col, tile.row,
                            IdStringList::concat(ctx->id(site_name), ctx->id("RAM32"))), ctx->id("RAM32X1S"),
                            Loc(tile.col, tile.row, z + 6), false, false);
                        bel_site_info[composite.index] = BelSiteInfo{IseSiteRef{int(ti), int(si)}, "F5MUX"};
                        connect_ise_ram32(ctx, composite, [&](const std::string &element, const std::string &pin) {
                            return site_elem_wire(ti, si, element, pin);
                        });
                        IseRam32Resources resources{composite, {carry.lut[0], carry.lut[1],
                            bels[0], bels[1], bels[2], bels[3], wide[0], wide[1], carry.andcell[0], carry.andcell[1]}, carry.ff};
                        BelId dual = ctx->addBel(h.xy_id(tile.col, tile.row,
                            IdStringList::concat(ctx->id(site_name), ctx->id("RAM16D"))), ctx->id("RAM16X1D"),
                            Loc(tile.col, tile.row, z + 7), false, false);
                        bel_site_info[dual.index] = BelSiteInfo{IseSiteRef{int(ti), int(si)}, "F"};
                        connect_ise_ram16_dual(ctx, dual, [&](const std::string &element, const std::string &pin) {
                            return site_elem_wire(ti, si, element, pin);
                        });
                        IseRam16DualResources dual_resources{dual, resources.members};
                        dual_resources.excluded.push_back(composite);
                        for (BelId ff : carry.ff) dual_resources.excluded.push_back(ff);
                        ram16_dual_resource_by_bel[dual.index] = ram16_dual_resources.size();
                        for (BelId member : dual_resources.excluded)
                            ram16_dual_resource_by_bel[member.index] = ram16_dual_resources.size();
                        ram16_dual_resources.push_back(std::move(dual_resources));
                        resources.members.push_back(dual);
                        ram32_resource_by_bel[composite.index] = ram32_resources.size();
                        for (BelId member : resources.members) ram32_resource_by_bel[member.index] = ram32_resources.size();
                        for (BelId ff : resources.ffs) ram32_resource_by_bel[ff.index] = ram32_resources.size();
                        ram32_resources.push_back(std::move(resources));
                    }
                    for (int half = 0; half < 2; half++) {
                        carry.mux[half] = bels[2 * half];
                        carry.xorcy[half] = bels[2 * half + 1];
                        for (auto bel : {carry.lut[half], carry.ff[half], carry.mux[half], carry.xorcy[half], carry.andcell[half]})
                            carry_bel_slot[bel.index] = {int(carry_sites.size()), half};
                    }
                    carry_sites.push_back(carry);
                    z += site.type == "SLICEM" ? 10 : 8; // Includes memory composites and both product BELs.
                    nlut += 2;
                    nff += 2;
                } else if (is_io_site_type(site.type) && si < tile.bonding.size() && tile.bonding[si] == "bonded") {
                    int io_z = z++;
                    BelId pad = add_iob_bel(tile, ti, si, site, site_name, io_z);
                    iob_if_bels[pad.index] = add_ff_bel(tile, ti, si, site_name, "IFF1", z++);
                    if (site_elem_index.at(site.type).count("OUTBUF")) {
                        BelId ff = add_ff_bel(tile, ti, si, site_name, "OFF1", z++);
                        iob_off_bels[pad.index] = ff;
                        if (site_elem_index.at(site.type).count("TFF1"))
                            iob_tff_bels[pad.index] = add_ff_bel(tile, ti, si, site_name, "TFF1", z++);
                    }
                    nio++;
                } else if (site.type == "RAMB16") {
                    add_bram_bel(tile, int(ti), int(si), site_name, z++);
                } else if (site.type == "MULT18X18SIO") {
                    BelId bel = ctx->addBel(h.xy_id(tile.col, tile.row,
                        IdStringList::concat(ctx->id(site_name), ctx->id("MULT18X18SIO"))), ctx->id("MULT18X18SIO"),
                        Loc(tile.col, tile.row, z++), false, false);
                    bel_site_info[bel.index] = BelSiteInfo{IseSiteRef{int(ti), int(si)}, "MULT18X18SIO"};
                    mult_site_by_bel[bel.index] = int(mult_sites.size());
                    mult_sites.push_back({IseSiteRef{int(ti), int(si)}, bel, -1});
                    const auto &elements = db.site_defs.at("MULT18X18SIO").elements;
                    auto multiplier = std::find_if(elements.begin(), elements.end(), [](const auto &element) {
                        return element.name == "MULT18X18SIO";
                    });
                    if (multiplier == elements.end()) log_error("spartan3e-ise: missing multiplier site element\n");
                    for (const auto &pin : multiplier->pins) {
                        WireId wire = site_elem_wire(ti, si, multiplier->name, pin.first);
                        if (pin.second == "input") ctx->addBelInput(bel, ctx->id(pin.first), wire);
                        else ctx->addBelOutput(bel, ctx->id(pin.first), wire);
                    }
                } else if (site.type == "BUFGMUX") {
                    BelId bel = ctx->addBel(h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id("GCLK_BUFFER"))),
                                           id_BUFGMUX, Loc(tile.col, tile.row, z++), true, true);
                    IseSiteRef ref{int(ti), int(si)};
                    bel_site_info[bel.index] = BelSiteInfo{ref, "GCLK_BUFFER"};
                    ctx->addBelInput(bel, id_I, site_elem_wire(ti, si, "GCLKMUX", "I0"));
                    ctx->addBelInput(bel, id_I0, site_elem_wire(ti, si, "GCLKMUX", "I0"));
                    ctx->addBelInput(bel, id_I1, site_elem_wire(ti, si, "GCLKMUX", "I1"));
                    ctx->addBelInput(bel, id_S, site_elem_wire(ti, si, "GCLKMUX", "S"));
                    ctx->addBelOutput(bel, id_O, site_elem_wire(ti, si, "GCLK_BUFFER", "OUT"));
                    IseClockSite clock;
                    clock.site = ref;
                    clock.bel = bel;
                    clock.inputs = {{site_pin_wire(tile, ti, site, "I0"), site_pin_wire(tile, ti, site, "I1")}};
                    clock.output = site_pin_wire(tile, ti, site, "O");
                    clock_sites.push_back(std::move(clock));
                } else if (site.type == "VCC") {
                    BelId bel = ctx->addBel(h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id("POWER"))),
                                           ctx->id("VCC"), Loc(tile.col, tile.row, z++), false, false);
                    bel_site_info[bel.index] = BelSiteInfo{IseSiteRef{int(ti), int(si)}, "POWER"};
                    ctx->addBelOutput(bel, ctx->id("P"), site_elem_wire(ti, si, "POWER", "1"));
                    clock_power_bels[int(ti)] = bel;
                } else if (site.type == "STARTUP" || site.type == "CAPTURE" || site.type == "BSCAN") {
                    // Fixed single-site control primitives; the BEL is the site's
                    // blackbox element and the pins route through the corner muxes.
                    const std::string element = site.type == "STARTUP" ? "STARTUP" :
                                                site.type == "CAPTURE" ? "CAPTURE_BLACKBOX" : "BSCAN_BLACKBOX";
                    const IseSiteElement *bel_elem = nullptr;
                    for (const auto &candidate : db.site_defs.at(site.type).elements)
                        if (candidate.name == element)
                            bel_elem = &candidate;
                    if (bel_elem == nullptr)
                        log_error("spartan3e-ise: %s site has no %s element\n", site.type.c_str(), element.c_str());
                    BelId bel = ctx->addBel(h.xy_id(tile.col, tile.row,
                                                   IdStringList::concat(ctx->id(site_name), ctx->id(site.type))),
                                           ctx->id(site.type), Loc(tile.col, tile.row, z++), false, false);
                    bel_site_info[bel.index] = BelSiteInfo{IseSiteRef{int(ti), int(si)}, element};
                    for (const auto &pin : bel_elem->pins) {
                        WireId w = site_elem_wire(ti, si, element, pin.first);
                        if (pin.second == "input")
                            ctx->addBelInput(bel, ctx->id(pin.first), w);
                        else if (pin.second == "output")
                            ctx->addBelOutput(bel, ctx->id(pin.first), w);
                    }
                }
            }
        }
        log_info("Spartan-3E ISE: created %zu LUT4 bels, %zu DFF bels, %zu IOB bels\n", nlut, nff, nio);
    }

    void index_carry_topology()
    {
        std::unordered_map<int32_t, int> cin_site;
        auto pin_wire = [&](const CarrySite &carry, const std::string &pin) {
            const auto &tile = db.tiles[carry.site.tile];
            return site_pin_wire(tile, carry.site.tile, db.variants[tile.variant].sites[carry.site.site], pin);
        };
        for (size_t i = 0; i < carry_sites.size(); i++)
            cin_site.emplace(pin_wire(carry_sites[i], "CIN").index, int(i));
        size_t links = 0;
        for (auto &carry : carry_sites) {
            WireId cout = pin_wire(carry, "COUT");
            auto match = [&](WireId wire) {
                auto it = cin_site.find(wire.index);
                if (it == cin_site.end())
                    return;
                if (carry.next != -1 && carry.next != it->second)
                    log_error("Spartan-3E ISE: ambiguous COUT/CIN topology\n");
                carry.next = it->second;
            };
            match(cout);
            for (auto pip : ctx->getPipsDownhill(cout))
                match(ctx->getPipDstWire(pip));
            if (carry.next != -1)
                links++;
        }
        log_info("Spartan-3E ISE: indexed %zu dedicated carry site links\n", links);
    }

    void index_clock_topology()
    {
        std::unordered_map<int32_t, BelId> pads;
        std::unordered_set<int32_t> pins;
        IseClockPins physical_clock_pins;
        for (const auto &entry : bel_package_pin) {
            BelId bel{entry.first};
            const auto &ref = bel_site_info.at(bel.index).site;
            const auto &tile = db.tiles[ref.tile];
            const auto &def = db.variants[tile.variant].sites[ref.site];
            pads[site_pin_wire(tile, ref.tile, def, "I").index] = bel;
        }
        for (const auto &carry : carry_sites) {
            const auto &tile = db.tiles[carry.site.tile];
            const auto &def = db.variants[tile.variant].sites[carry.site.site];
            WireId pin = site_pin_wire(tile, carry.site.tile, def, "CLK");
            pins.insert(pin.index);
            for (BelId bel : carry.ff) {
                physical_clock_pins[bel.index][ctx->id("C")] = {pin};
                physical_clock_pins[bel.index][ctx->id("G")] = {pin};
            }
            for (BelId bel : carry.lut) physical_clock_pins[bel.index][ctx->id("WCLK")] = {pin};
            for (BelId bel : carry.lut) physical_clock_pins[bel.index][ctx->id("CLK")] = {pin};
        }
        for (const auto &entry : bel_site_info) {
            if (ctx->getBelType(BelId{entry.first}).in(ctx->id("RAM32X1S"), ctx->id("RAM16X1D"))) {
                const auto &ref = entry.second.site;
                const auto &tile = db.tiles[ref.tile];
                const auto &site = db.variants[tile.variant].sites[ref.site];
                WireId pin = site_pin_wire(tile, ref.tile, site, "CLK");
                pins.insert(pin.index);
                physical_clock_pins[entry.first][ctx->id("WCLK")] = {pin};
            }
            if (entry.second.element == "OFF1" || entry.second.element == "IFF1" || entry.second.element == "TFF1") {
                const auto &ref = entry.second.site;
                const auto &tile = db.tiles[ref.tile];
                const auto &site = db.variants[tile.variant].sites[ref.site];
                WireId pin = site_pin_wire(tile, ref.tile, site, entry.second.element == "IFF1" ? "ICLK1" : "OTCLK1");
                pins.insert(pin.index);
                physical_clock_pins[entry.first][ctx->id("C")] = {pin};
            }
            if (entry.second.element == "STARTUP" || entry.second.element == "CAPTURE_BLACKBOX") {
                const auto &ref = entry.second.site;
                const auto &tile = db.tiles[ref.tile];
                const auto &site = db.variants[tile.variant].sites[ref.site];
                WireId pin = site_pin_wire(tile, ref.tile, site, "CLK");
                pins.insert(pin.index);
                physical_clock_pins[entry.first][ctx->id("CLK")] = {pin};
            }
            if (entry.second.element == "MULT18X18SIO") {
                const auto &ref = entry.second.site;
                const auto &tile = db.tiles[ref.tile];
                const auto &site = db.variants[tile.variant].sites[ref.site];
                WireId pin = site_pin_wire(tile, ref.tile, site, "CLK");
                pins.insert(pin.index);
                physical_clock_pins[entry.first][ctx->id("CLK")] = {pin};
                physical_clock_pins[entry.first][ctx->id("C")] = {pin};
            }
            if (entry.second.element != "RAMB16") continue;
            const auto &ref = entry.second.site;
            const auto &tile = db.tiles[ref.tile];
            const auto &site = db.variants[tile.variant].sites[ref.site];
            for (const std::string &port : {std::string("CLKA"), std::string("CLKB")}) {
                WireId pin = site_pin_wire(tile, ref.tile, site, port);
                pins.insert(pin.index);
                physical_clock_pins[entry.first][ctx->id(port)] = {pin};
                if (port == "CLKA") physical_clock_pins[entry.first][ctx->id("CLK")] = {pin};
            }
        }
        index_ise_clock_paths(ctx, clock_sites, pads, pins, ctx->getWires().size, int(pip_meta.size()),
                              [&](PipId pip) {
            const auto &meta = pip_meta.at(pip.index);
            const auto &variant = db.variants[db.tiles[meta.tile].variant];
            return ise_clock_distribution_pip(db.wires[variant.wires[meta.from]], db.wires[variant.wires[meta.to]]);
        });
        for (const auto &memory : ram64_resources)
            for (int index : memory.sites) {
                const auto &ref = carry_sites[index].site;
                const auto &tile = db.tiles[ref.tile];
                const auto &def = db.variants[tile.variant].sites[ref.site];
                WireId pin = site_pin_wire(tile, ref.tile, def, "CLK");
                pins.insert(pin.index);
                physical_clock_pins[memory.composite.index][ctx->id("WCLK")].push_back(pin);
            }
        clock_model = std::make_unique<IseClockModel>(ctx, db, std::move(clock_sites),
                                                     std::move(clock_power_bels), std::move(physical_clock_pins));
    }





    void index_wide_topology()
    {
        wide_sources.resize(carry_sites.size(), {{{{-1, -1}}, {{-1, -1}}}});
        std::array<std::unordered_map<int32_t, int>, 2> sources, destinations;
        for (size_t i = 0; i < carry_sites.size(); ++i) {
            const auto &ref = carry_sites[i].site;
            const auto &tile = db.tiles[ref.tile];
            const auto &def = db.variants[tile.variant].sites[ref.site];
            for (int j = 0; j < 2; ++j) {
                sources[j][site_pin_wire(tile, ref.tile, def, j == 0 ? "F5" : "FX").index] = int(i);
                destinations[j][site_pin_wire(tile, ref.tile, def, j == 0 ? "FXINB" : "FXINA").index] = int(i);
            }
        }
        size_t count = 0;
        for (size_t pi = 0; pi < pip_meta.size(); ++pi) {
            PipId pip{int32_t(pi)};
            for (int kind = 0; kind < 2; ++kind) {
                auto from = sources[kind].find(ctx->getPipSrcWire(pip).index);
                if (from == sources[kind].end()) continue;
                for (int input = 0; input < 2; ++input) {
                    auto to = destinations[input].find(ctx->getPipDstWire(pip).index);
                    if (to == destinations[input].end()) continue;
                    int &source = wide_sources[to->second][kind][input];
                    if (source != -1 && source != from->second)
                        log_error("spartan3e-ise: ambiguous dedicated wide cascade link\n");
                    source = from->second;
                    ++count;
                }
            }
        }
        log_info("Spartan-3E ISE: indexed %zu dedicated wide cascade links\n", count);
    }

    void init_ram64_bels()
    {
        for (size_t high = 0; high < carry_sites.size(); ++high) {
            int low = wide_sources[high][0][0];
            if (low < 0 || low == int(high) || wide_sources[high][0][1] != int(high)) continue;
            const auto &hr = carry_sites[high].site;
            const auto &lr = carry_sites[low].site;
            const auto &tile = db.tiles[hr.tile];
            if (hr.tile != lr.tile || db.variants[tile.variant].sites[hr.site].type != "SLICEM" ||
                db.variants[tile.variant].sites[lr.site].type != "SLICEM") continue;
            int z = 0;
            for (const auto &entry : bel_site_info) {
                Loc loc = ctx->getBelLocation(BelId{entry.first});
                if (loc.x == tile.col && loc.y == tile.row) z = std::max(z, loc.z + 1);
            }
            BelId bel = ctx->addBel(h.xy_id(tile.col, tile.row,
                IdStringList::concat(ctx->id(tile.sites[hr.site]), ctx->id("RAM64"))),
                ctx->id("RAM64X1S"), Loc(tile.col, tile.row, z), false, false);
            bel_site_info[bel.index] = BelSiteInfo{hr, "F6MUX"};
            IseRam64Resources resources{bel, {low, int(high)}, {}};
            for (int half = 0; half < 2; ++half) {
                const auto &ref = carry_sites[resources.sites[half]].site;
                std::string prefix = half == 0 ? "L" : "H";
                auto input = [&](const std::string &name, const std::string &element, const std::string &pin) {
                    ctx->addBelInput(bel, ctx->id(name), site_elem_wire(ref.tile, ref.site, element, pin));
                };
                for (const std::string bank : {"F", "G"}) {
                    for (int i = 0; i < 4; ++i) {
                        input(prefix + bank + "A" + std::to_string(i), bank, "A" + std::to_string(i + 1));
                        input(prefix + bank + "W" + std::to_string(i), bank, "W" + bank + std::to_string(i + 1));
                    }
                    input(prefix + bank + "D", bank, "DI");
                }
                input(prefix + "A4", "F5MUX", "S0");
                input(prefix + "WA4", "WSGEN", "WE0");
                input(prefix + "WA5", "WSGEN", "WE1");
                input(prefix + "WE", "WSGEN", "WE");
                input(prefix + "CLK", "WSGEN", "CK");
                ctx->addBelOutput(bel, ctx->id("$RAM64_F5_" + prefix), site_elem_wire(ref.tile, ref.site, "F5MUX", "OUT"));
                ctx->addBelInput(bel, ctx->id("$RAM64_F6_" + prefix),
                    site_elem_wire(hr.tile, hr.site, "F6MUX", half == 0 ? "0" : "1"));
                for (const auto &entry : bel_site_info)
                    if (entry.first != bel.index && entry.second.site.tile == ref.tile && entry.second.site.site == ref.site)
                        resources.excluded.push_back(BelId{entry.first});
            }
            ctx->addBelInput(bel, ctx->id("A5"), site_elem_wire(hr.tile, hr.site, "F6MUX", "S0"));
            ctx->addBelOutput(bel, ctx->id("O"), site_elem_wire(hr.tile, hr.site, "F6MUX", "OUT"));
            ram64_by_bel[bel.index] = ram64_resources.size();
            ram64_resources.push_back(std::move(resources));
        }
    }



    void index_mult_topology()
    {
        std::unordered_map<int32_t, std::pair<int, int>> inputs;
        for (size_t i = 0; i < mult_sites.size(); ++i) {
            const auto &ref = mult_sites[i].site;
            const auto &tile = db.tiles[ref.tile];
            const auto &def = db.variants[tile.variant].sites[ref.site];
            for (int bit = 0; bit < 18; ++bit)
                inputs[site_pin_wire(tile, ref.tile, def, "BCIN" + std::to_string(bit)).index] = {int(i), bit};
        }
        for (size_t i = 0; i < mult_sites.size(); ++i) {
            const auto &ref = mult_sites[i].site;
            const auto &tile = db.tiles[ref.tile];
            const auto &def = db.variants[tile.variant].sites[ref.site];
            int next = -1;
            bool complete = true;
            for (int bit = 0; bit < 18; ++bit) {
                WireId output = site_pin_wire(tile, ref.tile, def, "BCOUT" + std::to_string(bit));
                int target = -1;
                auto match = [&](WireId wire) {
                    auto found = inputs.find(wire.index);
                    if (found == inputs.end() || found->second.second != bit) return;
                    if (target >= 0 && target != found->second.first) log_error("spartan3e-ise: ambiguous MULT cascade bit\n");
                    target = found->second.first;
                };
                match(output);
                for (PipId pip : ctx->getPipsDownhill(output)) match(ctx->getPipDstWire(pip));
                if (target < 0 || (next >= 0 && next != target)) complete = false;
                if (target >= 0) next = target;
            }
            if (complete) mult_sites[i].next = next;
        }
    }

    bool place_mult_chain(const IseMultChain &chain, BelId root,
                          std::vector<std::pair<CellInfo *, BelId>> &placement) const
    {
        auto found = mult_site_by_bel.find(root.index);
        if (found == mult_site_by_bel.end()) return false;
        int site = found->second;
        placement.clear();
        std::set<int32_t> used;
        for (CellInfo *cell : chain.cells) {
            if (site < 0) return false;
            const auto &target = mult_sites[site];
            if (!used.insert(target.bel.index).second) return false;
            auto fixed = cell->attrs.find(ctx->id("BEL"));
            if (fixed != cell->attrs.end() && fixed->second.as_string() != ctx->getBelName(target.bel).str(ctx)) return false;
            auto loc = cell->attrs.find(ctx->id("LOC"));
            if (loc != cell->attrs.end() && loc->second.as_string() != db.tiles[target.site.tile].sites[target.site.site]) return false;
            CellInfo *occupied = ctx->getBoundBelCell(target.bel);
            if (occupied != nullptr && occupied != cell && occupied->cluster != chain.root->name) return false;
            placement.emplace_back(cell, target.bel);
            site = target.next;
        }
        return true;
    }

    void index_srl_topology()
    {
        srl_next_site.assign(carry_sites.size(), -1);
        std::unordered_map<int32_t, int> inputs;
        for (size_t i = 0; i < carry_sites.size(); ++i) {
            const auto &ref = carry_sites[i].site;
            const auto &tile = db.tiles[ref.tile];
            const auto &def = db.variants[tile.variant].sites[ref.site];
            if (def.type == "SLICEM") inputs[site_pin_wire(tile, ref.tile, def, "SHIFTIN").index] = int(i);
        }
        for (size_t i = 0; i < carry_sites.size(); ++i) {
            const auto &ref = carry_sites[i].site;
            const auto &tile = db.tiles[ref.tile];
            const auto &def = db.variants[tile.variant].sites[ref.site];
            if (def.type != "SLICEM") continue;
            WireId output = site_pin_wire(tile, ref.tile, def, "SHIFTOUT");
            auto assign = [&](WireId target) {
                auto found = inputs.find(target.index);
                if (found == inputs.end()) return;
                int &next = srl_next_site[i];
                if (next >= 0 && next != found->second) log_error("spartan3e-ise: ambiguous dedicated SRL site link\n");
                next = found->second;
            };
            assign(output);
            for (PipId pip : ctx->getPipsDownhill(output)) assign(ctx->getPipDstWire(pip));
        }
    }

    bool place_srl_chain(const IseSrlChain &chain, BelId root_bel,
                         std::vector<std::pair<CellInfo *, BelId>> &placement) const
    {
        auto root = carry_bel_slot.find(root_bel.index);
        if (root == carry_bel_slot.end() || root->second.second != 1 ||
            carry_sites[root->second.first].lut[1] != root_bel) return false;
        placement.clear();
        int site = root->second.first;
        std::set<int32_t> used;
        for (size_t i = 0; i < chain.stages.size(); ++i) {
            if (site < 0) return false;
            const auto &ref = carry_sites[site].site;
            if (db.variants[db.tiles[ref.tile].variant].sites[ref.site].type != "SLICEM") return false;
            int half = i % 2 == 0 ? 1 : 0;
            for (auto target : {std::make_pair(chain.stages[i].cell, carry_sites[site].lut[half]),
                                std::make_pair(chain.stages[i].ff, carry_sites[site].ff[half])}) {
                if (target.first == nullptr) continue;
                if (!used.insert(target.second.index).second) return false;
                auto fixed = target.first->attrs.find(ctx->id("BEL"));
                if (fixed != target.first->attrs.end() && fixed->second.as_string() != ctx->getBelName(target.second).str(ctx)) return false;
                auto loc = target.first->attrs.find(ctx->id("LOC"));
                if (loc != target.first->attrs.end() && loc->second.as_string() != db.tiles[ref.tile].sites[ref.site]) return false;
                CellInfo *occupied = ctx->getBoundBelCell(target.second);
                if (occupied != nullptr && occupied != target.first && occupied->cluster != chain.root->name) return false;
                placement.push_back(target);
            }
            if (half == 0 && i + 1 < chain.stages.size()) site = srl_next_site[site];
        }
        return true;
    }

    bool place_wide_tree(const IseWideTree &tree, BelId root_bel,
                         std::vector<std::pair<CellInfo *, BelId>> &placement) const
    {
        auto root = wide_bel_slot.find(root_bel.index);
        if (root == wide_bel_slot.end() || (tree.root->type == id_MUXF5 ? 0 : 1) != root->second.second)
            return false;
        using Assignment = std::vector<int32_t>;
        std::function<bool(int, int, int, Assignment &)> assign = [&](int node, int site, int half, Assignment &state) {
            if (site < 0) return false;
            const auto &entry = tree.nodes[node];
            bool leaf = is_lut_cell(entry.cell->type);
            int kind = entry.cell->type == id_MUXF5 ? 0 : 1;
            BelId bel = leaf ? carry_sites[site].lut[half] : wide_bels[site][kind];
            if (state[node] >= 0) return state[node] == bel.index;
            for (size_t i = 0; i < state.size(); ++i)
                if (int(i) != node && state[i] == bel.index) return false;
            auto fixed = entry.cell->attrs.find(ctx->id("BEL"));
            if (fixed != entry.cell->attrs.end() && fixed->second.as_string() != ctx->getBelName(bel).str(ctx))
                return false;
            auto loc = entry.cell->attrs.find(ctx->id("LOC"));
            const auto &ref = carry_sites[site].site;
            if (loc != entry.cell->attrs.end() && loc->second.as_string() != db.tiles[ref.tile].sites[ref.site])
                return false;
            state[node] = bel.index;
            if (leaf) return true;
            for (int input = 0; input < 2; ++input) {
                int source_kind = entry.cell->type == id_MUXF6 ? 0 : 1;
                int source_site = kind == 0 ? site : wide_sources[site][source_kind][input];
                if (!assign(entry.children[input], source_site, 1 - input, state)) return false;
            }
            return true;
        };
        Assignment state(tree.nodes.size(), -1);
        if (!assign(0, root->second.first, 0, state)) return false;
        std::function<bool(Assignment &)> complete = [&](Assignment &current) {
            int pending = -1;
            for (size_t i = 0; i < tree.nodes.size(); ++i) {
                if (current[i] >= 0) continue;
                for (int child : tree.nodes[i].children)
                    if (child >= 0 && current[child] >= 0) { pending = int(i); break; }
                if (pending >= 0) break;
            }
            if (pending < 0)
                return std::all_of(current.begin(), current.end(), [](int32_t bel) { return bel >= 0; });
            // Another root must agree with an already placed shared child.
            // Candidate relations come from indexed directed chipdb links.
            for (size_t site = 0; site < carry_sites.size(); ++site) {
                Assignment candidate = current;
                if (!assign(pending, int(site), 0, candidate)) continue;
                if (complete(candidate)) { current = std::move(candidate); return true; }
            }
            return false;
        };
        if (!complete(state)) return false;
        placement.clear();
        for (size_t i = 0; i < tree.nodes.size(); ++i)
            placement.emplace_back(tree.nodes[i].cell, BelId{state[i]});
        return true;
    }

    std::optional<bool> getClusterPlacement(ClusterId cluster, BelId root_bel,
                                            std::vector<std::pair<CellInfo *, BelId>> &placement) const override
    {
        auto clock = clock_model->get_cluster_placement(cluster, root_bel, placement);
        if (clock.has_value()) return clock;
        auto io = iob_off_clusters.find(cluster);
        auto input_io = iob_if_clusters.find(cluster);
        auto tristate_io = iob_tff_clusters.find(cluster);
        if (io != iob_off_clusters.end() || input_io != iob_if_clusters.end() || tristate_io != iob_tff_clusters.end()) {
            placement = {{ctx->cells.at(cluster).get(), root_bel}};
        }
        if (io != iob_off_clusters.end()) {
            auto off = iob_off_bels.find(root_bel.index);
            if (off == iob_off_bels.end()) return false;
            placement.emplace_back(io->second, off->second);
        }
        if (input_io != iob_if_clusters.end()) {
            auto ff = iob_if_bels.find(root_bel.index);
            if (ff == iob_if_bels.end()) return false;
            placement.emplace_back(input_io->second, ff->second);
        }
        if (tristate_io != iob_tff_clusters.end()) {
            auto ff = iob_tff_bels.find(root_bel.index);
            if (ff == iob_tff_bels.end()) return false;
            placement.emplace_back(tristate_io->second, ff->second);
        }
        if (io != iob_off_clusters.end() || input_io != iob_if_clusters.end() || tristate_io != iob_tff_clusters.end()) return true;
        auto mult = mult_cluster.find(cluster);
        if (mult != mult_cluster.end()) return place_mult_chain(mult_chains[mult->second], root_bel, placement);
        auto srl = srl_cluster.find(cluster);
        if (srl != srl_cluster.end()) return place_srl_chain(srl_chains[srl->second], root_bel, placement);
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

    bool select_site_pip(const IseSiteRef &site, const std::string &element, const std::string &cfg)
    {
        auto it = site_pip.find(SitePipKey{site.tile, site.site, element, cfg});
        if (it == site_pip.end())
            return false;
        selected_site_pips.insert(it->second.index);
        return true;
    }

    // Dedicated cascade/propagate pins do not consume X/Y. Everything else,
    // including a local output FF, requires the half's output mux.
    bool needs_slice_output(CellInfo *cell) const
    {
        NetInfo *net = cell->getPort(id_O);
        if (net == nullptr) return false;
        for (const auto &user : net->users) {
            if (user.cell->type.in(id_MUXF5, id_MUXF6, id_MUXF7, id_MUXF8) &&
                user.port.in(id_I0, id_I1)) continue;
            if ((user.cell->type == id_MUXCY && user.port == id_S) ||
                (user.cell->type == id_XORCY && user.port == id_LI)) continue;
            return true;
        }
        return false;
    }

    bool local_ff_data(CellInfo *ff, const IseSiteRef &site, int half) const
    {
        NetInfo *d = ff->getPort(id_D);
        if (d == nullptr || d->driver.cell == nullptr) return false;
        auto src = bel_site_info.find(d->driver.cell->bel.index);
        if (src == bel_site_info.end() || src->second.site.tile != site.tile || src->second.site.site != site.site)
            return false;
        const auto &element = src->second.element;
        return half == 0 ? element == "F" || element == "XORF" || element == "F5MUX"
                         : element == "G" || element == "XORG" || element == "F6MUX";
    }

    bool dedicated_carry_input(CellInfo *cell) const
    {
        NetInfo *input = cell->getPort(id_CI);
        if (input == nullptr || input->driver.cell == nullptr || input->driver.cell->type != id_MUXCY ||
            input->driver.port != id_O || input->driver.cell->cluster != cell->cluster) return false;
        auto chain = carry_cluster.find(cell->cluster);
        return chain != carry_cluster.end() && carry_chains[chain->second].root != cell;
    }

    bool wide_resources_valid(int index) const
    {
        const auto &site = carry_sites[index];
        for (int half = 0; half < 2; ++half) {
            CellInfo *wide = ctx->getBoundBelCell(wide_bels[index][half]);
            if (wide == nullptr || !wide->type.in(id_MUXF5, id_MUXF6, id_MUXF7, id_MUXF8)) continue;
            CellInfo *lut = ctx->getBoundBelCell(site.lut[half]);
            CellInfo *sum = ctx->getBoundBelCell(site.xorcy[half]);
            int outputs = int(needs_slice_output(wide));
            outputs += int(lut != nullptr && is_lut_cell(lut->type) && needs_slice_output(lut));
            outputs += int(sum != nullptr && needs_slice_output(sum));
            if (outputs > 1) return false;
            NetInfo *select = wide->getPort(id_S);
            CellInfo *ff = ctx->getBoundBelCell(site.ff[half]);
            if (ff != nullptr && !local_ff_data(ff, site.site, half) && ff->getPort(id_D) != select)
                return false; // External FF data and the mux select share BX/BY.
            if (half == 0) {
                for (BelId carry : {site.mux[0], site.xorcy[0]}) {
                    CellInfo *cell = ctx->getBoundBelCell(carry);
                    if (cell == nullptr) continue;
                    NetInfo *ci = cell->getPort(id_CI);
                    bool dedicated = dedicated_carry_input(cell);
                    if (!dedicated && ci != select) return false; // CYINIT=BX.
                }
            }
        }
        return true;
    }

    void assign_site_info()
    {
        selected_site_pips.clear();
        size_t selected = 0, missing = 0;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->bel == BelId())
                continue;
            auto bs_it = bel_site_info.find(ci->bel.index);
            if (bs_it == bel_site_info.end())
                continue;
            const IseSiteRef &site = bs_it->second.site;
            const std::string &element = bs_it->second.element;
            auto select = [&](const std::string &elem, const std::string &cfg) {
                if (select_site_pip(site, elem, cfg))
                    selected++;
                else
                    missing++;
            };
            if (is_lut_cell(ci->type)) {
                if (!needs_slice_output(ci)) continue;
                auto slot = carry_bel_slot.at(ci->bel.index);
                if (ctx->getBoundBelCell(carry_sites[slot.first].xorcy[slot.second]) != nullptr)
                    continue; // The half's fabric output is the arithmetic sum.
                if (element == "F") {
                    select("FXMUX", "F");
                    select("XUSED", "0");
                } else if (element == "G") {
                    select("GYMUX", "G");
                    select("YUSED", "0");
                }
            } else if (auto shape = ise_bram_shape(ci->type.str(ctx))) {
                for (const std::string &port : {std::string("A"), std::string("B")}) {
                    if (port == "B" && !shape->dual) continue;
                    for (const std::string &control : {std::string("CLK"), std::string("EN"), std::string("WE"), std::string("SSR")}) {
                        std::string logical = control + (shape->dual ? port : "");
                        bool inverted = ci->attrs.count(ctx->id("S3E_BRAM_CONTROL_INV_" + logical));
                        select(control + port + "INV", control + port + (inverted ? "_B" : ""));
                    }
                }
            } else if (auto shape = ise_mult_shape(ctx, ci)) {
                for (const std::string &physical : {std::string("CLK"), std::string("CEA"), std::string("CEB"),
                    std::string("CEP"), std::string("RSTA"), std::string("RSTB"), std::string("RSTP")}) {
                    std::string logical = physical;
                    if (shape->wrapper && physical == "CLK") logical = "C";
                    if (shape->wrapper && physical == "CEP") logical = "CE";
                    if (shape->wrapper && physical == "RSTP") logical = "R";
                    if (ci->getPort(ctx->id(logical)) == nullptr) continue;
                    select(physical + "INV", physical + (ise_mult_inverted(ctx, ci, logical) ? "_B" : ""));
                }
            } else if (ci->type == ctx->id("RAM64X1S")) {
                const auto &resources = ram64_resources.at(ram64_by_bel.at(ci->bel.index));
                for (int half = 0; half < 2; ++half) {
                    const auto &ref = carry_sites[resources.sites[half]].site;
                    auto choose = [&](const std::string &elem, const std::string &cfg) {
                        if (select_site_pip(ref, elem, cfg)) ++selected;
                        else ++missing;
                    };
                    for (const std::string bank : {"F", "G"})
                        for (int i = 1; i <= 4; ++i) choose("W" + bank + std::to_string(i) + "USED", "0");
                    choose("CLKINV", "CLK");
                    choose("SRINV", "SR");
                    choose("BXINV", "BX");
                    choose("BYINV", "BY");
                    choose("DIF_MUX", "ALTDIF");
                    choose("DIG_MUX", half == 0 ? "BY" : "ALTDIG");
                    choose("SLICEWE0USED", "0");
                    choose("SLICEWE1USED", "0");
                    choose("F5USED", "0");
                    if (half == 0) choose("DIGUSED", "0");
                    else {
                        choose("BYOUTUSED", "0");
                        choose("BYINVOUTUSED", "0");
                        choose("GYMUX", "FX");
                        choose("YUSED", "0");
                    }
                }
            } else if (ci->type == ctx->id("RAM32X1S")) {
                for (const std::string bank : {"F", "G"})
                    for (int i = 1; i <= 4; ++i) select("W" + bank + std::to_string(i) + "USED", "0");
                select("CLKINV", "CLK");
                select("SRINV", "SR");
                select("BXINV", "BX");
                select("BYINV", "BY");
                select("DIG_MUX", "BY");
                select("DIF_MUX", "ALTDIF");
                select("SLICEWE0USED", "0");
                select("FXMUX", "F5");
                select("XUSED", "0");
            } else if (ci->type == ctx->id("RAM16X1D")) {
                for (const std::string bank : {"F", "G"})
                    for (int i = 1; i <= 4; ++i) select("W" + bank + std::to_string(i) + "USED", "0");
                select("CLKINV", "CLK");
                select("SRINV", "SR");
                select("BYINV", "BY");
                select("DIG_MUX", "BY");
                select("DIF_MUX", "ALTDIF");
                select("FXMUX", "F");
                select("GYMUX", "G");
                select("XUSED", "0");
                select("YUSED", "0");
            } else if (is_ise_srl(ctx, ci->type)) {
                select("CLKINV", "CLK");
                select("SRINV", "SR");
                bool local_shift = false;
                bool cross_shift = false;
                NetInfo *data = ci->getPort(id_D);
                if (element == "F" && data != nullptr && data->driver.cell != nullptr &&
                    data->driver.port == ctx->id("Q15") && ise_srl_cascade(ctx, data->driver.cell)) {
                    CellInfo *source = data->driver.cell;
                    auto bound = bel_site_info.find(source->bel.index);
                    local_shift = bound != bel_site_info.end() && bound->second.element == "G" &&
                        bound->second.site.tile == site.tile && bound->second.site.site == site.site &&
                        ctx->getBoundBelCell(source->bel) == source && source->getPort(ctx->id("Q15")) == data;
                }
                if (element == "G" && data != nullptr && data->driver.cell != nullptr &&
                    data->driver.port == ctx->id("Q15") && ise_srl_cascade(ctx, data->driver.cell)) {
                    auto source = carry_bel_slot.find(data->driver.cell->bel.index);
                    auto target = carry_bel_slot.find(ci->bel.index);
                    cross_shift = source != carry_bel_slot.end() && target != carry_bel_slot.end() &&
                        source->second.second == 0 && srl_next_site[source->second.first] == target->second.first;
                    if (cross_shift) {
                        const auto &ref = carry_sites[source->second.first].site;
                        if (select_site_pip(ref, "SHIFTOUTUSED", "0")) ++selected;
                        else ++missing;
                    }
                }
                select(element == "F" ? "DIF_MUX" : "DIG_MUX", (local_shift || cross_shift) ? "SHIFTIN" : element == "F" ? "BX" : "BY");
                if (!local_shift && !cross_shift)
                    select(element == "F" ? "BXINV" : "BYINV", element == "F" ? "BX" : "BY");
                select(element == "F" ? "FXMUX" : "GYMUX", element);
                select(element == "F" ? "XUSED" : "YUSED", "0");
                if (ise_srl_cascade(ctx, ci)) {
                    select(element == "F" ? "XBMUX" : "YBMUX", "0");
                    if (element == "G") select("YBUSED", "0");
                }
            } else if (ci->type == ctx->id("RAM16X1S")) {
                for (int i = 1; i <= 4; ++i)
                    select((element == "F" ? "WF" : "WG") + std::to_string(i) + "USED", "0");
                select("CLKINV", "CLK");
                select("SRINV", "SR");
                select(element == "F" ? "DIF_MUX" : "DIG_MUX", element == "F" ? "BX" : "BY");
                select(element == "F" ? "BXINV" : "BYINV", element == "F" ? "BX" : "BY");
                select(element == "F" ? "FXMUX" : "GYMUX", element);
                select(element == "F" ? "XUSED" : "YUSED", "0");
            } else if (is_xilinx_ff_cell(ci->type)) {
                if (element == "IFF1") {
                    select("IFFDELMUX", "1");
                    select("IFFDMUX", "1");
                    select("IDDRIN_MUX", "2");
                    select("ICLK1INV", "ICLK1");
                    select("ICEINV", "ICE");
                    select("SRINV", "SR");
                    select("ISR_USED", "0");
                    continue;
                }
                if (element == "OFF1" || element == "TFF1") {
                    bool tristate = element == "TFF1";
                    select(tristate ? "T1INV" : "O1INV", tristate ? "T1" : "O1");
                    if (!tristate) select("O1_DDRMUX", "1");
                    select("OTCLK1INV", "OTCLK1");
                    if (ci->getPort(id_CE) != nullptr) {
                        select(tristate ? "TCEINV" : "OCEINV", tristate ? "TCE" : "OCE");
                        if (!tristate) select("PCICE_MUX", "OCE");
                    }
                    for (IdString port : {ctx->id("R"), id_S, ctx->id("CLR"), ctx->id("PRE")})
                        if (ci->getPort(port) != nullptr) {
                            select("SRINV", "SR");
                            select(tristate ? "TSR_USED" : "OSR_USED", "0");
                        }
                    continue;
                }
                bool local_d = local_ff_data(ci, site, element == "FFX" ? 0 : 1);
                if (element == "FFX")
                    select("DXMUX", local_d ? "1" : "0");
                else if (element == "FFY")
                    select("DYMUX", local_d ? "1" : "0");
                if (!local_d)
                    select(element == "FFX" ? "BXINV" : "BYINV", element == "FFX" ? "BX" : "BY");
                const auto shape = *ise_ff_shape(ci->type.str(ctx));
                select("CLKINV", ise_ff_clock_inverted(ctx, ci) ? "CLK_B" : "CLK");
                const char *enable = shape.latch ? "GE" : "CE";
                if (ci->getPort(ctx->id(enable)) != nullptr)
                    select("CEINV", ise_ff_inverted(ctx, ci, enable) ? "CE_B" : "CE");
                bool has_sr = false;
                for (IdString pin : {ctx->id("R"), id_S, ctx->id("CLR"), ctx->id("PRE")})
                    has_sr |= ci->getPort(pin) != nullptr;
                if (has_sr) {
                    select("SRINV", shape.sr != nullptr && ise_ff_inverted(ctx, ci, shape.sr) ? "SR_B" : "SR");
                    if (db.variants[db.tiles[site.tile].variant].sites[site.site].type == "SLICEM")
                        select("SRFFMUX", "0");
                }
                if (shape.rev != nullptr && ci->getPort(ctx->id(shape.rev)) != nullptr) {
                    select("BYINV", ise_ff_inverted(ctx, ci, shape.rev) ? "BY_B" : "BY");
                    select("REVUSED", "0");
                }
            } else if (ci->type == id_MUXCY) {
                bool is_g = element == "CYMUXG";
                select(is_g ? "CYSELG" : "CYSELF", is_g ? "G" : "F");
                IseCarryDi di = carry_di.at(ci->name);
                std::string di_cfg = di == IseCarryDi::Product ? "PROD"
                                     : di == IseCarryDi::LutInput ? (is_g ? "G1" : "F1")
                                     : di == IseCarryDi::One    ? "1"
                                                                : "0";
                select(is_g ? "CY0G" : "CY0F", di_cfg);
                ci->attrs[ctx->id("S3E_CY0")] = Property(di_cfg);
                if (is_g)
                    select("COUTUSED", "0");
                // Carry results may also have ordinary fabric consumers.
                auto &tile = db.tiles[site.tile];
                if (db.variants[tile.variant].sites[site.site].type == "SLICEM")
                    select(is_g ? "YBMUX" : "XBMUX", "1");
                if (is_g || db.variants[tile.variant].sites[site.site].type == "SLICEL")
                    select(is_g ? "YBUSED" : "XBUSED", "0");
            } else if (ci->type == id_XORCY) {
                bool is_g = element == "XORG";
                if (needs_slice_output(ci)) {
                    select(is_g ? "GYMUX" : "FXMUX", is_g ? "GXOR" : "FXOR");
                    select(is_g ? "YUSED" : "XUSED", "0");
                }
            } else if (ci->type.in(id_MUXF5, id_MUXF6, id_MUXF7, id_MUXF8)) {
                if (ci->type == id_MUXF5) {
                    select("BXINV", "BX");
                    if (needs_slice_output(ci)) {
                        select("FXMUX", "F5");
                        select("XUSED", "0");
                    }
                    select("F5USED", "0");
                } else {
                    select("BYINV", "BY");
                    if (needs_slice_output(ci)) {
                        select("GYMUX", "FX");
                        select("YUSED", "0");
                    }
                    select("FXUSED", "0");
                }
            } else if (ci->type.in(id_BUFG, id_BUFGMUX, ctx->id("BUFGMUX_1"))) {
                select("I0_USED", "0");
                if (ci->type != id_BUFG) select("I1_USED", "0");
                select("SINV", ci->type == id_BUFG ? "S_B" : "S");
            } else if (ci->type == ctx->id("STARTUP_SPARTAN3E")) {
                select("CLKINV", "CLK");
                select("GSRINV", "GSR");
                select("GTSINV", "GTS");
            } else if (ci->type == ctx->id("CAPTURE_SPARTAN3")) {
                select("CLKINV", "CLK");
                select("CAPINV", "CAP");
            } else if (is_xilinx_io_cell(ci->type)) {
                json11::Json::object cfg;
                // An occupied I/O site starts disabled; only the selected mode is enabled.
                const auto &type = db.variants[db.tiles[site.tile].variant].sites[site.site].type;
                for (const auto &el : db.site_defs.at(type).elements)
                    if (!el.cfg.empty())
                        cfg[el.name] = "#OFF";
                cfg["IOATTRBOX"] = ci->attrs.at(ctx->id("IOSTANDARD")).as_string();
                cfg["PULL"] = ci->attrs.at(ctx->id("PULL")).as_string();
                CellInfo *iff = nullptr;
                auto iff_bel = iob_if_bels.find(ci->bel.index);
                if (iff_bel != iob_if_bels.end()) iff = ctx->getBoundBelCell(iff_bel->second);
                if (iff != nullptr) {
                    cfg["IFF1"] = "#FF";
                    auto init = iff->params.find(ctx->id("INIT"));
                    cfg["IFF1_INIT_ATTR"] = init != iff->params.end() && init->second.as_int64() == 1 ? "INIT1" : "INIT0";
                    cfg["IFF1_SR_ATTR"] = iff->type == id_FDPE ? "SRHIGH" : "SRLOW";
                    cfg["IFFATTRBOX"] = "ASYNC";
                    cfg["IFFDELMUX"] = "1";
                    cfg["IFFDMUX"] = "1";
                    cfg["IDDRIN_MUX"] = "2";
                    cfg["ICLK1INV"] = "ICLK1";
                    cfg["ICEINV"] = "ICE";
                    cfg["SRINV"] = "SR";
                    cfg["ISR_USED"] = "0";
                    ci->attrs[ctx->id("S3E_IO_IFF1")] = Property(iff->name.str(ctx));
                } else if (ci->type.in(id_IBUF, id_IOBUF)) {
                    select("IDELMUX", "1");
                    select("IMUX", "1");
                    cfg["IDELMUX"] = "1";
                    cfg["IMUX"] = "1";
                }
                if (ci->type != id_IBUF) {
                    CellInfo *off = nullptr;
                    auto off_bel = iob_off_bels.find(ci->bel.index);
                    if (off_bel != iob_off_bels.end()) off = ctx->getBoundBelCell(off_bel->second);
                    select("O1INV", "O1");
                    select("OMUX", off == nullptr ? "O1" : "OFF1");
                    cfg["O1INV"] = "O1";
                    cfg["OMUX"] = off == nullptr ? "O1" : "OFF1";
                    if (off != nullptr) {
                        cfg["OFF1"] = "#FF";
                        auto init = off->params.find(ctx->id("INIT"));
                        cfg["OFF1_INIT_ATTR"] = init != off->params.end() && init->second.as_int64() == 1 ? "INIT1" : "INIT0";
                        cfg["O1_DDRMUX"] = "1";
                        cfg["OTCLK1INV"] = "OTCLK1";
                        if (off->getPort(id_CE) != nullptr) {
                            cfg["OCEINV"] = "OCE";
                            cfg["PCICE_MUX"] = "OCE";
                        }
                        for (IdString port : {ctx->id("R"), id_S, ctx->id("CLR"), ctx->id("PRE")})
                            if (off->getPort(port) != nullptr) {
                                cfg["SRINV"] = "SR";
                                cfg["OSR_USED"] = "0";
                                cfg["OFF1_SR_ATTR"] = off->type.in(id_FDPE, id_FDP, id_FDSE) ? "SRHIGH" : "SRLOW";
                                cfg["OFFATTRBOX"] = off->type.in(id_FDRE, id_FDSE) ? "SYNC" : "ASYNC";
                            }
                        ci->attrs[ctx->id("S3E_IO_OFF1")] = Property(off->name.str(ctx));
                    }
                    cfg["DRIVEATTRBOX"] = ci->attrs.at(ctx->id("DRIVE")).as_string();
                    cfg["SLEW"] = ci->attrs.at(ctx->id("SLEW")).as_string();
                    if (ci->type.in(id_OBUFT, id_IOBUF)) {
                        CellInfo *tff = nullptr;
                        auto tff_bel = iob_tff_bels.find(ci->bel.index);
                        if (tff_bel != iob_tff_bels.end()) tff = ctx->getBoundBelCell(tff_bel->second);
                        select("T1INV", "T1");
                        select("TMUX", tff == nullptr ? "T1" : "TFF1");
                        cfg["T1INV"] = "T1";
                        cfg["TMUX"] = tff == nullptr ? "T1" : "TFF1";
                        if (tff != nullptr) {
                            cfg["TFF1"] = "#FF";
                            auto init = tff->params.find(ctx->id("INIT"));
                            cfg["TFF1_INIT_ATTR"] = init != tff->params.end() && init->second.as_int64() == 1 ? "INIT1" : "INIT0";
                            cfg["OTCLK1INV"] = "OTCLK1";
                            if (tff->getPort(id_CE) != nullptr) cfg["TCEINV"] = "TCE";
                            for (IdString port : {ctx->id("R"), id_S, ctx->id("CLR"), ctx->id("PRE")})
                                if (tff->getPort(port) != nullptr) {
                                    cfg["SRINV"] = "SR";
                                    cfg["TSR_USED"] = "0";
                                    cfg["TFF1_SR_ATTR"] = tff->type.in(id_FDPE, id_FDP, id_FDSE) ? "SRHIGH" : "SRLOW";
                                    cfg["TFFATTRBOX"] = tff->type.in(id_FDRE, id_FDSE) ? "SYNC" : "ASYNC";
                                }
                            ci->attrs[ctx->id("S3E_IO_TFF1")] = Property(tff->name.str(ctx));
                        }
                    }
                }
                ci->attrs[ctx->id("S3E_IO_CFG")] = Property(json11::Json(cfg).dump());
                ci->attrs[ctx->id("S3E_IO_DIAGNOSTIC_VERSION")] = Property("1");
                ci->attrs[ctx->id("S3E_IO_MODE")] = Property(ci->type == id_IBUF ? "INPUT" : ci->type == id_IOBUF ? "BIDIRECTIONAL" : ci->type == id_OBUFT ? "TRISTATE" : "OUTPUT");
                ci->attrs[ctx->id("S3E_IO_SITE")] = Property(db.tiles[site.tile].sites[site.site]);
                ci->attrs[ctx->id("S3E_IO_PIN")] = Property(bel_package_pin.at(ci->bel.index));
                ci->attrs[ctx->id("S3E_IO_PACKAGE")] = Property(db.package);
            }
            if ((ci->type == id_MUXCY && element == "CYMUXF") || (ci->type == id_XORCY && element == "XORF")) {
                bool dedicated = dedicated_carry_input(ci);
                select("CYINIT", dedicated ? "CIN" : "BX");
                if (!dedicated)
                    select("BXINV", "BX");
            }
        }
        log_info("Spartan-3E ISE: site pip selection: %zu selected, %zu missing\n", selected, missing);
        if (missing != 0)
            log_error("spartan3e-ise: required site configuration path is missing\n");
    }

    bool checkPipAvail(PipId pip) const override
    {
        auto it = site_pip_by_pip.find(pip.index);
        if (it == site_pip_by_pip.end())
            return true;
        return selected_site_pips.count(pip.index) > 0;
    }

    bool checkPipAvailForNet(PipId pip, const NetInfo *net) const override
    {
        if (!checkPipAvail(pip)) return false;
        if (site_route_meta.count(pip.index)) return true;
        return clock_model->pip_available(pip, net);
    }

    static bool is_io_site_type(const std::string &type)
    {
        // `IBUF` is an ISE site type for input-only pads, not the Xilinx IBUF cell type.
        return type == "IOB" || type == "DIFFM" || type == "DIFFS" || type == "DIFFMI" || type == "DIFFSI" ||
               type == "IBUF";
    }

    static bool is_lut_cell(IdString t) { return t.in(id_LUT1, id_LUT2, id_LUT3, id_LUT4); }

    bool is_xilinx_ff_cell(IdString t) const
    {
        return ise_ff_shape(t.str(ctx)).has_value();
    }

    static bool is_xilinx_io_cell(IdString t) { return t.in(id_IBUF, id_OBUF, id_OBUFT, id_IOBUF); }

    void pack_io_constraints()
    {
        // Preserve top-port constraints before the generic frontend buffers disappear.
        for (auto &cell : ctx->cells) {
            CellInfo *port = cell.second.get();
            bool input = port->type == ctx->id("$nextpnr_ibuf");
            bool output = port->type == ctx->id("$nextpnr_obuf");
            bool bidirectional = port->type == ctx->id("$nextpnr_iobuf");
            if (!input && !output && !bidirectional)
                continue;
            NetInfo *pad = port->getPort(input || bidirectional ? id_O : id_I);
            CellInfo *buffer = nullptr;
            if ((input || bidirectional) && pad != nullptr && pad->users.entries() == 1)
                buffer = (*pad->users.begin()).cell;
            if (output && pad != nullptr)
                buffer = pad->driver.cell;
            if (buffer == nullptr || (bidirectional ? buffer->type != id_IOBUF : input ? buffer->type != id_IBUF : !buffer->type.in(id_OBUF, id_OBUFT)))
                log_error("spartan3e-ise: port %s requires a top-level IBUF/OBUF/OBUFT/IOBUF\n", ctx->nameOf(port));
            for (const char *key : {"LOC", "PACKAGE_PIN", "BEL", "IOSTANDARD", "DRIVE", "SLEW", "PULL",
                                    "PULLUP", "PULLDOWN", "KEEPER", "DIFF_TERM", "IBUF_LOW_PWR", "IOB",
                                    "IN_TERM", "OUT_TERM", "IBUF_DELAY_VALUE", "IFD_DELAY_VALUE"}) {
                IdString attr = ctx->id(key);
                auto it = port->attrs.find(attr);
                if (it == port->attrs.end())
                    continue;
                auto existing = buffer->attrs.find(attr);
                if (existing != buffer->attrs.end() && existing->second != it->second)
                    log_error("spartan3e-ise: conflicting %s on port %s and buffer\n", key, ctx->nameOf(port));
                buffer->attrs[attr] = it->second;
            }
            buffer->attrs[ctx->id("S3E_IO_PORT")] = Property(port->name.str(ctx));
        }
        std::unordered_map<int32_t, CellInfo *> constrained;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (!is_xilinx_io_cell(ci->type))
                continue;
            if (!ci->attrs.count(ctx->id("S3E_IO_PORT")))
                log_error("spartan3e-ise: only top-level I/O buffers are supported\n");
            auto value = [&](const char *key, const std::string &fallback) {
                auto it = ci->attrs.find(ctx->id(key));
                if (it == ci->attrs.end())
                    return fallback;
                return it->second.is_string ? it->second.as_string() : std::to_string(it->second.as_int64());
            };
            if (value("IOSTANDARD", "LVCMOS33") != "LVCMOS33")
                log_error("spartan3e-ise: unsupported IOSTANDARD on %s (supported: LVCMOS33)\n", ctx->nameOf(ci));
            const std::set<std::string> drives = {"2", "4", "6", "8", "12", "16"};
            if (!drives.count(value("DRIVE", "12")))
                log_error("spartan3e-ise: unsupported LVCMOS33 DRIVE on %s (supported: 2/4/6/8/12/16)\n", ctx->nameOf(ci));
            if (value("SLEW", "SLOW") != "SLOW" && value("SLEW", "SLOW") != "FAST")
                log_error("spartan3e-ise: unsupported SLEW on %s (supported: SLOW/FAST)\n", ctx->nameOf(ci));
            for (const char *key : {"PULL", "DIFF_TERM", "IBUF_LOW_PWR", "IOB",
                                    "IN_TERM", "OUT_TERM", "IBUF_DELAY_VALUE", "IFD_DELAY_VALUE"})
                if (ci->attrs.count(ctx->id(key)) || ci->params.count(ctx->id(key)))
                    log_error("spartan3e-ise: unsupported I/O attribute %s on %s\n", key, ctx->nameOf(ci));
            std::string pull = "#OFF";
            for (const char *key : {"PULLUP", "PULLDOWN", "KEEPER"}) {
                if (ci->params.count(ctx->id(key)))
                    log_error("spartan3e-ise: use port/cell attributes for %s, not parameters\n", key);
                if (!ci->attrs.count(ctx->id(key))) continue;
                std::string flag = value(key, "");
                if (flag != "TRUE" && flag != "FALSE" && flag != "1" && flag != "0")
                    log_error("spartan3e-ise: %s must be TRUE/FALSE or 1/0 on %s\n", key, ctx->nameOf(ci));
                if (flag == "TRUE" || flag == "1") {
                    if (pull != "#OFF") log_error("spartan3e-ise: conflicting I/O pull attributes on %s\n", ctx->nameOf(ci));
                    pull = key;
                }
            }
            // PULL is a backend diagnostic selection, not an ISE user attribute.
            ci->attrs[ctx->id("PULL")] = Property(pull);
            // Attribute and parameter forms must not silently disagree.
            for (const char *key : {"IOSTANDARD", "DRIVE", "SLEW"}) {
                IdString attr = ctx->id(key);
                if (ci->params.count(attr))
                    log_error("spartan3e-ise: use port/cell attributes for %s, not parameters\n", key);
                ci->attrs[attr] = Property(value(key, std::string(key) == "IOSTANDARD" ? "LVCMOS33" :
                                                     std::string(key) == "DRIVE" ? "12" : "SLOW"));
            }
            std::string loc = value("LOC", ""), pin = value("PACKAGE_PIN", "");
            if (!loc.empty() && !pin.empty() && loc != pin)
                log_error("spartan3e-ise: conflicting LOC/PACKAGE_PIN on %s\n", ctx->nameOf(ci));
            if (pin.empty())
                pin = loc;
            BelId bel;
            if (!pin.empty()) {
                auto it = package_bels.find(pin);
                if (it == package_bels.end())
                    log_error("spartan3e-ise: pin %s is not a bonded user I/O in package %s\n", pin.c_str(), db.package.c_str());
                bel = it->second;
            }
            std::string bel_name = value("BEL", "");
            if (!bel_name.empty()) {
                BelId named;
                // Generic getBelByName asserts for unknown names; resolve against
                // the small validated package catalogue to provide a user error.
                for (const auto &candidate : package_bels)
                    if (ctx->getBelName(candidate.second).str(ctx) == bel_name)
                        named = candidate.second;
                if (named == BelId() || !bel_package_pin.count(named.index) || (bel != BelId() && bel != named))
                    log_error("spartan3e-ise: invalid or conflicting I/O BEL on %s\n", ctx->nameOf(ci));
                bel = named;
            }
            if (bel == BelId())
                continue;
            if (!isValidBelForCellType(ci->type, bel))
                log_error("spartan3e-ise: output %s cannot use input-only pin %s\n", ctx->nameOf(ci), bel_package_pin.at(bel.index).c_str());
            if (!constrained.emplace(bel.index, ci).second)
                log_error("spartan3e-ise: duplicate I/O pin binding for %s\n", bel_package_pin.at(bel.index).c_str());
            ci->attrs[ctx->id("BEL")] = Property(ctx->getBelName(bel).str(ctx));
        }
    }

    json11::Json diagnostic_io_route(CellInfo *ci, IdString port)
    {
        NetInfo *net = ci->getPort(port);
        if (net == nullptr || net->wires.empty())
            log_error("spartan3e-ise: disconnected I/O fabric port on %s\n", ctx->nameOf(ci));
        json11::Json::array route;
        for (const auto &wire : net->wires) {
            PipId pip = wire.second.pip;
            if (pip == PipId())
                continue;
            auto site = site_route_meta.find(pip.index);
            if (site != site_route_meta.end()) {
                const auto &meta = site->second;
                const auto &conn = meta.connection;
                json11::Json::object entry{
                    {"site", db.tiles[meta.site.tile].sites[meta.site.site]},
                    {"from", json11::Json::array{conn.src_el, conn.src_pin}},
                    {"to", json11::Json::array{conn.dst_el, conn.dst_pin}},
                    {"pip", ctx->getPipName(pip).str(ctx)}};
                if (meta.selection) {
                    entry["element"] = meta.selection->element;
                    entry["value"] = meta.selection->cfg;
                }
                route.push_back(entry);
            } else {
                const auto &meta = pip_meta.at(pip.index);
                const auto &tile = db.tiles[meta.tile];
                const auto &v = db.variants[tile.variant];
                route.push_back(json11::Json::object{{"tile", tile.name}, {"from", db.wires[v.wires[meta.from]]},
                                                   {"to", db.wires[v.wires[meta.to]]},
                                                   {"pip", ctx->getPipName(pip).str(ctx)}});
            }
        }
        return route;
    }

    void inherit_iob_register_attrs(CellInfo *ff)
    {
        // Yosys retains attributes on an inferred register's Q net, whereas
        // explicit primitive instances carry them on the cell itself.
        NetInfo *q = ff->getPort(id_Q);
        if (q == nullptr)
            return;
        for (const char *key : {"IOB", "S3E_IOB_KIND"}) {
            IdString id = ctx->id(key);
            auto net_attr = q->attrs.find(id);
            if (net_attr == q->attrs.end())
                continue;
            auto cell_attr = ff->attrs.find(id);
            if (cell_attr != ff->attrs.end() && cell_attr->second.as_string() != net_attr->second.as_string())
                log_error("spartan3e-ise: conflicting cell/Q-net %s on register %s\n", key, ctx->nameOf(ff));
            ff->attrs[id] = net_attr->second;
        }
    }

    void pack_iob_off()
    {
        for (auto &entry : ctx->cells) {
            CellInfo *ff = entry.second.get();
            if (!is_xilinx_ff_cell(ff->type))
                continue;
            inherit_iob_register_attrs(ff);
            auto request = ff->attrs.find(ctx->id("IOB"));
            if (request == ff->attrs.end() || request->second.as_string() != "TRUE") continue;
            if (!ise_legacy_iob_ff(ctx, ff))
                log_error("spartan3e-ise: IOB=TRUE does not support extended FF polarity or dual controls\n");
            NetInfo *q = ff->getPort(id_Q);
            CellInfo *buffer = q != nullptr && q->users.entries() == 1 ? (*q->users.begin()).cell : nullptr;
            bool tristate = buffer != nullptr && buffer->type.in(id_OBUFT, id_IOBUF) &&
                            (*q->users.begin()).port == ctx->id("T");
            bool input = !tristate && (buffer == nullptr || !buffer->type.in(id_OBUF, id_OBUFT, id_IOBUF) || (*q->users.begin()).port != id_I);
            auto kind = ff->attrs.find(ctx->id("S3E_IOB_KIND"));
            if (kind != ff->attrs.end()) {
                std::string role = kind->second.as_string();
                if (role != "IFF1" && role != "OFF1" && role != "TFF1") log_error("spartan3e-ise: unsupported S3E_IOB_KIND\n");
                if (role == "OFF1" && (input || tristate)) log_error("spartan3e-ise: requested OFF1 lacks a dedicated output-buffer consumer\n");
                if (role == "TFF1" && !tristate) log_error("spartan3e-ise: requested TFF1 lacks a dedicated tristate-buffer consumer\n");
                tristate = role == "TFF1";
                input = role == "IFF1";
            }
            if (input) {
                NetInfo *d = ff->getPort(id_D);
                buffer = d != nullptr && d->driver.cell != nullptr &&
                                 d->driver.cell->type.in(id_IBUF, id_IOBUF) && d->driver.port == id_O &&
                                 d->users.entries() == 1
                             ? d->driver.cell : nullptr;
                if (buffer == nullptr) log_error("spartan3e-ise: IOB=TRUE requires a dedicated input/output buffer\n");
                if (!ff->type.in(id_FDCE, id_FDPE)) log_error("spartan3e-ise: IFF1 currently requires FDCE/FDPE\n");
            }
            auto &clusters = input ? iob_if_clusters : tristate ? iob_tff_clusters : iob_off_clusters;
            if (ff->cluster != ClusterId() || clusters.count(buffer->name) ||
                (buffer->cluster != ClusterId() &&
                 (buffer->cluster != buffer->name ||
                  (!iob_off_clusters.count(buffer->name) && !iob_if_clusters.count(buffer->name) && !iob_tff_clusters.count(buffer->name)))))
                log_error("spartan3e-ise: IOB register conflicts with another cluster\n");
            for (const auto *other_clusters : {&iob_if_clusters, &iob_off_clusters, &iob_tff_clusters}) {
                auto other = other_clusters->find(buffer->name);
                if (other == other_clusters->end()) continue;
                auto sr = [&](CellInfo *cell) -> NetInfo * {
                    for (IdString port : {ctx->id("R"), id_S, ctx->id("CLR"), ctx->id("PRE")})
                        if (cell->getPort(port) != nullptr) return cell->getPort(port);
                    return nullptr;
                };
                if (sr(ff) != nullptr && sr(other->second) != nullptr && sr(ff) != sr(other->second))
                    log_error("spartan3e-ise: IOB registers share one physical SR input\n");
                if (!input && other_clusters != &iob_if_clusters && ff->getPort(ctx->id("C")) != other->second->getPort(ctx->id("C")))
                    log_error("spartan3e-ise: OFF1/TFF1 share one physical output clock\n");
            }
            ff->cluster = buffer->name;
            buffer->cluster = buffer->name;
            buffer->constr_children.push_back(ff);
            ff->constr_x = ff->constr_y = 0;
            ff->constr_abs_z = false;
            const auto &register_bels = input ? iob_if_bels : tristate ? iob_tff_bels : iob_off_bels;
            clusters[buffer->name] = ff;
            auto fixed = buffer->attrs.find(ctx->id("BEL"));
            if (fixed != buffer->attrs.end()) {
                BelId pad = ctx->getBelByNameStr(fixed->second.as_string());
                auto target = register_bels.find(pad.index);
                if (target == register_bels.end()) log_error("spartan3e-ise: constrained pad lacks requested IOB register\n");
                auto ff_fixed = ff->attrs.find(ctx->id("BEL"));
                if (ff_fixed != ff->attrs.end() && ctx->getBelByNameStr(ff_fixed->second.as_string()) != target->second)
                    log_error("spartan3e-ise: conflicting IOB output-register BEL\n");
                // HeAP excludes a fixed root from cluster placement. Fix its
                // child too, so no unplaced register reaches its cost model.
                ff->attrs[ctx->id("BEL")] = Property(ctx->getBelName(target->second).str(ctx));
            } else if (ff->attrs.count(ctx->id("BEL"))) {
                log_error("spartan3e-ise: IOB output-register BEL requires a constrained output pad\n");
            }
        }
    }

    void postRoute() override
    {
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (!is_xilinx_io_cell(ci->type)) continue;
            ci->attrs[ctx->id("S3E_IO_ROUTE")] = Property(diagnostic_io_route(ci, ci->type == id_IBUF ? id_O : id_I).dump());
            if (ci->type == id_IOBUF)
                ci->attrs[ctx->id("S3E_IO_INPUT_ROUTE")] = Property(diagnostic_io_route(ci, id_O).dump());
            if (ci->type.in(id_OBUFT, id_IOBUF))
                ci->attrs[ctx->id("S3E_IO_T_ROUTE")] = Property(diagnostic_io_route(ci, ctx->id("T")).dump());
        }
        if (!dump_path.empty()) write_ise_dump();
    }

    std::set<int32_t> route_subtree(const NetInfo *net, WireId root) const
    {
        std::set<int32_t> result{root.index};
        bool changed = true;
        while (changed) {
            changed = false;
            for (const auto &wire : net->wires) {
                if (wire.second.pip == PipId() || result.count(wire.first.index)) continue;
                if (result.count(ctx->getPipSrcWire(wire.second.pip).index)) {
                    result.insert(wire.first.index);
                    changed = true;
                }
            }
        }
        return result;
    }

    json11::Json::object dump_route_fragment(const NetInfo *net, WireId root,
        const std::set<int32_t> &wires, const std::function<json11::Json(WireId)> &endpoint) const
    {
        using J = json11::Json;
        J::array pips, pins, routes;
        std::set<std::pair<std::string, std::string>> seen;
        for (const auto &wire : net->wires) {
            if (!wires.count(wire.first.index) || wire.first == root || wire.second.pip == PipId()) continue;
            PipId pip = wire.second.pip;
            auto internal = site_route_meta.find(pip.index);
            if (internal == site_route_meta.end()) {
                const auto &meta = pip_meta.at(pip.index);
                const auto &tile = db.tiles[meta.tile];
                const auto &variant = db.variants[tile.variant];
                std::string op;
                for (const auto &row : variant.pips)
                    if (row.from == meta.from && row.to == meta.to && row.rt == -1) { op = db.pip_ops[row.op]; break; }
                if (op.empty()) log_error("spartan3e-ise: unknown component pip operator\n");
                pips.push_back(J::object{{"tile", tile.name}, {"from", db.wires[variant.wires[meta.from]]},
                    {"to", db.wires[variant.wires[meta.to]]}, {"op", op}});
                continue;
            }
            const auto &meta = internal->second;
            const auto &tile = db.tiles[meta.site.tile];
            const auto &def = db.variants[tile.variant].sites[meta.site.site];
            const auto &conn = meta.connection;
            J selection;
            if (meta.selection) selection = J::object{{"element", meta.selection->element}, {"value", meta.selection->cfg}};
            routes.push_back(J::object{{"site", tile.sites[meta.site.site]},
                {"from", J::array{conn.src_el, conn.src_pin}}, {"to", J::array{conn.dst_el, conn.dst_pin}}, {"selection", selection}});
            for (const auto &element : {conn.src_el, conn.dst_el}) {
                if (!def.pins.count(element) || !seen.emplace(tile.sites[meta.site.site], element).second) continue;
                std::string direction;
                for (const auto &pin : db.site_defs.at(def.type).pins)
                    if (pin.first == element) direction = pin.second;
                if (direction.empty()) log_error("spartan3e-ise: unknown component boundary pin\n");
                pins.push_back(J::object{{"site", tile.sites[meta.site.site]}, {"pin", element}, {"direction", direction}});
            }
        }
        return {{"root", endpoint(root)}, {"pins", pins}, {"pips", pips}, {"site_routes", routes}};
    }

    void write_ise_dump()
    {
        using J = json11::Json;
        // Resolve endpoint identity from typed wire metadata, never BEL names.
        std::map<int, SiteElemKey> endpoint_by_wire;
        for (const auto &entry : site_wire)
            if (!endpoint_by_wire.emplace(entry.second.index, entry.first).second)
                log_error("spartan3e-ise: ambiguous typed site endpoint\n");
        auto endpoint = [&](WireId wire) -> J {
            auto it = endpoint_by_wire.find(wire.index);
            if (it == endpoint_by_wire.end())
                log_error("spartan3e-ise: cannot resolve typed endpoint wire\n");
            const auto &key = it->second;
            return J::object{{"site", db.tiles[key.tile].sites[key.site]},
                             {"element", key.element}, {"pin", key.pin}};
        };
        std::map<std::pair<int, int>, J::array> cells;
        for (const auto &item : ctx->cells) {
            const CellInfo *cell = item.second.get();
            auto placed = bel_site_info.find(cell->bel.index);
            if (placed == bel_site_info.end())
                log_error("spartan3e-ise: cannot export unplaced/unknown cell %s\n", ctx->nameOf(cell));
            const auto &site = placed->second.site;
            J::object params, attrs, ports;
            for (const auto &param : cell->params)
                params[param.first.str(ctx)] = param.second.to_string();
            for (const auto &attr : cell->attrs)
                if (attr.first.str(ctx).find("S3E_IO_ROUTE") != 0)
                    attrs[attr.first.str(ctx)] = attr.second.to_string();
            for (const auto &port : cell->ports)
                if (port.second.net != nullptr && port.first.str(ctx).find("$RAM64_") != 0)
                    ports[port.first.str(ctx)] = port.second.net->name.str(ctx);
            J::object exported{
                {"name", cell->name.str(ctx)}, {"type", cell->type.str(ctx)}, {"element", placed->second.element},
                {"params", params}, {"attrs", attrs}, {"ports", ports}};
            auto memory = ram64_by_bel.find(cell->bel.index);
            if (memory != ram64_by_bel.end()) {
                J::array footprint, components;
                const auto &resources = ram64_resources[memory->second];
                for (int half = 0; half < 2; ++half) {
                    const auto &ref = carry_sites[resources.sites[half]].site;
                    footprint.push_back(J::object{{"site", db.tiles[ref.tile].sites[ref.site]}, {"address_base", half * 32}});
                    const std::string prefix = half == 0 ? "L" : "H";
                    const NetInfo *net = cell->getPort(ctx->id("$RAM64_F5_" + prefix));
                    WireId root = ctx->getNetinfoSourceWire(net);
                    auto component = dump_route_fragment(net, root, route_subtree(net, root), endpoint);
                    component["kind"] = half == 0 ? "ram64_read_low" : "ram64_read_high";
                    component["sinks"] = J::array{endpoint(ctx->getBelPinWire(cell->bel, ctx->id("$RAM64_F6_" + prefix)))};
                    components.push_back(component);
                }
                exported["memory_sites"] = footprint;
                exported["component_routes"] = components;
            }
            cells[{site.tile, site.site}].push_back(exported);
        }
        J::array sites, nets;
        bool has_ram64 = false;
        for (const auto &memory : ram64_resources)
            has_ram64 |= ctx->getBoundBelCell(memory.composite) != nullptr;
        for (const auto &site : active_sites) {
            const auto &tile = db.tiles[site.first];
            J::array selections;
            for (int32_t index : selected_site_pips) {
                const auto &key = site_pip_by_pip.at(index);
                // Placement enables legal routing alternatives. Only a used
                // alternative is configuration, except disconnected constant DI.
                if (ctx->getBoundPipNet(PipId(index)) == nullptr && key.element != "CY0F" &&
                    key.element != "CY0G")
                    continue;
                if (key.tile == site.first && key.site == site.second)
                    selections.push_back(J::object{{"element", key.element}, {"value", key.cfg}});
            }
            sites.push_back(J::object{{"site", tile.sites[site.second]}, {"tile", tile.name},
                {"type", db.variants[tile.variant].sites[site.second].type},
                {"cells", cells[site]}, {"site_pips", selections}});
        }
        for (const auto &item : ctx->nets) {
            const NetInfo *net = item.second.get();
            if (net->driver.cell == nullptr || net->wires.empty())
                continue;
            if (net->driver.port.str(ctx).find("$RAM64_") == 0) continue;
            // Package-side nets have no fabric representation.
            if (net->driver.cell->type == id_OBUF && net->driver.port == id_O)
                continue;
            J::array pips, pins, sinks, site_routes, branches;
            std::set<int32_t> branch_wires;
            for (const auto &memory : ram64_resources) {
                CellInfo *owner = ctx->getBoundBelCell(memory.composite);
                if (owner == nullptr) continue;
                const auto &low = carry_sites[memory.sites[0]].site;
                const auto &high = carry_sites[memory.sites[1]].site;
                struct Branch { IseSiteRef site; const char *port; const char *kind; const char *transform;
                    const char *source_element; const char *source_pin; const char *root_element; };
                const std::array<Branch, 3> definitions{{
                    {high, "A5", "ram64_byout", "identity", "BYINV", "BY", "BYOUTUSED"},
                    {high, "A5", "ram64_byinvout", "inverted", "BYINV", "BY", "BYINVOUTUSED"},
                    {low, "D", "ram64_dig", "identity", "DIG_MUX", "BY", "DIGUSED"}}};
                for (const auto &definition : definitions) {
                    if (owner->getPort(ctx->id(definition.port)) != net) continue;
                    WireId root = site_wire.at(SiteElemKey{definition.site.tile, definition.site.site, definition.root_element, "OUT"});
                    if (!net->wires.count(root)) log_error("spartan3e-ise: RAM64 physical branch root is unrouted\n");
                    auto wires = route_subtree(net, root);
                    auto fragment = dump_route_fragment(net, root, wires, endpoint);
                    J::array branch_sinks;
                    for (const PortRef &user : net->users)
                        for (WireId wire : ctx->getNetinfoSinkWires(net, user))
                            if (wires.count(wire.index)) branch_sinks.push_back(J::object{
                                {"cell", user.cell->name.str(ctx)}, {"port", user.port.str(ctx)}, {"endpoint", endpoint(wire)}});
                    fragment["owner"] = owner->name.str(ctx);
                    fragment["port"] = definition.port;
                    fragment["kind"] = definition.kind;
                    fragment["transform"] = definition.transform;
                    fragment["source"] = endpoint(site_wire.at(SiteElemKey{definition.site.tile, definition.site.site,
                        definition.source_element, definition.source_pin}));
                    fragment["sinks"] = branch_sinks;
                    branches.push_back(fragment);
                    branch_wires.insert(wires.begin(), wires.end());
                }
            }
            for (const PortRef &user : net->users) {
                auto wires = ctx->getNetinfoSinkWires(net, user);
                if (wires.size() == 0)
                    log_error("spartan3e-ise: no physical sinks for %s.%s\n",
                              ctx->nameOf(user.cell), user.port.c_str(ctx));
                for (WireId wire : wires)
                    if (!branch_wires.count(wire.index))
                    sinks.push_back(J::object{{"cell", user.cell->name.str(ctx)},
                                             {"port", user.port.str(ctx)}, {"endpoint", endpoint(wire)}});
            }
            std::set<std::pair<std::string, std::string>> seen_pins;
            for (const auto &wire : net->wires) {
                if (branch_wires.count(wire.first.index)) continue;
                PipId pip = wire.second.pip;
                if (pip == PipId())
                    continue;
                auto internal = site_route_meta.find(pip.index);
                if (internal == site_route_meta.end()) {
                    const auto &meta = pip_meta.at(pip.index);
                    const auto &tile = db.tiles[meta.tile];
                    const auto &v = db.variants[tile.variant];
                    std::string op;
                    for (const auto &row : v.pips)
                        if (row.from == meta.from && row.to == meta.to && row.rt == -1) {
                            op = db.pip_ops[row.op];
                            break;
                        }
                    if (op.empty())
                        log_error("spartan3e-ise: cannot resolve exported pip operator\n");
                    pips.push_back(J::object{{"tile", tile.name}, {"from", db.wires[v.wires[meta.from]]},
                        {"to", db.wires[v.wires[meta.to]]}, {"op", op}});
                    continue;
                }
                const auto &meta = internal->second;
                const auto &tile = db.tiles[meta.site.tile];
                const auto &def = db.variants[tile.variant].sites[meta.site.site];
                const auto &conn = meta.connection;
                J selection;
                if (meta.selection)
                    selection = J::object{{"element", meta.selection->element}, {"value", meta.selection->cfg}};
                site_routes.push_back(J::object{{"site", tile.sites[meta.site.site]},
                    {"from", J::array{conn.src_el, conn.src_pin}},
                    {"to", J::array{conn.dst_el, conn.dst_pin}}, {"selection", selection}});
                for (const auto &element : {conn.src_el, conn.dst_el}) {
                    if (!def.pins.count(element) || !seen_pins.emplace(tile.sites[meta.site.site], element).second)
                        continue;
                    std::string direction;
                    for (const auto &pin : db.site_defs.at(def.type).pins)
                        if (pin.first == element)
                            direction = pin.second;
                    if (direction.empty())
                        log_error("spartan3e-ise: missing direction for site pin %s\n", element.c_str());
                    pins.push_back(J::object{{"site", tile.sites[meta.site.site]}, {"pin", element},
                        {"direction", direction}});
                }
            }
            const CellInfo *driver = net->driver.cell;
            const auto &placed = bel_site_info.at(driver->bel.index);
            WireId source = ctx->getNetinfoSourceWire(net);
            J root = endpoint(source);
            J::object exported_net{{"name", net->name.str(ctx)}, {"driver", driver->name.str(ctx)},
                {"driver_port", net->driver.port.str(ctx)}, {"source_site", db.tiles[placed.site.tile].sites[placed.site.site]},
                {"root", root}, {"pins", pins}, {"pips", pips},
                {"sinks", sinks}, {"site_routes", site_routes}};
            if (has_ram64) exported_net["branches"] = branches;
            nets.push_back(exported_net);
        }
        J dump = J::object{{"format", has_ram64 ? "spark-ise-route/3" : "spark-ise-route/2"}, {"part", db.part}, {"package", db.package},
            {"top", "top"}, {"sites", sites}, {"nets", nets}};
        std::ofstream file(dump_path);
        if (!file || !(file << dump.dump() << '\n'))
            log_error("spartan3e-ise: cannot write dump %s\n", dump_path.c_str());
    }

    void remove_transparent_buffers()
    {
        std::vector<IdString> dead_cells, dead_nets;
        for (auto &entry : ctx->cells) {
            CellInfo *cell = entry.second.get();
            if (!cell->type.in(ctx->id("$buf"), ctx->id("$_BUF_")))
                continue;
            for (auto &port : cell->ports) {
                if (port.second.type != PORT_OUT)
                    continue;
                std::string output = port.first.str(ctx);
                if (output.empty() || output[0] != 'Y')
                    log_error("spartan3e-ise: malformed transparent-buffer output\n");
                IdString input = ctx->id("A" + output.substr(1));
                if (!cell->ports.count(input))
                    log_error("spartan3e-ise: transparent buffer lacks matching input\n");
                NetInfo *y = port.second.net;
                NetInfo *a = cell->getPort(input);
                if (y == nullptr)
                    continue;
                if (a == nullptr) {
                    // The frontend leaves x/z inputs disconnected. Keep the
                    // output net undriven; never turn a don't-care into logic.
                    cell->disconnectPort(port.first);
                    continue;
                }
                if (a == y)
                    log_error("spartan3e-ise: transparent-buffer combinational loop\n");
                for (const auto &attr : y->attrs) {
                    auto existing = a->attrs.find(attr.first);
                    const std::string key = attr.first.str(ctx);
                    bool request = key == "IOB" || key == "S3E_IOB_KIND" || key == "LOC" ||
                            key == "PACKAGE_PIN" || key == "BEL" || key == "IOSTANDARD" ||
                            key == "DRIVE" || key == "SLEW" || key == "PULL" ||
                            key == "PULLUP" || key == "PULLDOWN" || key == "KEEPER";
                    if (request && existing != a->attrs.end() && existing->second != attr.second)
                        log_error("spartan3e-ise: conflicting %s across transparent buffer\n", key.c_str());
                    if (existing == a->attrs.end())
                        a->attrs[attr.first] = attr.second;
                }
                if (a->clkconstr && y->clkconstr)
                    log_error("spartan3e-ise: conflicting clock constraints across transparent buffer\n");
                if (!a->clkconstr)
                    a->clkconstr = std::move(y->clkconstr);
                if (a->region && y->region && a->region != y->region)
                    log_error("spartan3e-ise: conflicting regions across transparent buffer\n");
                if (!a->region)
                    a->region = y->region;
                if (a->constant_value != IdString() && y->constant_value != IdString() &&
                        a->constant_value != y->constant_value)
                    log_error("spartan3e-ise: conflicting constants across transparent buffer\n");
                if (a->constant_value == IdString())
                    a->constant_value = y->constant_value;
                std::vector<PortRef> users;
                for (const auto &user : y->users)
                    users.push_back(user);
                for (const auto &user : users) {
                    user.cell->disconnectPort(user.port);
                    user.cell->connectPort(user.port, a);
                }
                a->aliases.push_back(y->name);
                ctx->net_aliases[y->name] = a->name;
                for (IdString alias : y->aliases) {
                    a->aliases.push_back(alias);
                    ctx->net_aliases[alias] = a->name;
                }
                for (auto &top_port : ctx->ports)
                    if (top_port.second.net == y)
                        top_port.second.net = a;
                cell->disconnectPort(port.first);
                dead_nets.push_back(y->name);
            }
            for (auto &port : cell->ports)
                cell->disconnectPort(port.first);
            dead_cells.push_back(cell->name);
        }
        for (IdString name : dead_cells)
            ctx->cells.erase(name);
        for (IdString name : dead_nets)
            ctx->nets.erase(name);
    }

    void normalize_ibufg()
    {
        for (auto &entry : ctx->cells) {
            CellInfo *cell = entry.second.get();
            if (cell->type != ctx->id("IBUFG"))
                continue;
            NetInfo *output = cell->getPort(id_O);
            bool global_input = false;
            if (output != nullptr) {
                for (const auto &user : output->users) {
                    if ((user.cell->type == id_BUFG && user.port == id_I) ||
                            (user.cell->type.in(id_BUFGMUX, ctx->id("BUFGMUX_1")) &&
                             user.port.in(ctx->id("I0"), ctx->id("I1"))))
                        global_input = true;
                }
            }
            if (!global_input)
                log_error("spartan3e-ise: IBUFG currently requires a BUFG/BUFGMUX input consumer\n");
            // Same single-ended pad circuitry as IBUF. Preserve identity and
            // requests; the clock model still requires a dedicated pad path.
            cell->type = id_IBUF;
        }
    }

    void pack() override
    {
        // Yosys clean retains vector $buf cells with unused x/z input bits.
        // Eliminate aliases before I/O, constants, clock and carry packing.
        remove_transparent_buffers();
        normalize_ibufg();
        IdString id_IO = ctx->id("IO"), id_INV = ctx->id("INV");
        const pool<CellTypePort> top_ports{
                CellTypePort(id_IBUF, id_I),
                CellTypePort(id_OBUF, id_O),
                CellTypePort(id_OBUFT, id_O),
                CellTypePort(id_IOBUF, id_IO),
        };
        pack_io_constraints();
        h.remove_nextpnr_iobs(top_ports);
        fold_ise_bram_control_inverters(ctx);

        // ISE #LUT expressions expose only truth-table-dependent input pins.
        // Kept constant leaves must not export fabric nets to unused F/G pins.
        for (auto &entry : ctx->cells) {
            CellInfo *cell = entry.second.get();
            if (!is_lut_cell(cell->type)) continue;
            int width = cell->type.str(ctx).back() - '0';
            auto init = cell->params.find(id_INIT);
            if (init == cell->params.end())
                log_error("spartan3e-ise: LUT %s has no INIT\n", ctx->nameOf(cell));
            uint64_t truth = uint64_t(init->second.as_int64());
            for (int input = 0; input < width; ++input) {
                bool dependent = false;
                for (int address = 0; address < (1 << width); ++address)
                    dependent |= ((truth >> address) & 1) != ((truth >> (address ^ (1 << input))) & 1);
                IdString pin = ctx->idf("I%d", input);
                if (!dependent && cell->ports.count(pin)) cell->disconnectPort(pin);
            }
        }

        // synth_xilinx emits INV for single-input inverters.
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type == id_INV) {
                if (ci->ports.count(id_I))
                    ci->renamePort(id_I, ctx->id("I0"));
                ci->type = id_LUT4;
                ci->params[id_INIT] = Property(0x5555, 16);
            }
        }

        const dict<IdString, Property> vcc_params = {{id_INIT, Property(0xFFFF, 16)}};
        const dict<IdString, Property> gnd_params = {{id_INIT, Property(0x0000, 16)}};
        std::vector<CellInfo *> always_enabled_srls;
        for (auto &entry : ctx->cells) {
            CellInfo *cell = entry.second.get();
            if (!is_ise_srl(ctx, cell->type)) continue;
            validate_ise_srl(ctx, cell);
            if (cell->type.in(ctx->id("SRL16"), ctx->id("SRLC16"))) always_enabled_srls.push_back(cell);
        }
        if (!always_enabled_srls.empty()) {
            CellInfo *one = ctx->createCell(ctx->id("$ISE_SRL_ENABLE"), id_LUT4);
            one->params[id_INIT] = Property(0xffff, 16);
            one->addOutput(id_O);
            NetInfo *enable = ctx->createNet(ctx->id("$ISE_SRL_ENABLE_NET"));
            one->connectPort(id_O, enable);
            for (CellInfo *cell : always_enabled_srls) {
                cell->addInput(id_CE);
                cell->connectPort(id_CE, enable);
            }
        }
        h.replace_constants(CellTypePort(id_LUT4, id_O), CellTypePort(id_LUT4, id_O), vcc_params, gnd_params);

        pool<CellTypePort> lut_src_ports;
        for (IdString lut : {id_LUT1, id_LUT2, id_LUT3, id_LUT4})
            lut_src_ports.insert(CellTypePort(lut, id_O));

        pool<CellTypePort> ff_sink_ports;
        for (auto &entry : ctx->cells)
            if (ise_mult_type(ctx, entry.second->type)) remap_ise_mult(ctx, entry.second.get());
        for (auto &entry : ctx->cells)
            if (is_xilinx_ff_cell(entry.second->type)) {
                validate_ise_ff(ctx, entry.second.get());
                ff_sink_ports.insert(CellTypePort(entry.second->type, id_D));
            }

        wide_trees = pack_ise_wide(ctx);
        mult_chains = pack_ise_mult_chains(ctx);
        for (size_t i = 0; i < mult_chains.size(); ++i) {
            const auto &chain = mult_chains[i];
            mult_cluster[chain.root->name] = i;
            std::vector<std::pair<CellInfo *, BelId>> placement;
            bool legal = false;
            for (const auto &site : mult_sites)
                if (place_mult_chain(chain, site.bel, placement)) { legal = true; break; }
            if (!legal)
                log_error("spartan3e-ise: MULT cascade has no legal dedicated footprint (fixed adjacency or occupancy conflict)\n");
            if (chain.root->attrs.count(ctx->id("BEL"))) fix_cluster_placement(placement);
        }
        clock_model->pack();
        pack_iob_off();
        // A non-inverted literal-low SR does not consume slice SR. Preserve
        // FF identity/type/CE; IOB controls retain their separate contract.
        for (auto &entry : ctx->cells) {
            CellInfo *ff = entry.second.get();
            if (iob_off_clusters.count(ff->cluster) || iob_if_clusters.count(ff->cluster) || iob_tff_clusters.count(ff->cluster))
                continue;
            disconnect_ise_inactive_ff_sr(ctx, ff, [&](IdString type) { return is_lut_cell(type); });
        }
        srl_chains = pack_ise_srl_chains(ctx);
        for (size_t i = 0; i < srl_chains.size(); ++i) {
            const auto &chain = srl_chains[i];
            srl_cluster[chain.root->name] = i;
            std::vector<std::pair<CellInfo *, BelId>> placement;
            bool legal = false;
            for (const auto &site : carry_sites)
                if (place_srl_chain(chain, site.lut[1], placement)) { legal = true; break; }
            if (!legal) log_error("spartan3e-ise: SRL chain has no legal dedicated footprint (fixed adjacency or occupancy conflict)\n");
            if (chain.root->attrs.count(ctx->id("BEL"))) fix_cluster_placement(placement);
        }
        for (size_t i = 0; i < wide_trees.size(); ++i) {
            wide_cluster[wide_trees[i].root->name] = i;
            std::vector<std::pair<CellInfo *, BelId>> placement;
            bool legal = false;
            for (const auto &site : wide_bels)
                if (place_wide_tree(wide_trees[i], site[wide_trees[i].root->type == id_MUXF5 ? 0 : 1], placement)) {
                    legal = true;
                    break;
                }
            if (!legal)
                log_error("spartan3e-ise: wide DAG %s has no legal dedicated footprint (sharing or fixed BEL conflict)\n",
                          wide_trees[i].root->name.c_str(ctx));
            if (wide_trees[i].root->attrs.count(ctx->id("BEL")))
                fix_cluster_placement(placement);
        }
        int pairs = h.constrain_cell_pairs(lut_src_ports, ff_sink_ports, 1);
        log_info("Spartan-3E ISE: constrained %d LUT/DFF pairs\n", pairs);

        // synth_xilinx IO buffers use Xilinx port names; the physical IOB bel
        // exposes both fabric and pad pins, so remap the pad-side ports.
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->type == id_IBUF)
                ci->bel_pins[id_I] = {id_IO};
            else if (ci->type == id_OBUF || ci->type == id_OBUFT)
                ci->bel_pins[id_O] = {id_IO};
            else if (ci->type == ctx->id("RAM32X1S"))
                remap_ise_ram32(ctx, ci);
            else if (ci->type == ctx->id("RAM64X1S"))
                remap_ise_ram64(ctx, ci);
            else if (ci->type == ctx->id("RAM16X1D"))
                remap_ise_ram16_dual(ctx, ci);
            else if (ci->type == ctx->id("RAM16X1S"))
                for (int i = 0; i < 4; ++i)
                    ci->bel_pins[ctx->idf("A%d", i)] = {ctx->idf("A%d", i), ctx->idf("WA%d", i)};
            else if (auto shape = ise_ff_shape(ci->type.str(ctx))) {
                if (shape->latch) {
                    ci->bel_pins[ctx->id("G")] = {ctx->id("C")};
                    ci->bel_pins[ctx->id("GE")] = {ctx->id("CE")};
                }
                if (shape->rev != nullptr)
                    ci->bel_pins[ctx->id(shape->rev)] = {ctx->id(shape->sr_high ? "REV_R" : "REV_S")};
            }
            else if (auto shape = ise_bram_shape(ci->type.str(ctx))) remap_ise_bram_ports(ctx, ci, *shape);
        }

        carry_chains = pack_ise_carries(ctx);
        for (size_t i = 0; i < carry_chains.size(); i++) {
            carry_cluster[carry_chains[i].root->name] = i;
            for (auto &bit : carry_chains[i].bits)
                if (bit.mux != nullptr)
                    carry_di[bit.mux->name] = bit.di;
            auto fixed = carry_chains[i].root->attrs.find(ctx->id("BEL"));
            if (fixed != carry_chains[i].root->attrs.end()) {
                std::vector<std::pair<CellInfo *, BelId>> placement;
                auto legal = getClusterPlacement(carry_chains[i].root->name,
                    ctx->getBelByNameStr(fixed->second.as_string()), placement);
                if (!legal.value_or(false)) log_error("spartan3e-ise: invalid fixed carry footprint\n");
                fix_cluster_placement(placement);
            }
        }
        constrain_ff_locations();
    }

    void constrain_ff_locations()
    {
        // LOC selects a physical slice, not one arbitrary FFX/FFY BEL. Give
        // HeAP the actual search region as well as the legality predicate.
        // Otherwise a valid isolated FF can exhaust random placement attempts.
        for (auto &entry : ctx->cells) {
            CellInfo *cell = entry.second.get();
            if (!ise_ff_shape(cell->type.str(ctx))) continue;
            auto loc = cell->attrs.find(ctx->id("LOC"));
            if (loc == cell->attrs.end()) continue;
            std::string site = loc->second.as_string();
            if (site.compare(0, 6, "SLICE_") != 0) continue;
            auto region = cell->region == nullptr ? std::make_unique<Region>() :
                std::make_unique<Region>(*cell->region);
            region->name = ctx->id("$ISE_FF_LOC_" + cell->name.str(ctx));
            region->constr_bels = true;
            region->bels.clear();
            for (const auto &bel : bel_site_info) {
                if (bel.second.element != "FFX" && bel.second.element != "FFY") continue;
                const auto &ref = bel.second.site;
                BelId candidate{bel.first};
                if (db.tiles[ref.tile].sites[ref.site] == site && cell->testRegion(candidate))
                    region->bels.insert(candidate);
            }
            if (region->bels.empty())
                log_error("spartan3e-ise: FF LOC has no available slice BEL: %s (%s)\n",
                          ctx->nameOf(cell), site.c_str());
            if (ctx->region.count(region->name))
                log_error("spartan3e-ise: duplicate FF LOC region for %s\n", ctx->nameOf(cell));
            cell->region = region.get();
            ctx->region[cell->region->name] = std::move(region);
        }
    }

    void fix_cluster_placement(const std::vector<std::pair<CellInfo *, BelId>> &placement)
    {
        // HeAP does not place children of a fixed root. Pin the entire actual
        // footprint, preserving every pre-existing child BEL constraint.
        for (const auto &entry : placement) {
            std::string name = ctx->getBelName(entry.second).str(ctx);
            auto fixed = entry.first->attrs.find(ctx->id("BEL"));
            if (fixed != entry.first->attrs.end() && fixed->second.as_string() != name)
                log_error("spartan3e-ise: conflicting fixed cluster BEL for %s\n", ctx->nameOf(entry.first));
            entry.first->attrs[ctx->id("BEL")] = Property(name);
        }
    }

    void postPlace() override
    {
        clock_model->add_ties();
        active_sites.clear();
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->bel == BelId())
                continue;
            auto it = bel_site_info.find(ci->bel.index);
            if (it != bel_site_info.end())
                active_sites.insert({it->second.site.tile, it->second.site.site});
            auto memory = ram64_by_bel.find(ci->bel.index);
            if (memory != ram64_by_bel.end())
                for (int index : ram64_resources[memory->second].sites) {
                    const auto &ref = carry_sites[index].site;
                    active_sites.insert({ref.tile, ref.site});
                }
        }
        build_site_graph();
        assign_site_info();
        site_elem_index.clear();
    }



    void preRoute() override
    {
        clock_model->pre_route([&](PipId pip) { return site_route_meta.count(pip.index) != 0; },
                               [&](PipId pip) { return checkPipAvail(pip); });
    }

    IdString getBelBucketForCellType(IdString cell_type) const override
    {
        if (is_xilinx_io_cell(cell_type))
            return id_IOB;
        if (is_xilinx_ff_cell(cell_type))
            return id_DFF;
        if (is_lut_cell(cell_type))
            return id_LUT4;
        if (cell_type == ctx->id("RAM16X1S") || is_ise_srl(ctx, cell_type)) return id_LUT4;
        if (ise_bram_shape(cell_type.str(ctx))) return ctx->id("RAMB16");
        if (ise_mult_type(ctx, cell_type)) return ctx->id("MULT18X18SIO");
        if (cell_type.in(id_MUXF6, id_MUXF7, id_MUXF8))
            return id_MUXF6;
        if (cell_type.in(id_BUFG, ctx->id("BUFGMUX_1"))) return id_BUFGMUX;
        if (cell_type == ctx->id("STARTUP_SPARTAN3E")) return id_STARTUP;
        if (cell_type == ctx->id("CAPTURE_SPARTAN3")) return id_CAPTURE;
        if (cell_type == ctx->id("BSCAN_SPARTAN3")) return id_BSCAN;
        return cell_type;
    }

    BelBucketId getBelBucketForBel(BelId bel) const override
    { return ctx->getBelType(bel).in(ctx->id("IOB_OFF1"), ctx->id("IOB_IFF1"), ctx->id("IOB_TFF1")) ? id_DFF : ctx->getBelType(bel); }

    bool isValidBelForCellType(IdString cell_type, BelId bel) const override
    {
        IdString bel_type = ctx->getBelType(bel);
        if (bel_type == ctx->id("IOB_OFF1")) return cell_type.in(id_FD, id_FDCE, id_FDC, id_FDPE, id_FDP, id_FDRE, id_FDSE);
        if (bel_type == ctx->id("IOB_TFF1")) return cell_type.in(id_FD, id_FDCE, id_FDC, id_FDPE, id_FDP, id_FDRE, id_FDSE);
        if (bel_type == ctx->id("IOB_IFF1")) return cell_type.in(id_FDCE, id_FDPE);
        if (bel_type == id_IOB) {
            const auto &ref = bel_site_info.at(bel.index).site;
            const auto &tile = db.tiles[ref.tile];
            const auto &type = db.variants[tile.variant].sites[ref.site].type;
            return cell_type == id_IBUF || (cell_type.in(id_OBUF, id_OBUFT, id_IOBUF) && type != "IBUF" && type != "DIFFMI" && type != "DIFFSI");
        }
        if (bel_type == id_DFF)
            return is_xilinx_ff_cell(cell_type);
        if (bel_type == id_LUT4) {
            if (cell_type == ctx->id("RAM16X1S") || is_ise_srl(ctx, cell_type)) {
                const auto &ref = bel_site_info.at(bel.index).site;
                return db.variants[db.tiles[ref.tile].variant].sites[ref.site].type == "SLICEM";
            }
            return is_lut_cell(cell_type);
        }
        if (bel_type == id_MUXF6)
            return cell_type.in(id_MUXF6, id_MUXF7, id_MUXF8);
        if (bel_type == id_BUFGMUX) return cell_type.in(id_BUFG, id_BUFGMUX, ctx->id("BUFGMUX_1"));
        if (bel_type == id_STARTUP) return cell_type == ctx->id("STARTUP_SPARTAN3E");
        if (bel_type == id_CAPTURE) return cell_type == ctx->id("CAPTURE_SPARTAN3");
        if (bel_type == id_BSCAN) return cell_type == ctx->id("BSCAN_SPARTAN3");
        if (bel_type == ctx->id("RAMB16")) return ise_bram_shape(cell_type.str(ctx)).has_value();
        if (bel_type == ctx->id("MULT18X18SIO")) return ise_mult_type(ctx, cell_type);
        return bel_type == cell_type;
    }

    bool ff_rev_resources_valid(int index) const
    {
        const auto &site = carry_sites[index];
        for (int half = 0; half < 2; ++half) {
            CellInfo *ff = ctx->getBoundBelCell(site.ff[half]);
            if (ff == nullptr) continue;
            auto shape = ise_ff_shape(ff->type.str(ctx));
            if (!shape || shape->rev == nullptr) continue;
            NetInfo *rev = ff->getPort(ctx->id(shape->rev));
            if (rev == nullptr) continue;
            // REV takes the shared BY input. Keep unverified memory/carry/FX
            // mixtures closed; ordinary LUT data paths do not consume BY.
            if (ctx->getBoundBelCell(wide_bels[index][1]) != nullptr) return false;
            for (int other = 0; other < 2; ++other) {
                CellInfo *logic = ctx->getBoundBelCell(site.lut[other]);
                if (logic != nullptr && !is_lut_cell(logic->type)) return false;
                if (ctx->getBoundBelCell(site.mux[other]) != nullptr ||
                    ctx->getBoundBelCell(site.xorcy[other]) != nullptr) return false;
            }
            CellInfo *y = ctx->getBoundBelCell(site.ff[1]);
            if (y != nullptr && !local_ff_data(y, site.site, 1) &&
                (y->getPort(id_D) != rev || ise_ff_inverted(ctx, ff, shape->rev))) return false;
        }
        return true;
    }

    bool isBelLocationValid(BelId bel, bool) const override
    {
        auto wide_slot = wide_bel_slot.find(bel.index);
        auto carry_slot = carry_bel_slot.find(bel.index);
        if (wide_slot != wide_bel_slot.end() || carry_slot != carry_bel_slot.end()) {
            int index = carry_slot != carry_bel_slot.end() ? carry_slot->second.first : wide_slot->second.first;
            if (!ff_rev_resources_valid(index)) return false;
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
            if (logic[0] != nullptr && is_ise_srl(ctx, logic[0]->type) && logic[1] == nullptr) {
                const std::string fixed_g = ctx->getBelName(site.lut[1]).str(ctx);
                CellInfo *pending = nullptr;
                auto cluster = srl_cluster.find(logic[0]->cluster);
                if (cluster != srl_cluster.end()) {
                    const auto &stages = srl_chains[cluster->second].stages;
                    for (size_t i = 1; i < stages.size(); i += 2)
                        if (stages[i].cell == logic[0]) {
                            CellInfo *peer = stages[i - 1].cell;
                            auto fixed = peer->attrs.find(ctx->id("BEL"));
                            if (peer->bel == BelId() && (fixed == peer->attrs.end() || fixed->second.as_string() == fixed_g))
                                pending = peer;
                        }
                }
                for (const auto &entry : ctx->cells) {
                    CellInfo *candidate = entry.second.get();
                    auto fixed = candidate->attrs.find(ctx->id("BEL"));
                    if (!is_ise_srl(ctx, candidate->type) || candidate->bel != BelId() ||
                        fixed == candidate->attrs.end() || fixed->second.as_string() != fixed_g) continue;
                    if (pending != nullptr && pending != candidate) return false;
                    pending = candidate;
                }
                logic[1] = pending;
            }
            if (!ise_srl_resources_valid(ctx, logic, ffs, excluded)) return false;
        }
        if (wide_slot != wide_bel_slot.end() && !wide_resources_valid(wide_slot->second.first)) return false;
        if (carry_slot != carry_bel_slot.end() && !wide_resources_valid(carry_slot->second.first)) return false;
        for (const auto &memory : ram64_resources) {
            if (bel != memory.composite && std::find(memory.excluded.begin(), memory.excluded.end(), bel) == memory.excluded.end()) continue;
            if (ctx->getBoundBelCell(memory.composite) == nullptr) continue;
            for (BelId excluded : memory.excluded)
                if (ctx->getBoundBelCell(excluded) != nullptr) return false;
        }
        auto dual = ram16_dual_resource_by_bel.find(bel.index);
        if (dual != ram16_dual_resource_by_bel.end() &&
            !ise_ram16_dual_resources_valid(ctx, ram16_dual_resources[dual->second])) return false;
        auto ram32 = ram32_resource_by_bel.find(bel.index);
        if (ram32 != ram32_resource_by_bel.end() && !ise_ram32_resources_valid(ctx, ram32_resources[ram32->second]))
            return false;
        CellInfo *bound = ctx->getBoundBelCell(bel);
        if (bound != nullptr && ise_ff_shape(bound->type.str(ctx)).has_value()) {
            auto loc = bound->attrs.find(ctx->id("LOC"));
            if (loc != bound->attrs.end()) {
                const auto &ref = bel_site_info.at(bel.index).site;
                if (loc->second.as_string() != db.tiles[ref.tile].sites[ref.site]) return false;
            }
        }
        if (bound != nullptr && ctx->getBelType(bel) == ctx->id("IOB_OFF1") && !iob_off_clusters.count(bound->cluster)) return false;
        if (bound != nullptr && ctx->getBelType(bel) == ctx->id("IOB_TFF1") && !iob_tff_clusters.count(bound->cluster)) return false;
        if (bound != nullptr && ctx->getBelType(bel) == ctx->id("IOB_IFF1") && !iob_if_clusters.count(bound->cluster)) return false;
        if (bound != nullptr && !clock_model->region_valid(bound, bel)) return false;
        auto slot = carry_bel_slot.find(bel.index);
        if (slot != carry_bel_slot.end()) {
            const auto &site = carry_sites[slot->second.first];
            std::array<CellInfo *, 2> logic{{ctx->getBoundBelCell(site.lut[0]), ctx->getBoundBelCell(site.lut[1])}};
            auto is_ram = [&](CellInfo *cell) { return cell != nullptr && cell->type == ctx->id("RAM16X1S"); };
            for (int half = 0; half < 2; ++half) {
                CellInfo *ram = logic[half];
                if (!is_ram(ram)) continue;
                for (int other = 0; other < 2; ++other) {
                    CellInfo *peer = logic[other];
                    if (is_ram(peer) && (peer->getPort(ctx->id("WCLK")) != ram->getPort(ctx->id("WCLK")) ||
                                        peer->getPort(ctx->id("WE")) != ram->getPort(ctx->id("WE")))) return false;
                    CellInfo *ff = ctx->getBoundBelCell(site.ff[other]);
                    if (ff == nullptr) continue;
                    if (ff->getPort(ctx->id("C")) != ram->getPort(ctx->id("WCLK")) || ise_ff_clock_inverted(ctx, ff)) return false;
                    for (IdString pin : {ctx->id("R"), id_S, ctx->id("CLR"), ctx->id("PRE")})
                        // ISE rejects simultaneous SR->WSGEN.WE and SR->FF.SR,
                        // including aliases of the same logical net. Keep the
                        // output-register form without SR until a legal case exists.
                        if (ff->getPort(pin) != nullptr) return false;
                    if (half == other) {
                        NetInfo *d = ff->getPort(id_D);
                        bool local = d != nullptr && d->driver.cell == ram;
                        if (!local && d != ram->getPort(id_D)) return false;
                    }
                }
                // F RAM's write decoder uses the G address pins, even in
                // single-port mode. A co-resident G resource must agree.
                if (half == 0 && logic[1] != nullptr)
                    for (int i = 0; i < 4; ++i) {
                        NetInfo *address = logic[1]->getPort(ctx->idf(is_ram(logic[1]) ? "A%d" : "I%d", i));
                        if (address != nullptr && address != ram->getPort(ctx->idf("A%d", i))) return false;
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
        return ise_ff_pair_valid(ctx, a, b);
    }

    void build_wires()
    {
        tile_base.assign(db.tiles.size(), 0);
        uint32_t total = 0;
        for (size_t i = 0; i < db.tiles.size(); i++) {
            const auto &tile = db.tiles[i];
            if (tile.variant < 0 || tile.variant >= (int)db.variants.size())
                log_error("spartan3e-ise: tile %s has bad variant %d\n", tile.name.c_str(), tile.variant);
            tile_base[i] = total;
            uint64_t next = (uint64_t)total + db.variants[tile.variant].wires.size();
            if (next > std::numeric_limits<uint32_t>::max())
                log_error("spartan3e-ise: too many wire members\n");
            total = (uint32_t)next;
        }

        Dsu dsu(total);
        auto member_at = [&](int tile, int wire) -> uint32_t {
            if (tile < 0 || tile >= (int)db.tiles.size())
                log_error("spartan3e-ise: node has bad tile index %d\n", tile);
            const auto &variant = db.variants[db.tiles[tile].variant];
            if (wire < 0 || wire >= (int)variant.wires.size())
                log_error("spartan3e-ise: node has bad wire index %d in tile %s\n", wire, db.tiles[tile].name.c_str());
            return tile_base[tile] + (uint32_t)wire;
        };
        for (auto &node : db.nodes) {
            if (node.empty())
                continue;
            uint32_t first = member_at(node[0].first, node[0].second);
            for (size_t i = 1; i < node.size(); i++)
                dsu.unite(first, member_at(node[i].first, node[i].second));
        }

        std::vector<uint32_t> min_member(total, std::numeric_limits<uint32_t>::max());
        for (uint32_t i = 0; i < total; i++) {
            uint32_t root = dsu.find(i);
            if (i < min_member[root])
                min_member[root] = i;
        }

        member_wire.assign(total, WireId());
        std::vector<WireId> root_wire(total);
        auto member_name = [&](uint32_t member) -> std::pair<const IseTile *, int> {
            auto it = std::upper_bound(tile_base.begin(), tile_base.end(), member);
            size_t tile = (size_t)(it - tile_base.begin()) - 1;
            return {&db.tiles[tile], (int)(member - tile_base[tile])};
        };
        for (uint32_t i = 0; i < total; i++) {
            uint32_t root = dsu.find(i);
            if (i != min_member[root])
                continue;
            auto [tile, wire] = member_name(i);
            int global = db.variants[tile->variant].wires[wire];
            WireId id = ctx->addWire(IdStringList::concat(ctx->id(tile->name), ctx->id(db.wires[global])),
                                     ctx->id("WIRE"), tile->col, tile->row);
            root_wire[root] = id;
        }
        for (uint32_t i = 0; i < total; i++)
            member_wire[i] = root_wire[dsu.find(i)];
    }

    void build_pips()
    {
        size_t expected = 0;
        for (auto &tile : db.tiles)
            for (auto &pip : db.variants[tile.variant].pips)
                if (pip.rt == -1)
                    expected++;
        pip_meta.assign(expected, IsePipMeta{});

        int neg_op = -1;
        for (size_t i = 0; i < db.pip_ops.size(); i++)
            if (db.pip_ops[i] == "=-")
                neg_op = (int)i;
        auto key = [](int from, int to) { return ((uint64_t)(uint32_t)from << 32) | (uint32_t)to; };

        size_t built = 0, skipped_rt = 0, twinned = 0, missing_twin = 0;
        for (size_t ti = 0; ti < db.tiles.size(); ti++) {
            const auto &tile = db.tiles[ti];
            const auto &variant = db.variants[tile.variant];
            std::unordered_map<uint64_t, PipId> by_pair;
            by_pair.reserve(variant.pips.size());
            struct Built
            {
                int from, to, op;
                PipId pip;
            };
            std::vector<Built> local;
            for (auto &p : variant.pips) {
                if (p.rt != -1) {
                    skipped_rt++;
                    continue;
                }
                if (p.from < 0 || p.from >= (int)variant.wires.size() || p.to < 0 || p.to >= (int)variant.wires.size())
                    log_error("spartan3e-ise: bad pip endpoint in tile %s\n", tile.name.c_str());
                WireId src = member_wire[tile_base[ti] + (uint32_t)p.from];
                WireId dst = member_wire[tile_base[ti] + (uint32_t)p.to];
                int from_global = variant.wires[p.from];
                int to_global = variant.wires[p.to];
                IdStringList name =
                        IdStringList::concat(IdStringList::concat(ctx->id(tile.name), ctx->id(db.wires[from_global])),
                                             ctx->id(db.wires[to_global]));
                PipId pip = ctx->addPip(name, id_PIP, src, dst, 0, Loc(tile.col, tile.row, 0));
                if (pip_meta.size() <= (size_t)pip.index)
                    pip_meta.resize(std::max<size_t>(pip.index + 1, pip_meta.size() * 2));
                pip_meta[pip.index] = IsePipMeta{(int)ti, p.from, p.to, -1, -1};
                by_pair[key(p.from, p.to)] = pip;
                local.push_back(Built{p.from, p.to, p.op, pip});
                built++;
            }
            for (auto &b : local) {
                if (b.op != neg_op)
                    continue;
                auto it = by_pair.find(key(b.to, b.from));
                if (it == by_pair.end()) {
                    missing_twin++;
                    continue;
                }
                pip_meta[b.pip.index].twin = it->second.index;
                pip_meta[it->second.index].twin = b.pip.index;
                twinned++;
            }
        }
        if (built != expected)
            log_error("spartan3e-ise: pip count mismatch: built %zu expected %zu route-through %zu\n", built, expected,
                      skipped_rt);
        log_info("Spartan-3E ISE: route-through skipped %zu, bidirectional twins %zu/%zu, missing %zu\n", skipped_rt,
                 twinned, twinned + missing_twin, missing_twin);
    }
};

} // namespace

std::unique_ptr<ViaductAPI> make_spartan3e_ise_impl(const std::string &db_path, const std::string &check_xdl_path,
                                                const std::string &dump_path)
{
    return std::make_unique<Spartan3eIseImpl>(db_path, check_xdl_path, dump_path);
}

NEXTPNR_NAMESPACE_END
