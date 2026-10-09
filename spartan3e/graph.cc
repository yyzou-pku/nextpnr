#include "implementation.h"

#include "constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace spartan3e_detail {

void Spartan3eImpl::index_sites()
{
    site_by_name.reserve(db.tiles.size() * 2); // ~2 sites per tile on average
    for (size_t ti = 0; ti < db.tiles.size(); ti++) {
        const auto &tile = db.tiles[ti];
        if (tile.variant < 0 || tile.variant >= (int)db.variants.size())
            log_error("spartan3e: tile %s has bad variant %d\n", tile.name.c_str(), tile.variant);
        if (tile.sites.size() > db.variants[tile.variant].sites.size())
            log_error("spartan3e: tile %s has too many sites\n", tile.name.c_str());
        for (size_t si = 0; si < tile.sites.size(); si++) {
            if (!site_by_name.emplace(tile.sites[si], SiteRef{(int)ti, (int)si}).second)
                log_error("spartan3e: duplicate site name %s\n", tile.sites[si].c_str());
        }
    }
    for (const auto &kv : db.site_defs) {
        auto &index = site_elem_index[kv.first];
        for (size_t ei = 0; ei < kv.second.elements.size(); ei++)
            index[kv.second.elements[ei].name] = ei;
    }
}

WireId Spartan3eImpl::site_pin_wire(const Tile &tile, size_t ti, const SiteDef &site, const std::string &pin) const
{
    auto it = site.pins.find(pin);
    if (it == site.pins.end() || it->second < 0)
        return WireId();
    uint32_t member = tile_base[ti] + (uint32_t)it->second;
    if (member >= member_wire.size())
        return WireId();
    return member_wire[member];
}

std::array<BelId, 4> Spartan3eImpl::add_carry_bels(const Tile &tile, size_t ti, size_t si, const std::string &site_name,
                                                   int z)
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
        bel_site_info[muxbel.index] = BelSiteInfo{SiteRef{(int)ti, (int)si}, cymux};
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
        bel_site_info[xorbel.index] = BelSiteInfo{SiteRef{(int)ti, (int)si}, xor_el};
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

BelId Spartan3eImpl::add_lut4_bel(const Tile &tile, size_t ti, size_t si, const std::string &site_name,
                                  const std::string &element, int z)
{
    IdStringList name = h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id(element)));
    BelId bel = ctx->addBel(name, id_LUT4, Loc(tile.col, tile.row, z), false, false);
    bel_site_info[bel.index] = BelSiteInfo{SiteRef{(int)ti, (int)si}, element};
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

BelId Spartan3eImpl::add_ff_bel(const Tile &tile, size_t ti, size_t si, const std::string &site_name,
                                const std::string &element, int z)
{
    IdStringList name = h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id(element)));
    BelId bel = ctx->addBel(name,
                            element == "OFF1"   ? ctx->id("IOB_OFF1")
                            : element == "IFF1" ? ctx->id("IOB_IFF1")
                            : element == "TFF1" ? ctx->id("IOB_TFF1")
                                                : id_DFF,
                            Loc(tile.col, tile.row, z), false, false);
    bel_site_info[bel.index] = BelSiteInfo{SiteRef{(int)ti, (int)si}, element};
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

BelId Spartan3eImpl::add_iob_bel(const Tile &tile, size_t ti, size_t si, const SiteDef &site,
                                 const std::string &site_name, int z)
{
    IdStringList name = h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id("IOB")));
    BelId bel = ctx->addBel(name, id_IOB, Loc(tile.col, tile.row, z), false, false);
    bel_site_info[bel.index] = BelSiteInfo{SiteRef{(int)ti, (int)si}, "IOB"};
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
            log_error("spartan3e: ambiguous package binding for %s\n", site_name.c_str());
        bel_package_pin[bel.index] = pin.first;
    }
    if (!bel_package_pin.count(bel.index))
        log_error("spartan3e: bonded I/O site %s missing from package table\n", site_name.c_str());
    return bel;
}

void Spartan3eImpl::add_bram_bel(const Tile &tile, int ti, int si, const std::string &name, int z)
{
    BelId bel = ctx->addBel(h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(name), ctx->id("RAMB16"))),
                            ctx->id("RAMB16"), Loc(tile.col, tile.row, z), false, false);
    bel_site_info[bel.index] = BelSiteInfo{SiteRef{ti, si}, "RAMB16"};
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
                if (base == "DI" || base == "DIP")
                    ctx->addBelInput(bel, ctx->id(pin), wire);
                else
                    ctx->addBelOutput(bel, ctx->id(pin), wire);
            }
        for (const std::string &control :
             {std::string("CLK"), std::string("EN"), std::string("WE"), std::string("SSR")})
            ctx->addBelInput(bel, ctx->id(control + port), site_elem_wire(ti, si, wrapper, control + port));
    }
}

void Spartan3eImpl::init_bels()
{
    log_info("Spartan-3E: creating bels...\n");
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
                carry.site = SiteRef{int(ti), int(si)};
                carry.lut[0] = add_lut4_bel(tile, ti, si, site_name, "F", z++);
                carry.ff[0] = add_ff_bel(tile, ti, si, site_name, "FFX", z++);
                carry.lut[1] = add_lut4_bel(tile, ti, si, site_name, "G", z++);
                carry.ff[1] = add_ff_bel(tile, ti, si, site_name, "FFY", z++);
                auto bels = add_carry_bels(tile, ti, si, site_name, z);
                std::array<BelId, 2> wide;
                for (int kind = 0; kind < 2; ++kind) {
                    const std::string element = kind == 0 ? "F5MUX" : "F6MUX";
                    BelId bel = ctx->addBel(
                            h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id(element))),
                            kind == 0 ? id_MUXF5 : id_MUXF6, Loc(tile.col, tile.row, z + 4 + kind), false, false);
                    wide[kind] = bel;
                    bel_site_info[bel.index] = BelSiteInfo{SiteRef{int(ti), int(si)}, element};
                    wide_bel_slot[bel.index] = {int(carry_sites.size()), kind};
                    ctx->addBelInput(bel, id_I0, site_elem_wire(ti, si, element, kind == 0 ? "G" : "0"));
                    ctx->addBelInput(bel, id_I1, site_elem_wire(ti, si, element, kind == 0 ? "F" : "1"));
                    ctx->addBelInput(bel, id_S, site_elem_wire(ti, si, element, "S0"));
                    ctx->addBelOutput(bel, id_O, site_elem_wire(ti, si, element, "OUT"));
                }
                wide_bels.push_back(wide);
                for (int half = 0; half < 2; ++half) {
                    const std::string element = half == 0 ? "FAND" : "GAND";
                    BelId bel = ctx->addBel(
                            h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id(element))),
                            ctx->id("MULT_AND"), Loc(tile.col, tile.row, z + (site.type == "SLICEM" ? 8 : 6) + half),
                            false, false);
                    carry.andcell[half] = bel;
                    bel_site_info[bel.index] = BelSiteInfo{SiteRef{int(ti), int(si)}, element};
                    ctx->addBelInput(bel, id_I0, site_elem_wire(ti, si, element, "0"));
                    ctx->addBelInput(bel, id_I1, site_elem_wire(ti, si, element, "1"));
                    ctx->addBelOutput(bel, ctx->id("LO"), site_elem_wire(ti, si, element, "O"));
                }
                if (site.type == "SLICEM") {
                    BelId composite = ctx->addBel(
                            h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id("RAM32"))),
                            ctx->id("RAM32X1S"), Loc(tile.col, tile.row, z + 6), false, false);
                    bel_site_info[composite.index] = BelSiteInfo{SiteRef{int(ti), int(si)}, "F5MUX"};
                    connect_ram32(ctx, composite, [&](const std::string &element, const std::string &pin) {
                        return site_elem_wire(ti, si, element, pin);
                    });
                    Ram32Resources resources{composite,
                                             {carry.lut[0], carry.lut[1], bels[0], bels[1], bels[2], bels[3], wide[0],
                                              wide[1], carry.andcell[0], carry.andcell[1]},
                                             carry.ff};
                    BelId dual = ctx->addBel(
                            h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id("RAM16D"))),
                            ctx->id("RAM16X1D"), Loc(tile.col, tile.row, z + 7), false, false);
                    bel_site_info[dual.index] = BelSiteInfo{SiteRef{int(ti), int(si)}, "F"};
                    connect_ram16_dual(ctx, dual, [&](const std::string &element, const std::string &pin) {
                        return site_elem_wire(ti, si, element, pin);
                    });
                    Ram16DualResources dual_resources{dual, resources.members};
                    dual_resources.excluded.push_back(composite);
                    for (BelId ff : carry.ff)
                        dual_resources.excluded.push_back(ff);
                    ram16_dual_resource_by_bel[dual.index] = ram16_dual_resources.size();
                    for (BelId member : dual_resources.excluded)
                        ram16_dual_resource_by_bel[member.index] = ram16_dual_resources.size();
                    ram16_dual_resources.push_back(std::move(dual_resources));
                    resources.members.push_back(dual);
                    ram32_resource_by_bel[composite.index] = ram32_resources.size();
                    for (BelId member : resources.members)
                        ram32_resource_by_bel[member.index] = ram32_resources.size();
                    for (BelId ff : resources.ffs)
                        ram32_resource_by_bel[ff.index] = ram32_resources.size();
                    ram32_resources.push_back(std::move(resources));
                }
                for (int half = 0; half < 2; half++) {
                    carry.mux[half] = bels[2 * half];
                    carry.xorcy[half] = bels[2 * half + 1];
                    for (auto bel :
                         {carry.lut[half], carry.ff[half], carry.mux[half], carry.xorcy[half], carry.andcell[half]})
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
                BelId bel = ctx->addBel(
                        h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id("MULT18X18SIO"))),
                        ctx->id("MULT18X18SIO"), Loc(tile.col, tile.row, z++), false, false);
                bel_site_info[bel.index] = BelSiteInfo{SiteRef{int(ti), int(si)}, "MULT18X18SIO"};
                mult_site_by_bel[bel.index] = int(mult_sites.size());
                mult_sites.push_back({SiteRef{int(ti), int(si)}, bel, -1});
                const auto &elements = db.site_defs.at("MULT18X18SIO").elements;
                auto multiplier = std::find_if(elements.begin(), elements.end(),
                                               [](const auto &element) { return element.name == "MULT18X18SIO"; });
                if (multiplier == elements.end())
                    log_error("spartan3e: missing multiplier site element\n");
                for (const auto &pin : multiplier->pins) {
                    WireId wire = site_elem_wire(ti, si, multiplier->name, pin.first);
                    if (pin.second == "input")
                        ctx->addBelInput(bel, ctx->id(pin.first), wire);
                    else
                        ctx->addBelOutput(bel, ctx->id(pin.first), wire);
                }
            } else if (site.type == "BUFGMUX") {
                BelId bel = ctx->addBel(
                        h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id("GCLK_BUFFER"))),
                        id_BUFGMUX, Loc(tile.col, tile.row, z++), true, true);
                SiteRef ref{int(ti), int(si)};
                bel_site_info[bel.index] = BelSiteInfo{ref, "GCLK_BUFFER"};
                ctx->addBelInput(bel, id_I, site_elem_wire(ti, si, "GCLKMUX", "I0"));
                ctx->addBelInput(bel, id_I0, site_elem_wire(ti, si, "GCLKMUX", "I0"));
                ctx->addBelInput(bel, id_I1, site_elem_wire(ti, si, "GCLKMUX", "I1"));
                ctx->addBelInput(bel, id_S, site_elem_wire(ti, si, "GCLKMUX", "S"));
                ctx->addBelOutput(bel, id_O, site_elem_wire(ti, si, "GCLK_BUFFER", "OUT"));
                ClockSite clock;
                clock.site = ref;
                clock.bel = bel;
                clock.inputs = {{site_pin_wire(tile, ti, site, "I0"), site_pin_wire(tile, ti, site, "I1")}};
                clock.output = site_pin_wire(tile, ti, site, "O");
                clock_sites.push_back(std::move(clock));
            } else if (site.type == "VCC") {
                BelId bel = ctx->addBel(
                        h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id("POWER"))),
                        ctx->id("VCC"), Loc(tile.col, tile.row, z++), false, false);
                bel_site_info[bel.index] = BelSiteInfo{SiteRef{int(ti), int(si)}, "POWER"};
                ctx->addBelOutput(bel, ctx->id("P"), site_elem_wire(ti, si, "POWER", "1"));
                clock_power_bels[int(ti)] = bel;
            } else if (site.type == "STARTUP" || site.type == "CAPTURE" || site.type == "BSCAN") {
                // Fixed single-site control primitives; the BEL is the site's
                // blackbox element and the pins route through the corner muxes.
                const std::string element = site.type == "STARTUP"   ? "STARTUP"
                                            : site.type == "CAPTURE" ? "CAPTURE_BLACKBOX"
                                                                     : "BSCAN_BLACKBOX";
                const SiteElement *bel_elem = nullptr;
                for (const auto &candidate : db.site_defs.at(site.type).elements)
                    if (candidate.name == element)
                        bel_elem = &candidate;
                if (bel_elem == nullptr)
                    log_error("spartan3e: %s site has no %s element\n", site.type.c_str(), element.c_str());
                BelId bel = ctx->addBel(
                        h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(site_name), ctx->id(site.type))),
                        ctx->id(site.type), Loc(tile.col, tile.row, z++), false, false);
                bel_site_info[bel.index] = BelSiteInfo{SiteRef{int(ti), int(si)}, element};
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
    log_info("Spartan-3E: created %zu LUT4 bels, %zu DFF bels, %zu IOB bels\n", nlut, nff, nio);
}

void Spartan3eImpl::index_carry_topology()
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
                log_error("Spartan-3E: ambiguous COUT/CIN topology\n");
            carry.next = it->second;
        };
        match(cout);
        for (auto pip : ctx->getPipsDownhill(cout))
            match(ctx->getPipDstWire(pip));
        if (carry.next != -1)
            links++;
    }
    log_info("Spartan-3E: indexed %zu dedicated carry site links\n", links);
}

void Spartan3eImpl::index_clock_topology()
{
    std::unordered_map<int32_t, BelId> pads;
    std::unordered_set<int32_t> pins;
    ClockPins physical_clock_pins;
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
        for (BelId bel : carry.lut)
            physical_clock_pins[bel.index][ctx->id("WCLK")] = {pin};
        for (BelId bel : carry.lut)
            physical_clock_pins[bel.index][ctx->id("CLK")] = {pin};
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
        if (entry.second.element != "RAMB16")
            continue;
        const auto &ref = entry.second.site;
        const auto &tile = db.tiles[ref.tile];
        const auto &site = db.variants[tile.variant].sites[ref.site];
        for (const std::string &port : {std::string("CLKA"), std::string("CLKB")}) {
            WireId pin = site_pin_wire(tile, ref.tile, site, port);
            pins.insert(pin.index);
            physical_clock_pins[entry.first][ctx->id(port)] = {pin};
            if (port == "CLKA")
                physical_clock_pins[entry.first][ctx->id("CLK")] = {pin};
        }
    }
    index_clock_paths(ctx, clock_sites, pads, pins, ctx->getWires().size, int(pip_meta.size()), [&](PipId pip) {
        const auto &meta = pip_meta.at(pip.index);
        const auto &variant = db.variants[db.tiles[meta.tile].variant];
        return clock_distribution_pip(db.wires[variant.wires[meta.from]], db.wires[variant.wires[meta.to]]);
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
    clock_model = std::make_unique<ClockModel>(ctx, db, std::move(clock_sites), std::move(clock_power_bels),
                                               std::move(physical_clock_pins));
}

void Spartan3eImpl::index_wide_topology()
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
            if (from == sources[kind].end())
                continue;
            for (int input = 0; input < 2; ++input) {
                auto to = destinations[input].find(ctx->getPipDstWire(pip).index);
                if (to == destinations[input].end())
                    continue;
                int &source = wide_sources[to->second][kind][input];
                if (source != -1 && source != from->second)
                    log_error("spartan3e: ambiguous dedicated wide cascade link\n");
                source = from->second;
                ++count;
            }
        }
    }
    log_info("Spartan-3E: indexed %zu dedicated wide cascade links\n", count);
}

void Spartan3eImpl::init_ram64_bels()
{
    for (size_t high = 0; high < carry_sites.size(); ++high) {
        int low = wide_sources[high][0][0];
        if (low < 0 || low == int(high) || wide_sources[high][0][1] != int(high))
            continue;
        const auto &hr = carry_sites[high].site;
        const auto &lr = carry_sites[low].site;
        const auto &tile = db.tiles[hr.tile];
        if (hr.tile != lr.tile || db.variants[tile.variant].sites[hr.site].type != "SLICEM" ||
            db.variants[tile.variant].sites[lr.site].type != "SLICEM")
            continue;
        int z = 0;
        for (const auto &entry : bel_site_info) {
            Loc loc = ctx->getBelLocation(BelId{entry.first});
            if (loc.x == tile.col && loc.y == tile.row)
                z = std::max(z, loc.z + 1);
        }
        BelId bel = ctx->addBel(
                h.xy_id(tile.col, tile.row, IdStringList::concat(ctx->id(tile.sites[hr.site]), ctx->id("RAM64"))),
                ctx->id("RAM64X1S"), Loc(tile.col, tile.row, z), false, false);
        bel_site_info[bel.index] = BelSiteInfo{hr, "F6MUX"};
        Ram64Resources resources{bel, {low, int(high)}, {}};
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
                if (entry.first != bel.index && entry.second.site.tile == ref.tile &&
                    entry.second.site.site == ref.site)
                    resources.excluded.push_back(BelId{entry.first});
        }
        ctx->addBelInput(bel, ctx->id("A5"), site_elem_wire(hr.tile, hr.site, "F6MUX", "S0"));
        ctx->addBelOutput(bel, ctx->id("O"), site_elem_wire(hr.tile, hr.site, "F6MUX", "OUT"));
        ram64_by_bel[bel.index] = ram64_resources.size();
        ram64_resources.push_back(std::move(resources));
    }
}

void Spartan3eImpl::index_mult_topology()
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
                if (found == inputs.end() || found->second.second != bit)
                    return;
                if (target >= 0 && target != found->second.first)
                    log_error("spartan3e: ambiguous MULT cascade bit\n");
                target = found->second.first;
            };
            match(output);
            for (PipId pip : ctx->getPipsDownhill(output))
                match(ctx->getPipDstWire(pip));
            if (target < 0 || (next >= 0 && next != target))
                complete = false;
            if (target >= 0)
                next = target;
        }
        if (complete)
            mult_sites[i].next = next;
    }
}

void Spartan3eImpl::index_srl_topology()
{
    srl_next_site.assign(carry_sites.size(), -1);
    std::unordered_map<int32_t, int> inputs;
    for (size_t i = 0; i < carry_sites.size(); ++i) {
        const auto &ref = carry_sites[i].site;
        const auto &tile = db.tiles[ref.tile];
        const auto &def = db.variants[tile.variant].sites[ref.site];
        if (def.type == "SLICEM")
            inputs[site_pin_wire(tile, ref.tile, def, "SHIFTIN").index] = int(i);
    }
    for (size_t i = 0; i < carry_sites.size(); ++i) {
        const auto &ref = carry_sites[i].site;
        const auto &tile = db.tiles[ref.tile];
        const auto &def = db.variants[tile.variant].sites[ref.site];
        if (def.type != "SLICEM")
            continue;
        WireId output = site_pin_wire(tile, ref.tile, def, "SHIFTOUT");
        auto assign = [&](WireId target) {
            auto found = inputs.find(target.index);
            if (found == inputs.end())
                return;
            int &next = srl_next_site[i];
            if (next >= 0 && next != found->second)
                log_error("spartan3e: ambiguous dedicated SRL site link\n");
            next = found->second;
        };
        assign(output);
        for (PipId pip : ctx->getPipsDownhill(output))
            assign(ctx->getPipDstWire(pip));
    }
}

void Spartan3eImpl::build_wires()
{
    tile_base.assign(db.tiles.size(), 0);
    uint32_t total = 0;
    for (size_t i = 0; i < db.tiles.size(); i++) {
        const auto &tile = db.tiles[i];
        if (tile.variant < 0 || tile.variant >= (int)db.variants.size())
            log_error("spartan3e: tile %s has bad variant %d\n", tile.name.c_str(), tile.variant);
        tile_base[i] = total;
        uint64_t next = (uint64_t)total + db.variants[tile.variant].wires.size();
        if (next > std::numeric_limits<uint32_t>::max())
            log_error("spartan3e: too many wire members\n");
        total = (uint32_t)next;
    }

    Dsu dsu(total);
    auto member_at = [&](int tile, int wire) -> uint32_t {
        if (tile < 0 || tile >= (int)db.tiles.size())
            log_error("spartan3e: node has bad tile index %d\n", tile);
        const auto &variant = db.variants[db.tiles[tile].variant];
        if (wire < 0 || wire >= (int)variant.wires.size())
            log_error("spartan3e: node has bad wire index %d in tile %s\n", wire, db.tiles[tile].name.c_str());
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
    auto member_name = [&](uint32_t member) -> std::pair<const Tile *, int> {
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
        WireId id = ctx->addWire(IdStringList::concat(ctx->id(tile->name), ctx->id(db.wires[global])), ctx->id("WIRE"),
                                 tile->col, tile->row);
        root_wire[root] = id;
    }
    for (uint32_t i = 0; i < total; i++)
        member_wire[i] = root_wire[dsu.find(i)];
}

void Spartan3eImpl::build_pips()
{
    size_t expected = 0;
    for (auto &tile : db.tiles)
        for (auto &pip : db.variants[tile.variant].pips)
            if (pip.rt == -1)
                expected++;
    pip_meta.assign(expected, PipMeta{});

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
                log_error("spartan3e: bad pip endpoint in tile %s\n", tile.name.c_str());
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
            pip_meta[pip.index] = PipMeta{(int)ti, p.from, p.to, -1, -1};
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
        log_error("spartan3e: pip count mismatch: built %zu expected %zu route-through %zu\n", built, expected,
                  skipped_rt);
    log_info("Spartan-3E: route-through skipped %zu, bidirectional twins %zu/%zu, missing %zu\n", skipped_rt, twinned,
             twinned + missing_twin, missing_twin);
}

} // namespace spartan3e_detail

NEXTPNR_NAMESPACE_END
