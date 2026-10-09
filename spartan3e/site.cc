#include "implementation.h"

#include "constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace spartan3e_detail {

WireId Spartan3eImpl::site_elem_wire(size_t ti, size_t si, const std::string &element, const std::string &pin)
{
    SiteElemKey key{(int)ti, (int)si, element, pin};
    auto it = site_wire.find(key);
    if (it != site_wire.end())
        return it->second;
    const auto &tile = db.tiles[ti];
    IdStringList name =
            h.xy_id(tile.col, tile.row, ctx->idf("S_%s_%s_%s", tile.sites[si].c_str(), element.c_str(), pin.c_str()));
    WireId wire = ctx->addWire(name, ctx->id("WIRE"), tile.col, tile.row);
    site_wire.emplace(key, wire);
    return wire;
}

void Spartan3eImpl::build_site_graph()
{
    log_info("Spartan-3E: creating occupied site graphs...\n");
    static const std::set<std::string> clb_selected = {
            "F",
            "G",
            "FFX",
            "FFY",
            "FXMUX",
            "GYMUX",
            "DXMUX",
            "DYMUX",
            "XUSED",
            "YUSED",
            "CLKINV",
            "CEINV",
            "SRINV",
            "BXINV",
            "BYINV",
            "XORF",
            "XORG",
            "FAND",
            "GAND",
            "CYINIT",
            "CY0F",
            "CY0G",
            "CYSELF",
            "CYSELG",
            "CYMUXF",
            "CYMUXG",
            "COUTUSED",
            "XBUSED",
            "YBUSED",
            "XBMUX",
            "YBMUX",
            "SRFFMUX",
            "REVUSED",
            "F5MUX",
            "F6MUX",
            "F5USED",
            "FXUSED",
            "DIF_MUX",
            "DIG_MUX",
            "WSGEN",
            "SLICEWE0USED",
            "SLICEWE1USED",
            "BYOUTUSED",
            "BYINVOUTUSED",
            "DIGUSED",
            "SHIFTIN",
            "SHIFTOUT",
            "SHIFTOUTUSED",
            "FMC15_BLACKBOX",
            "GMC15_BLACKBOX",
            "WF1USED",
            "WF2USED",
            "WF3USED",
            "WF4USED",
            "WG1USED",
            "WG2USED",
            "WG3USED",
            "WG4USED",
    };
    static const std::set<std::string> io_selected = {
            "INBUF",  "IDELMUX",   "IMUX",    "O1INV",      "OMUX",      "OUTBUF", "T1INV",     "TMUX",  "TFF1",
            "TCEINV", "TSR_USED",  "OFF1",    "O1_DDRMUX",  "OTCLK1INV", "OCEINV", "PCICE_MUX", "SRINV", "OSR_USED",
            "IFF1",   "IFFDELMUX", "IFFDMUX", "IDDRIN_MUX", "ICLK1INV",  "ICEINV", "ISR_USED"};
    static const std::set<std::string> clock_selected = {"I0_USED", "I1_USED", "SINV", "GCLKMUX", "GCLK_BUFFER"};
    static const std::set<std::string> power_selected = {"POWER"};
    static const std::set<std::string> bram_selected = {"RAMB16", "RAMB16A", "RAMB16B", "CLKAINV", "CLKBINV", "ENAINV",
                                                        "ENBINV", "WEAINV",  "WEBINV",  "SSRAINV", "SSRBINV"};
    static const std::set<std::string> mult_selected = {"MULT18X18SIO", "CLKINV",  "CEAINV",  "CEBINV",
                                                        "CEPINV",       "RSTAINV", "RSTBINV", "RSTPINV"};
    static const std::set<std::string> startup_selected = {"STARTUP", "CLKINV", "GTSINV", "GSRINV",
                                                           "CLK",     "GSR",    "GTS",    "MBT"};
    static const std::set<std::string> capture_selected = {"CAPTURE_BLACKBOX", "CLKINV", "CAPINV",
                                                           "ONESHOT_ATTR",     "CLK",    "CAP"};
    static const std::set<std::string> bscan_selected = {"BSCAN_BLACKBOX", "TDO1",  "TDO2",  "SEL1",
                                                         "SEL2",           "TDI",   "SHIFT", "UPDATE",
                                                         "DRCK1",          "DRCK2", "RESET", "CAPTURE"};
    // Logic BELs compute their outputs; they are not routing multiplexers.
    static const std::set<std::string> no_internal_pips = {"F",
                                                           "G",
                                                           "FFX",
                                                           "FFY",
                                                           "CYMUXF",
                                                           "CYMUXG",
                                                           "XORF",
                                                           "XORG",
                                                           "FAND",
                                                           "GAND",
                                                           "F5MUX",
                                                           "F6MUX",
                                                           "GCLKMUX",
                                                           "GCLK_BUFFER",
                                                           "POWER",
                                                           "WSGEN",
                                                           "FMC15_BLACKBOX",
                                                           "GMC15_BLACKBOX",
                                                           "RAMB16",
                                                           "RAMB16A",
                                                           "RAMB16B",
                                                           "MULT18X18SIO",
                                                           "STARTUP",
                                                           "CAPTURE_BLACKBOX",
                                                           "BSCAN_BLACKBOX"};
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
                site.type != "RAMB16" && site.type != "MULT18X18SIO" && site.type != "STARTUP" &&
                site.type != "CAPTURE" && site.type != "BSCAN")
                continue;
            const auto &selected = io                            ? io_selected
                                   : site.type == "BUFGMUX"      ? clock_selected
                                   : site.type == "VCC"          ? power_selected
                                   : site.type == "RAMB16"       ? bram_selected
                                   : site.type == "MULT18X18SIO" ? mult_selected
                                   : site.type == "STARTUP"      ? startup_selected
                                   : site.type == "CAPTURE"      ? capture_selected
                                   : site.type == "BSCAN"        ? bscan_selected
                                                                 : clb_selected;
            auto st_it = db.site_defs.find(site.type);
            if (st_it == db.site_defs.end())
                log_error("spartan3e: missing site_defs for %s\n", site.type.c_str());
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
                    IdStringList name =
                            h.xy_id(tile.col, tile.row, ctx->idf("SP_%s_%u", tile.sites[si].c_str(), site_pip_seq++));
                    PipId pip = ctx->addPip(name, id_PIP, sw, dw, 0, Loc(tile.col, tile.row, 0));
                    site_route_meta[pip.index] = SiteRouteMeta{SiteRef{int(ti), int(si)}, key, std::nullopt};
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
                    IdStringList name =
                            h.xy_id(tile.col, tile.row, ctx->idf("SP_%s_%u", tile.sites[si].c_str(), site_pip_seq++));
                    PipId pip = ctx->addPip(name, id_PIP, sw, dw, 0, Loc(tile.col, tile.row, 0));
                    site_route_meta[pip.index] = SiteRouteMeta{SiteRef{int(ti), int(si)},
                                                               SiteConnKey{elem.name, pin.first, elem.name, out_pin},
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
    log_info("Spartan-3E: occupied site graphs: %zu wires, %zu pips\n", wires, pips);
}

bool Spartan3eImpl::select_site_pip(const SiteRef &site, const std::string &element, const std::string &cfg)
{
    auto it = site_pip.find(SitePipKey{site.tile, site.site, element, cfg});
    if (it == site_pip.end())
        return false;
    selected_site_pips.insert(it->second.index);
    return true;
}

bool Spartan3eImpl::needs_slice_output(CellInfo *cell) const
{
    NetInfo *net = cell->getPort(id_O);
    if (net == nullptr)
        return false;
    for (const auto &user : net->users) {
        if (user.cell->type.in(id_MUXF5, id_MUXF6, id_MUXF7, id_MUXF8) && user.port.in(id_I0, id_I1))
            continue;
        if ((user.cell->type == id_MUXCY && user.port == id_S) || (user.cell->type == id_XORCY && user.port == id_LI))
            continue;
        return true;
    }
    return false;
}

bool Spartan3eImpl::local_ff_data(CellInfo *ff, const SiteRef &site, int half) const
{
    NetInfo *d = ff->getPort(id_D);
    if (d == nullptr || d->driver.cell == nullptr)
        return false;
    auto src = bel_site_info.find(d->driver.cell->bel.index);
    if (src == bel_site_info.end() || src->second.site.tile != site.tile || src->second.site.site != site.site)
        return false;
    const auto &element = src->second.element;
    return half == 0 ? element == "F" || element == "XORF" || element == "F5MUX"
                     : element == "G" || element == "XORG" || element == "F6MUX";
}

bool Spartan3eImpl::dedicated_carry_input(CellInfo *cell) const
{
    NetInfo *input = cell->getPort(id_CI);
    if (input == nullptr || input->driver.cell == nullptr || input->driver.cell->type != id_MUXCY ||
        input->driver.port != id_O || input->driver.cell->cluster != cell->cluster)
        return false;
    auto chain = carry_cluster.find(cell->cluster);
    return chain != carry_cluster.end() && carry_chains[chain->second].root != cell;
}

void Spartan3eImpl::assign_site_info()
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
        const SiteRef &site = bs_it->second.site;
        const std::string &element = bs_it->second.element;
        auto select = [&](const std::string &elem, const std::string &cfg) {
            if (select_site_pip(site, elem, cfg))
                selected++;
            else
                missing++;
        };
        if (is_lut_cell(ci->type)) {
            if (!needs_slice_output(ci))
                continue;
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
        } else if (auto shape = bram_shape(ci->type.str(ctx))) {
            for (const std::string &port : {std::string("A"), std::string("B")}) {
                if (port == "B" && !shape->dual)
                    continue;
                for (const std::string &control :
                     {std::string("CLK"), std::string("EN"), std::string("WE"), std::string("SSR")}) {
                    std::string logical = control + (shape->dual ? port : "");
                    bool inverted = ci->attrs.count(ctx->id("S3E_BRAM_CONTROL_INV_" + logical));
                    select(control + port + "INV", control + port + (inverted ? "_B" : ""));
                }
            }
        } else if (auto shape = mult_shape(ctx, ci)) {
            for (const std::string &physical :
                 {std::string("CLK"), std::string("CEA"), std::string("CEB"), std::string("CEP"), std::string("RSTA"),
                  std::string("RSTB"), std::string("RSTP")}) {
                std::string logical = physical;
                if (shape->wrapper && physical == "CLK")
                    logical = "C";
                if (shape->wrapper && physical == "CEP")
                    logical = "CE";
                if (shape->wrapper && physical == "RSTP")
                    logical = "R";
                if (ci->getPort(ctx->id(logical)) == nullptr)
                    continue;
                select(physical + "INV", physical + (mult_inverted(ctx, ci, logical) ? "_B" : ""));
            }
        } else if (ci->type == ctx->id("RAM64X1S")) {
            const auto &resources = ram64_resources.at(ram64_by_bel.at(ci->bel.index));
            for (int half = 0; half < 2; ++half) {
                const auto &ref = carry_sites[resources.sites[half]].site;
                auto choose = [&](const std::string &elem, const std::string &cfg) {
                    if (select_site_pip(ref, elem, cfg))
                        ++selected;
                    else
                        ++missing;
                };
                for (const std::string bank : {"F", "G"})
                    for (int i = 1; i <= 4; ++i)
                        choose("W" + bank + std::to_string(i) + "USED", "0");
                choose("CLKINV", "CLK");
                choose("SRINV", "SR");
                choose("BXINV", "BX");
                choose("BYINV", "BY");
                choose("DIF_MUX", "ALTDIF");
                choose("DIG_MUX", half == 0 ? "BY" : "ALTDIG");
                choose("SLICEWE0USED", "0");
                choose("SLICEWE1USED", "0");
                choose("F5USED", "0");
                if (half == 0)
                    choose("DIGUSED", "0");
                else {
                    choose("BYOUTUSED", "0");
                    choose("BYINVOUTUSED", "0");
                    choose("GYMUX", "FX");
                    choose("YUSED", "0");
                }
            }
        } else if (ci->type == ctx->id("RAM32X1S")) {
            for (const std::string bank : {"F", "G"})
                for (int i = 1; i <= 4; ++i)
                    select("W" + bank + std::to_string(i) + "USED", "0");
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
                for (int i = 1; i <= 4; ++i)
                    select("W" + bank + std::to_string(i) + "USED", "0");
            select("CLKINV", "CLK");
            select("SRINV", "SR");
            select("BYINV", "BY");
            select("DIG_MUX", "BY");
            select("DIF_MUX", "ALTDIF");
            select("FXMUX", "F");
            select("GYMUX", "G");
            select("XUSED", "0");
            select("YUSED", "0");
        } else if (is_srl(ctx, ci->type)) {
            select("CLKINV", "CLK");
            select("SRINV", "SR");
            bool local_shift = false;
            bool cross_shift = false;
            NetInfo *data = ci->getPort(id_D);
            if (element == "F" && data != nullptr && data->driver.cell != nullptr &&
                data->driver.port == ctx->id("Q15") && srl_cascade(ctx, data->driver.cell)) {
                CellInfo *source = data->driver.cell;
                auto bound = bel_site_info.find(source->bel.index);
                local_shift = bound != bel_site_info.end() && bound->second.element == "G" &&
                              bound->second.site.tile == site.tile && bound->second.site.site == site.site &&
                              ctx->getBoundBelCell(source->bel) == source && source->getPort(ctx->id("Q15")) == data;
            }
            if (element == "G" && data != nullptr && data->driver.cell != nullptr &&
                data->driver.port == ctx->id("Q15") && srl_cascade(ctx, data->driver.cell)) {
                auto source = carry_bel_slot.find(data->driver.cell->bel.index);
                auto target = carry_bel_slot.find(ci->bel.index);
                cross_shift = source != carry_bel_slot.end() && target != carry_bel_slot.end() &&
                              source->second.second == 0 && srl_next_site[source->second.first] == target->second.first;
                if (cross_shift) {
                    const auto &ref = carry_sites[source->second.first].site;
                    if (select_site_pip(ref, "SHIFTOUTUSED", "0"))
                        ++selected;
                    else
                        ++missing;
                }
            }
            select(element == "F" ? "DIF_MUX" : "DIG_MUX", (local_shift || cross_shift) ? "SHIFTIN"
                                                           : element == "F"             ? "BX"
                                                                                        : "BY");
            if (!local_shift && !cross_shift)
                select(element == "F" ? "BXINV" : "BYINV", element == "F" ? "BX" : "BY");
            select(element == "F" ? "FXMUX" : "GYMUX", element);
            select(element == "F" ? "XUSED" : "YUSED", "0");
            if (srl_cascade(ctx, ci)) {
                select(element == "F" ? "XBMUX" : "YBMUX", "0");
                if (element == "G")
                    select("YBUSED", "0");
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
                if (!tristate)
                    select("O1_DDRMUX", "1");
                select("OTCLK1INV", "OTCLK1");
                if (ci->getPort(id_CE) != nullptr) {
                    select(tristate ? "TCEINV" : "OCEINV", tristate ? "TCE" : "OCE");
                    if (!tristate)
                        select("PCICE_MUX", "OCE");
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
            const auto shape = *ff_shape(ci->type.str(ctx));
            select("CLKINV", ff_clock_inverted(ctx, ci) ? "CLK_B" : "CLK");
            const char *enable = shape.latch ? "GE" : "CE";
            if (ci->getPort(ctx->id(enable)) != nullptr)
                select("CEINV", ff_inverted(ctx, ci, enable) ? "CE_B" : "CE");
            bool has_sr = false;
            for (IdString pin : {ctx->id("R"), id_S, ctx->id("CLR"), ctx->id("PRE")})
                has_sr |= ci->getPort(pin) != nullptr;
            if (has_sr) {
                select("SRINV", shape.sr != nullptr && ff_inverted(ctx, ci, shape.sr) ? "SR_B" : "SR");
                if (db.variants[db.tiles[site.tile].variant].sites[site.site].type == "SLICEM")
                    select("SRFFMUX", "0");
            }
            if (shape.rev != nullptr && ci->getPort(ctx->id(shape.rev)) != nullptr) {
                select("BYINV", ff_inverted(ctx, ci, shape.rev) ? "BY_B" : "BY");
                select("REVUSED", "0");
            }
        } else if (ci->type == id_MUXCY) {
            bool is_g = element == "CYMUXG";
            select(is_g ? "CYSELG" : "CYSELF", is_g ? "G" : "F");
            CarryDi di = carry_di.at(ci->name);
            std::string di_cfg = di == CarryDi::Product    ? "PROD"
                                 : di == CarryDi::LutInput ? (is_g ? "G1" : "F1")
                                 : di == CarryDi::One      ? "1"
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
            if (ci->type != id_BUFG)
                select("I1_USED", "0");
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
            if (iff_bel != iob_if_bels.end())
                iff = ctx->getBoundBelCell(iff_bel->second);
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
                if (off_bel != iob_off_bels.end())
                    off = ctx->getBoundBelCell(off_bel->second);
                select("O1INV", "O1");
                select("OMUX", off == nullptr ? "O1" : "OFF1");
                cfg["O1INV"] = "O1";
                cfg["OMUX"] = off == nullptr ? "O1" : "OFF1";
                if (off != nullptr) {
                    cfg["OFF1"] = "#FF";
                    auto init = off->params.find(ctx->id("INIT"));
                    cfg["OFF1_INIT_ATTR"] =
                            init != off->params.end() && init->second.as_int64() == 1 ? "INIT1" : "INIT0";
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
                    if (tff_bel != iob_tff_bels.end())
                        tff = ctx->getBoundBelCell(tff_bel->second);
                    select("T1INV", "T1");
                    select("TMUX", tff == nullptr ? "T1" : "TFF1");
                    cfg["T1INV"] = "T1";
                    cfg["TMUX"] = tff == nullptr ? "T1" : "TFF1";
                    if (tff != nullptr) {
                        cfg["TFF1"] = "#FF";
                        auto init = tff->params.find(ctx->id("INIT"));
                        cfg["TFF1_INIT_ATTR"] =
                                init != tff->params.end() && init->second.as_int64() == 1 ? "INIT1" : "INIT0";
                        cfg["OTCLK1INV"] = "OTCLK1";
                        if (tff->getPort(id_CE) != nullptr)
                            cfg["TCEINV"] = "TCE";
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
            ci->attrs[ctx->id("S3E_IO_MODE")] = Property(ci->type == id_IBUF    ? "INPUT"
                                                         : ci->type == id_IOBUF ? "BIDIRECTIONAL"
                                                         : ci->type == id_OBUFT ? "TRISTATE"
                                                                                : "OUTPUT");
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
    log_info("Spartan-3E: site pip selection: %zu selected, %zu missing\n", selected, missing);
    if (missing != 0)
        log_error("spartan3e: required site configuration path is missing\n");
}

} // namespace spartan3e_detail

NEXTPNR_NAMESPACE_END
