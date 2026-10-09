#include "implementation.h"

#include "constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace spartan3e_detail {

bool Spartan3eImpl::is_io_site_type(const std::string &type)
{
    // The input-only pad site type is distinct from the IBUF cell primitive.
    return type == "IOB" || type == "DIFFM" || type == "DIFFS" || type == "DIFFMI" || type == "DIFFSI" ||
           type == "IBUF";
}

bool Spartan3eImpl::is_lut_cell(IdString t) { return t.in(id_LUT1, id_LUT2, id_LUT3, id_LUT4); }

bool Spartan3eImpl::is_xilinx_ff_cell(IdString t) const { return ff_shape(t.str(ctx)).has_value(); }

bool Spartan3eImpl::is_xilinx_io_cell(IdString t) { return t.in(id_IBUF, id_OBUF, id_OBUFT, id_IOBUF); }

void Spartan3eImpl::pack_io_constraints()
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
        if (buffer == nullptr || (bidirectional ? buffer->type != id_IOBUF
                                  : input       ? buffer->type != id_IBUF
                                                : !buffer->type.in(id_OBUF, id_OBUFT)))
            log_error("spartan3e: port %s requires a top-level IBUF/OBUF/OBUFT/IOBUF\n", ctx->nameOf(port));
        for (const char *key :
             {"LOC", "PACKAGE_PIN", "BEL", "IOSTANDARD", "DRIVE", "SLEW", "PULL", "PULLUP", "PULLDOWN", "KEEPER",
              "DIFF_TERM", "IBUF_LOW_PWR", "IOB", "IN_TERM", "OUT_TERM", "IBUF_DELAY_VALUE", "IFD_DELAY_VALUE"}) {
            IdString attr = ctx->id(key);
            auto it = port->attrs.find(attr);
            if (it == port->attrs.end())
                continue;
            auto existing = buffer->attrs.find(attr);
            if (existing != buffer->attrs.end() && existing->second != it->second)
                log_error("spartan3e: conflicting %s on port %s and buffer\n", key, ctx->nameOf(port));
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
            log_error("spartan3e: only top-level I/O buffers are supported\n");
        auto value = [&](const char *key, const std::string &fallback) {
            auto it = ci->attrs.find(ctx->id(key));
            if (it == ci->attrs.end())
                return fallback;
            return it->second.is_string ? it->second.as_string() : std::to_string(it->second.as_int64());
        };
        if (value("IOSTANDARD", "LVCMOS33") != "LVCMOS33")
            log_error("spartan3e: unsupported IOSTANDARD on %s (supported: LVCMOS33)\n", ctx->nameOf(ci));
        const std::set<std::string> drives = {"2", "4", "6", "8", "12", "16"};
        if (!drives.count(value("DRIVE", "12")))
            log_error("spartan3e: unsupported LVCMOS33 DRIVE on %s (supported: 2/4/6/8/12/16)\n", ctx->nameOf(ci));
        if (value("SLEW", "SLOW") != "SLOW" && value("SLEW", "SLOW") != "FAST")
            log_error("spartan3e: unsupported SLEW on %s (supported: SLOW/FAST)\n", ctx->nameOf(ci));
        for (const char *key :
             {"PULL", "DIFF_TERM", "IBUF_LOW_PWR", "IOB", "IN_TERM", "OUT_TERM", "IBUF_DELAY_VALUE", "IFD_DELAY_VALUE"})
            if (ci->attrs.count(ctx->id(key)) || ci->params.count(ctx->id(key)))
                log_error("spartan3e: unsupported I/O attribute %s on %s\n", key, ctx->nameOf(ci));
        std::string pull = "#OFF";
        for (const char *key : {"PULLUP", "PULLDOWN", "KEEPER"}) {
            if (ci->params.count(ctx->id(key)))
                log_error("spartan3e: use port/cell attributes for %s, not parameters\n", key);
            if (!ci->attrs.count(ctx->id(key)))
                continue;
            std::string flag = value(key, "");
            if (flag != "TRUE" && flag != "FALSE" && flag != "1" && flag != "0")
                log_error("spartan3e: %s must be TRUE/FALSE or 1/0 on %s\n", key, ctx->nameOf(ci));
            if (flag == "TRUE" || flag == "1") {
                if (pull != "#OFF")
                    log_error("spartan3e: conflicting I/O pull attributes on %s\n", ctx->nameOf(ci));
                pull = key;
            }
        }
        // PULL is an internal selection; user constraints use named pull attributes.
        ci->attrs[ctx->id("PULL")] = Property(pull);
        // Attribute and parameter forms must not silently disagree.
        for (const char *key : {"IOSTANDARD", "DRIVE", "SLEW"}) {
            IdString attr = ctx->id(key);
            if (ci->params.count(attr))
                log_error("spartan3e: use port/cell attributes for %s, not parameters\n", key);
            ci->attrs[attr] = Property(value(key, std::string(key) == "IOSTANDARD" ? "LVCMOS33"
                                                  : std::string(key) == "DRIVE"    ? "12"
                                                                                   : "SLOW"));
        }
        std::string loc = value("LOC", ""), pin = value("PACKAGE_PIN", "");
        if (!loc.empty() && !pin.empty() && loc != pin)
            log_error("spartan3e: conflicting LOC/PACKAGE_PIN on %s\n", ctx->nameOf(ci));
        if (pin.empty())
            pin = loc;
        BelId bel;
        if (!pin.empty()) {
            auto it = package_bels.find(pin);
            if (it == package_bels.end())
                log_error("spartan3e: pin %s is not a bonded user I/O in package %s\n", pin.c_str(),
                          db.package.c_str());
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
                log_error("spartan3e: invalid or conflicting I/O BEL on %s\n", ctx->nameOf(ci));
            bel = named;
        }
        if (bel == BelId())
            continue;
        if (!isValidBelForCellType(ci->type, bel))
            log_error("spartan3e: output %s cannot use input-only pin %s\n", ctx->nameOf(ci),
                      bel_package_pin.at(bel.index).c_str());
        if (!constrained.emplace(bel.index, ci).second)
            log_error("spartan3e: duplicate I/O pin binding for %s\n", bel_package_pin.at(bel.index).c_str());
        ci->attrs[ctx->id("BEL")] = Property(ctx->getBelName(bel).str(ctx));
    }
}

void Spartan3eImpl::inherit_iob_register_attrs(CellInfo *ff)
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
            log_error("spartan3e: conflicting cell/Q-net %s on register %s\n", key, ctx->nameOf(ff));
        ff->attrs[id] = net_attr->second;
    }
}

void Spartan3eImpl::pack_iob_off()
{
    for (auto &entry : ctx->cells) {
        CellInfo *ff = entry.second.get();
        if (!is_xilinx_ff_cell(ff->type))
            continue;
        inherit_iob_register_attrs(ff);
        auto request = ff->attrs.find(ctx->id("IOB"));
        if (request == ff->attrs.end() || request->second.as_string() != "TRUE")
            continue;
        if (!legacy_iob_ff(ctx, ff))
            log_error("spartan3e: IOB=TRUE does not support extended FF polarity or dual controls\n");
        NetInfo *q = ff->getPort(id_Q);
        CellInfo *buffer = q != nullptr && q->users.entries() == 1 ? (*q->users.begin()).cell : nullptr;
        bool tristate =
                buffer != nullptr && buffer->type.in(id_OBUFT, id_IOBUF) && (*q->users.begin()).port == ctx->id("T");
        bool input = !tristate && (buffer == nullptr || !buffer->type.in(id_OBUF, id_OBUFT, id_IOBUF) ||
                                   (*q->users.begin()).port != id_I);
        auto kind = ff->attrs.find(ctx->id("S3E_IOB_KIND"));
        if (kind != ff->attrs.end()) {
            std::string role = kind->second.as_string();
            if (role != "IFF1" && role != "OFF1" && role != "TFF1")
                log_error("spartan3e: unsupported S3E_IOB_KIND\n");
            if (role == "OFF1" && (input || tristate))
                log_error("spartan3e: requested OFF1 lacks a dedicated output-buffer consumer\n");
            if (role == "TFF1" && !tristate)
                log_error("spartan3e: requested TFF1 lacks a dedicated tristate-buffer consumer\n");
            tristate = role == "TFF1";
            input = role == "IFF1";
        }
        if (input) {
            NetInfo *d = ff->getPort(id_D);
            buffer = d != nullptr && d->driver.cell != nullptr && d->driver.cell->type.in(id_IBUF, id_IOBUF) &&
                                     d->driver.port == id_O && d->users.entries() == 1
                             ? d->driver.cell
                             : nullptr;
            if (buffer == nullptr)
                log_error("spartan3e: IOB=TRUE requires a dedicated input/output buffer\n");
            if (!ff->type.in(id_FDCE, id_FDPE))
                log_error("spartan3e: IFF1 currently requires FDCE/FDPE\n");
        }
        auto &clusters = input ? iob_if_clusters : tristate ? iob_tff_clusters : iob_off_clusters;
        if (ff->cluster != ClusterId() || clusters.count(buffer->name) ||
            (buffer->cluster != ClusterId() &&
             (buffer->cluster != buffer->name ||
              (!iob_off_clusters.count(buffer->name) && !iob_if_clusters.count(buffer->name) &&
               !iob_tff_clusters.count(buffer->name)))))
            log_error("spartan3e: IOB register conflicts with another cluster\n");
        for (const auto *other_clusters : {&iob_if_clusters, &iob_off_clusters, &iob_tff_clusters}) {
            auto other = other_clusters->find(buffer->name);
            if (other == other_clusters->end())
                continue;
            auto sr = [&](CellInfo *cell) -> NetInfo * {
                for (IdString port : {ctx->id("R"), id_S, ctx->id("CLR"), ctx->id("PRE")})
                    if (cell->getPort(port) != nullptr)
                        return cell->getPort(port);
                return nullptr;
            };
            if (sr(ff) != nullptr && sr(other->second) != nullptr && sr(ff) != sr(other->second))
                log_error("spartan3e: IOB registers share one physical SR input\n");
            if (!input && other_clusters != &iob_if_clusters &&
                ff->getPort(ctx->id("C")) != other->second->getPort(ctx->id("C")))
                log_error("spartan3e: OFF1/TFF1 share one physical output clock\n");
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
            if (target == register_bels.end())
                log_error("spartan3e: constrained pad lacks requested IOB register\n");
            auto ff_fixed = ff->attrs.find(ctx->id("BEL"));
            if (ff_fixed != ff->attrs.end() && ctx->getBelByNameStr(ff_fixed->second.as_string()) != target->second)
                log_error("spartan3e: conflicting IOB output-register BEL\n");
            // HeAP excludes a fixed root from cluster placement. Fix its
            // child too, so no unplaced register reaches its cost model.
            ff->attrs[ctx->id("BEL")] = Property(ctx->getBelName(target->second).str(ctx));
        } else if (ff->attrs.count(ctx->id("BEL"))) {
            log_error("spartan3e: IOB output-register BEL requires a constrained output pad\n");
        }
    }
}

void Spartan3eImpl::remove_transparent_buffers()
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
                log_error("spartan3e: malformed transparent-buffer output\n");
            IdString input = ctx->id("A" + output.substr(1));
            if (!cell->ports.count(input))
                log_error("spartan3e: transparent buffer lacks matching input\n");
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
                log_error("spartan3e: transparent-buffer combinational loop\n");
            for (const auto &attr : y->attrs) {
                auto existing = a->attrs.find(attr.first);
                const std::string key = attr.first.str(ctx);
                bool request = key == "IOB" || key == "S3E_IOB_KIND" || key == "LOC" || key == "PACKAGE_PIN" ||
                               key == "BEL" || key == "IOSTANDARD" || key == "DRIVE" || key == "SLEW" ||
                               key == "PULL" || key == "PULLUP" || key == "PULLDOWN" || key == "KEEPER";
                if (request && existing != a->attrs.end() && existing->second != attr.second)
                    log_error("spartan3e: conflicting %s across transparent buffer\n", key.c_str());
                if (existing == a->attrs.end())
                    a->attrs[attr.first] = attr.second;
            }
            if (a->clkconstr && y->clkconstr)
                log_error("spartan3e: conflicting clock constraints across transparent buffer\n");
            if (!a->clkconstr)
                a->clkconstr = std::move(y->clkconstr);
            if (a->region && y->region && a->region != y->region)
                log_error("spartan3e: conflicting regions across transparent buffer\n");
            if (!a->region)
                a->region = y->region;
            if (a->constant_value != IdString() && y->constant_value != IdString() &&
                a->constant_value != y->constant_value)
                log_error("spartan3e: conflicting constants across transparent buffer\n");
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

void Spartan3eImpl::normalize_ibufg()
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
            log_error("spartan3e: IBUFG currently requires a BUFG/BUFGMUX input consumer\n");
        // Same single-ended pad circuitry as IBUF. Preserve identity and
        // requests; the clock model still requires a dedicated pad path.
        cell->type = id_IBUF;
    }
}

void Spartan3eImpl::pack()
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
    fold_bram_control_inverters(ctx);

    // ISE #LUT expressions expose only truth-table-dependent input pins.
    // Kept constant leaves must not export fabric nets to unused F/G pins.
    for (auto &entry : ctx->cells) {
        CellInfo *cell = entry.second.get();
        if (!is_lut_cell(cell->type))
            continue;
        int width = cell->type.str(ctx).back() - '0';
        auto init = cell->params.find(id_INIT);
        if (init == cell->params.end())
            log_error("spartan3e: LUT %s has no INIT\n", ctx->nameOf(cell));
        uint64_t truth = uint64_t(init->second.as_int64());
        for (int input = 0; input < width; ++input) {
            bool dependent = false;
            for (int address = 0; address < (1 << width); ++address)
                dependent |= ((truth >> address) & 1) != ((truth >> (address ^ (1 << input))) & 1);
            IdString pin = ctx->idf("I%d", input);
            if (!dependent && cell->ports.count(pin))
                cell->disconnectPort(pin);
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
        if (!is_srl(ctx, cell->type))
            continue;
        validate_srl(ctx, cell);
        if (cell->type.in(ctx->id("SRL16"), ctx->id("SRLC16")))
            always_enabled_srls.push_back(cell);
    }
    if (!always_enabled_srls.empty()) {
        CellInfo *one = ctx->createCell(ctx->id("$SRL_ENABLE"), id_LUT4);
        one->params[id_INIT] = Property(0xffff, 16);
        one->addOutput(id_O);
        NetInfo *enable = ctx->createNet(ctx->id("$SRL_ENABLE_NET"));
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
        if (mult_type(ctx, entry.second->type))
            remap_mult(ctx, entry.second.get());
    for (auto &entry : ctx->cells)
        if (is_xilinx_ff_cell(entry.second->type)) {
            validate_ff(ctx, entry.second.get());
            ff_sink_ports.insert(CellTypePort(entry.second->type, id_D));
        }

    wide_trees = pack_wide(ctx);
    mult_chains = pack_mult_chains(ctx);
    for (size_t i = 0; i < mult_chains.size(); ++i) {
        const auto &chain = mult_chains[i];
        mult_cluster[chain.root->name] = i;
        std::vector<std::pair<CellInfo *, BelId>> placement;
        bool legal = false;
        for (const auto &site : mult_sites)
            if (place_mult_chain(chain, site.bel, placement)) {
                legal = true;
                break;
            }
        if (!legal)
            log_error("spartan3e: MULT cascade has no legal dedicated footprint (fixed adjacency or occupancy "
                      "conflict)\n");
        if (chain.root->attrs.count(ctx->id("BEL")))
            fix_cluster_placement(placement);
    }
    clock_model->pack();
    pack_iob_off();
    // A non-inverted literal-low SR does not consume slice SR. Preserve
    // FF identity/type/CE; IOB controls retain their separate contract.
    for (auto &entry : ctx->cells) {
        CellInfo *ff = entry.second.get();
        if (iob_off_clusters.count(ff->cluster) || iob_if_clusters.count(ff->cluster) ||
            iob_tff_clusters.count(ff->cluster))
            continue;
        disconnect_inactive_ff_sr(ctx, ff, [&](IdString type) { return is_lut_cell(type); });
    }
    srl_chains = pack_srl_chains(ctx);
    for (size_t i = 0; i < srl_chains.size(); ++i) {
        const auto &chain = srl_chains[i];
        srl_cluster[chain.root->name] = i;
        std::vector<std::pair<CellInfo *, BelId>> placement;
        bool legal = false;
        for (const auto &site : carry_sites)
            if (place_srl_chain(chain, site.lut[1], placement)) {
                legal = true;
                break;
            }
        if (!legal)
            log_error(
                    "spartan3e: SRL chain has no legal dedicated footprint (fixed adjacency or occupancy conflict)\n");
        if (chain.root->attrs.count(ctx->id("BEL")))
            fix_cluster_placement(placement);
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
            log_error("spartan3e: wide DAG %s has no legal dedicated footprint (sharing or fixed BEL conflict)\n",
                      wide_trees[i].root->name.c_str(ctx));
        if (wide_trees[i].root->attrs.count(ctx->id("BEL")))
            fix_cluster_placement(placement);
    }
    int pairs = h.constrain_cell_pairs(lut_src_ports, ff_sink_ports, 1);
    log_info("Spartan-3E: constrained %d LUT/DFF pairs\n", pairs);

    // synth_xilinx IO buffers use Xilinx port names; the physical IOB bel
    // exposes both fabric and pad pins, so remap the pad-side ports.
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_IBUF)
            ci->bel_pins[id_I] = {id_IO};
        else if (ci->type == id_OBUF || ci->type == id_OBUFT)
            ci->bel_pins[id_O] = {id_IO};
        else if (ci->type == ctx->id("RAM32X1S"))
            remap_ram32(ctx, ci);
        else if (ci->type == ctx->id("RAM64X1S"))
            remap_ram64(ctx, ci);
        else if (ci->type == ctx->id("RAM16X1D"))
            remap_ram16_dual(ctx, ci);
        else if (ci->type == ctx->id("RAM16X1S"))
            for (int i = 0; i < 4; ++i)
                ci->bel_pins[ctx->idf("A%d", i)] = {ctx->idf("A%d", i), ctx->idf("WA%d", i)};
        else if (auto shape = ff_shape(ci->type.str(ctx))) {
            if (shape->latch) {
                ci->bel_pins[ctx->id("G")] = {ctx->id("C")};
                ci->bel_pins[ctx->id("GE")] = {ctx->id("CE")};
            }
            if (shape->rev != nullptr)
                ci->bel_pins[ctx->id(shape->rev)] = {ctx->id(shape->sr_high ? "REV_R" : "REV_S")};
        } else if (auto shape = bram_shape(ci->type.str(ctx)))
            remap_bram_ports(ctx, ci, *shape);
    }

    carry_chains = pack_carries(ctx);
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
            if (!legal.value_or(false))
                log_error("spartan3e: invalid fixed carry footprint\n");
            fix_cluster_placement(placement);
        }
    }
    constrain_ff_locations();
}

void Spartan3eImpl::constrain_ff_locations()
{
    // LOC selects a physical slice, not one arbitrary FFX/FFY BEL. Give
    // HeAP the actual search region as well as the legality predicate.
    // Otherwise a valid isolated FF can exhaust random placement attempts.
    for (auto &entry : ctx->cells) {
        CellInfo *cell = entry.second.get();
        if (!ff_shape(cell->type.str(ctx)))
            continue;
        auto loc = cell->attrs.find(ctx->id("LOC"));
        if (loc == cell->attrs.end())
            continue;
        std::string site = loc->second.as_string();
        if (site.compare(0, 6, "SLICE_") != 0)
            continue;
        auto region = cell->region == nullptr ? std::make_unique<Region>() : std::make_unique<Region>(*cell->region);
        region->name = ctx->id("$FF_LOC_" + cell->name.str(ctx));
        region->constr_bels = true;
        region->bels.clear();
        for (const auto &bel : bel_site_info) {
            if (bel.second.element != "FFX" && bel.second.element != "FFY")
                continue;
            const auto &ref = bel.second.site;
            BelId candidate{bel.first};
            if (db.tiles[ref.tile].sites[ref.site] == site && cell->testRegion(candidate))
                region->bels.insert(candidate);
        }
        if (region->bels.empty())
            log_error("spartan3e: FF LOC has no available slice BEL: %s (%s)\n", ctx->nameOf(cell), site.c_str());
        if (ctx->region.count(region->name))
            log_error("spartan3e: duplicate FF LOC region for %s\n", ctx->nameOf(cell));
        cell->region = region.get();
        ctx->region[cell->region->name] = std::move(region);
    }
}

void Spartan3eImpl::fix_cluster_placement(const std::vector<std::pair<CellInfo *, BelId>> &placement)
{
    // HeAP does not place children of a fixed root. Pin the entire actual
    // footprint, preserving every pre-existing child BEL constraint.
    for (const auto &entry : placement) {
        std::string name = ctx->getBelName(entry.second).str(ctx);
        auto fixed = entry.first->attrs.find(ctx->id("BEL"));
        if (fixed != entry.first->attrs.end() && fixed->second.as_string() != name)
            log_error("spartan3e: conflicting fixed cluster BEL for %s\n", ctx->nameOf(entry.first));
        entry.first->attrs[ctx->id("BEL")] = Property(name);
    }
}

} // namespace spartan3e_detail

NEXTPNR_NAMESPACE_END
