#include "implementation.h"

#include "constids.h"
#include "version.h"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <unistd.h>

NEXTPNR_NAMESPACE_BEGIN

namespace spartan3e_detail {

namespace {
struct PendingPhysicalOutput
{
    std::string path;
    int fd = -1;
    bool published = false;
    ~PendingPhysicalOutput()
    {
        if (fd >= 0)
            ::close(fd);
        if (!published && !path.empty())
            ::unlink(path.c_str());
    }
};

void publish_physical(const std::string &path, const std::string &bytes)
{
    namespace fs = std::filesystem;
    std::error_code ec;
    auto status = fs::symlink_status(path, ec);
    if (ec && ec != std::errc::no_such_file_or_directory)
        throw std::runtime_error(ec.message());
    if (fs::exists(status) && !fs::is_regular_file(status))
        throw std::runtime_error("output must be a regular file, not a directory or symlink");
    auto parent = fs::path(path).parent_path();
    if (parent.empty())
        parent = ".";
    auto pattern = (parent / ".spark-physical-XXXXXX").string();
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    PendingPhysicalOutput pending;
    pending.fd = ::mkstemp(name.data());
    if (pending.fd < 0)
        throw std::runtime_error(std::strerror(errno));
    pending.path = name.data();
    for (size_t offset = 0; offset < bytes.size();) {
        auto written = ::write(pending.fd, bytes.data() + offset, bytes.size() - offset);
        if (written < 0 && errno == EINTR)
            continue;
        if (written <= 0)
            throw std::runtime_error(written < 0 ? std::strerror(errno) : "short output write");
        offset += size_t(written);
    }
    if (::fsync(pending.fd) != 0)
        throw std::runtime_error(std::strerror(errno));
    int fd = pending.fd;
    pending.fd = -1;
    if (::close(fd) != 0)
        throw std::runtime_error(std::strerror(errno));
    if (::rename(pending.path.c_str(), path.c_str()) != 0)
        throw std::runtime_error(std::strerror(errno));
    pending.published = true;
}
} // namespace

json11::Json Spartan3eImpl::diagnostic_io_route(CellInfo *ci, IdString port)
{
    NetInfo *net = ci->getPort(port);
    if (net == nullptr || net->wires.empty())
        log_error("spartan3e: disconnected I/O fabric port on %s\n", ctx->nameOf(ci));
    json11::Json::array route;
    for (const auto &wire : net->wires) {
        PipId pip = wire.second.pip;
        if (pip == PipId())
            continue;
        auto site = site_route_meta.find(pip.index);
        if (site != site_route_meta.end()) {
            const auto &meta = site->second;
            const auto &conn = meta.connection;
            json11::Json::object entry{{"site", db.tiles[meta.site.tile].sites[meta.site.site]},
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
            route.push_back(json11::Json::object{{"tile", tile.name},
                                                 {"from", db.wires[v.wires[meta.from]]},
                                                 {"to", db.wires[v.wires[meta.to]]},
                                                 {"pip", ctx->getPipName(pip).str(ctx)}});
        }
    }
    return route;
}

std::set<int32_t> Spartan3eImpl::route_subtree(const NetInfo *net, WireId root) const
{
    std::set<int32_t> result{root.index};
    bool changed = true;
    while (changed) {
        changed = false;
        for (const auto &wire : net->wires) {
            if (wire.second.pip == PipId() || result.count(wire.first.index))
                continue;
            if (result.count(ctx->getPipSrcWire(wire.second.pip).index)) {
                result.insert(wire.first.index);
                changed = true;
            }
        }
    }
    return result;
}

json11::Json::object Spartan3eImpl::physical_route_fragment(const NetInfo *net, WireId root,
                                                            const std::set<int32_t> &wires,
                                                            const std::function<json11::Json(WireId)> &endpoint) const
{
    using J = json11::Json;
    J::array pips, pins, routes;
    std::set<std::pair<std::string, std::string>> seen;
    for (const auto &wire : net->wires) {
        if (!wires.count(wire.first.index) || wire.first == root || wire.second.pip == PipId())
            continue;
        PipId pip = wire.second.pip;
        auto internal = site_route_meta.find(pip.index);
        if (internal == site_route_meta.end()) {
            const auto &meta = pip_meta.at(pip.index);
            const auto &tile = db.tiles[meta.tile];
            const auto &variant = db.variants[tile.variant];
            std::string op;
            for (const auto &row : variant.pips)
                if (row.from == meta.from && row.to == meta.to && row.rt == -1) {
                    op = db.pip_ops[row.op];
                    break;
                }
            if (op.empty())
                log_error("spartan3e: unknown component pip operator\n");
            pips.push_back(J::object{{"tile", tile.name},
                                     {"from", db.wires[variant.wires[meta.from]]},
                                     {"to", db.wires[variant.wires[meta.to]]},
                                     {"op", op}});
            continue;
        }
        const auto &meta = internal->second;
        const auto &tile = db.tiles[meta.site.tile];
        const auto &def = db.variants[tile.variant].sites[meta.site.site];
        const auto &conn = meta.connection;
        J selection;
        if (meta.selection)
            selection = J::object{{"element", meta.selection->element}, {"value", meta.selection->cfg}};
        routes.push_back(J::object{{"site", tile.sites[meta.site.site]},
                                   {"from", J::array{conn.src_el, conn.src_pin}},
                                   {"to", J::array{conn.dst_el, conn.dst_pin}},
                                   {"selection", selection}});
        for (const auto &element : {conn.src_el, conn.dst_el}) {
            if (!def.pins.count(element) || !seen.emplace(tile.sites[meta.site.site], element).second)
                continue;
            std::string direction;
            for (const auto &pin : db.site_defs.at(def.type).pins)
                if (pin.first == element)
                    direction = pin.second;
            if (direction.empty())
                log_error("spartan3e: unknown component boundary pin\n");
            pins.push_back(J::object{{"site", tile.sites[meta.site.site]}, {"pin", element}, {"direction", direction}});
        }
    }
    return {{"root", endpoint(root)}, {"pins", pins}, {"pips", pips}, {"site_routes", routes}};
}

void Spartan3eImpl::write_physical_design()
{
    using J = json11::Json;
    // Resolve endpoint identity from typed wire metadata, never BEL names.
    std::map<int, SiteElemKey> endpoint_by_wire;
    for (const auto &entry : site_wire)
        if (!endpoint_by_wire.emplace(entry.second.index, entry.first).second)
            log_error("spartan3e: ambiguous typed site endpoint\n");
    auto endpoint = [&](WireId wire) -> J {
        auto it = endpoint_by_wire.find(wire.index);
        if (it == endpoint_by_wire.end())
            log_error("spartan3e: cannot resolve typed endpoint wire\n");
        const auto &key = it->second;
        return J::object{{"site", db.tiles[key.tile].sites[key.site]}, {"element", key.element}, {"pin", key.pin}};
    };
    std::map<std::pair<int, int>, J::array> cells;
    for (const auto &item : ctx->cells) {
        const CellInfo *cell = item.second.get();
        auto placed = bel_site_info.find(cell->bel.index);
        if (placed == bel_site_info.end())
            log_error("spartan3e: cannot export unplaced/unknown cell %s\n", ctx->nameOf(cell));
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
        J::object exported{{"name", cell->name.str(ctx)},
                           {"type", cell->type.str(ctx)},
                           {"element", placed->second.element},
                           {"params", params},
                           {"attrs", attrs},
                           {"ports", ports}};
        auto memory = ram64_by_bel.find(cell->bel.index);
        if (memory != ram64_by_bel.end()) {
            J::array footprint, components;
            const auto &resources = ram64_resources[memory->second];
            for (int half = 0; half < 2; ++half) {
                const auto &ref = carry_sites[resources.sites[half]].site;
                footprint.push_back(
                        J::object{{"site", db.tiles[ref.tile].sites[ref.site]}, {"address_base", half * 32}});
                const std::string prefix = half == 0 ? "L" : "H";
                const NetInfo *net = cell->getPort(ctx->id("$RAM64_F5_" + prefix));
                WireId root = ctx->getNetinfoSourceWire(net);
                auto component = physical_route_fragment(net, root, route_subtree(net, root), endpoint);
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
            if (ctx->getBoundPipNet(PipId(index)) == nullptr && key.element != "CY0F" && key.element != "CY0G")
                continue;
            if (key.tile == site.first && key.site == site.second)
                selections.push_back(J::object{{"element", key.element}, {"value", key.cfg}});
        }
        sites.push_back(J::object{{"site", tile.sites[site.second]},
                                  {"tile", tile.name},
                                  {"type", db.variants[tile.variant].sites[site.second].type},
                                  {"cells", cells[site]},
                                  {"site_pips", selections}});
    }
    for (const auto &item : ctx->nets) {
        const NetInfo *net = item.second.get();
        if (net->driver.cell == nullptr || net->wires.empty())
            continue;
        if (net->driver.port.str(ctx).find("$RAM64_") == 0)
            continue;
        // Package-side nets have no fabric representation.
        if (net->driver.cell->type == id_OBUF && net->driver.port == id_O)
            continue;
        J::array pips, pins, sinks, site_routes, branches;
        std::set<int32_t> branch_wires;
        for (const auto &memory : ram64_resources) {
            CellInfo *owner = ctx->getBoundBelCell(memory.composite);
            if (owner == nullptr)
                continue;
            const auto &low = carry_sites[memory.sites[0]].site;
            const auto &high = carry_sites[memory.sites[1]].site;
            struct Branch
            {
                SiteRef site;
                const char *port;
                const char *kind;
                const char *transform;
                const char *source_element;
                const char *source_pin;
                const char *root_element;
            };
            const std::array<Branch, 3> definitions{
                    {{high, "A5", "ram64_byout", "identity", "BYINV", "BY", "BYOUTUSED"},
                     {high, "A5", "ram64_byinvout", "inverted", "BYINV", "BY", "BYINVOUTUSED"},
                     {low, "D", "ram64_dig", "identity", "DIG_MUX", "BY", "DIGUSED"}}};
            for (const auto &definition : definitions) {
                if (owner->getPort(ctx->id(definition.port)) != net)
                    continue;
                WireId root = site_wire.at(
                        SiteElemKey{definition.site.tile, definition.site.site, definition.root_element, "OUT"});
                if (!net->wires.count(root))
                    log_error("spartan3e: RAM64 physical branch root is unrouted\n");
                auto wires = route_subtree(net, root);
                auto fragment = physical_route_fragment(net, root, wires, endpoint);
                J::array branch_sinks;
                for (const PortRef &user : net->users)
                    for (WireId wire : ctx->getNetinfoSinkWires(net, user))
                        if (wires.count(wire.index))
                            branch_sinks.push_back(J::object{{"cell", user.cell->name.str(ctx)},
                                                             {"port", user.port.str(ctx)},
                                                             {"endpoint", endpoint(wire)}});
                fragment["owner"] = owner->name.str(ctx);
                fragment["port"] = definition.port;
                fragment["kind"] = definition.kind;
                fragment["transform"] = definition.transform;
                fragment["source"] = endpoint(site_wire.at(SiteElemKey{
                        definition.site.tile, definition.site.site, definition.source_element, definition.source_pin}));
                fragment["sinks"] = branch_sinks;
                branches.push_back(fragment);
                branch_wires.insert(wires.begin(), wires.end());
            }
        }
        for (const PortRef &user : net->users) {
            auto wires = ctx->getNetinfoSinkWires(net, user);
            if (wires.size() == 0)
                log_error("spartan3e: no physical sinks for %s.%s\n", ctx->nameOf(user.cell), user.port.c_str(ctx));
            for (WireId wire : wires)
                if (!branch_wires.count(wire.index))
                    sinks.push_back(J::object{{"cell", user.cell->name.str(ctx)},
                                              {"port", user.port.str(ctx)},
                                              {"endpoint", endpoint(wire)}});
        }
        std::set<std::pair<std::string, std::string>> seen_pins;
        for (const auto &wire : net->wires) {
            if (branch_wires.count(wire.first.index))
                continue;
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
                    log_error("spartan3e: cannot resolve exported pip operator\n");
                pips.push_back(J::object{{"tile", tile.name},
                                         {"from", db.wires[v.wires[meta.from]]},
                                         {"to", db.wires[v.wires[meta.to]]},
                                         {"op", op}});
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
                                            {"to", J::array{conn.dst_el, conn.dst_pin}},
                                            {"selection", selection}});
            for (const auto &element : {conn.src_el, conn.dst_el}) {
                if (!def.pins.count(element) || !seen_pins.emplace(tile.sites[meta.site.site], element).second)
                    continue;
                std::string direction;
                for (const auto &pin : db.site_defs.at(def.type).pins)
                    if (pin.first == element)
                        direction = pin.second;
                if (direction.empty())
                    log_error("spartan3e: missing direction for site pin %s\n", element.c_str());
                pins.push_back(
                        J::object{{"site", tile.sites[meta.site.site]}, {"pin", element}, {"direction", direction}});
            }
        }
        const CellInfo *driver = net->driver.cell;
        const auto &placed = bel_site_info.at(driver->bel.index);
        WireId source = ctx->getNetinfoSourceWire(net);
        J root = endpoint(source);
        J::object exported_net{{"name", net->name.str(ctx)},
                               {"driver", driver->name.str(ctx)},
                               {"driver_port", net->driver.port.str(ctx)},
                               {"source_site", db.tiles[placed.site.tile].sites[placed.site.site]},
                               {"root", root},
                               {"pins", pins},
                               {"pips", pips},
                               {"sinks", sinks},
                               {"site_routes", site_routes}};
        if (has_ram64)
            exported_net["branches"] = branches;
        nets.push_back(exported_net);
    }
    J::object settings;
    for (const auto &setting : ctx->settings)
        settings[setting.first.str(ctx)] = J::object{{"kind", setting.second.is_string ? "string" : "bits"},
                                                     {"value", setting.second.is_string ? setting.second.str : setting.second.to_string()}};
    J physical_design = J::object{{"format", has_ram64 ? "spark-physical/3" : "spark-physical/2"},
                                  {"part", db.part},
                                  {"package", db.package},
                                  {"top", ctx->top_module.str(ctx)},
                                  {"sites", sites},
                                  {"nets", nets},
                                  {"provenance", J::object{{"format", "spark-implementation/1"},
                                      {"producer", J::object{{"name", "nextpnr-spartan3e"}, {"version", GIT_DESCRIBE_STR}}},
                                      {"settings", settings}}}};
    try {
        publish_physical(physical_path, physical_design.dump() + '\n');
    } catch (const std::exception &e) {
        log_error("spartan3e: cannot publish physical design %s: %s\n", physical_path.c_str(), e.what());
    }
}

} // namespace spartan3e_detail

NEXTPNR_NAMESPACE_END
