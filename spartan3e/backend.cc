/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2021  gatecat <gatecat@ds0.me>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "implementation.h"

#define GEN_INIT_CONSTIDS
#include "constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace spartan3e_detail {

Spartan3eImpl::Spartan3eImpl(std::string path, std::string physical_design)
        : db_path(std::move(path)), physical_path(std::move(physical_design))
{
}

void Spartan3eImpl::init(Context *ctx)
{
    init_constids(ctx);
    DeviceBackend::init(ctx);
    h.init(ctx);
    db = parse_chipdb(db_path);
    index_sites();
    build_wires();
    build_pips();
    log_info("Spartan-3E: loaded '%s': tiles=%zu nodes=%d multi_nodes=%zu wires=%d pips=%d members=%zu\n",
             db_path.c_str(), db.tiles.size(), ctx->getWires().size, db.nodes.size(), ctx->getWires().size,
             ctx->getPips().size, member_wire.size());
    init_bels();
    index_carry_topology();
    index_wide_topology();
    index_srl_topology();
    index_mult_topology();
    init_ram64_bels();
    index_clock_topology();
    site_by_name.clear();
}

bool Spartan3eImpl::checkPipAvail(PipId pip) const
{
    auto it = site_pip_by_pip.find(pip.index);
    if (it == site_pip_by_pip.end())
        return true;
    return selected_site_pips.count(pip.index) > 0;
}

bool Spartan3eImpl::checkPipAvailForNet(PipId pip, const NetInfo *net) const
{
    if (!checkPipAvail(pip))
        return false;
    if (site_route_meta.count(pip.index))
        return true;
    return clock_model->pip_available(pip, net);
}

void Spartan3eImpl::postRoute()
{
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (!is_xilinx_io_cell(ci->type))
            continue;
        ci->attrs[ctx->id("S3E_IO_ROUTE")] =
                Property(diagnostic_io_route(ci, ci->type == id_IBUF ? id_O : id_I).dump());
        if (ci->type == id_IOBUF)
            ci->attrs[ctx->id("S3E_IO_INPUT_ROUTE")] = Property(diagnostic_io_route(ci, id_O).dump());
        if (ci->type.in(id_OBUFT, id_IOBUF))
            ci->attrs[ctx->id("S3E_IO_T_ROUTE")] = Property(diagnostic_io_route(ci, ctx->id("T")).dump());
    }
    if (!physical_path.empty())
        write_physical_design();
}

void Spartan3eImpl::postPlace()
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

void Spartan3eImpl::preRoute()
{
    clock_model->pre_route([&](PipId pip) { return site_route_meta.count(pip.index) != 0; },
                           [&](PipId pip) { return checkPipAvail(pip); });
}

} // namespace spartan3e_detail

std::unique_ptr<DeviceBackend> make_backend(const std::string &db_path, const std::string &physical_path)
{
    return std::make_unique<spartan3e_detail::Spartan3eImpl>(db_path, physical_path);
}

// Default device policy and routing lookahead.

void DeviceBackend::init(Context *ctx) { this->ctx = ctx; }

std::vector<IdString> DeviceBackend::getCellTypes() const
{
    pool<IdString> cell_types;
    for (auto bel : ctx->bels) {
        cell_types.insert(bel.type);
    }

    return std::vector<IdString>{cell_types.begin(), cell_types.end()};
}
BelBucketId DeviceBackend::getBelBucketForBel(BelId bel) const { return ctx->getBelType(bel); }
BelBucketId DeviceBackend::getBelBucketForCellType(IdString cell_type) const { return cell_type; }
bool DeviceBackend::isValidBelForCellType(IdString cell_type, BelId bel) const
{
    return ctx->getBelType(bel) == cell_type;
}

delay_t DeviceBackend::estimateDelay(WireId src, WireId dst) const
{
    const WireInfo &s = ctx->wire_info(src);
    const WireInfo &d = ctx->wire_info(dst);
    int dx = abs(s.x - d.x);
    int dy = abs(s.y - d.y);
    return (dx + dy) * ctx->args.delayScale + ctx->args.delayOffset;
}
delay_t DeviceBackend::predictDelay(BelId src_bel, IdString src_pin, BelId dst_bel, IdString dst_pin) const
{
    NPNR_UNUSED(src_pin);
    NPNR_UNUSED(dst_pin);
    auto driver_loc = ctx->getBelLocation(src_bel);
    auto sink_loc = ctx->getBelLocation(dst_bel);

    int dx = abs(sink_loc.x - driver_loc.x);
    int dy = abs(sink_loc.y - driver_loc.y);
    return (dx + dy) * ctx->args.delayScale + ctx->args.delayOffset;
}
BoundingBox DeviceBackend::getRouteBoundingBox(WireId src, WireId dst) const
{
    BoundingBox bb;
    int src_x = ctx->wire_info(src).x;
    int src_y = ctx->wire_info(src).y;
    int dst_x = ctx->wire_info(dst).x;
    int dst_y = ctx->wire_info(dst).y;

    bb.x0 = src_x;
    bb.y0 = src_y;
    bb.x1 = src_x;
    bb.y1 = src_y;

    auto extend = [&](int x, int y) {
        bb.x0 = std::min(bb.x0, x);
        bb.x1 = std::max(bb.x1, x);
        bb.y0 = std::min(bb.y0, y);
        bb.y1 = std::max(bb.y1, y);
    };
    extend(dst_x, dst_y);
    return bb;
}

NEXTPNR_NAMESPACE_END
