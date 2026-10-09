/* Primitive BRAM port policy; physical routing remains sourced from primitive_defs. */
#include "bram.h"
#include "log.h"

NEXTPNR_NAMESPACE_BEGIN

void fold_bram_control_inverters(Context *ctx)
{
    std::vector<IdString> removed;
    for (auto &entry : ctx->cells) {
        CellInfo *bram = entry.second.get();
        auto shape = bram_shape(bram->type.str(ctx));
        if (!shape)
            continue;
        for (const auto &attr : bram->attrs)
            if (attr.first.str(ctx).compare(0, 21, "S3E_BRAM_CONTROL_INV_") == 0)
                log_error("spartan3e: BRAM control inversion provenance is backend-owned\n");
        for (int side = 0; side < (shape->dual ? 2 : 1); ++side) {
            const std::string suffix = shape->dual ? (side == 0 ? "A" : "B") : "";
            for (const std::string control : {"CLK", "EN", "WE", "SSR"}) {
                const std::string logical = control + suffix;
                IdString port = ctx->id(logical);
                NetInfo *output = bram->getPort(port);
                if (output == nullptr || output->driver.cell == nullptr || output->users.entries() != 1)
                    continue;
                CellInfo *inv = output->driver.cell;
                const PortRef &sink = *output->users.begin();
                if (inv->type != ctx->id("INV") || output->driver.port != ctx->id("O") || sink.cell != bram ||
                    sink.port != port || inv->ports.size() != 2 || !inv->params.empty())
                    continue;
                bool top_output = false;
                for (const auto &top : ctx->ports)
                    top_output |= top.second.net == output;
                if (top_output || inv->attrs.count(ctx->id("BEL")) || inv->attrs.count(ctx->id("LOC")))
                    continue;
                NetInfo *input = inv->getPort(ctx->id("I"));
                if (input == nullptr || input == output)
                    continue;
                if (output->clkconstr && input->clkconstr)
                    log_error("spartan3e: conflicting clock constraints across BRAM control inverter\n");
                if (!input->clkconstr)
                    input->clkconstr = std::move(output->clkconstr);
                bram->disconnectPort(port);
                bram->connectPort(port, input);
                bram->attrs[ctx->id("S3E_BRAM_CONTROL_INV_" + logical)] = Property(inv->name.str(ctx));
                inv->disconnectPort(ctx->id("I"));
                inv->disconnectPort(ctx->id("O"));
                removed.push_back(inv->name);
            }
        }
    }
    for (IdString cell : removed)
        ctx->cells.erase(cell);
}

std::optional<BramShape> bram_shape(const std::string &kind)
{
    const std::string prefix = "RAMB16_S";
    if (kind.compare(0, prefix.size(), prefix) != 0)
        return std::nullopt;
    std::string rest = kind.substr(prefix.size());
    size_t split = rest.find("_S");
    std::array<std::string, 2> widths{
            {rest.substr(0, split), split == std::string::npos ? "" : rest.substr(split + 2)}};
    static const std::array<BramPort, 6> layouts{
            {{1, 1, 0, 0}, {2, 2, 0, 1}, {4, 4, 0, 2}, {9, 8, 1, 3}, {18, 16, 2, 4}, {36, 32, 4, 5}}};
    BramShape shape{};
    shape.dual = split != std::string::npos;
    for (int port = 0; port < (shape.dual ? 2 : 1); ++port) {
        bool found = false;
        for (const auto &layout : layouts)
            if (std::to_string(layout.width) == widths[port]) {
                shape.ports[port] = layout;
                found = true;
            }
        if (!found)
            return std::nullopt;
    }
    if (shape.dual && shape.ports[0].width > shape.ports[1].width)
        return std::nullopt;
    return shape;
}

void remap_bram_ports(Context *ctx, CellInfo *cell, const BramShape &shape)
{
    for (int port = 0; port < (shape.dual ? 2 : 1); ++port) {
        const auto &layout = shape.ports[port];
        std::string physical = port == 0 ? "A" : "B", logical = shape.dual ? physical : "";
        auto bit = [&](const std::string &base, int index, int count, int shift = 0) {
            std::string source = base + logical + (count == 1 ? "" : "[" + std::to_string(index) + "]");
            cell->bel_pins[ctx->id(source)] = {ctx->id(base + physical + std::to_string(index + shift))};
        };
        for (int i = 0; i < 14 - layout.address_shift; ++i)
            bit("ADDR", i, 14 - layout.address_shift, layout.address_shift);
        for (int i = 0; i < layout.data_bits; ++i) {
            bit("DI", i, layout.data_bits);
            bit("DO", i, layout.data_bits);
        }
        for (int i = 0; i < layout.parity_bits; ++i) {
            bit("DIP", i, layout.parity_bits);
            bit("DOP", i, layout.parity_bits);
        }
        for (const std::string &control :
             {std::string("CLK"), std::string("EN"), std::string("WE"), std::string("SSR")})
            cell->bel_pins[ctx->id(control + logical)] = {ctx->id(control + physical)};
    }
}

NEXTPNR_NAMESPACE_END
