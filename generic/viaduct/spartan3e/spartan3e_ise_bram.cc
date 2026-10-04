/* Primitive BRAM port policy; physical routing remains sourced from primitive_defs. */
#include "spartan3e_ise_bram.h"

NEXTPNR_NAMESPACE_BEGIN

std::optional<IseBramShape> ise_bram_shape(const std::string &kind)
{
    const std::string prefix = "RAMB16_S";
    if (kind.compare(0, prefix.size(), prefix) != 0) return std::nullopt;
    std::string rest = kind.substr(prefix.size());
    size_t split = rest.find("_S");
    std::array<std::string, 2> widths{{rest.substr(0, split), split == std::string::npos ? "" : rest.substr(split + 2)}};
    static const std::array<IseBramPort, 6> layouts{{{1, 1, 0, 0}, {2, 2, 0, 1}, {4, 4, 0, 2},
                                                   {9, 8, 1, 3}, {18, 16, 2, 4}, {36, 32, 4, 5}}};
    IseBramShape shape{};
    shape.dual = split != std::string::npos;
    for (int port = 0; port < (shape.dual ? 2 : 1); ++port) {
        bool found = false;
        for (const auto &layout : layouts)
            if (std::to_string(layout.width) == widths[port]) { shape.ports[port] = layout; found = true; }
        if (!found) return std::nullopt;
    }
    if (shape.dual && shape.ports[0].width > shape.ports[1].width) return std::nullopt;
    return shape;
}

void remap_ise_bram_ports(Context *ctx, CellInfo *cell, const IseBramShape &shape)
{
    for (int port = 0; port < (shape.dual ? 2 : 1); ++port) {
        const auto &layout = shape.ports[port];
        std::string physical = port == 0 ? "A" : "B", logical = shape.dual ? physical : "";
        auto bit = [&](const std::string &base, int index, int count, int shift = 0) {
            std::string source = base + logical + (count == 1 ? "" : "[" + std::to_string(index) + "]");
            cell->bel_pins[ctx->id(source)] = {ctx->id(base + physical + std::to_string(index + shift))};
        };
        for (int i = 0; i < 14 - layout.address_shift; ++i) bit("ADDR", i, 14 - layout.address_shift, layout.address_shift);
        for (int i = 0; i < layout.data_bits; ++i) { bit("DI", i, layout.data_bits); bit("DO", i, layout.data_bits); }
        for (int i = 0; i < layout.parity_bits; ++i) { bit("DIP", i, layout.parity_bits); bit("DOP", i, layout.parity_bits); }
        for (const std::string &control : {std::string("CLK"), std::string("EN"), std::string("WE"), std::string("SSR")})
            cell->bel_pins[ctx->id(control + logical)] = {ctx->id(control + physical)};
    }
}

NEXTPNR_NAMESPACE_END
