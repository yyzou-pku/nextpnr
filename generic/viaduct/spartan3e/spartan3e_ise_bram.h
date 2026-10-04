#ifndef SPARTAN3E_ISE_BRAM_H
#define SPARTAN3E_ISE_BRAM_H

#include <array>
#include <optional>
#include <string>
#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN

struct IseBramPort {
    int width, data_bits, parity_bits, address_shift;
};
struct IseBramShape {
    std::array<IseBramPort, 2> ports;
    bool dual;
};
std::optional<IseBramShape> ise_bram_shape(const std::string &kind);
void remap_ise_bram_ports(Context *ctx, CellInfo *cell, const IseBramShape &shape);

NEXTPNR_NAMESPACE_END
#endif
