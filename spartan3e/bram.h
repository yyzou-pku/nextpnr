#ifndef SPARTAN3E_BRAM_H
#define SPARTAN3E_BRAM_H

#include <array>
#include <optional>
#include <string>
#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN

struct BramPort
{
    int width, data_bits, parity_bits, address_shift;
};
struct BramShape
{
    std::array<BramPort, 2> ports;
    bool dual;
};
std::optional<BramShape> bram_shape(const std::string &kind);
void remap_bram_ports(Context *ctx, CellInfo *cell, const BramShape &shape);
void fold_bram_control_inverters(Context *ctx);

NEXTPNR_NAMESPACE_END
#endif
