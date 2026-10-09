#ifndef SPARTAN3E_CARRY_H
#define SPARTAN3E_CARRY_H

#include <vector>

#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN

enum class CarryDi
{
    Zero,
    One,
    LutInput,
    Product
};

struct CarryBit
{
    CellInfo *mux = nullptr;
    CellInfo *xorcy = nullptr;
    CellInfo *lut = nullptr;
    CellInfo *ff = nullptr;
    CellInfo *andcell = nullptr;
    CarryDi di = CarryDi::Zero;
};

struct CarryChain
{
    CellInfo *root = nullptr;
    std::vector<CarryBit> bits;
};

// Normalize each propagate function to a private LUT and cluster entire chains,
// including the terminal XORCY whose unused MUXCY Yosys has removed.
std::vector<CarryChain> pack_carries(Context *ctx);

NEXTPNR_NAMESPACE_END

#endif
