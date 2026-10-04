#ifndef SPARTAN3E_ISE_CARRY_H
#define SPARTAN3E_ISE_CARRY_H

#include <vector>

#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN

enum class IseCarryDi
{
    Zero,
    One,
    LutInput
};

struct IseCarryBit
{
    CellInfo *mux = nullptr;
    CellInfo *xorcy = nullptr;
    CellInfo *lut = nullptr;
    CellInfo *ff = nullptr;
    IseCarryDi di = IseCarryDi::Zero;
};

struct IseCarryChain
{
    CellInfo *root = nullptr;
    std::vector<IseCarryBit> bits;
};

// Normalize each propagate function to a private LUT and cluster entire chains,
// including the terminal XORCY whose unused MUXCY Yosys has removed.
std::vector<IseCarryChain> pack_ise_carries(Context *ctx);

NEXTPNR_NAMESPACE_END

#endif
