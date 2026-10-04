#ifndef SPARTAN3E_ISE_WIDE_H
#define SPARTAN3E_ISE_WIDE_H

#include <array>
#include <vector>
#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN

struct IseWideNode
{
    CellInfo *cell = nullptr;
    std::array<int, 2> children{{-1, -1}};
};

struct IseWideTree
{
    CellInfo *root = nullptr;
    std::vector<IseWideNode> nodes;
};

// Keep the Xilinx primitive INIT/port semantics; only cluster the dedicated tree.
std::vector<IseWideTree> pack_ise_wide(Context *ctx);

NEXTPNR_NAMESPACE_END
#endif
