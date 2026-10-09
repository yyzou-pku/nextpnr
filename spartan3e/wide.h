#ifndef SPARTAN3E_WIDE_H
#define SPARTAN3E_WIDE_H

#include <array>
#include <vector>
#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN

struct WideNode
{
    CellInfo *cell = nullptr;
    std::array<int, 2> children{{-1, -1}};
};

struct WideTree
{
    CellInfo *root = nullptr;
    // Connected dedicated DAG; each cell appears once. root is the anchor.
    std::vector<WideNode> nodes;
};

// Keep the Xilinx primitive INIT/port semantics; only cluster the dedicated tree.
std::vector<WideTree> pack_wide(Context *ctx);

NEXTPNR_NAMESPACE_END
#endif
