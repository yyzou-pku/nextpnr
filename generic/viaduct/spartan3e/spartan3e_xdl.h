#ifndef SPARTAN3E_XDL_H
#define SPARTAN3E_XDL_H

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "nextpnr.h"
#include "spartan3e_ise_db.h"

NEXTPNR_NAMESPACE_BEGIN

// g3b: replay one XDL design against the loaded ISE graph.
bool check_xdl_routing(Context *ctx, const IseChipDb &db, const std::vector<uint32_t> &tile_base,
                       const std::vector<WireId> &member_wire,
                       const std::unordered_map<std::string, IseSiteRef> &site_by_name, const std::string &path);

NEXTPNR_NAMESPACE_END

#endif
