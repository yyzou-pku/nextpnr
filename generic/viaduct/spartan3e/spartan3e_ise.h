#ifndef SPARTAN3E_ISE_H
#define SPARTAN3E_ISE_H

#include <memory>
#include <string>

#include "viaduct_api.h"

NEXTPNR_NAMESPACE_BEGIN

// Create the ISE-native Spartan-3E uarch.  `check_xdl_path`, when non-empty,
// makes init replay one XDL design and report missing pips/broken nets (g3b).
std::unique_ptr<ViaductAPI> make_spartan3e_ise_impl(const std::string &db_path, const std::string &check_xdl_path,
                                                const std::string &dump_path);

NEXTPNR_NAMESPACE_END

#endif
