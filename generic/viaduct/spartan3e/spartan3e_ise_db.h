#ifndef SPARTAN3E_ISE_DB_H
#define SPARTAN3E_ISE_DB_H

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nextpnr_namespaces.h"

NEXTPNR_NAMESPACE_BEGIN

struct IsePip
{
    int from = -1, to = -1, op = -1, rt = -1;
};

struct IseSiteDef
{
    std::string type;
    std::unordered_map<std::string, int> pins; // pin name -> local wire index
};

struct IseVariant
{
    std::vector<int> wires;
    std::vector<IsePip> pips;
    std::vector<IseSiteDef> sites;
};

struct IseTile
{
    int row = 0, col = 0;
    std::string name, kind;
    int variant = 0;
    std::vector<std::string> sites;
    std::vector<std::string> bonding;
};

struct IseSiteConn
{
    std::string from, from_pin, op, to, to_pin;
};

struct IseSiteElement
{
    std::string name;
    bool bel = false;
    std::vector<std::pair<std::string, std::string>> pins; // (name, dir)
    std::vector<std::string> cfg;
    std::vector<IseSiteConn> conns;
};

struct IseSiteType
{
    std::vector<std::pair<std::string, std::string>> pins; // (name, dir)
    std::vector<IseSiteElement> elements;
};

struct IseChipDb
{
    std::string part;
    std::string package;
    // Only user I/O pins (not power/configuration pkgpin rows).
    std::unordered_map<std::string, std::string> package_pads;
    int rows = 0, cols = 0;
    std::vector<std::string> wires;
    std::vector<std::string> pip_ops;
    std::vector<std::string> routethroughs;
    std::vector<IseVariant> variants;
    std::vector<IseTile> tiles;
    /// Electrical nodes with more than one member: `(tile index, local wire index)`.
    std::vector<std::vector<std::pair<int, int>>> nodes;
    std::unordered_map<std::string, IseSiteType> site_defs;
};

struct IseSiteRef
{
    int tile = -1;
    int site = -1;
};

// True if `path` starts with a spark-ise-chipdb/2 JSON object.
bool is_spark_ise_chipdb(const std::string &path);

// Parse a spark-ise-chipdb/2 file into the raw structural model.
IseChipDb parse_ise_chipdb(const std::string &path);

NEXTPNR_NAMESPACE_END

#endif
