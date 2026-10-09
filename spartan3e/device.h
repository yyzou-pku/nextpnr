#ifndef SPARTAN3E_DB_H
#define SPARTAN3E_DB_H

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "nextpnr_namespaces.h"

NEXTPNR_NAMESPACE_BEGIN

struct Pip
{
    int from = -1, to = -1, op = -1, rt = -1;
};

struct SiteDef
{
    std::string type;
    std::unordered_map<std::string, int> pins; // pin name -> local wire index
};

struct Variant
{
    std::vector<int> wires;
    std::vector<Pip> pips;
    std::vector<SiteDef> sites;
};

struct Tile
{
    int row = 0, col = 0;
    std::string name, kind;
    int variant = 0;
    std::vector<std::string> sites;
    std::vector<std::string> bonding;
};

struct SiteConn
{
    std::string from, from_pin, op, to, to_pin;
};

struct SiteElement
{
    std::string name;
    bool bel = false;
    std::vector<std::pair<std::string, std::string>> pins; // (name, dir)
    std::vector<std::string> cfg;
    std::vector<SiteConn> conns;
};

struct SiteType
{
    std::vector<std::pair<std::string, std::string>> pins; // (name, dir)
    std::vector<SiteElement> elements;
};

struct ChipDb
{
    std::string part;
    std::string package;
    // Only user I/O pins (not power/configuration pkgpin rows).
    std::unordered_map<std::string, std::string> package_pads;
    int rows = 0, cols = 0;
    std::vector<std::string> wires;
    std::vector<std::string> pip_ops;
    std::vector<std::string> routethroughs;
    std::vector<Variant> variants;
    std::vector<Tile> tiles;
    /// Electrical nodes with more than one member: `(tile index, local wire index)`.
    std::vector<std::vector<std::pair<int, int>>> nodes;
    std::unordered_map<std::string, SiteType> site_defs;
};

struct SiteRef
{
    int tile = -1;
    int site = -1;
};

// Parse a spark-chipdb/2 file into the raw structural model.
ChipDb parse_chipdb(const std::string &path);

NEXTPNR_NAMESPACE_END

#endif
