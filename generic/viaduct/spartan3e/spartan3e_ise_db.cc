/*
 * nextpnr -- Spartan-3E ISE-native chipdb parsing (T40 stage 3).
 */

#include "spartan3e_ise_db.h"

#include <fstream>
#include <iterator>
#include <string>

#include "json11.hpp"
#include "log.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {

using json11::Json;

std::string read_file(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        log_error("spartan3e-ise: cannot open '%s'\n", path.c_str());
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

} // namespace

IseChipDb parse_ise_chipdb(const std::string &path)
{
    std::string text = read_file(path);
    std::string err;
    Json root = Json::parse(text, err);
    if (!err.empty())
        log_error("spartan3e-ise: chipdb parse error: %s\n", err.c_str());
    if (root["format"].string_value() != "spark-ise-chipdb/2")
        log_error("spartan3e-ise: '%s' is not a spark-ise-chipdb/2 file\n", path.c_str());

    IseChipDb db;
    db.part = root["part"].string_value();
    db.package = root["package"]["name"].string_value();
    for (const auto &row : root["package"]["pins"].array_items()) {
        const auto &a = row.array_items();
        if (a.size() < 3)
            log_error("spartan3e-ise: malformed package pin row\n");
        if (a[0].string_value() == "pin" &&
            !db.package_pads.emplace(a[2].string_value(), a[1].string_value()).second)
            log_error("spartan3e-ise: duplicate package pin\n");
    }
    db.rows = root["rows"].int_value();
    db.cols = root["cols"].int_value();
    for (auto &w : root["wires"].array_items())
        db.wires.push_back(w.string_value());
    for (auto &op : root["pip_ops"].array_items())
        db.pip_ops.push_back(op.string_value());
    for (auto &rt : root["routethroughs"].array_items())
        db.routethroughs.push_back(rt.string_value());

    for (auto &v : root["variants"].array_items()) {
        IseVariant out;
        for (auto &w : v["wires"].array_items())
            out.wires.push_back(w.int_value());
        for (auto &p : v["pips"].array_items()) {
            auto a = p.array_items();
            if (a.size() < 4)
                log_error("spartan3e-ise: bad pip row in variant\n");
            out.pips.push_back(IsePip{a[0].int_value(), a[1].int_value(), a[2].int_value(), a[3].int_value()});
        }
        for (auto &s : v["sites"].array_items()) {
            IseSiteDef site;
            site.type = s["type"].string_value();
            for (auto &pin : s["pins"].object_items()) {
                auto a = pin.second.array_items();
                if (a.size() < 2)
                    log_error("spartan3e-ise: bad site pin row\n");
                site.pins[pin.first] = a[1].int_value();
            }
            out.sites.push_back(std::move(site));
        }
        db.variants.push_back(std::move(out));
    }
    for (auto &t : root["tiles"].array_items()) {
        IseTile tile;
        tile.row = t["row"].int_value();
        tile.col = t["col"].int_value();
        tile.name = t["name"].string_value();
        tile.kind = t["kind"].string_value();
        tile.variant = t["variant"].int_value();
        for (auto &s : t["sites"].array_items())
            tile.sites.push_back(s.string_value());
        for (auto &s : t["bonding"].array_items())
            tile.bonding.push_back(s.string_value());
        db.tiles.push_back(std::move(tile));
    }
    for (auto &n : root["nodes"].array_items()) {
        std::vector<std::pair<int, int>> node;
        for (auto &m : n.array_items()) {
            auto a = m.array_items();
            if (a.size() < 2)
                log_error("spartan3e-ise: bad node member\n");
            node.emplace_back(a[0].int_value(), a[1].int_value());
        }
        db.nodes.push_back(std::move(node));
    }
    for (auto &kv : root["site_defs"].object_items()) {
        IseSiteType type;
        for (auto &p : kv.second["pins"].array_items())
            type.pins.emplace_back(p[0].string_value(), p[1].string_value());
        for (auto &e : kv.second["elements"].array_items()) {
            IseSiteElement el;
            el.name = e["name"].string_value();
            el.bel = e["bel"].bool_value();
            for (auto &p : e["pins"].array_items())
                el.pins.emplace_back(p[0].string_value(), p[1].string_value());
            for (auto &c : e["cfg"].array_items())
                el.cfg.push_back(c.string_value());
            for (auto &c : e["conns"].array_items()) {
                auto a = c.array_items();
                if (a.size() < 5)
                    log_error("spartan3e-ise: bad site conn row\n");
                el.conns.push_back(IseSiteConn{a[0].string_value(), a[1].string_value(), a[2].string_value(),
                                               a[3].string_value(), a[4].string_value()});
            }
            type.elements.push_back(std::move(el));
        }
        db.site_defs.emplace(kv.first, std::move(type));
    }
    return db;
}

bool is_spark_ise_chipdb(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    char buf[4096] = {};
    f.read(buf, sizeof(buf));
    std::string head(buf, (size_t)f.gcount());
    return head.find("\"format\"") != std::string::npos && head.find("spark-ise-chipdb/2") != std::string::npos;
}

NEXTPNR_NAMESPACE_END
