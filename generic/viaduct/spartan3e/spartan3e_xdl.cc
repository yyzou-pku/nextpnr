/*
 * nextpnr -- Spartan-3E ISE-native XDL replay for gate g3b (T40 stage 3, step 3).
 */

#include "spartan3e_xdl.h"

#include <cctype>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "log.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {

std::string read_xdl_file(const std::string &path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        log_error("spartan3e-ise: cannot open XDL '%s'\n", path.c_str());
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

std::string unquote(const std::string &s)
{
    if (s.size() >= 2 && s.front() == '"' && s.back() == '"')
        return s.substr(1, s.size() - 2);
    return s;
}

std::string strip_xdl_comments(const std::string &raw)
{
    std::string body;
    size_t start = 0;
    while (start <= raw.size()) {
        size_t end = raw.find('\n', start);
        std::string line = raw.substr(start, end == std::string::npos ? std::string::npos : end - start);
        size_t first = line.find_first_not_of(" \t\r");
        if (first != std::string::npos && line[first] == '#')
            line.clear();
        body += line;
        body += '\n';
        if (end == std::string::npos)
            break;
        start = end + 1;
    }
    return body;
}

std::vector<std::string> split_xdl_statements(const std::string &s)
{
    std::vector<std::string> out;
    bool quoted = false;
    size_t start = 0;
    for (size_t i = 0; i < s.size(); i++) {
        if (s[i] == '"') {
            quoted = !quoted;
        } else if (s[i] == ';' && !quoted) {
            out.push_back(s.substr(start, i - start));
            start = i + 1;
        }
    }
    if (start < s.size())
        out.push_back(s.substr(start));
    return out;
}

std::vector<std::string> xdl_tokens(const std::string &s)
{
    std::vector<std::string> out;
    std::string cur;
    bool quoted = false;
    for (char c : s) {
        if (quoted) {
            cur.push_back(c);
            if (c == '"') {
                quoted = false;
                out.push_back(std::move(cur));
                cur.clear();
            }
        } else if (c == '"') {
            if (!cur.empty()) {
                out.push_back(std::move(cur));
                cur.clear();
            }
            cur.push_back(c);
            quoted = true;
        } else if (c == ',' || std::isspace(static_cast<unsigned char>(c))) {
            if (!cur.empty()) {
                out.push_back(std::move(cur));
                cur.clear();
            }
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty())
        out.push_back(std::move(cur));
    return out;
}

int find_token(const std::vector<std::string> &tokens, const char *what)
{
    for (size_t i = 0; i < tokens.size(); i++)
        if (tokens[i] == what)
            return (int)i;
    return -1;
}

std::unordered_map<std::string, std::string> collect_placements(const std::vector<std::string> &statements)
{
    std::unordered_map<std::string, std::string> site_of;
    for (auto &stmt : statements) {
        auto t = xdl_tokens(stmt);
        if (t.size() < 3 || t[0] != "inst")
            continue;
        int pi = find_token(t, "placed");
        if (pi >= 0 && pi + 2 < (int)t.size())
            site_of[unquote(t[1])] = t[pi + 2];
    }
    return site_of;
}

struct NetDsu
{
    std::unordered_map<int32_t, int> ids;
    std::vector<int> parent;

    int get(WireId w)
    {
        if (w.index < 0)
            return -1;
        auto it = ids.find(w.index);
        if (it != ids.end())
            return it->second;
        int id = (int)parent.size();
        ids[w.index] = id;
        parent.push_back(id);
        return id;
    }

    int find(int x)
    {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }

    void unite(int a, int b)
    {
        if (a < 0 || b < 0)
            return;
        a = find(a);
        b = find(b);
        if (a != b)
            parent[b] = a;
    }

    bool one_component()
    {
        if (ids.empty())
            return true;
        int root = find(0);
        for (auto &kv : ids)
            if (find(kv.second) != root)
                return false;
        return true;
    }
};

std::optional<WireId> resolve_site_pin(const std::string &inst, const std::string &pin,
                                       const std::unordered_map<std::string, std::string> &site_of,
                                       const std::unordered_map<std::string, IseSiteRef> &site_by_name,
                                       const IseChipDb &db, const std::vector<uint32_t> &tile_base,
                                       const std::vector<WireId> &member_wire)
{
    auto sit = site_of.find(inst);
    if (sit == site_of.end())
        return std::nullopt;
    auto sref = site_by_name.find(sit->second);
    if (sref == site_by_name.end())
        return std::nullopt;
    const auto &tile = db.tiles[sref->second.tile];
    const auto &variant = db.variants[tile.variant];
    if (sref->second.site < 0 || sref->second.site >= (int)variant.sites.size())
        return std::nullopt;
    const auto &site = variant.sites[sref->second.site];
    auto pit = site.pins.find(pin);
    if (pit == site.pins.end() || pit->second < 0 || pit->second >= (int)variant.wires.size())
        return std::nullopt;
    uint32_t member = tile_base[sref->second.tile] + (uint32_t)pit->second;
    if (member >= member_wire.size())
        return std::nullopt;
    return member_wire[member];
}

void replay_net(Context *ctx, const std::vector<std::string> &tokens,
                const std::unordered_map<std::string, std::string> &site_of,
                const std::unordered_map<std::string, IseSiteRef> &site_by_name, const IseChipDb &db,
                const std::vector<uint32_t> &tile_base, const std::vector<WireId> &member_wire, NetDsu &dsu,
                size_t &pips, size_t &pips_ok, bool &unresolved, std::vector<std::string> &bad)
{
    for (size_t i = 2; i < tokens.size();) {
        if ((tokens[i] == "outpin" || tokens[i] == "inpin") && i + 2 < tokens.size()) {
            std::string inst = unquote(tokens[i + 1]);
            std::string pin = tokens[i + 2];
            auto wire = resolve_site_pin(inst, pin, site_of, site_by_name, db, tile_base, member_wire);
            if (wire) {
                dsu.get(*wire);
            } else {
                unresolved = true;
                if (bad.size() < 20)
                    bad.push_back("g3b net " + unquote(tokens[1]) + ": unresolved site pin " + inst + "." + pin);
            }
            i += 3;
        } else if (tokens[i] == "pip" && i + 4 < tokens.size()) {
            const std::string &tile = tokens[i + 1];
            const std::string &from = tokens[i + 2];
            const std::string &to = tokens[i + 4];
            IdStringList name =
                    IdStringList::concat(IdStringList::concat(ctx->id(tile), ctx->id(from)), ctx->id(to));
            PipId pip = ctx->getPipByName(name);
            pips++;
            if (pip != PipId()) {
                pips_ok++;
                dsu.unite(dsu.get(ctx->getPipSrcWire(pip)), dsu.get(ctx->getPipDstWire(pip)));
            } else if (bad.size() < 20) {
                bad.push_back("g3b net " + unquote(tokens[1]) + ": missing pip " + tile + " " + from + " " +
                              tokens[i + 3] + " " + to);
            }
            i += 5;
        } else {
            i++;
        }
    }
}

} // namespace

bool check_xdl_routing(Context *ctx, const IseChipDb &db, const std::vector<uint32_t> &tile_base,
                       const std::vector<WireId> &member_wire,
                       const std::unordered_map<std::string, IseSiteRef> &site_by_name, const std::string &path)
{
    std::string body = strip_xdl_comments(read_xdl_file(path));
    auto statements = split_xdl_statements(body);
    auto site_of = collect_placements(statements);

    size_t pips = 0, pips_ok = 0, nets = 0, nets_ok = 0, unresolved = 0;
    std::vector<std::string> bad;
    for (auto &stmt : statements) {
        auto t = xdl_tokens(stmt);
        if (t.empty() || t[0] != "net")
            continue;
        NetDsu dsu;
        bool net_unresolved = false;
        replay_net(ctx, t, site_of, site_by_name, db, tile_base, member_wire, dsu, pips, pips_ok, net_unresolved,
                   bad);
        if (net_unresolved)
            unresolved++;
        if (dsu.ids.empty())
            continue;
        nets++;
        if (!net_unresolved && dsu.one_component()) {
            nets_ok++;
        } else if (bad.size() < 20) {
            bad.push_back("g3b net " + unquote(t[1]) + ": not connected");
        }
    }

    log_info("Spartan-3E ISE g3b: pips %zu/%zu in chipdb, nets connected %zu/%zu (%zu unresolved nets)\n", pips_ok,
             pips, nets_ok, nets, unresolved);
    for (auto &b : bad)
        log_info("  %s\n", b.c_str());
    if (pips_ok != pips || nets_ok != nets)
        log_error("spartan3e-ise g3b FAIL: %zu/%zu pips, %zu/%zu nets\n", pips_ok, pips, nets_ok, nets);
    return true;
}

NEXTPNR_NAMESPACE_END
