#include "ucf.h"
#include <algorithm>
#include <cctype>
#include <fstream>
#include <set>
#include <tuple>
#include "log.h"
#include "nextpnr.h"

NEXTPNR_NAMESPACE_BEGIN
namespace {
struct Token
{
    std::string text;
    int line;
    bool quoted = false;
};
std::string upper(std::string s)
{
    for (char &c : s)
        c = std::toupper(static_cast<unsigned char>(c));
    return s;
}
bool matches(const std::string &pattern, const std::string &name)
{
    size_t p = 0, n = 0, star = std::string::npos, retry = 0;
    while (n < name.size()) {
        if (p < pattern.size() && (pattern[p] == '?' || pattern[p] == name[n])) {
            ++p;
            ++n;
        } else if (p < pattern.size() && pattern[p] == '*') {
            star = p++;
            retry = n;
        } else if (star != std::string::npos) {
            p = star + 1;
            n = ++retry;
        } else
            return false;
    }
    while (p < pattern.size() && pattern[p] == '*')
        ++p;
    return p == pattern.size();
}
class Reader
{
    Context *ctx;
    std::string filename;
    std::vector<Token> tokens;
    size_t pos = 0;
    [[noreturn]] void fail(int line, const std::string &message)
    {
        log_error("%s:%d: %s\n", filename.c_str(), line, message.c_str());
    }
    Token take()
    {
        if (pos == tokens.size())
            fail(tokens.empty() ? 1 : tokens.back().line, "unexpected end of UCF");
        return tokens[pos++];
    }
    void expect(const std::string &s)
    {
        auto t = take();
        if (t.quoted || t.text != s)
            fail(t.line, "expected '" + s + "'");
    }
    void lex(std::istream &input)
    {
        int line = 1;
        while (input.peek() != EOF) {
            char c = input.get();
            if (c == '\n') {
                ++line;
                continue;
            }
            if (std::isspace(static_cast<unsigned char>(c)))
                continue;
            if (c == '#') {
                while (input.peek() != EOF && input.peek() != '\n')
                    input.get();
                continue;
            }
            Token t{"", line, c == '"'};
            if (t.quoted) {
                while (input.peek() != EOF && input.peek() != '"') {
                    char next = input.get();
                    if (next == '\n')
                        fail(line, "newline in quoted name");
                    t.text += next;
                }
                if (input.get() != '"')
                    fail(line, "unterminated quoted name");
            } else if (c == '=' || c == '|' || c == ';')
                t.text += c;
            else {
                t.text += c;
                while (input.peek() != EOF) {
                    char next = input.peek();
                    if (std::isspace(static_cast<unsigned char>(next)) || next == '#' || next == '=' || next == '|' ||
                        next == ';')
                        break;
                    if (next == '"')
                        fail(line, "unexpected quote in token");
                    t.text += input.get();
                }
            }
            tokens.push_back(t);
        }
    }
    std::set<CellInfo *> targets(const Token &kind, const Token &name)
    {
        std::set<CellInfo *> result;
        std::string type = upper(kind.text);
        if (type == "INST") {
            for (auto &cell : ctx->cells)
                if (matches(name.text, cell.first.str(ctx)))
                    result.insert(cell.second.get());
        } else if (type == "NET") {
            // NET constraints in this subset address top-level pad nets, not internal timing nets.
            std::set<NetInfo *> nets;
            for (const auto &alias : ctx->net_aliases)
                if (matches(name.text, alias.first.str(ctx)))
                    nets.insert(ctx->getNetByAlias(alias.first));
            for (const auto &net : ctx->nets)
                if (matches(name.text, net.first.str(ctx)))
                    nets.insert(net.second.get());
            for (auto &cell : ctx->cells) {
                CellInfo *c = cell.second.get();
                std::string t = c->type.str(ctx);
                if (t != "$nextpnr_ibuf" && t != "$nextpnr_obuf" && t != "$nextpnr_iobuf")
                    continue;
                NetInfo *pad = c->getPort(ctx->id(t == "$nextpnr_obuf" ? "I" : "O"));
                if (matches(name.text, c->name.str(ctx)) || nets.count(pad))
                    result.insert(c);
            }
        } else
            fail(kind.line, "unsupported UCF statement '" + kind.text + "'");
        if (name.text.empty() || name.text.back() == '/')
            fail(name.line, "unsupported empty or level-only target pattern");
        if (result.empty())
            fail(name.line, "no supported target matches '" + name.text + "'");
        return result;
    }
    void apply(CellInfo *cell, const Token &key, std::string value)
    {
        std::string k = upper(key.text), type = cell->type.str(ctx);
        bool io = type == "$nextpnr_ibuf" || type == "$nextpnr_obuf" || type == "$nextpnr_iobuf" || type == "IBUF" ||
                  type == "OBUF" || type == "OBUFT" || type == "IOBUF";
        bool pull = k == "PULLUP" || k == "PULLDOWN" || k == "KEEPER";
        if (k != "LOC" && k != "BEL" && k != "IOSTANDARD" && k != "DRIVE" && k != "SLEW" && !pull)
            fail(key.line, "unsupported constraint '" + key.text + "'");
        if (!io && k != "LOC" && k != "BEL")
            fail(key.line, k + " requires an I/O target");
        if (k == "IOSTANDARD" && upper(value) != "LVCMOS33")
            fail(key.line, "supported IOSTANDARD: LVCMOS33");
        if (k == "DRIVE" && !std::set<std::string>{"2", "4", "6", "8", "12", "16"}.count(value))
            fail(key.line, "supported DRIVE: 2/4/6/8/12/16");
        if (k == "SLEW" && upper(value) != "FAST" && upper(value) != "SLOW")
            fail(key.line, "supported SLEW: FAST/SLOW");
        if (pull && upper(value) != "TRUE" && upper(value) != "FALSE" && value != "1" && value != "0")
            fail(key.line, k + " requires TRUE/FALSE or 1/0");
        if (pull || k == "IOSTANDARD" || k == "SLEW")
            value = upper(value);
        if (k == "LOC" || k == "BEL") {
            if (value.empty() || value.find_first_of("*?,:{} ") != std::string::npos)
                fail(key.line, k + " requires one exact pin/site/BEL");
            if (!io) {
                auto region = std::make_unique<Region>();
                region->name = ctx->id("$ucf/" + cell->name.str(ctx));
                region->constr_bels = true;
                for (BelId bel : ctx->getBels()) {
                    std::string full = ctx->getBelName(bel).str(ctx);
                    bool selected = k == "BEL" ? full == value : full.find("/" + value + "/") != std::string::npos;
                    auto other = cell->attrs.find(ctx->id(k == "BEL" ? "LOC" : "BEL"));
                    if (other != cell->attrs.end()) {
                        std::string v = other->second.as_string();
                        selected &= k == "BEL" ? full.find("/" + v + "/") != std::string::npos : full == v;
                    }
                    if (selected && ctx->isValidBelForCellType(cell->type, bel))
                        region->bels.insert(bel);
                }
                if (region->bels.empty())
                    fail(key.line, "no compatible placement for " + cell->name.str(ctx) + " " + k + "=" + value);
                cell->region = region.get();
                ctx->region[region->name] = std::move(region);
            }
        }
        cell->attrs[ctx->id(k)] = Property(value);
        auto retain_source = [&](CellInfo *target) {
            ctx->ucf_sources[{target->name.str(ctx), k}] = UcfSource{filename, key.line, value};
        };
        retain_source(cell);
        if (io) {
            // UCF has priority over netlist attributes; update both port and native buffer.
            if (k == "LOC")
                cell->attrs.erase(ctx->id("PACKAGE_PIN"));
            std::string port = type == "$nextpnr_obuf" || type == "IBUF" ? "I" : type == "IOBUF" ? "IO" : "O";
            NetInfo *pad = cell->getPort(ctx->id(port));
            if (pad != nullptr) {
                auto update = [&](CellInfo *buffer) {
                    if (!buffer ||
                        !buffer->type.in(ctx->id("IBUF"), ctx->id("OBUF"), ctx->id("OBUFT"), ctx->id("IOBUF"),
                                         ctx->id("$nextpnr_ibuf"), ctx->id("$nextpnr_obuf"), ctx->id("$nextpnr_iobuf")))
                        return;
                    buffer->attrs[ctx->id(k)] = Property(value);
                    retain_source(buffer);
                    if (k == "LOC")
                        buffer->attrs.erase(ctx->id("PACKAGE_PIN"));
                };
                update(pad->driver.cell);
                for (auto user : pad->users)
                    update(user.cell);
            }
        }
    }

  public:
    Reader(Context *ctx, std::string filename) : ctx(ctx), filename(std::move(filename)) {}
    void read()
    {
        std::ifstream input(filename);
        if (!input)
            fail(1, "cannot open UCF file");
        lex(input);
        while (pos < tokens.size()) {
            Token kind = take(), name = take();
            auto selected = targets(kind, name);
            while (true) {
                Token key = take();
                std::string value = "TRUE";
                if (pos < tokens.size() && tokens[pos].text == "=") {
                    expect("=");
                    value = take().text;
                } else if (upper(key.text) != "PULLUP" && upper(key.text) != "PULLDOWN" && upper(key.text) != "KEEPER")
                    fail(key.line, "expected '=' after " + key.text);
                for (auto cell : selected)
                    apply(cell, key, value);
                Token end = take();
                if (!end.quoted && end.text == ";")
                    break;
                if (end.quoted || end.text != "|")
                    fail(end.line, "expected '|' or ';'");
            }
        }
        log_info("Applied UCF constraints from %s.\n", filename.c_str());
    }
};
} // namespace
void read_ucf(Context *ctx, const std::string &filename) { Reader(ctx, filename).read(); }

void report_ucf_context(Context *ctx, const char *phase)
{
    if (ctx->ucf_sources.empty())
        return;
    // Generic placer failures do not identify a unique offending constraint.
    // Report effective source context without presenting it as a diagnosed cause.
    log_info("Effective UCF constraints at %s failure (context, not a conflict diagnosis):\n", phase);
    using SourceKey = std::tuple<std::string, int, std::string, std::string>;
    std::map<SourceKey, std::set<std::string>> sources;
    for (const auto &entry : ctx->ucf_sources) {
        const auto &source = entry.second;
        sources[{source.filename, source.line, entry.first.second, source.value}].insert(entry.first.first);
    }
    for (const auto &entry : sources) {
        const auto &[filename, line, key, value] = entry.first;
        std::string targets;
        for (const auto &target : entry.second) {
            if (!targets.empty())
                targets += ", ";
            targets += target;
        }
        log_info("  %s:%d: %s=%s on %s\n", filename.c_str(), line, key.c_str(), value.c_str(), targets.c_str());
    }
}
NEXTPNR_NAMESPACE_END
