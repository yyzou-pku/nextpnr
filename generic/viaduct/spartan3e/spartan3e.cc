/*
 * nextpnr -- Spartan-3E (xc3s500e) Viaduct uarch
 *
 * 数据来源：`chipdb/xc3s500e.json`（由 ~/Documents/spark/tools/s3e-tools 从 prjcombine 的
 * databases/spartan3.zstd 导出）。用 nextpnr 自带的 json11 解析，无额外依赖。
 *
 * 建模要点：
 *  - 每个 canonical 节点 → 一条 nextpnr wire（名字 = chipdb 的 node name）；
 *  - 每个 S3E SLICE → 拆成两个 nextpnr bel：`SLICE<i>_LUT`(z=2i) 与 `SLICE<i>_FF`(z=2i+1)，
 *    这样 Yosys 的 LUT4/DFF 可直接落位，用 constrain_cell_pairs(delta_z=1) 保证同 SLICE；
 *  - LUT→FF 的直连用「每个 SLICE 内部加一条 X → BX 的 pip」来模拟 XMUX（无需占用互连）；
 *  - IO 用 IOI bel：`O` ← IOI.I 输出节点，`I` ← IOI.O1 输入节点，`PAD` ← 自建 pad wire。
 */

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "json11.hpp"
#include "log.h"
#include "nextpnr.h"
#include "util.h"
#include "viaduct_api.h"
#include "viaduct_helpers.h"

#define GEN_INIT_CONSTIDS
#define VIADUCT_CONSTIDS "viaduct/spartan3e/constids.inc"
#include "viaduct_constids.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {

using json11::Json;

struct PipTemplate
{
    std::string kind, dst, src;
    std::vector<std::string> bits;
    std::vector<std::pair<std::string, std::string>> mux_src; // wire -> pattern
};

struct BelTemplate
{
    std::string slot, cls, kind;
    std::vector<std::pair<std::string, std::string>> inputs; // pin -> wire
    std::vector<std::pair<std::string, std::vector<std::string>>> outputs;
};

struct TileClass
{
    std::vector<std::string> cells;
    std::vector<BelTemplate> bels;
    std::vector<PipTemplate> pips;
};

struct TileInst
{
    int x, y;
    std::string slot, cls;
    // I19：tile 覆盖的 cell 列表（prjcombine 的 tile 是「锚点 + 覆盖 cell 集合」）
    std::vector<std::pair<int, int>> cells;
    std::string key() const { return std::to_string(x) + "," + std::to_string(y) + "," + slot; }
};

struct Node
{
    std::string name, kind;
    int x, y;
};

static int parse_bracket_index(const std::string &s)
{
    // "SLICE[3]" -> 3 ; 无方括号 -> 0
    auto lb = s.find('[');
    if (lb == std::string::npos)
        return 0;
    return std::atoi(s.c_str() + lb + 1);
}

struct Spartan3eImpl : ViaductAPI
{
    // pip 元数据：高 32 位 = tile class 下标，16..31 = pip 模板下标，0..15 = mux 源下标；
    // 若最高位为 1，则为「内部特殊 pip」（目前只有 XMUX，低 16 位 = slice 下标）。
    static constexpr uint64_t META_SPECIAL = 1ull << 63;
    /// 虚拟 pip（不入位流）：目前只有「输入 buffer 的 fan-out」I→CLKPAD。
    static constexpr uint64_t META_VIRTUAL = 1ull << 61;

    ViaductHelpers h;
    std::string db_path;
    std::string dump_path;

    std::map<std::string, TileClass> classes;
    std::map<std::string, int> class_index;
    std::vector<std::string> class_list;
    std::vector<TileInst> tiles;
    /// 权威键合 pad："cellx,celly,ioibel"（来自 chipdb 的 bond 表）
    std::set<std::string> bonded_pads;
    /// chipdb `iob_map`：IOB 类 -> [(cell_idx, ioi, kind)]（来自 prjcombine `IobTileData.iobs`）
    std::map<std::string, std::vector<std::tuple<int, int, std::string>>> iob_map;
    /// 只能输入的 pad："cellx,celly,ioi"
    std::set<std::string> input_only_ioi;
    /// 上面这些 pad 对应的 IOI bel（用来在布局合法性里拒绝输出）
    std::set<BelId> input_only_bel;

    // ---- W5：引脚约束（PCF）----
    /// "cellx,celly,ioi" -> 该站点键合的引脚名（如 "P78"）
    std::map<std::string, std::vector<std::string>> bond_pins;
    /// 封装引脚名 -> IOB bel
    std::map<std::string, BelId> pin_bel;
    /// 端口名 -> 引脚名（来自 PCF 的 `set_io`）
    std::map<std::string, std::string> pin_constraints;
    std::string pcf_path;

    // ---- T4：专用时钟输入（IBUFG/CLKPAD）的 IOB↔BUFG 配对 ----
    /// IOI 的 `CLKPAD` 输出线 -> 该 IOI bel
    std::map<WireId, BelId> clkpad_src;
    /// BUFGMUX bel -> 能经专用直连（`OUT_CLKPAD → IMUX_BUFG_CLK`）喂它的 IOI bel
    std::map<BelId, std::vector<BelId>> bufg_pairs;
    /// 反向：IOI bel -> 配对 BUFGMUX bel
    std::map<BelId, std::vector<BelId>> iob_pairs;
    /// "x,y,BUFGMUX[k]" -> bel（建配对时查）
    std::map<std::string, BelId> bufg_bel_by_key;
    std::vector<Node> nodes;
    std::map<std::string, std::map<std::string, int>> wiremap;

    std::vector<WireId> node_wires;
    std::vector<uint64_t> pip_meta; // pip.index -> meta
    unsigned pip_seq = 0;

    static uint64_t meta_of(int cidx, size_t pi, size_t si)
    {
        return ((uint64_t)cidx << 32) | ((uint64_t)pi << 16) | (uint64_t)si;
    }

    ~Spartan3eImpl() {};

    // ---------------- chipdb 载入 ----------------
    void load_db()
    {
        if (db_path.empty()) {
            const char *env = std::getenv("SPARTAN3E_DB");
            db_path = env ? env : "xc3s500e.json";
        }
        std::ifstream f(db_path);
        if (!f)
            log_error("spartan3e: cannot open chipdb '%s' (use --vopt db=<path> or $SPARTAN3E_DB)\n",
                      db_path.c_str());
        std::stringstream ss;
        ss << f.rdbuf();
        std::string text = ss.str();
        f.close();

        std::string err;
        Json root = Json::parse(text, err);
        if (!err.empty())
            log_error("spartan3e: chipdb parse error: %s\n", err.c_str());

        for (auto &kv : root["tile_classes"].object_items()) {
            TileClass tc;
            for (auto &c : kv.second["cells"].array_items())
                tc.cells.push_back(c.string_value());
            for (auto &b : kv.second["bels"].array_items()) {
                BelTemplate bt;
                bt.slot = b["slot"].string_value();
                bt.kind = b["kind"].string_value();
                bt.cls = b["class"].string_value();
                for (auto &i : b["inputs"].array_items())
                    bt.inputs.emplace_back(i["pin"].string_value(), i["wire"].string_value());
                for (auto &o : b["outputs"].array_items()) {
                    std::vector<std::string> ws;
                    for (auto &w : o["wires"].array_items())
                        ws.push_back(w.string_value());
                    bt.outputs.emplace_back(o["pin"].string_value(), std::move(ws));
                }
                tc.bels.push_back(std::move(bt));
            }
            for (auto &p : kv.second["pips"].array_items()) {
                PipTemplate pt;
                pt.kind = p["kind"].string_value();
                pt.dst = p["dst"].string_value();
                pt.src = p["src"].string_value();
                for (auto &b : p["bits"].array_items())
                    pt.bits.push_back(b.string_value());
                if (p["bit"].is_string())
                    pt.bits.push_back(p["bit"].string_value());
                for (auto &s : p["src"].object_items())
                    pt.mux_src.emplace_back(s.first, s.second.string_value());
                tc.pips.push_back(std::move(pt));
            }
            classes[kv.first] = std::move(tc);
        }

        // `bond` 表是**权威**：哪个 (cell, IOI bel) 真正键合到封装引脚。
        // 不是所有 IOI bel 都有引脚（西边 IOB tile 覆盖 4 个 cell，但只有部分有 pad）；
        // 无引脚的站点其 bel 映射会与邻站重叠（s3pack 的冲突检测会报出来）。
        for (auto &kv : root["bond"].object_items()) {
            for (auto &r : kv.second.array_items()) {
                int cx = 0, cy = 0, bi = 0;
                if (std::sscanf(r.string_value().c_str(), "D0X%dY%d.IOI[%d]", &cx, &cy, &bi) == 3) {
                    std::string key = std::to_string(cx) + "," + std::to_string(cy) + "," +
                                      std::to_string(bi);
                    bonded_pads.insert(key);
                    bond_pins[key].push_back(kv.first); // 引脚名（如 "P78"）
                }
            }
        }
        log_info("Spartan-3E: %zu bonded IOI pads from bond table\n", bonded_pads.size());

        // `iob_map`：IOB 类 -> [(cell_idx, ioi, kind)]，来自 prjcombine 的 `IobTileData.iobs`。
        // kind=="Ibuf" 的站点是**只能输入**的 pad（例如 P13/P30/P38/P39/P69/P88/P89）。
        for (auto &kv : root["iob_map"].object_items()) {
            std::vector<std::tuple<int, int, std::string>> entries;
            for (auto &e : kv.second.array_items())
                entries.emplace_back(e["cell"].int_value(), e["ioi"].int_value(),
                                     e["kind"].string_value());
            iob_map[kv.first] = std::move(entries);
        }

        for (auto &t : root["tiles"].array_items()) {
            TileInst ti;
            ti.x = t["x"].int_value();
            ti.y = t["y"].int_value();
            ti.slot = t["slot"].string_value();
            ti.cls = t["class"].string_value();
            for (auto &c : t["cells"].array_items())
                if (c.array_items().size() == 2)
                    ti.cells.emplace_back(c.array_items()[0].int_value(),
                                          c.array_items()[1].int_value());
            tiles.push_back(ti);
        }

        // 由 `iob_map` + IOB tile 实例，算出「只能输入的 (cellx,celly,ioi)」集合。
        for (auto &t : tiles) {
            if (t.slot != "IOB")
                continue;
            auto mit = iob_map.find(t.cls);
            if (mit == iob_map.end())
                continue;
            for (auto &e : mit->second) {
                int cell = std::get<0>(e), ioi = std::get<1>(e);
                const std::string &kind = std::get<2>(e);
                if (kind != "Ibuf" || cell < 0 || cell >= (int)t.cells.size())
                    continue;
                input_only_ioi.insert(std::to_string(t.cells[cell].first) + "," +
                                      std::to_string(t.cells[cell].second) + "," +
                                      std::to_string(ioi));
            }
        }
        log_info("Spartan-3E: %zu input-only bonded IOI pads\n", input_only_ioi.size());

        for (auto &n : root["nodes"].array_items())
            nodes.push_back(Node{n[0].string_value(), n[1].string_value(), n[2].int_value(), n[3].int_value()});

        for (auto &kv : root["wiremap"].object_items()) {
            std::map<std::string, int> m;
            for (auto &w : kv.second.object_items())
                m[w.first] = w.second.int_value();
            wiremap[kv.first] = std::move(m);
        }

        log_info("Spartan-3E: chipdb '%s': %zu tiles, %zu tile classes, %zu nodes\n", db_path.c_str(),
                 tiles.size(), classes.size(), nodes.size());

        for (auto &kv : classes) {
            class_index[kv.first] = (int)class_list.size();
            class_list.push_back(kv.first);
        }
    }

    // ---------------- 工具 ----------------
    const std::map<std::string, int> *tile_wm(const TileInst &t) const
    {
        auto it = wiremap.find(t.key());
        return it == wiremap.end() ? nullptr : &it->second;
    }
    WireId lookup(const std::map<std::string, int> &wm, const std::string &wire) const
    {
        auto it = wm.find(wire);
        if (it == wm.end())
            return WireId();
        return node_wires.at(it->second);
    }
    static const std::string *pin_of(const BelTemplate &b, const std::string &pin)
    {
        for (auto &i : b.inputs)
            if (i.first == pin)
                return &i.second;
        return nullptr;
    }
    static const std::string *out_of(const BelTemplate &b, const std::string &pin)
    {
        for (auto &o : b.outputs)
            if (o.first == pin && !o.second.empty())
                return &o.second.front();
        return nullptr;
    }
    PipId add_pip(WireId src, WireId dst, Loc loc, delay_t delay, uint64_t meta)
    {
        // 名字必须全局唯一：nextpnr 用 pip_by_name 做去重断言。
        // SSOArray<IdString,4> 支持最多 4 段内联，成本低。
        IdStringList name = IdStringList::concat(
                IdStringList::concat(ctx->getWireName(dst), ctx->getWireName(src)),
                ctx->idf("#%u", pip_seq++));
        PipId pip = ctx->addPip(name, id_PIP, src, dst, delay, loc);
        if (pip_meta.size() <= (size_t)pip.index)
            pip_meta.resize(pip.index + 1, 0);
        pip_meta[pip.index] = meta;
        return pip;
    }

    // ---------------- init ----------------
    void init(Context *ctx) override
    {
        load_db();
        init_uarch_constids(ctx);
        ViaductAPI::init(ctx);
        h.init(ctx);
        if (with_gui)
            init_bel_decals();
        init_wires();
        init_bels();
        init_clock_pairs();
        init_pips();
    }

    void init_wires()
    {
        log_info("Spartan-3E: creating %zu wires...\n", nodes.size());
        node_wires.resize(nodes.size());
        for (size_t i = 0; i < nodes.size(); i++) {
            node_wires[i] = ctx->addWire(IdStringList(ctx->id(nodes[i].name)), ctx->id(nodes[i].kind),
                                        nodes[i].x, nodes[i].y);
        }
    }

    // SLICE 内部 bel 的 z 次序：LUT(z+0) FF(z+1) LUTY(z+2) FFY(z+3)
    // 这样 `constrain_cell_pairs(delta_z=1)` 对 X/Y 两半都成立。
    void add_slice_bels(const TileInst &t, const BelTemplate &b, const std::map<std::string, int> &wm, int &z)
    {
        int i = parse_bracket_index(b.slot);
        add_slice_half(t, b, wm, z, i, "F", "X", "BX", "XQ", id_LUT4, id_DFF, "SLICE%d_LUT", "SLICE%d_FF",
                       0);
        add_slice_half(t, b, wm, z, i, "G", "Y", "BY", "YQ", id_LUT4, id_DFF, "SLICE%d_LUTY", "SLICE%d_FFY",
                       1);
        add_lut5_bel(t, b, wm, z, i);
    }

    // T7：slice 级 5 输入宽 LUT（F/G 两个 4-LUT + F5 宽 mux）。
    //   F5 = S ? F : G（S=BX）；F 用 I[0..3]（F1..F4），G 用 J[0..3]（G1..G4），输出到 X。
    void add_lut5_bel(const TileInst &t, const BelTemplate &b, const std::map<std::string, int> &wm, int &z,
                      int i)
    {
        int x = t.x, y = t.y;
        BelId l5 = ctx->addBel(h.xy_id(x, y, ctx->idf("SLICE%d_LUT5", i)), id_LUT5, Loc(x, y, z), false, false);
        z++;
        for (int k = 0; k < 4; k++) {
            if (const std::string *w = pin_of(b, "F" + std::to_string(k + 1))) {
                WireId nw = lookup(wm, *w);
                if (nw != WireId())
                    ctx->addBelInput(l5, ctx->idf("I[%d]", k), nw);
            }
            if (const std::string *w = pin_of(b, "G" + std::to_string(k + 1))) {
                WireId nw = lookup(wm, *w);
                if (nw != WireId())
                    ctx->addBelInput(l5, ctx->idf("J[%d]", k), nw);
            }
        }
        if (const std::string *w = pin_of(b, "BX")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(l5, id_S, nw);
        }
        if (const std::string *w = out_of(b, "X")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelOutput(l5, id_O, nw);
        }
    }

    // T7：F6（6 输入宽 LUT），跨两个 slice：
    //   X 半 = slice0(F/G) + slice2(F/G)，输出到 slice0 的 Y（GYMUX=FX）；
    //   Y 半 = slice1 + slice3，输出到 slice1 的 Y。
    //   BX0/BX2 作 I4，BY0 作 I5。
    void add_lut6_bels(const TileInst &t, const std::vector<BelTemplate> &sl,
                       const std::map<std::string, int> &wm, int &z)
    {
        int x = t.x, y = t.y;
        for (int hf = 0; hf < 2; hf++) {
            const BelTemplate &ba = sl[hf];
            const BelTemplate &bb = sl[hf + 2];
            BelId b6 = ctx->addBel(h.xy_id(x, y, ctx->idf("CLB_LUT6_%d", hf)), id_LUT6, Loc(x, y, z),
                                   false, false);
            z++;
            auto bind = [&](const char *pin, const BelTemplate &bt, const std::string &p) {
                if (const std::string *w = pin_of(bt, p)) {
                    WireId nw = lookup(wm, *w);
                    if (nw != WireId())
                        ctx->addBelInput(b6, ctx->id(pin), nw);
                }
            };
            for (int k = 0; k < 4; k++) {
                std::string sf = "F" + std::to_string(k + 1), sg = "G" + std::to_string(k + 1);
                bind(("FI[" + std::to_string(k) + "]").c_str(), ba, sf);
                bind(("GI[" + std::to_string(k) + "]").c_str(), ba, sg);
                bind(("FJ[" + std::to_string(k) + "]").c_str(), bb, sf);
                bind(("GJ[" + std::to_string(k) + "]").c_str(), bb, sg);
            }
            bind("BX0", ba, "BX");
            bind("BX2", bb, "BX");
            bind("BY0", ba, "BY");
            if (const std::string *w = out_of(ba, "Y")) {
                WireId nw = lookup(wm, *w);
                if (nw != WireId())
                    ctx->addBelOutput(b6, id_O, nw);
            }
        }
    }

    // T7：F7（7 输入宽 LUT），用 4 个 slice 的 F5 + 它们之间的 F7；输出到 slice2 的 Y。
    void add_lut7_bel(const TileInst &t, const std::vector<BelTemplate> &sl,
                      const std::map<std::string, int> &wm, int &z)
    {
        int x = t.x, y = t.y;
        BelId b7 = ctx->addBel(h.xy_id(x, y, ctx->id("CLB_LUT7")), id_LUT7, Loc(x, y, z), false, false);
        z++;
        static const char *fn[4] = {"FI", "FJ", "FK", "FL"};
        static const char *gn[4] = {"GI", "GJ", "GK", "GL"};
        static const char *bxn[4] = {"BXI", "BXJ", "BXK", "BXL"};
        for (int i = 0; i < 4; i++) {
            for (int k = 0; k < 4; k++) {
                std::string sf = "F" + std::to_string(k + 1), sg = "G" + std::to_string(k + 1);
                if (const std::string *w = pin_of(sl[i], sf)) {
                    WireId nw = lookup(wm, *w);
                    if (nw != WireId())
                        ctx->addBelInput(b7, ctx->idf("%s[%d]", fn[i], k), nw);
                }
                if (const std::string *w = pin_of(sl[i], sg)) {
                    WireId nw = lookup(wm, *w);
                    if (nw != WireId())
                        ctx->addBelInput(b7, ctx->idf("%s[%d]", gn[i], k), nw);
                }
            }
            if (const std::string *w = pin_of(sl[i], "BX")) {
                WireId nw = lookup(wm, *w);
                if (nw != WireId())
                    ctx->addBelInput(b7, ctx->id(bxn[i]), nw);
            }
        }
        static const char *byn[3] = {"BYI", "BYJ", "BYK"};
        for (int i = 0; i < 3; i++) {
            if (const std::string *w = pin_of(sl[i], "BY")) {
                WireId nw = lookup(wm, *w);
                if (nw != WireId())
                    ctx->addBelInput(b7, ctx->id(byn[i]), nw);
            }
        }
        if (const std::string *w = out_of(sl[2], "Y")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelOutput(b7, id_O, nw);
        }
    }

    // T7：F8（8 输入宽 LUT）= 本 CLB 的 F7 + **上方** CLB 的 F7 + F8。
    //   FX3 = BY3 ? FX2(this) : FX2(above)；输出到本 CLB slice3 的 Y。
    void add_lut8_bel(const TileInst &t, const std::vector<BelTemplate> &sl,
                      const std::map<std::string, int> &wm, const std::vector<BelTemplate> &sl2,
                      const std::map<std::string, int> &wm2, int &z)
    {
        int x = t.x, y = t.y;
        BelId b8 = ctx->addBel(h.xy_id(x, y, ctx->id("CLB_LUT8")), id_LUT8, Loc(x, y, z), false, false);
        z++;
        static const char *fn[4] = {"FI", "FJ", "FK", "FL"};
        static const char *gn[4] = {"GI", "GJ", "GK", "GL"};
        static const char *bxn[4] = {"BXI", "BXJ", "BXK", "BXL"};
        static const char *mn[4] = {"MI", "MJ", "MK", "ML"};
        static const char *nn[4] = {"NI", "NJ", "NK", "NL"};
        static const char *bxn2[4] = {"BXM", "BXN", "BXO", "BXP"};
        auto bin = [&](const char *pin, const BelTemplate &bt, const std::map<std::string, int> &m,
                       const std::string &p) {
            if (const std::string *w = pin_of(bt, p)) {
                WireId nw = lookup(m, *w);
                if (nw != WireId())
                    ctx->addBelInput(b8, ctx->id(pin), nw);
            }
        };
        for (int i = 0; i < 4; i++)
            for (int k = 0; k < 4; k++) {
                std::string sf = "F" + std::to_string(k + 1), sg = "G" + std::to_string(k + 1);
                bin((std::string(fn[i]) + "[" + std::to_string(k) + "]").c_str(), sl[i], wm, sf);
                bin((std::string(gn[i]) + "[" + std::to_string(k) + "]").c_str(), sl[i], wm, sg);
                bin((std::string(mn[i]) + "[" + std::to_string(k) + "]").c_str(), sl2[i], wm2, sf);
                bin((std::string(nn[i]) + "[" + std::to_string(k) + "]").c_str(), sl2[i], wm2, sg);
            }
        for (int i = 0; i < 4; i++) {
            bin(bxn[i], sl[i], wm, "BX");
            bin(bxn2[i], sl2[i], wm2, "BX");
        }
        static const char *byn[3] = {"BYI", "BYJ", "BYK"};
        static const char *byn2[3] = {"BYM", "BYN", "BYO"};
        for (int i = 0; i < 3; i++) {
            bin(byn[i], sl[i], wm, "BY");
            bin(byn2[i], sl2[i], wm2, "BY");
        }
        bin("BYL", sl[3], wm, "BY");
        if (const std::string *w = out_of(sl[3], "Y")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelOutput(b8, id_O, nw);
        }
    }

    void add_slice_half(const TileInst &t, const BelTemplate &b, const std::map<std::string, int> &wm, int &z,
                        int i, const char *lut_pin_prefix, const char *out_pin, const char *d_pin,
                        const char *q_pin, IdString lut_type, IdString ff_type, const char *lut_name,
                        const char *ff_name, int half)
    {
        int x = t.x, y = t.y;
        WireId on, dn, bn, qn;

        // --- LUT 半边 ---
        BelId lut = ctx->addBel(h.xy_id(x, y, ctx->idf(lut_name, i)), lut_type, Loc(x, y, z), false, false);
        int lut_z = z++;
        for (int k = 0; k < 4; k++) {
            const std::string *w = pin_of(b, std::string(lut_pin_prefix) + std::to_string(k + 1));
            if (!w)
                continue;
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(lut, ctx->idf("I[%d]", k), nw);
        }
        if (const std::string *w = out_of(b, out_pin)) {
            on = lookup(wm, *w);
            if (on != WireId())
                ctx->addBelOutput(lut, id_F, on);
        }

        // --- FF 半边 ---
        BelId ff = ctx->addBel(h.xy_id(x, y, ctx->idf(ff_name, i)), ff_type, Loc(x, y, z), false, false);
        z++;
        if (const std::string *w = pin_of(b, d_pin)) {
            dn = lookup(wm, *w);
            if (dn != WireId())
                ctx->addBelInput(ff, id_D, dn);
        }
        if (const std::string *w = pin_of(b, "CLK")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(ff, id_CLK, nw);
        }
        // CE / SR：S3E 的 CLK/CE/SR 都经一級「可编程反相器」（proginv）进 bel，
        // chipdb 里 bel 的引脚绑的是 IMUX_{CE,SR}_OPTINV，proginv pip 由 uarch
        // 当普通 pip 处理（bit=1 表示建立连接）。见 docs/bitstream-notes.md §6。
        if (const std::string *w = pin_of(b, "CE")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(ff, id_CE, nw);
        }
        if (const std::string *w = pin_of(b, "SR")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(ff, id_SR, nw);
        }
        if (const std::string *w = out_of(b, q_pin)) {
            qn = lookup(wm, *w);
            if (qn != WireId())
                ctx->addBelOutput(ff, id_Q, qn);
        }

        // --- 内部 MUX：LUT 输出 -> FF 的 D 缓冲线（仅本 SLICE 内）---
        // 物理上不是 pip，而是 bel 属性 DXMUX/DYMUX；s3pack 会翻译成属性。
        if (on != WireId() && dn != WireId() && on != dn) {
            uint64_t meta = META_SPECIAL | ((uint64_t)half << 62) | (uint64_t)i;
            // Loc 的 z 用 LUT 的（仅供 GUI/日志，不影响位流）
            add_pip(on, dn, Loc(x, y, lut_z), ctx->getDelayFromNS(0.05), meta);
        }
    }

    BelId add_ioi_bel(const TileInst &t, const BelTemplate &b, const std::map<std::string, int> &wm,
                      int &z, bool input_only)
    {
        int x = t.x, y = t.y;
        BelId iob = ctx->addBel(h.xy_id(x, y, ctx->id(b.slot)), id_IOB, Loc(x, y, z), false, false);
        int zi = z++;
        if (input_only)
            input_only_bel.insert(iob);
        WireId fan_out;
        if (const std::string *w = out_of(b, "I")) {
            fan_out = lookup(wm, *w);
            if (fan_out != WireId())
                ctx->addBelOutput(iob, id_O, fan_out);
        }
        // T4 专用时钟输入：IOI 的输入 buffer 同时驱动 fabric 输出 `I`（OUT_FAN）与
        // 专用时钟输出 `CLKPAD`（OUT_CLKPAD，进 `IMUX_BUFG_CLK`）。两个是**同一个 buffer
        // 的 fan-out**，用一条 0 延时的「虚拟 pip」表达，让布线器可以把时钟网从 OUT_FAN
        // 拐到 OUT_CLKPAD，进而走低 jitter 直连（`docs/clock-tree.md` §4.3）。
        // 该 pip **没有配置位**，postRoute 不写进 dump（META_VIRTUAL）。
        if (const std::string *w = out_of(b, "CLKPAD")) {
            WireId clkpad = lookup(wm, *w);
            if (clkpad != WireId()) {
                ctx->addBelOutput(iob, id_CLKPAD, clkpad);
                clkpad_src[clkpad] = iob;
                if (fan_out != WireId() && fan_out != clkpad)
                    add_pip(fan_out, clkpad, Loc(x, y, zi), ctx->getDelayFromNS(0.0), META_VIRTUAL);
            }
        }
        if (const std::string *w = pin_of(b, "O1")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(iob, id_I, nw);
        }
        // 三态控制：IOI 的 `T1` 输入 -> bel 引脚 `T`。
        // 配置位（MUX_O=O1/MUX_T=T1）与普通 OBUF **完全相同**（实测 pass/tristate 位流），
        // 区别只在于 T1 是否被布线驱动 —— 不驱动即恒导通。
        if (const std::string *w = pin_of(b, "T1")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(iob, id_T, nw);
        }
        WireId pad = ctx->addWire(h.xy_id(x, y, ctx->idf("PAD%d", zi)), id_PAD, x, y);
        ctx->addBelInout(iob, id_PAD, pad);

        // T5b：IOB 输入 FF（IOI 的 IFF1/IFF2）。D ← PAD，Q → IQ1，CLK ← ICLK1。
        // 独立 bel（与 IOB 同 (x,y)，z 紧随），pack() 把「INBUF 直接喂 DFF」的 DFF
        // 约束到这里（delta_z=1）；见 docs/t5-iob-ff.md §6。
        BelId ioff = ctx->addBel(h.xy_id(x, y, ctx->idf("IOB_FF%d", parse_bracket_index(b.slot))),
                                 id_IOB_FF, Loc(x, y, z), false, false);
        z++;
        // D 绑 IOI 的输入 buffer 输出（OUT_FAN，`I`）而不是 pad：这样 INBUF.O→DFF.D 这条网
        // 的 driver/sink 同线，无需布线；且 INBUF 仍有连接（HeAP 才把它放进 place_cells）。
        if (fan_out != WireId())
            ctx->addBelInput(ioff, id_D, fan_out);
        if (const std::string *w = out_of(b, "IQ1")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelOutput(ioff, id_Q, nw);
        }
        if (const std::string *w = pin_of(b, "ICLK1")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(ioff, id_CLK, nw);
        }
        if (const std::string *w = pin_of(b, "ICE")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(ioff, id_CE, nw);
        }
        if (const std::string *w = pin_of(b, "SR")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(ioff, id_SR, nw);
        }

        // T8c：IOB 输出 FF（IOI 的 OFF1/OFF2）。与输入 FF 相对：D ← 来自 fabric 的 `O1`，
        // Q → 只驱动 IOI 内部的输出缓冲（用一条哑线承载，不入 fabric）。
        // CLK/CE/SR ← `OTCLK1`/`OCE`/`SR`。pack() 把「DFF.Q 驱动 OUTBUF.I」的 FF 移到这里，
        // 并给 OUTBUF 注入 `OMUX::OFF1` 等参数；见 docs/t8c-iob-ff-sr.md。
        BelId ooff = ctx->addBel(h.xy_id(x, y, ctx->idf("IOB_OFF%d", parse_bracket_index(b.slot))),
                                 id_IOB_OFF, Loc(x, y, z), false, false);
        z++;
        if (const std::string *w = pin_of(b, "O1")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(ooff, id_D, nw);
        }
        {
            WireId qw = ctx->addWire(h.xy_id(x, y, ctx->idf("OFFQ%d", zi)), id_Q, x, y);
            ctx->addBelOutput(ooff, id_Q, qw);
        }
        if (const std::string *w = pin_of(b, "OTCLK1")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(ooff, id_CLK, nw);
        }
        if (const std::string *w = pin_of(b, "OCE")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(ooff, id_CE, nw);
        }
        if (const std::string *w = pin_of(b, "SR")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(ooff, id_SR, nw);
        }
        return iob;
    }

    // 全局时钟缓冲（BUFGMUX bel，class=BUFGMUX）。时钟树布线（`GCLK_N/S` → `GCLK_QUAD`
    // → `GCLK` → `IMUX_CLK`）在 chipdb 里已完整（见 prjcombine
    // `docs/src/spartan3/clock/README.md`），`init_pips()` 会把它们全部建出；
    // 这里的 bel 只需给出引脚：`I` ← `IMUX_BUFG_CLK[i]`（即 I0），`O` → `GCLK_N/S[i]`。
    void add_bufg_bel(const TileInst &t, const BelTemplate &b, const std::map<std::string, int> &wm, int &z)
    {
        int x = t.x, y = t.y;
        BelId bel = ctx->addBel(h.xy_id(x, y, ctx->id(b.slot)), id_BUFG, Loc(x, y, z), false, false);
        bufg_bel_by_key[std::to_string(x) + "," + std::to_string(y) + "," + b.slot] = bel;
        z++;
        if (const std::string *w = pin_of(b, "I0")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelInput(bel, id_I, nw);
        }
        if (const std::string *w = out_of(b, "O")) {
            WireId nw = lookup(wm, *w);
            if (nw != WireId())
                ctx->addBelOutput(bel, id_O, nw);
        }
    }

    // T4：建立「IOI bel（CLKPAD）<-> BUFGMUX bel」的专用直连配对表。
    // 逐 CLK tile 找 `IMUX_BUFG_CLK[k]`（= BUFGMUX[k].I0）mux 的 `OUT_CLKPAD` 源，
    // 解析成实际驱动它的 IOI bel。
    void init_clock_pairs()
    {
        for (auto &t : tiles) {
            if (t.slot != "CLK")
                continue;
            auto cit = classes.find(t.cls);
            if (cit == classes.end())
                continue;
            const std::map<std::string, int> *wmp = tile_wm(t);
            if (!wmp)
                continue;
            const auto &wm = *wmp;
            for (auto &b : cit->second.bels) {
                if (b.cls != "BUFGMUX")
                    continue;
                auto bit = bufg_bel_by_key.find(std::to_string(t.x) + "," + std::to_string(t.y) + "," +
                                                b.slot);
                if (bit == bufg_bel_by_key.end())
                    continue;
                BelId bbel = bit->second;
                const std::string *i0 = pin_of(b, "I0");
                if (!i0)
                    continue;
                for (auto &p : cit->second.pips) {
                    if (p.dst != *i0)
                        continue;
                    for (auto &s : p.mux_src) {
                        if (s.first.find("OUT_CLKPAD") == std::string::npos)
                            continue;
                        auto sit = wm.find(s.first);
                        if (sit == wm.end())
                            continue;
                        WireId sw = node_wires.at(sit->second);
                        auto iit = clkpad_src.find(sw);
                        if (iit == clkpad_src.end())
                            continue;
                        bufg_pairs[bbel].push_back(iit->second);
                        iob_pairs[iit->second].push_back(bbel);
                    }
                }
            }
        }
        size_t n = 0;
        for (auto &kv : bufg_pairs)
            n += kv.second.size();
        log_info("Spartan-3E: %zu BUFG<->clock-pad dedicated pairings\n", n);
    }

    void init_bels()
    {
        log_info("Spartan-3E: creating bels...\n");
        // I11: 只有 (x,y) 处存在同名 IOB[n] 的 IOI[n] 才有键合 pad，才建 IO bel。
        // S3E 有 160 个 IOI tile，但 vq100 只有约 44 个配有 IOB。
        // I19：IOB tile 是「锚点 + 覆盖 cell 集合」，一个 IOB tile 可覆盖多个 cell，
        // **同一个焊盘的 IOI 与 IOB 不保证在同一 (x,y)**。
        // 例：P24 的 IOI 在 (2,0)，其 IOB 在 (1,0) 的 `IOB[1]`（`cells=[[1,0],[2,0]]`）。
        // 所以 bonding 要按「cell 是否被某个 IOB tile 覆盖」判断，而不是 (x,y) 相等。
        std::set<std::string> io_bonded; // "cellx,celly,bel序号"
        for (auto &t : tiles) {
            if (t.slot != "IOB")
                continue;
            auto cit = classes.find(t.cls);
            if (cit == classes.end())
                continue;
            // 该 tile 的 IOB bel 数（按顺序对应 covered cells）
            std::vector<int> iob_bels;
            for (auto &b : cit->second.bels)
                if (b.kind == "bel" && b.cls == "IOB")
                    iob_bels.push_back(parse_bracket_index(b.slot));
            for (size_t i = 0; i < t.cells.size() && i < iob_bels.size(); i++) {
                io_bonded.insert(std::to_string(t.cells[i].first) + "," +
                                 std::to_string(t.cells[i].second) + "," + std::to_string(iob_bels[i]));
            }
        }
        size_t inter = 0;
        for (auto &k : io_bonded)
            if (bonded_pads.count(k))
                inter++;
        log_info("Spartan-3E: %zu bonded IOI positions (of %zu IOI bels); 与 bond 表交集=%zu\n",
                 io_bonded.size(), (size_t)160 * 3, inter);
        // Loc 的 z 必须**按 (x,y) 全局唯一**：同一 (x,y) 可能有多个 tile slot
        // （如 (18,47) 同时有 IOI 和 CLK），各自从 0 数会撞。
        std::map<std::pair<int, int>, int> zpos;
        for (auto &t : tiles) {
            auto cit = classes.find(t.cls);
            if (cit == classes.end())
                continue;
            const std::map<std::string, int> *wmp = tile_wm(t);
            if (!wmp)
                continue;
            const auto &wm = *wmp;
            int &z = zpos[{t.x, t.y}];
            std::vector<BelTemplate> slice_tmpls;
            for (auto &b : cit->second.bels) {
                if (b.kind != "bel")
                    continue;
                if (is_slice_cls(b.cls)) {
                    add_slice_bels(t, b, wm, z);
                    slice_tmpls.push_back(b);
                    if (slice_tmpls.size() == 4) {
                        add_lut6_bels(t, slice_tmpls, wm, z);
                        add_lut7_bel(t, slice_tmpls, wm, z);
                        // F8 需要上方 (x,y+1) 的 CLB
                        for (auto &t2 : tiles) {
                            if (t2.x != t.x || t2.y != t.y + 1 || t2.slot != "BEL" || t2.cls != t.cls)
                                continue;
                            auto cit2 = classes.find(t2.cls);
                            const std::map<std::string, int> *wm2p = tile_wm(t2);
                            if (cit2 == classes.end() || !wm2p)
                                break;
                            std::vector<BelTemplate> sl2;
                            for (auto &b2 : cit2->second.bels)
                                if (b2.kind == "bel" && is_slice_cls(b2.cls))
                                    sl2.push_back(b2);
                            if (sl2.size() == 4)
                                add_lut8_bel(t, slice_tmpls, wm, sl2, *wm2p, z);
                            break;
                        }
                    }
                }
                else if (b.cls == "BUFGMUX")
                    add_bufg_bel(t, b, wm, z);
                else if (is_ioi_cls(b.cls)) {
                    const std::string k = std::to_string(t.x) + "," + std::to_string(t.y) + "," +
                                          std::to_string(parse_bracket_index(b.slot));
                    const std::string bk = std::to_string(t.x) + "," + std::to_string(t.y) + "," +
                                           std::to_string(parse_bracket_index(b.slot));
                    // 注意：`io_bonded` 的 bel 下标是 **IOB** 空间，而 bond 表给的是
                    // **IOI** 空间，两者不能混用（曾因此把 66 个键合 pad 过滤成 21 个）。
                    // 权威判据只用 bond 表：该 (cell, IOI bel) 是不是真引脚。
                    (void)k;
                    if (bonded_pads.count(bk)) {
                        BelId iob = add_ioi_bel(t, b, wm, z, input_only_ioi.count(bk) > 0);
                        for (auto &p : bond_pins[bk])
                            pin_bel[p] = iob;
                    }
                }
            }
        }
    }

    // 时钟树延迟：让全局时钟走 GCLK 专用树（低 skew），而不是从 `GCLK_*`→`OMUX`
    // 拐进 fabric。pi p 延迟模型是单值，所以用「树内便宜、离开树贵」来引导布线器。
    delay_t pip_delay(const std::string &src, const std::string &dst) const
    {
        bool gs = src.find("GCLK") != std::string::npos;
        bool gd = dst.find("GCLK") != std::string::npos;
        if (!gs && !gd)
            return ctx->getDelayFromNS(0.05);
        // 从时钟树出去到普通互连（OMUX）——重罚
        if (gs && dst.find("OMUX") != std::string::npos)
            return ctx->getDelayFromNS(9.0);
        // 树内连线 / 到 FF 时钟输入（IMUX_CLK）——便宜
        return ctx->getDelayFromNS(0.005);
    }

    void init_pips()
    {
        log_info("Spartan-3E: creating pips...\n");
        for (auto &t : tiles) {
            auto cit = classes.find(t.cls);
            if (cit == classes.end())
                continue;
            const std::map<std::string, int> *wmp = tile_wm(t);
            if (!wmp)
                continue;
            const auto &wm = *wmp;
            int cidx = class_index.at(t.cls);
            Loc loc(t.x, t.y, 0);
            for (size_t pi = 0; pi < cit->second.pips.size(); pi++) {
                const auto &p = cit->second.pips[pi];
                if (p.kind == "mux") {
                    auto dit = wm.find(p.dst);
                    if (dit == wm.end())
                        continue;
                    WireId dw = node_wires.at(dit->second);
                    for (size_t si = 0; si < p.mux_src.size(); si++) {
                        auto sit = wm.find(p.mux_src[si].first);
                        if (sit == wm.end())
                            continue;
                        WireId sw = node_wires.at(sit->second);
                        if (sw == dw)
                            continue;
                        add_pip(sw, dw, loc, pip_delay(p.mux_src[si].first, p.dst), meta_of(cidx, pi, si));

                    }
                } else if (p.kind == "progbuf" || p.kind == "proginv" || p.kind == "permabuf") {
                    auto dit = wm.find(p.dst);
                    auto sit = wm.find(p.src);
                    if (dit == wm.end() || sit == wm.end())
                        continue;
                    WireId dw = node_wires.at(dit->second);
                    WireId sw = node_wires.at(sit->second);
                    if (sw == dw)
                        continue;
                    // T13: `proginv` is a *permanent* buffer whose bit only selects polarity
                    // (prjcombine `ProgInv`: bit=1 -> inverted, bit=0 -> straight). The uarch
                    // does not model per-FF polarity (CLKINV/CEINV/SRINV), so expose it as a
                    // permanent non-inverting (virtual) edge: the router traverses it for free
                    // and s3pack writes no bit -> the bit stays 0 (non-inverted), which is
                    // correct for active-high logic. See docs/known-issues.md I7.
                    uint64_t pmeta = (p.kind == "proginv") ? META_VIRTUAL : meta_of(cidx, pi, 0);
                    add_pip(sw, dw, loc, pip_delay(p.src, p.dst), pmeta);
                }
            }
        }
    }

    // ---------------- packing ----------------
    // ---- W5：引脚约束 ----
    /// 读 PCF 约束文件（只支持 `set_io <port> <pin>`，`#` 注释）
    void read_pcf()
    {
        if (pcf_path.empty())
            return;
        std::ifstream f(pcf_path);
        if (!f)
            log_error("spartan3e: cannot open PCF '%s'\n", pcf_path.c_str());
        std::string line;
        int n = 0;
        while (std::getline(f, line)) {
            auto cp = line.find('#');
            if (cp != std::string::npos)
                line = line.substr(0, cp);
            std::istringstream ss(line);
            std::string kw, port, pin;
            if (!(ss >> kw >> port >> pin))
                continue;
            if (kw == "set_io") {
                pin_constraints[port] = pin;
                n++;
            }
        }
        log_info("Spartan-3E: %d pin constraint(s) from '%s'\n", n, pcf_path.c_str());
    }

    /// 把 PCF 引脚约束绑到对应的 pad cell（`$iopadmap$top.<port>`），强度 USER = 固定。
    void apply_pin_constraints()
    {
        for (auto &kv : pin_constraints) {
            const std::string &port = kv.first;
            const std::string &pin = kv.second;
            auto bit = pin_bel.find(pin);
            if (bit == pin_bel.end()) {
                log_warning("spartan3e: PCF: unknown pin '%s' for port '%s'\n", pin.c_str(),
                            port.c_str());
                continue;
            }
            // iopadmap 的 cell 命名：bit0 = `q`（**无后缀**），bitn = `q_<n>`；标量 = 原名。
            std::string base = port, idx;
            auto lb = port.find('[');
            if (lb != std::string::npos) {
                base = port.substr(0, lb);
                idx = port.substr(lb + 1);
                if (!idx.empty() && idx.back() == ']')
                    idx.pop_back();
            }
            std::vector<std::string> cands;
            if (!idx.empty() && idx != "0")
                cands.push_back(base + "_" + idx);
            cands.push_back(base);
            if (!idx.empty() && idx != "0")
                cands.push_back(base + "_" + idx + "_"); // 宽松：旧命名
            CellInfo *c = nullptr;
            for (auto &cn : cands) {
                auto cit = ctx->cells.find(ctx->id("$iopadmap$top." + cn));
                if (cit != ctx->cells.end()) {
                    c = cit->second.get();
                    break;
                }
            }
            if (c == nullptr) {
                log_warning("spartan3e: PCF: no pad cell for port '%s'\n", port.c_str());
                continue;
            }
            if (c->bel != BelId())
                ctx->unbindBel(c->bel);
            ctx->bindBel(bit->second, c, STRENGTH_USER);
            log_info("Spartan-3E: port '%s' -> pin %s (%s)\n", port.c_str(), pin.c_str(),
                     ctx->getBelName(bit->second).str(ctx).c_str());
        }
    }

    void pack() override
    {
        // 去掉 nextpnr 自动插的 IOB（IO 缓冲已在综合阶段映射到 INBUF/OUTBUF）
        const pool<CellTypePort> top_ports{
                CellTypePort(id_INBUF, id_PAD),
                CellTypePort(id_OUTBUF, id_PAD),
                CellTypePort(id_OBUFT, id_PAD),
                CellTypePort(id_IOBUF, id_PAD),
        };
        h.remove_nextpnr_iobs(top_ports);
        read_pcf();
        apply_pin_constraints();
        // 常量 → LUT
        const dict<IdString, Property> vcc_params = {{id_INIT, Property(0xFFFF, 16)}};
        const dict<IdString, Property> gnd_params = {{id_INIT, Property(0x0000, 16)}};
        h.replace_constants(CellTypePort(id_LUT4, id_F), CellTypePort(id_LUT4, id_F), vcc_params, gnd_params);

        // T8c：IOB 输出 FF。必须**先于** LUT-FF 配对（否则 D 来自 LUT 的 FF 会先被绑到 slice）。
        // 把「DFF 的 Q 只驱动一个 OUTBUF/OBUFT/IOBUF 的 I」的 FF 打包进 `IOB_OFF` bel
        // （与 OUTBUF 同 tile，IOB 在 z、IOB_FF 在 z+1、IOB_OFF 在 z+2）。
        // 只接受无复位 / **异步**复位 FF（IOI 输出 FF 无 `SYNC_ATTR`）。
        if (getenv("S3E_NO_IOB_OFF") == nullptr) {
            int ioboffs = 0;
            for (auto &cell : ctx->cells) {
                CellInfo *ob = cell.second.get();
                if (!ob->type.in(id_OUTBUF, id_OBUFT, id_IOBUF) || ob->cluster != ClusterId())
                    continue;
                NetInfo *in = ob->getPort(id_I);
                if (!in || in->driver.cell == nullptr)
                    continue;
                CellInfo *ff = in->driver.cell;
                if (!ff->type.in(id_DFF, id_DFFE, id_DFFC, id_DFFP, id_DFFCE, id_DFFPE))
                    continue;
                if (ff->cluster != ClusterId())
                    continue;
                if (in->users.entries() > 1)
                    continue; // FF 的 Q 还驱动别处 -- 不能搬进 IOB
                // 若 D 直接由 pad（输入 buffer）驱动，优先让它进 **IOB 输入 FF**（无需布线，
                // 与 ISE 一致）；否则才考虑输出 FF。
                NetInfo *dn = ff->getPort(id_D);
                if (dn != nullptr && dn->driver.cell != nullptr &&
                    dn->driver.cell->type == id_INBUF)
                    continue;
                // 若 OUTBUF 已因 PCF 绑定了 bel（STRENGTH_USER），HeAP 不会把 cluster root
                // 放进 place_cells，于是子 FF 永远拿不到位置，`total_hpwl` 抛 `dict::at()`
                // （如 `reg q=0; always @(posedge clk) q<=1'b1;`）。直接把 FF 绑到同 tile
                // 的 `IOB_OFF` bel（z+2），让整个 cluster 已放置。
                BelId off_bel = BelId();
                Loc off_loc;
                if (ob->bel != BelId()) {
                    Loc ol = ctx->getBelLocation(ob->bel);
                    off_loc = ol;
                    for (auto bel : ctx->getBels()) {
                        if (ctx->getBelType(bel) != id_IOB_OFF)
                            continue;
                        Loc l = ctx->getBelLocation(bel);
                        if (l.x == ol.x && l.y == ol.y) {
                            off_bel = bel;
                            break;
                        }
                    }
                    if (off_bel == BelId()) {
                        log_error("spartan3e: IOB tile (%d,%d) has no IOB_OFF bel for the "
                                  "packed output FF '%s' (chipdb/uarch inconsistency)\n",
                                  ol.x, ol.y, ff->name.c_str(ctx));
                    } else if (CellInfo *bound = ctx->getBoundBelCell(off_bel)) {
                        log_warning("spartan3e: IOB_OFF bel at (%d,%d) already used by '%s'; "
                                    "leaving output FF '%s' for the CLB packer\n",
                                    off_loc.x, off_loc.y, bound->name.c_str(ctx), ff->name.c_str(ctx));
                        continue;
                    }
                }
                // 手动建 cluster：根 = pad cell（位置受焊盘约束），子 = FF（z+2 = IOB_OFF）
                ob->cluster = ob->name;
                ob->constr_abs_z = false;
                ob->constr_children.push_back(ff);
                ff->cluster = ob->name;
                ff->constr_x = 0;
                ff->constr_y = 0;
                ff->constr_z = 2;
                ff->constr_abs_z = false;
                ff->setAttr(id_IOB_OFF, Property(1));
                if (off_bel != BelId())
                    ctx->bindBel(off_bel, ff, STRENGTH_USER);
                // 物理上输出缓冲取样自 FF（OMUX=FFO1），O1 只接 FF.D。把 OUTBUF.I 从原
                // Q 网改接到 FF.D 网（= O1）：既让 O1 只承载 FF.D（否则和 OUTBUF.I 争线），
                // 又保持 OUTBUF 有输入连接（不能直接断开：HeAP 会把无连接的根 cell 当作
                // 「已锁定」而不放进 place_cells，total_hpwl 会抛 dict::at()）。
                NetInfo *dnet = ff->getPort(id_D);
                if (dnet != nullptr && dnet != in) {
                    ob->disconnectPort(id_I);
                    ob->connectPort(id_I, dnet);
                }
                ob->setParam(ctx->id("OMUX"), Property(std::string("OFF1")));
                ob->setParam(ctx->id("OFF1_INIT_ATTR"), Property(std::string("INIT0")));
                if (ff->type.in(id_DFFC, id_DFFCE, id_DFFP, id_DFFPE)) {
                    bool high = ff->type.in(id_DFFP, id_DFFPE);
                    ob->setParam(ctx->id("OFF1_SR_ATTR"),
                                 Property(std::string(high ? "SRHIGH" : "SRLOW")));
                    ob->setParam(ctx->id("OSR_USED"), Property(std::string("0")));
                    ob->setParam(ctx->id("SRINV"), Property(std::string("SR")));
                }
                ioboffs++;
            }
            log_info("Constrained %d IOB output-FF pairs.\n", ioboffs);
        }

        // LUT 与 FF 绑到同一个 SLICE：FF 相对 LUT 偏移 z+1
        int lutffs = h.constrain_cell_pairs(pool<CellTypePort>{{id_LUT4, id_F}}, pool<CellTypePort>{{id_DFF, id_D}}, 1);
        log_info("Constrained %d LUTFF pairs.\n", lutffs);

        int iobffs = 0;
        // T5b：IOB 输入 FF（默认开；`S3E_NO_IOB_FF=1` 关）。见 docs/t5-iob-ff.md §6。
        // 已知阻断：IOB 输入 FF 的时钟走 `IMUX_IOCLK`，而单边 BUFG（E/W 的 `GCLK_WE`）是
        // **regional**（只喂本侧半区）——若设计时钟在 E/W 而输入 FF 跨到另一侧，布线必失败。
        // 需先做「时钟区域/放置约束」或只在 N/S 全局时钟下启用，故先用 env 开关。
        if (getenv("S3E_NO_IOB_FF") != nullptr)
            iobffs = 0;
        else {
        // INBUF 直接喂 DFF → 把 DFF 打包进 IOB 输入 FF（`IOB_FF bel`，与 IOB 同 tile、z+1）。
        // 然后把 DFF.D 从 INBUF.O 改接到 pad（IOB_FF 的 D 绑的是 pad），并删掉旧网。
        pool<CellTypePort> ff_d_ports;
        // T8c：无复位（`DFF`/`DFFE`）与**异步**复位（`DFFC`/`DFFP`/`DFFCE`/`DFFPE`）的 FF 都可
        // 打包进 IOB 输入 FF —— IFD 的复位是异步的（IOI 无 `SYNC_ATTR`）。同步复位类
        // （`DFFR`/`DFFS`/`DFFRE`/`DFFSE`）留在 CLB，IOI 无法表达。位表见 patch-ioi-bits.py。
        for (IdString t : {id_DFF, id_DFFE, id_DFFC, id_DFFP, id_DFFCE, id_DFFPE})
            ff_d_ports.insert(CellTypePort(t, id_D));
        iobffs = h.constrain_cell_pairs(pool<CellTypePort>{{id_INBUF, id_O}}, ff_d_ports, 1, false);
        for (auto &cell : ctx->cells) {
            CellInfo *ff = cell.second.get();
            if (!is_ff_type(ff->type) || ff->cluster == ClusterId())
                continue;
            auto rit = ctx->cells.find(ff->cluster);
            if (rit == ctx->cells.end())
                continue;
            CellInfo *ib = rit->second.get();
            if (ib->type != id_INBUF)
                continue;
            NetInfo *padnet = ib->getPort(id_PAD);
            NetInfo *dn = ff->getPort(id_D);
            if (!padnet || !dn)
                continue;
            // 注意：**不能**断开 INBUF.O（否则 HeAP 认为该 cell 无连接而当作「已锁定」，
            // cluster root 不进 place_cells，DFF 子 cell 就不会被放置 → total_hpwl 抛 dict::at）。
            // DFF.D 保持接在 INBUF.O 网上；IOB_FF 的 D 也绑 OUT_FAN，同一根线。
            (void)ib;
            (void)dn;
            (void)padnet;
            // 同输出 FF：INBUF 若因 PCF 已绑定 bel，HeAP 不会把 cluster root 放进
            // place_cells，子 FF 拿不到位置 → `total_hpwl` 抛 `dict::at()`。直接绑到同 tile
            // 的 `IOB_FF` bel（z+1）。
            if (ib->bel != BelId()) {
                Loc il = ctx->getBelLocation(ib->bel);
                BelId ff_bel = BelId();
                for (auto bel : ctx->getBels()) {
                    if (ctx->getBelType(bel) != id_IOB_FF)
                        continue;
                    Loc l = ctx->getBelLocation(bel);
                    if (l.x == il.x && l.y == il.y) {
                        ff_bel = bel;
                        break;
                    }
                }
                if (ff_bel == BelId()) {
                    log_error("spartan3e: IOB tile (%d,%d) has no IOB_FF bel for the "
                              "packed input FF '%s' (chipdb/uarch inconsistency)\n",
                              il.x, il.y, ff->name.c_str(ctx));
                } else if (CellInfo *bound = ctx->getBoundBelCell(ff_bel)) {
                    log_warning("spartan3e: IOB_FF bel at (%d,%d) already used by '%s'; "
                                "leaving input FF '%s' for the CLB packer\n",
                                il.x, il.y, bound->name.c_str(ctx), ff->name.c_str(ctx));
                    continue;
                }
                ctx->bindBel(ff_bel, ff, STRENGTH_USER);
            }
            ff->setAttr(id_IOB_FF, Property(1));
            // T5b：给 INBUF 注入输入 FF 参数，使 s3pack 走与 ISE `rff4` 完全相同（已验证 0/0）
            // 的路径：IOB 侧写 IFD 延时（ISE 输入 FF 默认 `DLY3`）+ IOI 侧写 IFF 配置。
            if (ff->type.in(id_DFF, id_DFFE, id_DFFC, id_DFFP, id_DFFCE, id_DFFPE)) {
                ib->setParam(ctx->id("IFF1"), Property(std::string("1")));
                ib->setParam(ctx->id("ICLK1INV"), Property(std::string("ICLK1")));
                ib->setParam(ctx->id("IFF1_INIT_ATTR"), Property(std::string("INIT0")));
                ib->setParam(ctx->id("IFD_DELAY_VALUE"), Property(std::string("DLY3")));
                // T8c：异步复位的 FF 还要写 IOI 的 SR 配置（SRLOW/SRHIGH + used + 反相）。
                if (ff->type.in(id_DFFC, id_DFFCE, id_DFFP, id_DFFPE)) {
                    bool high = ff->type.in(id_DFFP, id_DFFPE);
                    ib->setParam(ctx->id("IFF1_SR_ATTR"),
                                 Property(std::string(high ? "SRHIGH" : "SRLOW")));
                    ib->setParam(ctx->id("ISR_USED"), Property(std::string("0")));
                    ib->setParam(ctx->id("SRINV"), Property(std::string("SR")));
                }
            }
        }
        }
        log_info("Constrained %d IOB input-FF pairs.\n", iobffs);
    }

    // 同一 SLICE 的两个 FF（X/Y 半边）**共用 CLK / CE / SR 三个输入**
    // （chipdb 里这三个引脚分别绑单条 `IMUX_{CLK,CE,SR}_OPTINV[n]`）。
    // 所以两半若都放了 FF，必须在这三个信号上一致：都有且同网，或都没有。
    // 否则布线时两条网会争同一个 sink（实测报 "Found two arcs with same sink wire"）。
    // 真实器件与 ISE 同此约束。
    bool isBelLocationValid(BelId bel, bool explain_invalid) const override
    {
        // T7/T8：CLB slice 内的「互斥资源」用 (x, y, slice, kind) 集合表示：
        //   kind: 0=F-LUT, 1=G-LUT, 2=BX, 3=BY
        // - LUT4 (F)/(G) 占 {*,F/G}；LUT5/6/7/8 额外占对应 slice 的 BX（LUT6/7/8 还占 BY）。
        // - slice FF 的 **D** 在 uarch 里绑到 BX（F 半）/ BY（G 半）——因为 DXMUX/DYMUX
        //   走的是 `X→BX`/`Y→BY` 的内部路径。所以 FF 也占一个 BX/BY。
        // 于是「宽 LUT 的 BX/BY」与「同 slice 的 FF」天然互斥（物理上也共享同一根 BX/BY 线），
        // 判定必须**对称**且跨 CLB（LUT8 影响 (x,y) 与 (x,y+1)）。
        IdString bt = ctx->getBelType(bel);
        auto resources = [](IdString t, Loc loc, const std::string &n) {
            std::vector<std::array<int, 4>> o; // (x, y, slice, kind)
            int x = loc.x, y = loc.y;
            auto add = [&](int yy, int sl, int k) { o.push_back({x, yy, sl, k}); };
            auto fg = [&](int yy, int sl) { add(yy, sl, 0); add(yy, sl, 1); };
            if (t == id_LUT5) {
                int sl = loc.z / 5;
                fg(y, sl); add(y, sl, 2);
            } else if (t == id_LUT6) {
                int hf = (!n.empty() && n.back() == '0') ? 0 : 1;
                for (int sl : {hf, hf + 2}) { fg(y, sl); add(y, sl, 2); }
                add(y, hf, 3);
            } else if (t == id_LUT7) {
                for (int sl = 0; sl < 4; sl++) { fg(y, sl); add(y, sl, 2); }
                for (int sl = 0; sl < 3; sl++) add(y, sl, 3);
            } else if (t == id_LUT8) {
                for (int yy : {y, y + 1}) {
                    for (int sl = 0; sl < 4; sl++) { fg(yy, sl); add(yy, sl, 2); }
                    for (int sl = 0; sl < 3; sl++) add(yy, sl, 3);
                }
            } else if (t == id_LUT4) {
                int r = loc.z % 5;
                add(y, loc.z / 5, (r == 0) ? 0 : 1);
            } else if (t == id_DFF) { // SLICE FF：D 绑 BX（F 半）/ BY（G 半）
                auto pos = n.find("SLICE");
                if (pos != std::string::npos) {
                    int sl = n[pos + 5] - '0';
                    bool is_y = n.size() >= 4 && n.compare(n.size() - 4, 4, "_FFY") == 0;
                    add(y, sl, is_y ? 3 : 2);
                }
            }
            return o;
        };
        auto intersects = [](const std::vector<std::array<int, 4>> &a,
                             const std::vector<std::array<int, 4>> &b) {
            for (auto &u : a)
                for (auto &v : b)
                    if (u == v)
                        return true;
            return false;
        };
        auto check_against = [&](IdString mine_type, Loc l, const std::string &nm,
                                 IdString other_type, const std::string &onm, Loc ol) {
            return intersects(resources(mine_type, l, nm), resources(other_type, ol, onm));
        };
        if (bt.in(id_LUT4, id_LUT5, id_LUT6, id_LUT7, id_LUT8)) {
            CellInfo *c = ctx->getBoundBelCell(bel);
            if (c == nullptr)
                return true;
            Loc l = ctx->getBelLocation(bel);
            std::string nm = ctx->getBelName(bel).str(ctx);
            for (auto &cl : ctx->cells) {
                CellInfo *o = cl.second.get();
                if (o == c || o->bel == BelId())
                    continue;
                IdString ot = ctx->getBelType(o->bel);
                if (!ot.in(id_LUT4, id_LUT5, id_LUT6, id_LUT7, id_LUT8) && ot != id_DFF)
                    continue;
                Loc ol = ctx->getBelLocation(o->bel);
                // 快速排除：不同列，或行距 >1（LUT8 最多影响相邻 CLB）
                if (ol.x != l.x || ol.y < l.y - 1 || ol.y > l.y + 1)
                    continue;
                if (check_against(bt, l, nm, ot, ctx->getBelName(o->bel).str(ctx), ol))
                    return false;
            }
            return true;
        }
        // T5b：只允许 pack 过（D 已接到 pad）的 FF 落到 IOB 输入 FF bel。
        // T8：且只接受**不带 SR** 的 FF（`DFF`/`DFFE`）—— IFD（IOB 输入 FF）的复位路径
        // 位表尚未建模（chipdb 缺 IOI SR 类 attr，见 TODO T8），带复位的 FF 留在 CLB。
        if (bt == id_IOB_FF || bt == id_IOB_OFF) {
            CellInfo *c = ctx->getBoundBelCell(bel);
            if (c == nullptr)
                return true;
            // T8c：只接受无复位与**异步**复位 FF（见 pack()）。
            return c->attrs.count(bt) > 0 &&
                   c->type.in(id_DFF, id_DFFE, id_DFFC, id_DFFP, id_DFFCE, id_DFFPE);
        }
        if (bt != id_DFF)
            return true;
        // FF 侧也要检查：是否与同 slice 的宽 LUT 争 BX/BY。
        {
            CellInfo *c = ctx->getBoundBelCell(bel);
            if (c != nullptr) {
                Loc l = ctx->getBelLocation(bel);
                std::string nm = ctx->getBelName(bel).str(ctx);
                for (auto &cl : ctx->cells) {
                    CellInfo *o = cl.second.get();
                    if (o == c || o->bel == BelId())
                        continue;
                    IdString ot = ctx->getBelType(o->bel);
                    if (!ot.in(id_LUT4, id_LUT5, id_LUT6, id_LUT7, id_LUT8))
                        continue;
                    Loc ol = ctx->getBelLocation(o->bel);
                    if (ol.x != l.x || ol.y < l.y - 1 || ol.y > l.y + 1)
                        continue;
                    if (check_against(bt, l, nm, ot, ctx->getBelName(o->bel).str(ctx), ol))
                        return false;
                }
            }
        }
        Loc l = ctx->getBelLocation(bel);
        // SLICE<i>_FF 在 z+1、SLICE<i>_FFY 在 z+3；配对的是另一半
        std::string nm = ctx->getBelName(bel).str(ctx);
        bool is_y = nm.size() >= 4 && nm.compare(nm.size() - 4, 4, "_FFY") == 0;
        int dz = is_y ? -2 : 2;
        if (l.z + dz < 0)
            return true;
        BelId other = ctx->getBelByLocation(Loc(l.x, l.y, l.z + dz));
        if (other == BelId() || ctx->getBelType(other) != id_DFF)
            return true;
        CellInfo *a = ctx->getBoundBelCell(bel);
        CellInfo *b = ctx->getBoundBelCell(other);
        if (a == nullptr || b == nullptr)
            return true;
        for (IdString pin : {id_CLK, id_CE, id_SR}) {
            auto get = [&](CellInfo *c) -> NetInfo * {
                auto it = c->ports.find(pin);
                return it == c->ports.end() ? nullptr : it->second.net;
            };
            NetInfo *na = get(a), *nb = get(b);
            if ((na == nullptr) != (nb == nullptr))
                return false; // 一个用、一个不用 → 共用线上会互相干扰
            if (na != nullptr && na != nb)
                return false;
        }
        return true;
    }

    // T4：把 BUFG 挪到与「喂它的时钟 pad」配对的 BUFGMUX 位置，使其能走低 jitter 专用直连
    // （`OUT_CLKPAD → IMUX_BUFG_CLK`）。用户固定过的、或目标位置被占的，不动。
    void postPlace() override
    {
        int moved = 0;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (!ci->type.in(id_BUFG, id_BUFGMUX) || ci->bel == BelId())
                continue;
            NetInfo *in = ci->getPort(id_I);
            if (in == nullptr || in->driver.cell == nullptr)
                continue;
            CellInfo *drv = in->driver.cell;
            if (drv->bel == BelId())
                continue;
            auto iit = iob_pairs.find(drv->bel);
            if (iit == iob_pairs.end())
                continue;
            const auto &paired = iit->second;
            if (std::find(paired.begin(), paired.end(), ci->bel) != paired.end())
                continue; // 已经配对
            if (ci->belStrength == STRENGTH_USER)
                continue; // 用户固定，不动
            for (BelId b2 : paired) {
                if (b2 == ci->bel || !ctx->checkBelAvail(b2))
                    continue;
                ctx->unbindBel(ci->bel);
                ctx->bindBel(b2, ci, STRENGTH_USER);
                moved++;
                break;
            }
        }
        if (moved)
            log_info("Spartan-3E: moved %d BUFG(s) to dedicated clock-input pairings\n", moved);

        // T5b：IOB 输入 FF 的时钟走 `IMUX_IOCLK`；E/W 的 BUFG 输出 `GCLK_WE` 只喂本侧半区。
        // 若设计里有 IOB 输入 FF 而该 BUFG 在 E/W，就把它挪到一个空闲的 N/S（全局 GCLK_S/N）站点。
        if (getenv("S3E_NO_IOB_FF") == nullptr) {
            std::set<CellInfo *> need_global;
            for (auto &cell : ctx->cells) {
                CellInfo *ff = cell.second.get();
                if (!ff->attrs.count(id_IOB_FF) && !ff->attrs.count(id_IOB_OFF))
                    continue;
                NetInfo *clk = ff->getPort(id_CLK);
                if (!clk || !clk->driver.cell)
                    continue;
                CellInfo *drv = clk->driver.cell;
                if (!drv->type.in(id_BUFG, id_BUFGMUX) || drv->bel == BelId())
                    continue;
                WireId ow = ctx->getNetinfoSourceWire(clk);
                if (ow == WireId())
                    continue;
                if (ctx->getWireName(ow).str(ctx).find("GCLK_WE") != std::string::npos)
                    need_global.insert(drv);
            }
            for (CellInfo *drv : need_global) {
                for (BelId b : ctx->getBels()) {
                    if (ctx->getBelType(b) != id_BUFG || !ctx->checkBelAvail(b))
                        continue;
                    WireId ow = ctx->getBelPinWire(b, id_O);
                    if (ow == WireId())
                        continue;
                    std::string w = ctx->getWireName(ow).str(ctx);
                    if (w.find("GCLK_S") == std::string::npos && w.find("GCLK_N") == std::string::npos)
                        continue;
                    ctx->unbindBel(drv->bel);
                    ctx->bindBel(b, drv, STRENGTH_USER);
                    log_info("Spartan-3E: moved BUFG %s to global clock site (IOB input FF)\n",
                             ctx->nameOf(drv));
                    break;
                }
            }
        }
    }

    // ---------------- postRoute：导出给 s3pack 的布局/布线 dump ----------------
    // nextpnr 的 --write JSON 不含路由，所以必须在这里自己写。
    void postRoute() override
    {
        if (dump_path.empty())
            return;
        log_info("Spartan-3E: writing dump to '%s'...\n", dump_path.c_str());

        json11::Json::object root;
        root["device"] = "xc3s500e";

        json11::Json::array bels;
        for (auto &cell : ctx->cells) {
            CellInfo *ci = cell.second.get();
            if (ci->bel == BelId())
                continue;
            Loc l = ctx->getBelLocation(ci->bel);
            json11::Json::object b;
            b["bel"] = ctx->getBelName(ci->bel).str(ctx);
            b["cell"] = ctx->nameOf(ci);
            b["x"] = l.x;
            b["y"] = l.y;
            b["z"] = l.z;
            b["type"] = ci->type.str(ctx);
            json11::Json::object params;
            for (auto &p : ci->params)
                params[p.first.str(ctx)] = p.second.to_string();
            b["params"] = params;
            // 记录端口→网名：s3pack 用它判断 OUTBUF 的 `I` / INBUF 的 `O` 是否真的接了。
            // 未接/常量驱动的输出 pad（如 multibufg 的 qb[3:1] 被 yosys 折成 x）不能被当成
            // 输出写 MUX_O/OUTPUT_ENABLE（ISE 对未接输出写 NONE）——见 T26/T33。
            json11::Json::object conn;
            for (auto &p : ci->ports) {
                if (p.second.net != nullptr)
                    conn[p.first.str(ctx)] = p.second.net->name.str(ctx);
            }
            b["conn"] = conn;
            if (ci->cluster != ClusterId())
                b["cluster"] = ci->cluster.str(ctx);
            bels.push_back(b);
        }
        root["bels"] = bels;

        json11::Json::array pips;
        json11::Json::array roots;
        for (auto &net : ctx->nets) {
            for (auto &w : net.second->wires) {
                PipId pip = w.second.pip;
                if (pip == PipId()) {
                    // T5b：根线（source / 恒连）——位流里无配置位，但是 net 连通性校验的起点
                    json11::Json::object r;
                    r["net"] = net.second->name.str(ctx);
                    r["wire"] = ctx->getWireName(w.first).str(ctx);
                    roots.push_back(r);
                    continue;
                }
                uint64_t m = (size_t)pip.index < pip_meta.size() ? pip_meta[pip.index] : 0;
                if (m & META_VIRTUAL)
                    continue; // 虚拟 pip：无配置位，不入 dump
                Loc l = ctx->getPipLocation(pip);
                json11::Json::object p;
                p["x"] = l.x;
                p["y"] = l.y;
                p["net"] = net.second->name.str(ctx);
                if (m & META_SPECIAL) {
                    p["special"] = (m & (1ull << 62)) ? "YMUX" : "XMUX";
                    p["slice"] = (int)(m & 0xffff);
                } else {
                    int cidx = (int)((m >> 32) & 0xffff);
                    p["tc"] = (cidx >= 0 && cidx < (int)class_list.size()) ? class_list[cidx] : "";
                    p["i"] = (int)((m >> 16) & 0xffff);
                    p["s"] = (int)(m & 0xffff);
                }
                pips.push_back(p);
            }
        }
        root["pips"] = pips;
        root["root_wires"] = roots;

        std::ofstream f(dump_path);
        if (!f)
            log_error("spartan3e: cannot write dump '%s'\n", dump_path.c_str());
        f << json11::Json(root).dump();
        log_info("Spartan-3E: dump written: %zu bels, %zu pips\n", bels.size(), pips.size());
    }

    // ---------------- bel bucket ----------------
    // 带复位的 FF 也放进 DFF 桶，这样它们能和 SLICE_FF 这个 bel 类型匹配。
    static bool is_ff_type(IdString t)
    {
        return t.in(id_DFF, id_DFFE, id_DFFR, id_DFFS, id_DFFC, id_DFFP, id_DFFRE, id_DFFSE,
                    id_DFFCE, id_DFFPE);
    }

    // T41 P4: accept both the prjcombine bel class (`SLICE`/`IOI`) and the ISE-facing one
    // emitted by `s3e-tools` (`SLICEM`/`SLICEL`, `IOB__IOI`).
    static bool is_slice_cls(const std::string &c)
    {
        return c == "SLICE" || c == "SLICEM" || c == "SLICEL";
    }
    static bool is_ioi_cls(const std::string &c) { return c == "IOI" || c == "IOB__IOI"; }
    IdString getBelBucketForCellType(IdString cell_type) const override
    {
        if (cell_type.in(id_INBUF, id_OUTBUF, id_OBUFT, id_IOBUF))
            return id_IOB;
        if (is_ff_type(cell_type))
            return id_DFF;
        return cell_type;
    }
    // T5b：IOB 输入 FF bel 与 SLICE FF 同桶（都是 DFF），
    // 这样 pack 的 cluster 能把 DFF 放到它上面。
    BelBucketId getBelBucketForBel(BelId bel) const override
    {
        IdString t = ctx->getBelType(bel);
        return (t == id_IOB_FF || t == id_IOB_OFF) ? id_DFF : t;
    }
    bool isValidBelForCellType(IdString cell_type, BelId bel) const override
    {
        IdString bel_type = ctx->getBelType(bel);
        if (bel_type == id_IOB_FF || bel_type == id_IOB_OFF)
            return is_ff_type(cell_type);
        if (bel_type == id_IOB) {
            // 只能输入的 pad 只接受 IBUF（其余带输出的 cell 拒绝）。
            if (input_only_bel.count(bel))
                return cell_type == id_INBUF;
            return cell_type.in(id_INBUF, id_OUTBUF, id_OBUFT, id_IOBUF);
        }
        if (bel_type == id_DFF)
            return is_ff_type(cell_type);
        if (bel_type.in(id_LUT5, id_LUT6, id_LUT7, id_LUT8))
            return cell_type == bel_type;
        return bel_type == cell_type;
    }



    // ---------------- GUI decals ----------------
    void init_bel_decals()
    {
        for (int z = 0; z < 4; z++) {
            float y1 = 0.2f + z * 0.05f;
            ctx->addDecalGraphic(IdStringList(ctx->idf("LUT%d", z)),
                                 GraphicElement(GraphicElement::TYPE_BOX, GraphicElement::STYLE_INACTIVE, 0.8f, y1,
                                                0.87f, y1 + 0.03f, 10.0));
        }
    }
};

struct Spartan3eArch : ViaductArch
{
    Spartan3eArch() : ViaductArch("spartan3e") {};
    std::unique_ptr<ViaductAPI> create(const dict<std::string, std::string> &args)
    {
        auto impl = std::make_unique<Spartan3eImpl>();
        if (args.count("db"))
            impl->db_path = args.at("db");
        if (args.count("dump"))
            impl->dump_path = args.at("dump");
        if (args.count("pcf"))
            impl->pcf_path = args.at("pcf");
        return impl;
    }
} spartan3eArch;
} // namespace

NEXTPNR_NAMESPACE_END
