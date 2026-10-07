/* Native clock topology: bounded pad entry and directed global distribution. */
#include "spartan3e_ise_clock.h"
#include <deque>
#include "log.h"

NEXTPNR_NAMESPACE_BEGIN

namespace {
bool numbered(const std::string &name, const char *prefix, int count, const char *suffix = "")
{
    const std::string head(prefix), tail(suffix);
    return name.size() == head.size() + 1 + tail.size() && name.compare(0, head.size(), head) == 0 &&
           name[head.size()] >= '0' && name[head.size()] < '0' + count &&
           name.compare(head.size() + 1, tail.size(), tail) == 0;
}

bool distribution_wire(const std::string &name)
{
    // Exact ISE local-wire grammar, not the DSU root name: E/W trunks
    // share nodes with IOIS fabric aliases which must not classify pips.
    for (const char *prefix : {"CLKT_GCLK_PINWIRE", "CLKB_GCLK_PINWIRE", "CLKT_GCLK_MAIN", "CLKB_GCLK_MAIN",
                              "CLKC_GCLK_MAIN_T", "CLKC_GCLK_MAIN_B"})
        if (numbered(name, prefix, 4)) return true;
    for (const char *prefix : {"CLKL_GCLK_PWO", "CLKR_GCLK_PWO", "CLKL_GCLKH_MAIN", "CLKR_GCLKH_MAIN",
                              "CLKC_GCLK", "GCLKC_GCLK", "GCLKC_GCLK_OUT_L", "GCLKC_GCLK_OUT_R",
                              "GCLKH_GCLK", "GCLKH_GCLK_UP", "GCLKH_GCLK_DN", "GCLKVML_GCLKLR",
                              "GCLKVML_GCLKCORE", "GCLKVMR_GCLKLR", "GCLKVMR_GCLKCORE",
                              "GCLKVMLR_GCLK_UP", "GCLKVMLR_GCLK_DN", "BRAMSITE2_GCLKH_GCLK",
                              "BRAMSITE2_GCLKH_GCLK_UP", "BRAMSITE2_GCLKH_GCLK_DN", "BRAMSITE2H_GCLK",
                              "BRAM2_FEEDTHRU_GCLK", "BRAM2_FEEDTHRUH_GCLK", "GCLK", "IOIS_CLK"})
        if (numbered(name, prefix, 8)) return true;
    if (numbered(name, "GCLK", 8, "_BRK") || numbered(name, "CLK", 4) || numbered(name, "BRAM_CLK", 4)) return true;
    for (const char *suffix : {"_INT0", "_INT1", "_INT2", "_INT3"})
        if (numbered(name, "BRAM_CLK", 4, suffix)) return true;
    return name == "CNR_CLK0" || name == "CNR_CLK2" || name == "DCM_CLKIN" ||
           name == "DCM_CLKFB" || name == "DCM_PSCLK";
}

bool sink_wire(const std::string &name)
{
    return numbered(name, "ICLK1", 3) || numbered(name, "OTCLK1", 3) || name == "RAMB16_CLKA" ||
           name == "RAMB16_CLKB" || name == "MULT18X18SIO_CLK" || name == "LR_STARTUP_CLK" ||
           name == "LR_CAPTURE_CLK" || name == "DCM_CLKIN_PINWIRE" || name == "DCM_CLKFB_PINWIRE" ||
           name == "DCM_PSCLK_PINWIRE";
}
}

bool ise_clock_distribution_pip(const std::string &from, const std::string &to)
{
    return distribution_wire(from) && (distribution_wire(to) || sink_wire(to));
}

void index_ise_clock_paths(Context *ctx, std::vector<IseClockSite> &clocks,
                           const std::unordered_map<int32_t, BelId> &pad_wires,
                           const std::unordered_set<int32_t> &clock_pins, int fabric_wires, int fabric_pips,
                           const std::function<bool(PipId)> &is_distribution)
{
    std::vector<std::vector<PipId>> uphill(fabric_wires), downhill(fabric_wires);
    for (int pi = 0; pi < fabric_pips; ++pi) {
        PipId pip{pi};
        uphill.at(ctx->getPipDstWire(pip).index).push_back(pip);
        downhill.at(ctx->getPipSrcWire(pip).index).push_back(pip);
    }
    size_t entries = 0, regions = 0;
    for (auto &clock : clocks) {
        for (int input = 0; input < 2; ++input) {
            std::deque<std::pair<WireId, int>> queue;
            std::unordered_map<int32_t, PipId> next;
            std::unordered_set<int32_t> seen;
            queue.emplace_back(clock.inputs[input], 0);
            seen.insert(clock.inputs[input].index);
            while (!queue.empty()) {
                auto item = queue.front();
                queue.pop_front();
                auto pad = pad_wires.find(item.first.index);
                if (pad != pad_wires.end()) {
                    std::vector<PipId> path;
                    WireId wire = item.first;
                    while (wire != clock.inputs[input]) {
                        PipId pip = next.at(wire.index);
                        path.push_back(pip);
                        wire = ctx->getPipDstWire(pip);
                    }
                    clock.pad_paths[input][pad->second.index] = std::move(path);
                    ++entries;
                    continue;
                }
                if (item.second == 3) continue;
                for (PipId pip : uphill.at(item.first.index)) {
                    WireId from = ctx->getPipSrcWire(pip);
                    if (!seen.insert(from.index).second) continue;
                    next[from.index] = pip;
                    queue.emplace_back(from, item.second + 1);
                }
            }
        }
        std::deque<WireId> queue{clock.output};
        clock.global_wires.insert(clock.output.index);
        while (!queue.empty()) {
            WireId wire = queue.front();
            queue.pop_front();
            if (clock_pins.count(wire.index)) { ++regions; continue; }
            for (PipId pip : downhill.at(wire.index)) {
                if (!is_distribution(pip)) continue;
                clock.global_pips.insert(pip.index);
                WireId to = ctx->getPipDstWire(pip);
                if (clock.global_wires.insert(to.index).second) queue.push_back(to);
            }
        }
        if (clock.global_pips.empty()) log_error("spartan3e-ise: empty dedicated global-clock distribution\n");
    }
    log_info("Spartan-3E ISE: indexed %zu clock-pad paths and %zu global CLK endpoints\n", entries, regions);
}

NEXTPNR_NAMESPACE_END
