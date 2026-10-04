/* Native clock topology: bounded pad entry and directed global distribution. */
#include "spartan3e_ise_clock.h"
#include <deque>
#include "log.h"

NEXTPNR_NAMESPACE_BEGIN

void index_ise_clock_paths(Context *ctx, std::vector<IseClockSite> &clocks,
                           const std::unordered_map<int32_t, BelId> &pad_wires,
                           const std::unordered_set<int32_t> &clock_pins, int fabric_wires, int fabric_pips)
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
                clock.global_pips.insert(pip.index);
                WireId to = ctx->getPipDstWire(pip);
                if (clock.global_wires.insert(to.index).second) queue.push_back(to);
            }
        }
    }
    log_info("Spartan-3E ISE: indexed %zu clock-pad paths and %zu global CLK endpoints\n", entries, regions);
}

NEXTPNR_NAMESPACE_END
