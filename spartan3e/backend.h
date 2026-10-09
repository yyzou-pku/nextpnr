/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2021  gatecat <gatecat@ds0.me>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#ifndef SPARTAN3E_BACKEND_H
#define SPARTAN3E_BACKEND_H

#include <optional>
#include <utility>
#include <vector>

#include "nextpnr_namespaces.h"
#include "nextpnr_types.h"

NEXTPNR_NAMESPACE_BEGIN

// Device resource policy and implementation phase hooks.

struct Context;

struct DeviceBackend
{
    virtual void init(Context *ctx);
    Context *ctx;
    bool with_gui = false;

    // --- Bel functions ---
    // Called when a bel is placed/unplaced (with cell=nullptr for a unbind)
    virtual void notifyBelChange(BelId bel, CellInfo *cell) {}
    // This only needs to return false if a bel is disabled for a device-specific reason and not just because it's
    // bound (which the graph bindings will deal with)
    virtual bool checkBelAvail(BelId bel) const { return true; }
    // Mirror the ArchAPI functions - see archapi.md
    virtual std::vector<IdString> getCellTypes() const;
    virtual BelBucketId getBelBucketForBel(BelId bel) const;
    virtual BelBucketId getBelBucketForCellType(IdString cell_type) const;
    virtual bool isValidBelForCellType(IdString cell_type, BelId bel) const;
    virtual bool isBelLocationValid(BelId bel, bool explain_invalid = false) const { return true; }

    // Return nullopt to use the generic relative-coordinate cluster placement.
    virtual std::optional<bool> getClusterPlacement(ClusterId cluster, BelId root_bel,
                                                    std::vector<std::pair<CellInfo *, BelId>> &placement) const
    {
        return std::nullopt;
    }

    // --- Wire and pip functions ---
    // Called when a wire/pip is placed/unplaced (with net=nullptr for a unbind)
    virtual void notifyWireChange(WireId wire, NetInfo *net) {}
    virtual void notifyPipChange(PipId pip, NetInfo *net) {}
    // These only need to return false if a wire/pip is disabled for a device-specific reason and not just because
    // it's bound (which the base arch will deal with)
    virtual bool checkWireAvail(WireId wire) const { return true; }
    virtual bool checkPipAvail(PipId pip) const { return true; }
    virtual bool checkPipAvailForNet(PipId pip, const NetInfo *net) const { return checkPipAvail(pip); };

    // --- Route lookahead ---
    virtual delay_t estimateDelay(WireId src, WireId dst) const;
    virtual delay_t predictDelay(BelId src_bel, IdString src_pin, BelId dst_bel, IdString dst_pin) const;
    virtual BoundingBox getRouteBoundingBox(WireId src, WireId dst) const;

    // --- Flow hooks ---
    virtual void pack() {}; // replaces the pack function
    // Called before and after main placement and routing
    virtual void prePlace() {};
    virtual void postPlace() {};
    virtual void preRoute() {};
    virtual void postRoute() {};

    virtual ~DeviceBackend() {};
};

std::unique_ptr<DeviceBackend> make_backend(const std::string &chipdb, const std::string &physical_design);

NEXTPNR_NAMESPACE_END

#endif
