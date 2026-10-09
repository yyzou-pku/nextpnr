/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2018  Claire Xenia Wolf <claire@yosyshq.com>
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

#ifndef SPARTAN3E_ARCHDEFS_H
#define SPARTAN3E_ARCHDEFS_H

#include "base_clusterinfo.h"
#include "hashlib.h"
#include "idstringlist.h"

NEXTPNR_NAMESPACE_BEGIN

typedef float delay_t;

struct BelId
{
    BelId() : index(-1) {};
    explicit BelId(int32_t index) : index(index) {};
    int32_t index = -1;

    bool operator==(const BelId &other) const { return index == other.index; }
    bool operator!=(const BelId &other) const { return index != other.index; }
    bool operator<(const BelId &other) const { return index < other.index; }
    unsigned int hash() const { return index; }
};

struct WireId
{
    WireId() : index(-1) {};
    explicit WireId(int32_t index) : index(index) {};
    int32_t index = -1;

    bool operator==(const WireId &other) const { return index == other.index; }
    bool operator!=(const WireId &other) const { return index != other.index; }
    bool operator<(const WireId &other) const { return index < other.index; }
    unsigned int hash() const { return index; }
};

struct PipId
{
    PipId() : index(-1) {};
    explicit PipId(int32_t index) : index(index) {};
    int32_t index = -1;

    bool operator==(const PipId &other) const { return index == other.index; }
    bool operator!=(const PipId &other) const { return index != other.index; }
    bool operator<(const PipId &other) const { return index < other.index; }
    unsigned int hash() const { return index; }
};

struct DecalId
{
    IdStringList name;
    bool active = false;
    DecalId() : name(), active(false) {};
    DecalId(IdStringList name, bool active) : name(name), active(active) {};
    bool operator==(const DecalId &other) const { return name == other.name && active == other.active; }
    bool operator!=(const DecalId &other) const { return name != other.name || active != other.active; }
    unsigned int hash() const { return mkhash(name.hash(), active); }
};

typedef IdStringList GroupId;
typedef IdString BelBucketId;
typedef IdString ClusterId;

struct ArchNetInfo
{
};

struct NetInfo;

struct ArchCellInfo : BaseClusterInfo
{
    // Stable index for resource validity caches.
    int flat_index;
    // Cell to bel pin mapping
    dict<IdString, std::vector<IdString>> bel_pins;
};

NEXTPNR_NAMESPACE_END

#endif /* SPARTAN3E_ARCHDEFS_H */
