/*
 * weil0ng: Sector cache implementation.
 * Author: Weilong Cui
 */

/**
 * @file
 * Cache definitions.
 */

#include "mem/cache/sector_cache.hh"

#include "base/misc.hh"
#include "base/types.hh"
#include "debug/Cache.hh"
#include "debug/CachePort.hh"
#include "debug/CacheTags.hh"
#include "debug/CacheVerbose.hh"
#include "mem/cache/blk.hh"
#include "mem/cache/mshr.hh"
#include "mem/cache/prefetch/base.hh"
#include "sim/sim_exit.hh"

SectorCache::SectorCache(const SectorCacheParams *p)
    : Cache(p) {}

SectorCache::~SectorCache()
{
    delete [] tempBlock->data;
    delete tempBlock;

    delete cpuSidePort;
    delete memSidePort;
}

/** ACCESS PATH.
 * Args:
 *  pkt: CPU pkt.
 * Returns:
 *  blks: cache blks that are to be accessed.
 *  lat: access latency.
 *  writebacks: list of packet to perform writebacks.
 *  return true on (full) hit, false on miss.
 */
bool SectorCache::access(PacketPtr pkt, BlkList *&blks, Cycles &lat,
                         PacketList &writebacks)
{
    // sanity check
    assert(pkt->isRequest());

    chatty_assert(!(isReadOnly && pkt->isWrite()),
                  "Should never see a write in a read-only cache %s\n",
                  name());

    DPRINTF(CacheVerbose, "%s for %s addr %#llx size %d\n", __func__,
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());

    // weil0ng: tmp prints.
    printf("[SectorCache] %s access: %s %#lx for size %d\n", name().c_str(),
            pkt->cmdString().c_str(), pkt->getAddr(), pkt->getSize());

    // weil0ng: we are building sector cache from subblock up, e.g. several
    // sub blocks could combine to support coarser access, not the other way
    // around.
    assert(pkt->getSize() >= blkSize);

    assert(pkt->getSize() % blkSize == 0);
    int numBlk = pkt->getSize() / blkSize;

    if (pkt->req->isUncacheable()) {
        DPRINTF(Cache, "%s%s addr %#llx uncacheable\n", pkt->cmdString(),
                pkt->req->isInstFetch() ? " (ifetch)" : "",
                pkt->getAddr());

        // Then flush and invalidate any existing block
        for (int i=0; i<numBlk; i++) {
            CacheBlk *old_blk(tags->findBlock(pkt->getAddr() + i*blkSize, pkt->isSecure()));
            if (old_blk && old_blk->isValid()) {
                if (old_blk->isDirty() || writebackClean)
                    writebacks.push_back(writebackBlk(old_blk));
                else
                    writebacks.push_back(cleanEvictBlk(old_blk));
                tags->invalidate(old_blk);
                old_blk->invalidate();
            }
        }

        blks = nullptr;
        // lookupLatency is the latency in case the request is uncacheable.
        lat = lookupLatency;
        return false;
    }

    return false;
}

SectorCache*
SectorCacheParams::create()
{
    assert(tags);

    return new SectorCache(this);
}
