/* weil0ng: sector cache support.
 * Author: Weilong Cui
 */

/**
 * @file
 * Describes a sector cache based on template policies.
 */

#ifndef __MEM_CACHE_SECTORCACHE_HH__
#define __MEM_CACHE_SECTORCACHE_HH__

#include "mem/cache/cache.hh"
#include "params/SectorCache.hh"
#include "sim/eventq.hh"

/**
 * A template-policy based cache. The behavior of the cache can be altered by
 * supplying different template policies. TagStore handles all tag and data
 * storage @sa TagStore, \ref gem5MemorySystem "gem5 Memory System"
 */
class SectorCache : public Cache
{
  public:
    /** Instantiates a basic cache object. */
    SectorCache(const SectorCacheParams *p);

    /** Non-default destructor is needed to deallocate memory. */
    virtual ~SectorCache();

    /** weil0ng: override access path. */
    bool access(PacketPtr pkt, BlkList *&blk, Cycles &lat, PacketList &writebacks);
};

/**
 * Wrap a method and present it as a cache block visitor.
 *
 * For example the forEachBlk method in the tag arrays expects a
 * callable object/function as their parameter. This class wraps a
 * method in an object and presents  callable object that adheres to
 * the cache block visitor protocol.
 */
class SectorCacheBlkVisitorWrapper : public CacheBlkVisitor
{
  public:
    typedef bool (SectorCache::*VisitorPtr)(CacheBlk &blk);

    SectorCacheBlkVisitorWrapper(SectorCache &_cache, VisitorPtr _visitor)
        : cache(_cache), visitor(_visitor) {}

    bool operator()(CacheBlk &blk) override {
        return (cache.*visitor)(blk);
    }

  private:
    SectorCache &cache;
    VisitorPtr visitor;
};

/**
 * Cache block visitor that determines if there are dirty blocks in a
 * cache.
 *
 * Use with the forEachBlk method in the tag array to determine if the
 * array contains dirty blocks.
 */
class SectorCacheBlkIsDirtyVisitor : public CacheBlkVisitor
{
  public:
    SectorCacheBlkIsDirtyVisitor()
        : _isDirty(false) {}

    bool operator()(CacheBlk &blk) override {
        if (blk.isDirty()) {
            _isDirty = true;
            return false;
        } else {
            return true;
        }
    }

    /**
     * Does the array contain a dirty line?
     *
     * \return true if yes, false otherwise.
     */
    bool isDirty() const { return _isDirty; };

  private:
    bool _isDirty;
};

#endif // __MEM_CACHE_SECTORCACHE_HH__
