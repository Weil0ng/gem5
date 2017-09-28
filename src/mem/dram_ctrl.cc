/*
 * Copyright (c) 2010-2017 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2013 Amin Farmahini-Farahani
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Andreas Hansson
 *          Ani Udipi
 *          Neha Agarwal
 *          Omar Naji
 *          Wendy Elsasser
 */

#include <sstream>
#include "mem/dram_ctrl.hh"

#include "base/bitfield.hh"
#include "base/trace.hh"
#include "debug/AddrRegs.hh"
#include "debug/MemCtx.hh"
#include "debug/DeviceState.hh"
#include "debug/DRAM.hh"
#include "debug/DRAMPower.hh"
#include "debug/DRAMState.hh"
#include "debug/Drain.hh"
#include "debug/VMC.hh"
#include "debug/Pack.hh"
#include "debug/Progress.hh"
#include "debug/Push.hh"
#include "sim/system.hh"

using namespace std;
using namespace Data;

DRAMCtrl::DRAMCtrl(const DRAMCtrlParams* p) :
    AbstractMemory(p),
    port(name() + ".port", *this),
    isTimingMode(false), isVMCMode(false),
    retryRdReq(false), retryWrReq(false),
    busState(READ), busStateNext(READ),
    nextReqEvent(this), respondEvent(this),
    deviceSize(p->device_size),
    deviceBusWidth(p->device_bus_width), burstLength(p->burst_length),
    deviceRowBufferSize(p->device_rowbuffer_size),
    devicesPerRank(p->devices_per_rank),
    burstSize((devicesPerRank * burstLength * deviceBusWidth) / 8),
    devBurstSize((burstLength * deviceBusWidth)/8),
    rowBufferSize(devicesPerRank * deviceRowBufferSize),
    columnsPerRowBuffer(rowBufferSize / burstSize),
    columnsPerStripe(range.interleaved() ? range.granularity() / burstSize : 1),
    ranksPerChannel(p->ranks_per_channel),
    bankGroupsPerRank(p->bank_groups_per_rank),
    bankGroupArch(p->bank_groups_per_rank > 0),
    banksPerRank(p->banks_per_rank), channels(p->channels), rowsPerBank(0),
    readBufferSize(p->read_buffer_size),
    addrRegsPerDevice(p->addr_regs_per_device),
    writeBufferSize(p->write_buffer_size),
    writeHighThresPerc(p->write_high_thresh_perc / 100.0),
    writeLowThresPerc(p->write_low_thresh_perc / 100.0),
    writeHighThreshold(writeBufferSize * p->write_high_thresh_perc / 100.0),
    writeLowThreshold(writeBufferSize * p->write_low_thresh_perc / 100.0),
    minWritesPerSwitch(p->min_writes_per_switch),
    writesThisTime(0), readsThisTime(0), packRdDrain(false), packWrDrain(false),
    tCK(p->tCK), tWTR(p->tWTR), tRTW(p->tRTW), tCS(p->tCS), tBURST(p->tBURST),
    tCCD_L(p->tCCD_L), tRCD(p->tRCD), tCL(p->tCL), tRP(p->tRP), tRAS(p->tRAS),
    tWR(p->tWR), tRTP(p->tRTP), tRFC(p->tRFC), tREFI(p->tREFI), tRRD(p->tRRD),
    tRRD_L(p->tRRD_L), tXAW(p->tXAW), tXP(p->tXP), tXS(p->tXS),
    activationLimit(p->activation_limit), memSchedPolicy(p->mem_sched_policy),
    addrMapping(p->addr_mapping), pageMgmt(p->page_policy),
    maxAccessesPerRow(p->max_accesses_per_row),
    frontendLatency(p->static_frontend_latency),
    backendLatency(p->static_backend_latency),
    busBusyUntil(0), prevArrival(0), prevPushScheduled(0),
    // weil0ng: It takes 3 cycles (2 cycles for addr trans + 1 dead cycle) to transfer addr.
    nextReqTime(0), packWaitTime(p->pack_latency),
    pushDelay(p->tBURST/2 + tCK),
    packRdDrainStartTick(0), packWrDrainStartTick(0),
    activeRank(0), timeStampOffset(0)
{
    // sanity check the ranks since we rely on bit slicing for the
    // address decoding
    fatal_if(!isPowerOf2(ranksPerChannel), "DRAM rank count of %d is not "
             "allowed, must be a power of two\n", ranksPerChannel);

    fatal_if(!isPowerOf2(burstSize), "DRAM burst size %d is not allowed, "
             "must be a power of two\n", burstSize);

    for (int i = 0; i < ranksPerChannel; i++) {
        Rank* rank = new Rank(*this, p, i);
        ranks.push_back(rank);

        rank->actTicks.resize(activationLimit, 0);
        rank->banks.reserve(banksPerRank);
        for (int j=0; j<banksPerRank; ++j)
            rank->banks.push_back(new Bank(*this, 0, 0));

        for (int b = 0; b < banksPerRank; b++) {
            rank->banks[b]->bank = b;
            // GDDR addressing of banks to BG is linear.
            // Here we assume that all DRAM generations address bank groups as
            // follows:
            if (bankGroupArch) {
                // Simply assign lower bits to bank group in order to
                // rotate across bank groups as banks are incremented
                // e.g. with 4 banks per bank group and 16 banks total:
                //    banks 0,4,8,12  are in bank group 0
                //    banks 1,5,9,13  are in bank group 1
                //    banks 2,6,10,14 are in bank group 2
                //    banks 3,7,11,15 are in bank group 3
                rank->banks[b]->bankgr = b % bankGroupsPerRank;
            } else {
                // No bank groups; simply assign to bank number
                rank->banks[b]->bankgr = b;
            }
        }
    }

    // perform a basic check of the write thresholds
    if (p->write_low_thresh_perc >= p->write_high_thresh_perc)
        fatal("Write buffer low threshold %d must be smaller than the "
              "high threshold %d\n", p->write_low_thresh_perc,
              p->write_high_thresh_perc);

    // determine the rows per bank by looking at the total capacity
    uint64_t capacity = ULL(1) << ceilLog2(AbstractMemory::size());

    // determine the dram actual capacity from the DRAM config in Mbytes
    uint64_t deviceCapacity = deviceSize / (1024 * 1024) * devicesPerRank *
        ranksPerChannel;

    // if actual DRAM size does not match memory capacity in system warn!
    if (deviceCapacity != capacity / (1024 * 1024))
        warn("DRAM device capacity (%d Mbytes) does not match the "
             "address range assigned (%d Mbytes)\n", deviceCapacity,
             capacity / (1024 * 1024));

    DPRINTF(DRAM, "Memory capacity %lld (%lld) bytes\n", capacity,
            AbstractMemory::size());

    DPRINTF(DRAM, "Row buffer size %d bytes with %d columns per row buffer\n",
            rowBufferSize, columnsPerRowBuffer);

    rowsPerBank = capacity / (rowBufferSize * banksPerRank * ranksPerChannel);

    // some basic sanity checks
    if (tREFI <= tRP || tREFI <= tRFC) {
        fatal("tREFI (%d) must be larger than tRP (%d) and tRFC (%d)\n",
              tREFI, tRP, tRFC);
    }

    // basic bank group architecture checks ->
    if (bankGroupArch) {
        // must have at least one bank per bank group
        if (bankGroupsPerRank > banksPerRank) {
            fatal("banks per rank (%d) must be equal to or larger than "
                  "banks groups per rank (%d)\n",
                  banksPerRank, bankGroupsPerRank);
        }
        // must have same number of banks in each bank group
        if ((banksPerRank % bankGroupsPerRank) != 0) {
            fatal("Banks per rank (%d) must be evenly divisible by bank groups "
                  "per rank (%d) for equal banks per bank group\n",
                  banksPerRank, bankGroupsPerRank);
        }
        // tCCD_L should be greater than minimal, back-to-back burst delay
        if (tCCD_L <= tBURST) {
            fatal("tCCD_L (%d) should be larger than tBURST (%d) when "
                  "bank groups per rank (%d) is greater than 1\n",
                  tCCD_L, tBURST, bankGroupsPerRank);
        }
        // tRRD_L is greater than minimal, same bank group ACT-to-ACT delay
        // some datasheets might specify it equal to tRRD
        if (tRRD_L < tRRD) {
            fatal("tRRD_L (%d) should be larger than tRRD (%d) when "
                  "bank groups per rank (%d) is greater than 1\n",
                  tRRD_L, tRRD, bankGroupsPerRank);
        }
    }

}

void
DRAMCtrl::init()
{
    AbstractMemory::init();

   if (!port.isConnected()) {
        fatal("DRAMCtrl %s is unconnected!\n", name());
    } else {
        port.sendRangeChange();
    }

    // a bit of sanity checks on the interleaving, save it for here to
    // ensure that the system pointer is initialised
    if (range.interleaved()) {
        if (channels != range.stripes())
            fatal("%s has %d interleaved address stripes but %d channel(s)\n",
                  name(), range.stripes(), channels);

        if (addrMapping == Enums::RoRaBaChCo) {
            if (rowBufferSize != range.granularity()) {
                fatal("Channel interleaving of %s doesn't match RoRaBaChCo "
                      "address map\n", name());
            }
        } else if (addrMapping == Enums::RoRaBaCoCh ||
                   addrMapping == Enums::RoCoRaBaCh) {
            // for the interleavings with channel bits in the bottom,
            // if the system uses a channel striping granularity that
            // is larger than the DRAM burst size, then map the
            // sequential accesses within a stripe to a number of
            // columns in the DRAM, effectively placing some of the
            // lower-order column bits as the least-significant bits
            // of the address (above the ones denoting the burst size)
            assert(columnsPerStripe >= 1);

            // channel striping has to be done at a granularity that
            // is equal or larger to a cache line
            if (system()->cacheLineSize() > range.granularity()) {
                fatal("Channel interleaving of %s must be at least as large "
                      "as the cache line size\n", name());
            }

            // ...and equal or smaller than the row-buffer size
            if (rowBufferSize < range.granularity()) {
                fatal("Channel interleaving of %s must be at most as large "
                      "as the row-buffer size\n", name());
            }
            // this is essentially the check above, so just to be sure
            assert(columnsPerStripe <= columnsPerRowBuffer);
        }
    }
}

std::atomic<int> DRAMCtrl::DRAMPacket::sid;

void
DRAMCtrl::startup()
{
    // remember the memory system mode of operation
    isTimingMode = system()->isTimingMode();

    // weil0ng: set the pageMgmt
    pageMgmt = system()->getPagePolicy();

    if (isTimingMode) {
        // timestamp offset should be in clock cycles for DRAMPower
        timeStampOffset = divCeil(curTick(), tCK);

        // update the start tick for the precharge accounting to the
        // current tick
        for (auto r : ranks) {
            r->startup(curTick() + tREFI - tRP);
        }

        // shift the bus busy time sufficiently far ahead that we never
        // have to worry about negative values when computing the time for
        // the next request, this will add an insignificant bubble at the
        // start of simulation
        busBusyUntil = curTick() + tRP + tRCD + tCL;
    }

    DPRINTF(VMC, "Startup params: timing %s, page policy %s,"
            "addrRegs %d, packLatency %d, readBufferSize %d,"
            "writeBufferSize %d\n",
            isTimingMode ? "True" : "False", pageMgmt, addrRegsPerDevice,
            packWaitTime, readBufferSize, writeBufferSize);
}

Tick
DRAMCtrl::recvAtomic(PacketPtr pkt)
{
    // DPRINTF(DRAM, "recvAtomic: %s 0x%x\n", pkt->cmdString(), pkt->getAddr());

    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    // do the actual memory access and turn the packet into a response
    access(pkt);

    Tick latency = 0;
    if (pkt->hasData()) {
        // this value is not supposed to be accurate, just enough to
        // keep things going, mimic a closed page
        latency = tRP + tRCD + tCL;
    }
    return latency;
}

bool
DRAMCtrl::readQueueFull(unsigned int neededEntries) const
{
    DPRINTF(DRAM, "Read queue limit %d, current size %d, entries needed %d\n",
            readBufferSize, readQueue.size() + respQueue.size(),
            neededEntries);

    return
        (readQueue.size() + respQueue.size() + neededEntries) > readBufferSize;
}

bool
DRAMCtrl::writeQueueFull(unsigned int neededEntries) const
{
    DPRINTF(DRAM, "Write queue limit %d, current size %d, entries needed %d\n",
            writeBufferSize, writeQueue.size(), neededEntries);
    return (writeQueue.size() + neededEntries) > writeBufferSize;
}

DRAMCtrl::DRAMPacket*
DRAMCtrl::decodeAddr(PacketPtr pkt, Addr dramPktAddr, unsigned size,
                       bool isRead)
{
    // decode the address based on the address mapping scheme, with
    // Ro, Ra, Co, Ba and Ch denoting row, rank, column, bank and
    // channel, respectively
    uint8_t rank;
    uint8_t bank;
    // use a 64-bit unsigned during the computations as the row is
    // always the top bits, and check before creating the DRAMPacket
    uint64_t row;
    // weil0ng: the device this req points to.
    uint8_t device;

    // weil0ng: let's get the device offset first.
    // This is essentially bytes into the 64B transaction batch,
    // we are assuming each 8B is consecutive on one device,
    // or each 8B comes from 8 devices?
    device = (dramPktAddr % burstSize) / devicesPerRank;

    // truncate the address to a DRAM burst, which makes it unique to
    // a specific column, row, bank, rank and channel
    Addr addr = dramPktAddr / burstSize;

    // we have removed the lowest order address bits that denote the
    // position within the column
    if (addrMapping == Enums::RoRaBaChCo) {
        // the lowest order bits denote the column to ensure that
        // sequential cache lines occupy the same row
        addr = addr / columnsPerRowBuffer;

        // take out the channel part of the address
        addr = addr / channels;

        // after the channel bits, get the bank bits to interleave
        // over the banks
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;

        // after the bank, we get the rank bits which thus interleaves
        // over the ranks
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;

        // lastly, get the row bits, no need to remove them from addr
        row = addr % rowsPerBank;
    } else if (addrMapping == Enums::RoRaBaCoCh) {
        // take out the lower-order column bits
        addr = addr / columnsPerStripe;

        // take out the channel part of the address
        addr = addr / channels;

        // next, the higher-order column bites
        addr = addr / (columnsPerRowBuffer / columnsPerStripe);

        // after the column bits, we get the bank bits to interleave
        // over the banks
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;

        // after the bank, we get the rank bits which thus interleaves
        // over the ranks
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;

        // lastly, get the row bits, no need to remove them from addr
        row = addr % rowsPerBank;
    } else if (addrMapping == Enums::RoCoRaBaCh) {
        // optimise for closed page mode and utilise maximum
        // parallelism of the DRAM (at the cost of power) 

        // take out the lower-order column bits
        addr = addr / columnsPerStripe;

        // take out the channel part of the address, not that this has
        // to match with how accesses are interleaved between the
        // controllers in the address mapping
        addr = addr / channels;

        // start with the bank bits, as this provides the maximum
        // opportunity for parallelism between requests
        bank = addr % banksPerRank;
        addr = addr / banksPerRank;

        // next get the rank bits
        rank = addr % ranksPerChannel;
        addr = addr / ranksPerChannel;

        // next, the higher-order column bites
        addr = addr / (columnsPerRowBuffer / columnsPerStripe);

        // lastly, get the row bits, no need to remove them from addr
        row = addr % rowsPerBank;
    } else
        panic("Unknown address mapping policy chosen!");

    assert(rank < ranksPerChannel);
    assert(bank < banksPerRank);
    assert(row < rowsPerBank);
    assert(row < Bank::NO_ROW);

    DPRINTF(DRAM, "Address: %#08x ReqSize: %d Burst %d "\
            "Rank %d Bank %d Device %d Row %d\n",
            dramPktAddr, size, burstSize, rank, bank, device, row);

    // create the corresponding DRAM packet with the entry time and
    // ready time set to the current tick, the latter will be updated
    // later
    uint16_t bank_id = banksPerRank * rank + bank;
    DRAMPacket* new_pkt = new DRAMPacket(pkt, isRead, isVMCMode, false,
            rank, bank, row, device, bank_id, dramPktAddr, size,
            *ranks[rank], *(ranks[rank]->devices[device]),
            *(ranks[rank]->devices[device]->banks[bank]));
    return new_pkt;
}

DRAMCtrl::DRAMPacket*
DRAMCtrl::tryPack(std::deque<DRAMPacket*>& queue, uint8_t rank) {
    assert(isVMCMode);
    assert(!queue.empty());
    bool isRead = queue.front()->isRead;
    bool found_packet = false;
    std::deque<DRAMPacket*> micro_pkts = std::deque<DRAMPacket*>();
    bool* occupied = new bool[devicesPerRank]();
    for (auto i=queue.begin(); i!=queue.end(); ++i) {
        DRAMPacket* dram_pkt = *i;
        //DPRINTF(Pack, "Checking pkt (%d) at rank%d_device%d(%d) for rank %d\n",
        //        dram_pkt->id, dram_pkt->rank, dram_pkt->device,
        //        occupied[dram_pkt->device], rank);
        if (dram_pkt->isMicro && !occupied[dram_pkt->device] &&
                dram_pkt->rank == rank &&
                !(isRead?dram_pkt->bankRef.pendingRdPushPkt:dram_pkt->bankRef.pendingWrPushPkt)) {
            micro_pkts.push_back(dram_pkt);
            occupied[dram_pkt->device] = true;
            found_packet = true;
        }
    }
    delete [] occupied;
    if (!found_packet)
        return NULL;
    
    DRAMPacket* packPkt = new DRAMPacket(new Packet(new Request(), MemCmd()), isRead, false, true,
            rank, 0, 0, 0, 0, 0, burstSize/devicesPerRank * micro_pkts.size(), *(ranks[rank]),
            *(ranks[rank]->devices[0]), *(ranks[rank]->devices[0]->banks[0]));

    // Prepare the PackHelper.
    std::deque<DRAMPacket*>* pkts = new std::deque<DRAMPacket*>();
    for (auto i=0; i<micro_pkts.size(); ++i) {
        pkts->push_back(micro_pkts[i]);
    }
    assert(micro_pkts.size() == pkts->size());
    PackHelper* pack_helper = new PackHelper(micro_pkts.size(), pkts);
    packPkt->packHelper = pack_helper;
    return packPkt;
}

bool
DRAMCtrl::packMicroPkts(std::deque<DRAMPacket*>& queue, uint8_t rank) {
    assert(isVMCMode);
    assert(!queue.empty());
    bool isRead = queue.front()->isRead;
    bool found_packet = false;
    std::deque<DRAMPacket*> micro_pkts = std::deque<DRAMPacket*>();
    bool* occupied = new bool[devicesPerRank]();
    std::stringstream id_stream;
    for (auto i=queue.begin(); i!=queue.end(); ++i) {
        DRAMPacket* dram_pkt = *i;
        //DPRINTF(Pack, "Checking pkt (%d) at rank%d_device%d(%d) for rank %d\n",
        //        dram_pkt->id, dram_pkt->rank, dram_pkt->device,
        //        occupied[dram_pkt->device], rank);

        // The following conditions must be identical to those in tryPack(),
        // otherwise, checkEqual would fail and we will not be packing the exact
        // same list of micro pkts between this and tryPack().
        if (dram_pkt->isMicro && !occupied[dram_pkt->device] &&
                dram_pkt->rank == rank &&
                !(isRead?dram_pkt->bankRef.pendingRdPushPkt:dram_pkt->bankRef.pendingWrPushPkt)) {
            queue.erase(i--);
            micro_pkts.push_back(dram_pkt);
            occupied[dram_pkt->device] = true;
            id_stream << "(" << dram_pkt->id << ")" << " ";
            found_packet = true;
        }
    }
    delete [] occupied;
    if (!found_packet)
        return found_packet;
    
    DRAMPacket* packPkt = new DRAMPacket(new Packet(new Request(), MemCmd()), isRead, false, true,
            rank, 0, 0, 0, 0, 0, burstSize/devicesPerRank * micro_pkts.size(), *ranks[rank],
            *(ranks[rank]->devices[0]), *(ranks[rank]->devices[0]->banks[0]));
    DPRINTF(Pack, "Packing %d %s pkts for rank %d\n", micro_pkts.size(), isRead?"read":"write", rank);
    DPRINTF(Pack, "Generating %s packPkt (%d) for rank %d\n",
            packPkt->isRead?"read":"write", packPkt->id, rank);
    DPRINTF(Pack, "packPkt (%d) has %s\n", packPkt->id, id_stream.str());

    // Prepare the PackHelper.
    std::deque<DRAMPacket*>* pkts = new std::deque<DRAMPacket*>();
    for (auto i=0; i<micro_pkts.size(); ++i) {
        pkts->push_back(micro_pkts[i]);
    }
    assert(micro_pkts.size() == pkts->size());
    PackHelper* pack_helper = new PackHelper(micro_pkts.size(), pkts);
    packPkt->packHelper = pack_helper;
    DPRINTF(Pack, "PackHelper of %d pkts, size %d bytes\n", pack_helper->pkt_cnt, sizeof(pack_helper));
    queue.push_front(packPkt);

    pckLength.sample(micro_pkts.size());
    if (isRead)
        pckRdLength.sample(micro_pkts.size());
    else
        pckWrLength.sample(micro_pkts.size());
    return found_packet;
}

bool
DRAMCtrl::checkPackEqual(DRAMPacket* pkt1, DRAMPacket* pkt2) {
    assert(pkt1->packHelper->pkt_cnt == pkt2->packHelper->pkt_cnt);
    for(int i=0; i<pkt1->packHelper->pkt_cnt; ++i) {
        DRAMPacket* cur_pkt1 = (*(pkt1->packHelper))[i];
        DRAMPacket* cur_pkt2 = (*(pkt2->packHelper))[i];
        assert(cur_pkt1->id == cur_pkt2->id);
    }
    return true;
}

bool
DRAMCtrl::destroyTrialPkt(DRAMPacket* packPkt) {
    assert(packPkt);
    assert(packPkt->isPack);
    delete packPkt->pkt->req;
    delete packPkt->pkt;
    delete packPkt->packHelper;
    if (packPkt->burstHelper)
        delete packPkt->burstHelper;
    delete packPkt;
    return true;
}

void
DRAMCtrl::addToReadQueue(PacketPtr pkt, unsigned int pktCount)
{
    // only add to the read queue here. whenever the request is
    // eventually done, set the readyTime, and call schedule()
    assert(!pkt->isWrite());

    assert(pktCount != 0);

    // if the request size is larger than burst size, the pkt is split into
    // multiple DRAM packets
    // Note if the pkt starting address is not aligened to burst size, the
    // address of first DRAM packet is kept unaliged. Subsequent DRAM packets
    // are aligned to burst size boundaries. This is to ensure we accurately
    // check read packets against packets in write queue.
    Addr addr = pkt->getAddr();
    unsigned pktsServicedByWrQ = 0;
    BurstHelper* burst_helper = NULL;
    uint32_t pktBurstSize = isVMCMode?devBurstSize:burstSize;
    for (int cnt = 0; cnt < pktCount; ++cnt) {
        unsigned size = std::min((addr | (pktBurstSize - 1)) + 1,
                        pkt->getAddr() + pkt->getSize()) - addr;
        readPktSize[ceilLog2(size)]++;
        readBursts++;

        // First check write buffer to see if the data is already at
        // the controller
        bool foundInWrQ = false;
        Addr burst_addr = isVMCMode?devBurstAlign(addr):burstAlign(addr);
        // if the burst address is not present then there is no need
        // looking any further
        if (isInWriteQueue.find(burst_addr) != isInWriteQueue.end()) {
            for (const auto& p : writeQueue) {
                // check if the read is subsumed in the write queue
                // packet we are looking at
                if (p->addr <= addr && (addr + size) <= (p->addr + p->size)) {
                    foundInWrQ = true;
                    servicedByWrQ++;
                    pktsServicedByWrQ++;
                    DPRINTF(DRAM, "Read to addr %lld with size %d serviced by "
                            "write queue\n", addr, size);
                    bytesReadWrQ += burstSize;
                    break;
                }
            }
        }

        // If not found in the write q, make a DRAM packet and
        // push it onto the read queue
        if (!foundInWrQ) {

            // Make the burst helper for split packets
            if (pktCount > 1 && burst_helper == NULL) {
                DPRINTF(DRAM, "Read to addr %lld translates to %d "
                        "dram requests\n", pkt->getAddr(), pktCount);
                burst_helper = new BurstHelper(pktCount);
            }

            DRAMPacket* dram_pkt = decodeAddr(pkt, addr, size, true);

            dram_pkt->burstHelper = burst_helper;

            assert(!readQueueFull(1));
            rdQLenPdf[readQueue.size() + respQueue.size()]++;

            DPRINTF(DRAM, "Adding to read queue\n");

            readQueue.push_back(dram_pkt);

            // Update stats
            avgRdQLen = readQueue.size() + respQueue.size();

            if (isVMCMode) {
                DPRINTF(VMC, "Generate dev read pkt (%d)\n", dram_pkt->id);
                uint8_t device = dram_pkt->device;
                uint8_t rank = dram_pkt->rank;
                ++devRdQLen[rank * devicesPerRank + device];
                ++dram_pkt->deviceRef.readEntries;
            } else {
                // increment read entries of the rank
                ++dram_pkt->rankRef.readEntries;
            }
        }

        // Starting address of next dram pkt (aligend to burstSize boundary)
        addr = (addr | (pktBurstSize - 1)) + 1;
    }

    // If all packets are serviced by write queue, we send the repsonse back
    if (pktsServicedByWrQ == pktCount) {
        accessAndRespond(pkt, frontendLatency);
        return;
    }

    // Update how many split packets are serviced by write queue
    if (burst_helper != NULL)
        burst_helper->burstsServiced = pktsServicedByWrQ;

    // If we are not already scheduled to get a request out of the
    // queue, do so now
    if (!nextReqEvent.scheduled()) {
        DPRINTF(DRAM, "Request scheduled immediately\n");
        schedule(nextReqEvent, curTick());
    }
}

void
DRAMCtrl::addToWriteQueue(PacketPtr pkt, unsigned int pktCount)
{
    // only add to the write queue here. whenever the request is
    // eventually done, set the readyTime, and call schedule()
    assert(pkt->isWrite());

    // if the request size is larger than burst size, the pkt is split into
    // multiple DRAM packets
    Addr addr = pkt->getAddr();
    uint32_t pktBurstSize = isVMCMode?devBurstSize:burstSize;
    for (int cnt = 0; cnt < pktCount; ++cnt) {
        unsigned size = std::min((addr | (pktBurstSize - 1)) + 1,
                        pkt->getAddr() + pkt->getSize()) - addr;
        writePktSize[ceilLog2(size)]++;
        writeBursts++;

        // see if we can merge with an existing item in the write
        // queue and keep track of whether we have merged or not
        Addr alignedAddr = isVMCMode?devBurstAlign(addr):burstAlign(addr);
        bool merged = isInWriteQueue.find(alignedAddr) !=
            isInWriteQueue.end();

        // if the item was not merged we need to create a new write
        // and enqueue it
        if (!merged) {
            DRAMPacket* dram_pkt = decodeAddr(pkt, addr, size, false);
            
            assert(writeQueue.size() < writeBufferSize);
            wrQLenPdf[writeQueue.size()]++;

            DPRINTF(DRAM, "Adding to write queue\n");

            writeQueue.push_back(dram_pkt);
            // Update stats
            avgWrQLen = writeQueue.size();

            if (isVMCMode) {
                uint8_t device = dram_pkt->device;
                uint8_t rank = dram_pkt->rank;
                DPRINTF(VMC, "Generate dev write pkt (%d)\n", dram_pkt->id);
                ++devWrQLen[rank * devicesPerRank + device];
                ++dram_pkt->deviceRef.writeEntries;
            } else {
                // increment write entries of the rank
                ++dram_pkt->rankRef.writeEntries;
            }

            isInWriteQueue.insert(alignedAddr);
            //assert(writeQueue.size() == isInWriteQueue.size());

        } else {
            DPRINTF(DRAM, "Merging write burst with existing queue entry\n");

            // keep track of the fact that this burst effectively
            // disappeared as it was merged with an existing one
            mergedWrBursts++;
        }

        // Starting address of next dram pkt (aligend to burstSize boundary)
        addr = (addr | (pktBurstSize - 1)) + 1;
    }

    // we do not wait for the writes to be send to the actual memory,
    // but instead take responsibility for the consistency here and
    // snoop the write queue for any upcoming reads
    // @todo, if a pkt size is larger than burst size, we might need a
    // different front end latency
    accessAndRespond(pkt, frontendLatency);

    // If we are not already scheduled to get a request out of the
    // queue, do so now
    if (!nextReqEvent.scheduled()) {
        DPRINTF(DRAM, "Request scheduled immediately\n");
        schedule(nextReqEvent, curTick());
    }
}

void
DRAMCtrl::printQs() const {
    DPRINTF(DRAM, "===READ QUEUE===\n\n");
    for (auto i = readQueue.begin() ;  i != readQueue.end() ; ++i) {
        DPRINTF(DRAM, "Read %lu\n", (*i)->addr);
    }
    DPRINTF(DRAM, "\n===RESP QUEUE===\n\n");
    for (auto i = respQueue.begin() ;  i != respQueue.end() ; ++i) {
        DPRINTF(DRAM, "Response %lu\n", (*i)->addr);
    }
    DPRINTF(DRAM, "\n===WRITE QUEUE===\n\n");
    for (auto i = writeQueue.begin() ;  i != writeQueue.end() ; ++i) {
        DPRINTF(DRAM, "Write %lu\n", (*i)->addr);
    }
}

void
DRAMCtrl::Rank::restart() {
    // TODO: assertions to check states of devices.

    // Reset internal states.
    for (auto& b : banks) {
        b->reset();
    }
    pwrStateTrans = PWR_IDLE;
    pwrStatePostRefresh = PWR_IDLE;
    pwrStateTick = 0;
    refreshDueAt = 0;
    pwrState = PWR_IDLE;
    refreshState = REF_IDLE;
    inLowPowerState = false;
    readEntries = 0;
    writeEntries = 0;
    outstandingEvents = 0;
    wakeUpAllowedAt = 0;
    numBanksActive = 0;

    // Restart the rank.
    startup(curTick() + memory.tREFI - memory.tRP);
}

void
DRAMCtrl::Device::copyStateFromRank() {
    assert(rankRef.rank == rank);
    this->pwrStateTrans = rankRef.pwrStateTrans;
    this->pwrStatePostRefresh = rankRef.pwrStatePostRefresh;
    this->pwrStateTick = rankRef.pwrStateTick;
    this->refreshDueAt = rankRef.refreshDueAt;
    this->pwrState = rankRef.pwrState;
    this->refreshState = rankRef.refreshState;
    this->inLowPowerState = rankRef.inLowPowerState;
    this->readEntries = rankRef.readEntries;
    this->writeEntries = rankRef.writeEntries;
    this->outstandingEvents = rankRef.outstandingEvents;
    this->wakeUpAllowedAt = rankRef.wakeUpAllowedAt;
    // copy over all power stats from rank
    this->power = rankRef.power;
    this->cmdList = rankRef.cmdList;
    // this->banks = rankRef.banks;
    this->numBanksActive = rankRef.numBanksActive;
    this->actTicks = rankRef.actTicks;
    for (int i=0; i<memory.banksPerRank; ++i) {
        Bank& deviceBankRef = *(this->banks[i]);
        Bank& bankRef = *(rankRef.banks[i]);
        assert(deviceBankRef.rank == bankRef.rank);
        assert(deviceBankRef.bank == bankRef.bank);
        assert(deviceBankRef.bankgr == bankRef.bankgr);
        deviceBankRef.openRow = bankRef.openRow;
        deviceBankRef.colAllowedAt = bankRef.colAllowedAt;
        deviceBankRef.preAllowedAt = bankRef.preAllowedAt;
        deviceBankRef.actAllowedAt = bankRef.actAllowedAt;
    }
}

void
DRAMCtrl::splitRankToDevices(uint8_t rank)
{
    for (auto d : ranks[rank]->devices) {
            d->copyStateFromRank();
    }
}

void
DRAMCtrl::mergeDevicesToRank(uint8_t rank)
{
    ranks[rank]->restart();
}

// weil0ng: this serves as the on-chip buffer for VMC.
bool
DRAMCtrl::recvTimingReq(PacketPtr pkt)
{
    // This is where we enter from the outside world
    DPRINTF(DRAM, "recvTimingReq: request %s addr %#08x size %d\n",
            pkt->cmdString(), pkt->getAddr(), pkt->getSize());
    if (pkt->req->hasContextId()) {
        DPRINTF(MemCtx, "from %d\n", pkt->req->contextId());
        ++perCoreReqs[pkt->req->contextId()];
    }
    if (system()->isVMCMode()) {
        if (!isVMCMode) {
            assert(drain() == DrainState::Drained);
            isVMCMode = true;
            DPRINTF(VMC, "Enter VMC mode\n");
            readBufferSize *= devicesPerRank;
            writeBufferSize *= devicesPerRank;
            writeHighThreshold = writeHighThresPerc * writeBufferSize;
            writeLowThreshold = writeLowThresPerc * writeBufferSize;
            for (int i=0; i<ranksPerChannel; ++i)
                splitRankToDevices(i);
        }
        DPRINTF(VMC, "=============%s============\n", curTick());
        if (pkt->req->hasContextId()) {
            DPRINTF(VMC, "recvTimingReq from %d: request %s addr %#08x size %d vmc %s\n",
                    pkt->req->contextId(), pkt->cmdString(), pkt->getAddr(),
                    pkt->getSize(), isVMCMode);
        } else {
            DPRINTF(VMC, "recvTimingReq from cache: request %s addr %#08x size %d vmc %s\n",
                    pkt->cmdString(), pkt->getAddr(),
                    pkt->getSize(), isVMCMode);
        }
    } else {
        if (isVMCMode) {
            DPRINTF(VMC, "Exit VMC mode\n");
            isVMCMode = false;
            for (int i=0; i<ranksPerChannel; ++i)
                mergeDevicesToRank(i);
        }
    }

    panic_if(pkt->cacheResponding(), "Should not see packets where cache "
             "is responding");

    panic_if(!(pkt->isRead() || pkt->isWrite()),
             "Should only see read and writes at memory controller\n");

    if (prevArrival != 0) {
        totGap += curTick() - prevArrival;
    }
    prevArrival = curTick();

    unsigned size = pkt->getSize();
    uint32_t pktBurstSize = isVMCMode?devBurstSize:burstSize;
    unsigned offset = pkt->getAddr() & (pktBurstSize - 1);
    unsigned short_pkt_count = divCeil(offset + size, pktBurstSize);
    // DRAMPacket* tmp_pkt = decodeAddr(pkt, pkt->getAddr(), size, pkt->isRead());
    // DPRINTF(VMC, "Generating temp pkt (%d) for device %d_%d\n",
    //        tmp_pkt->id, tmp_pkt->rank, tmp_pkt->device);
    // delete tmp_pkt;
    if (pkt->isRead()) {
        if (readQueueFull(short_pkt_count)) {
            DPRINTF(VMC, "Read queue full, not accepting\n");
            retryRdReq = true;
            ++rdRetry;
            return false;
        } else {
            addToReadQueue(pkt, short_pkt_count);
            readReqs++;
            bytesReadSys += size;
        }
    } else {
        assert(pkt->isWrite());
        assert(size != 0);
        if (writeQueueFull(short_pkt_count)) {
            DPRINTF(VMC, "Write queue full, not accepting\n");
            retryWrReq = true;
            ++wrRetry;
            return false;
        } else {
            addToWriteQueue(pkt, short_pkt_count);
            writeReqs++;
            bytesWrittenSys += size;
        }
    }

    return true;
}

/** weil0ng:
 * Prints status of outstandingEvents of a given rank.
 */
void
DRAMCtrl::dPrintEventCount(uint8_t rank, bool inc, std::string reason = std::string()) {
    // weil0ng: remove the following to start printing.
    return;
    if (!system()->isVMCMode())
        return;
    DPRINTF(VMC, "Outstanding events for rank %d: %s -> %d, %s\n",
            rank, inc?"+1":"-1", ranks[rank]->outstandingEvents, reason);
    return;
}

/** weil0ng:
 * Prints status of addrRegs of given rank.
 */
void
DRAMCtrl::dPrintAddrRegs(uint8_t rank, uint8_t device, bool isRead) {
    if (!isVMCMode)
        return;
    DPRINTF(AddrRegs, "%s addrRegs on rank%d_device%d:      |\n",
            isRead?"Read":"Write", rank, device);
    for (auto b : ranks[rank]->devices[device]->banks) {
        std::deque<DRAMPacket*>& addrRegs = isRead?b->rdAddrRegs:b->wrAddrRegs;
        for (auto p = addrRegs.begin(); p != addrRegs.end(); ++p) {
            DRAMPacket* push_pkt = *p;
            assert(!push_pkt->isPack);
            DPRINTF(AddrRegs, "Bank%d: Pkt (%d) |\n", b->bank, push_pkt->id);
        }
    }
    return;
}

void
DRAMCtrl::processRespondEvent()
{
    DPRINTF(DRAM,
            "processRespondEvent(): Some req has reached its readyTime\n");

    DRAMPacket* dram_pkt = respQueue.front();

    if (!dram_pkt->isPack) {
        assert(!isVMCMode);
        // if a read has reached its ready-time, decrement the number of reads
        // At this point the packet has been handled and there is a possibility
        // to switch to low-power mode if no other packet is available
        --dram_pkt->rankRef.readEntries;
        DPRINTF(DRAM, "number of read entries for rank %d is %d\n",
                dram_pkt->rank, dram_pkt->rankRef.readEntries);

        // counter should at least indicate one outstanding request
        // for this read
        assert(dram_pkt->rankRef.outstandingEvents > 0);
        // read response received, decrement count
        --dram_pkt->rankRef.outstandingEvents;
        std::ostringstream oss;
        oss << "read response (" << dram_pkt->id << ") received";
        dPrintEventCount(dram_pkt->rank, false, oss.str());

        // at this moment should not have transitioned to a low-power state
        assert((dram_pkt->rankRef.pwrState != PWR_SREF) &&
                (dram_pkt->rankRef.pwrState != PWR_PRE_PDN) &&
                (dram_pkt->rankRef.pwrState != PWR_ACT_PDN));

        // track if this is the last packet before idling
        // and that there are no outstanding commands to this rank
        // if REF in progress, transition to LP state should not occur
        // until REF completes
        if ((dram_pkt->rankRef.refreshState == REF_IDLE) &&
                (dram_pkt->rankRef.lowPowerEntryReady())) {
            // verify that there are no events scheduled
            assert(!dram_pkt->rankRef.activateEvent.scheduled());
            assert(!dram_pkt->rankRef.prechargeEvent.scheduled());

            // if coming from active state, schedule power event to
            // active power-down else go to precharge power-down
            DPRINTF(DRAMState, "Rank %d sleep at tick %d; current power state is "
                    "%d\n", dram_pkt->rank, curTick(), dram_pkt->rankRef.pwrState);

            // default to ACT power-down unless already in IDLE state
            // could be in IDLE if PRE issued before data returned
            PowerState next_pwr_state = PWR_ACT_PDN;
            if (dram_pkt->rankRef.pwrState == PWR_IDLE) {
                next_pwr_state = PWR_PRE_PDN;
            }

            dram_pkt->rankRef.powerDownSleep(next_pwr_state, curTick());
        }
    }

    // weil0ng: if this is a pack response.
    if (dram_pkt->packHelper) {
        assert(dram_pkt->isPack);
        DPRINTF(VMC, "Pack %s response (%d) on rank %d, has %d pkts\n",
                dram_pkt->isRead?"read":"write", dram_pkt->id, dram_pkt->rank,
                dram_pkt->packHelper->pkt_cnt);
        for (int i=0; i<dram_pkt->packHelper->pkt_cnt; ++i) {
            DRAMPacket* cur_pkt = (*(dram_pkt->packHelper))[i];

            --cur_pkt->deviceRef.readEntries;
            //DPRINTF(VMC, "number of read entries for rank%d_device%d is %d\n",
            //       cur_pkt->rank, cur_pkt->device, cur_pkt->deviceRef.readEntries);
            assert(cur_pkt->deviceRef.outstandingEvents > 0);
            --cur_pkt->deviceRef.outstandingEvents;
            assert((cur_pkt->deviceRef.pwrState != PWR_SREF) &&
                    (cur_pkt->deviceRef.pwrState != PWR_PRE_PDN) &&
                    (cur_pkt->deviceRef.pwrState != PWR_ACT_PDN));
            if ((cur_pkt->deviceRef.refreshState == REF_IDLE) &&
                    cur_pkt->deviceRef.lowPowerEntryReady()) {
                assert(!cur_pkt->deviceRef.activateEvent.scheduled());
                assert(!cur_pkt->deviceRef.prechargeEvent.scheduled());
                DPRINTF(DRAMState, "Device %d_%d sleep at tick %d; current power state is %d\n",
                        cur_pkt->rank, cur_pkt->device, curTick(), cur_pkt->deviceRef.pwrState);
                PowerState next_pwr_state = PWR_ACT_PDN;
                if (cur_pkt->deviceRef.pwrState == PWR_IDLE)
                    next_pwr_state = PWR_PRE_PDN;
                cur_pkt->deviceRef.powerDownSleep(next_pwr_state, curTick());
            }
            // weil0ng: cur_pkt should never be of multiple bursts.
            //accessAndRespond(cur_pkt->pkt, frontendLatency + backendLatency);
            if (cur_pkt->burstHelper) {
                DPRINTF(VMC, "Pack burstHelper has %d/%d serviced\n",
                        cur_pkt->burstHelper->burstsServiced, cur_pkt->burstHelper->burstCount);
                cur_pkt->burstHelper->burstsServiced++;
                if (cur_pkt->burstHelper->burstsServiced == cur_pkt->burstHelper->burstCount) {
                    accessAndRespond(cur_pkt->pkt, frontendLatency + backendLatency);
                    assert(cur_pkt != NULL && cur_pkt->burstHelper != NULL);
                    delete cur_pkt->burstHelper;
                    cur_pkt->burstHelper=NULL;
                }
            } else {
                accessAndRespond(cur_pkt->pkt, frontendLatency + backendLatency);
            }
        }
        // Destroy the packed dram pkts.
        assert(dram_pkt != NULL && dram_pkt->packHelper != NULL);
        // DESTROY actual read pkts.
        dram_pkt->packHelper->destroyPkts();
        delete dram_pkt->packHelper;
        dram_pkt->packHelper = NULL;
    } else if (dram_pkt->burstHelper) {
        // it is a split packet
        dram_pkt->burstHelper->burstsServiced++;
        if (dram_pkt->burstHelper->burstsServiced ==
            dram_pkt->burstHelper->burstCount) {
            // we have now serviced all children packets of a system packet
            // so we can now respond to the requester
            // @todo we probably want to have a different front end and back
            // end latency for split packets
            accessAndRespond(dram_pkt->pkt, frontendLatency + backendLatency);
            delete dram_pkt->burstHelper;
            dram_pkt->burstHelper = NULL;
        }
    } else {
        // it is not a split packet
        accessAndRespond(dram_pkt->pkt, frontendLatency + backendLatency);
    }

    assert(respQueue.front() != NULL);
    respQueue.pop_front();

    if (dram_pkt->isPack) {
        delete dram_pkt->pkt->req;
        delete dram_pkt->pkt;
    }
    delete dram_pkt;

    if (!respQueue.empty()) {
        assert(respQueue.front()->readyTime >= curTick());
        assert(!respondEvent.scheduled());
        schedule(respondEvent, respQueue.front()->readyTime);
    } else {
        // if there is nothing left in any queue, signal a drain
        if (drainState() == DrainState::Draining &&
            writeQueue.empty() && readQueue.empty() && allRanksDrained()) {

            DPRINTF(Drain, "DRAM controller done draining\n");
            signalDrainDone();
        }
    }
   
    // We have made a location in the queue available at this point,
    // so if there is a read that was forced to wait, retry now
    if (retryRdReq) {
        retryRdReq = false;
        port.sendRetryReq();
    }
}

bool
DRAMCtrl::DRAMPacket::packReady() {
    assert(this->isPack);
    // Check device availabilities.
    bool device_ready = true;
    for (int i=0; i<this->packHelper->pkt_cnt; ++i) {
        DRAMPacket* cur_pkt = (*(this->packHelper))[i];
        device_ready &= cur_pkt->deviceRef.isAvailable();
    }
    bool addr_ready = true;
    // Check address availabilities.
    for (int i=0; i<this->packHelper->pkt_cnt; ++i) {
        DRAMPacket* cur_pkt = (*(this->packHelper))[i];
        std::deque<DRAMPacket*>& addrRegs = isRead?
            cur_pkt->bankRef.rdAddrRegs:cur_pkt->bankRef.wrAddrRegs;
        addr_ready &= (std::find(addrRegs.begin(), addrRegs.end(),
                    cur_pkt) != addrRegs.end());
    }
    return device_ready & addr_ready;
}

bool
DRAMCtrl::DRAMPacket::canPush() {
    assert(this->isPack);
    DRAMCtrl& memory = this->deviceRef.memory;
    bool avail = true;
    // Check addrReg space and dbus.
    for (int i=0; i<this->packHelper->pkt_cnt; ++i) {
        DRAMPacket* cur_pkt = (*(this->packHelper))[i];
        avail &= ((isRead?
                    cur_pkt->bankRef.pendingRdPushPkt:cur_pkt->bankRef.pendingWrPushPkt)
                == NULL);
        std::deque<DRAMPacket*>& addrRegs = isRead?
            cur_pkt->bankRef.rdAddrRegs:cur_pkt->bankRef.wrAddrRegs;
        avail &= (addrRegs.size() < memory.addrRegsPerDevice);
        avail &= (curTick() >= cur_pkt->deviceRef.pushAllowedAt);
    }
    // Check bubble on bus.
    avail &= (curTick() + memory.pushDelay <= memory.busBusyUntil - memory.tBURST ||
            curTick() >= memory.busBusyUntil);
    // Check if we are draining pushed reqs.
    avail &= ((this->isRead && !memory.packRdDrain) || (!this->isRead && !memory.packWrDrain));
    return avail;
}

void
DRAMCtrl::DRAMPacket::preparePush() {
    assert(this->isPack);
    assert(this->canPush());
    if (isRead) {
        for (int i=0; i<this->packHelper->pkt_cnt; ++i) {
            DRAMPacket* cur_pkt = (*(this->packHelper))[i];
            assert(!cur_pkt->bankRef.pendingRdPushPkt);
            cur_pkt->bankRef.pendingRdPushPkt = (*(this->packHelper))[i];
        }
    } else {
        for (int i=0; i<this->packHelper->pkt_cnt; ++i) {
            DRAMPacket* cur_pkt = (*(this->packHelper))[i];
            assert(!cur_pkt->bankRef.pendingWrPushPkt);
            cur_pkt->bankRef.pendingWrPushPkt = (*(this->packHelper))[i];
        }
    }
}

void
DRAMCtrl::DRAMPacket::schedulePushEvents(Tick when) {
    assert(this->isPack);
    Tick nearestPush = 0;
    DRAMCtrl& memory = this->deviceRef.memory;
    if (isRead) {
        for (int i=0; i<this->packHelper->pkt_cnt; ++i) {
            DRAMPacket* cur_pkt = (*(this->packHelper))[i];
            assert(cur_pkt->bankRef.pendingRdPushPkt);
            assert(!cur_pkt->bankRef.rdPushEvent.scheduled());
            memory.schedule(cur_pkt->bankRef.rdPushEvent, when);
            cur_pkt->deviceRef.pushAllowedAt = curTick() + memory.pushDelay;
            if (nearestPush == 0)
                nearestPush = cur_pkt->deviceRef.pushAllowedAt;
            if (nearestPush > cur_pkt->deviceRef.pushAllowedAt)
                nearestPush = cur_pkt->deviceRef.pushAllowedAt;
        }
    } else {
        for (int i=0; i<this->packHelper->pkt_cnt; ++i) {
            DRAMPacket* cur_pkt = (*(this->packHelper))[i];
            assert(cur_pkt->bankRef.pendingWrPushPkt);
            assert(!cur_pkt->bankRef.wrPushEvent.scheduled());
            memory.schedule(cur_pkt->bankRef.wrPushEvent, when);
            cur_pkt->deviceRef.pushAllowedAt = curTick() + memory.pushDelay;
            if (nearestPush == 0)
                nearestPush = cur_pkt->deviceRef.pushAllowedAt;
            if (nearestPush > cur_pkt->deviceRef.pushAllowedAt)
                nearestPush = cur_pkt->deviceRef.pushAllowedAt;
        }
    }
    if (!memory.nextReqEvent.scheduled()) {
        memory.schedule(memory.nextReqEvent, nearestPush);
    }

}

void
DRAMCtrl::DRAMPacket::prepareClear() {
    assert(this->isPack);
    if (isRead) {
        for (int i=0; i<this->packHelper->pkt_cnt; ++i) {
            DRAMPacket* cur_pkt = (*(this->packHelper))[i];
            assert(!cur_pkt->bankRef.pendingRdClearPkt);
            cur_pkt->bankRef.pendingRdClearPkt = (*(this->packHelper))[i];
        }
    } else {
        for (int i=0; i<this->packHelper->pkt_cnt; ++i) {
            DRAMPacket* cur_pkt = (*(this->packHelper))[i];
            assert(!cur_pkt->bankRef.pendingWrClearPkt);
            cur_pkt->bankRef.pendingWrClearPkt = (*(this->packHelper))[i];
        }
    }
}

void
DRAMCtrl::DRAMPacket::scheduleClearEvents(Tick when) {
    assert(this->isPack);
    DRAMCtrl& memory = this->deviceRef.memory;
    if (isRead) {
        for (int i=0; i<this->packHelper->pkt_cnt; ++i) {
            DRAMPacket* cur_pkt = (*(this->packHelper))[i];
            assert(cur_pkt->bankRef.pendingRdClearPkt);
            assert(!cur_pkt->bankRef.rdClearEvent.scheduled());
            memory.schedule(cur_pkt->bankRef.rdClearEvent, when);
        }
    } else {
        for (int i=0; i<this->packHelper->pkt_cnt; ++i) {
            DRAMPacket* cur_pkt = (*(this->packHelper))[i];
            assert(cur_pkt->bankRef.pendingWrClearPkt);
            assert(!cur_pkt->bankRef.wrClearEvent.scheduled());
            memory.schedule(cur_pkt->bankRef.wrClearEvent, when);
        }
    }
}

bool
DRAMCtrl::chooseNext(std::deque<DRAMPacket*>& queue,
        Tick extra_col_delay)
{
    // This method does the arbitration between requests. The chosen
    // packet is simply moved to the head of the queue. The other
    // methods know that this is the place to look. For example, with
    // FCFS, this method does nothing
    assert(!queue.empty());
    
    int found_id = -1;

    // bool to indicate if a packet to an available rank is found
    bool found_packet = false;

    // Special case: if we only have 1 req.
    if (queue.size() == 1) {
        DRAMPacket* dram_pkt = queue.front();
        if (!dram_pkt->isPack && !dram_pkt->isMicro) {
            // case 1: wide access.
            // available rank corresponds to state refresh idle
            if (ranks[dram_pkt->rank]->isAvailable()) {
                found_packet = true;
                found_id = dram_pkt->id;
                DPRINTF(DRAM, "Single request, going to a free rank\n");
            } else {
                DPRINTF(DRAM, "Single request, going to a busy rank\n");
            }
        } else if (!dram_pkt->isPack) {
            // case 2: micro access.
            assert(isVMCMode);
            assert(dram_pkt->isMicro && !dram_pkt->isPack);
            // Try packing.
            DRAMPacket* trial_pkt = tryPack(queue, queue.front()->rank);
            assert(trial_pkt);
            found_packet = trial_pkt->packReady();
            if (found_packet) {
                // Do the actual packing.
                assert(packMicroPkts(queue, queue.front()->rank));
                // Sanity check.
                assert(checkPackEqual(trial_pkt, queue.front()));
                found_id = queue.front()->id;
            }
            else {
                if (trial_pkt->canPush()) {
                    // Do the actual packing.
                    assert(packMicroPkts(queue, queue.front()->rank));
                    // Sanity check.
                    assert(checkPackEqual(trial_pkt, queue.front()));
                    dram_pkt = queue.front();
                    dram_pkt->preparePush();
                    DPRINTF(Push, "pushEvent (from %d) scheduled now\n", dram_pkt->id);
                    // Do the push.
                    if (prevPushScheduled > 0)
                        totPushEventGap += curTick() - prevPushScheduled;
                    prevPushScheduled = curTick();
                    dram_pkt->schedulePushEvents(curTick());
                }
            }
            // Destroy trial pkt.
            assert(destroyTrialPkt(trial_pkt));
        } else {
            // case 3: pack access.
            assert(dram_pkt->isPack && !dram_pkt->isMicro);
            found_packet = dram_pkt->packReady();
            if (found_packet)
                found_id = dram_pkt->id;
        }
        if (isVMCMode) {
            if (found_packet)
                DPRINTF(VMC, "Pkt %d found\n", found_id);
            else
                DPRINTF(VMC, "Pkt not found\n");
        }
        return found_packet;
    }
    // Otherwise, if we have multiple reqs...
    if (isVMCMode) {
        for (uint8_t i=0; i<ranksPerChannel; ++i) {
            DRAMPacket* trial_pkt = tryPack(queue, i);
            if (trial_pkt) {
                if (trial_pkt->canPush()) {
                    assert(packMicroPkts(queue, i));
                    assert(checkPackEqual(trial_pkt, queue.front()));
                    assert(destroyTrialPkt(trial_pkt));
                    DRAMPacket* dram_pkt = queue.front();
                    dram_pkt->preparePush();
                    DPRINTF(Push, "pushEvent (from %d) scheduled now\n", dram_pkt->id);
                    if (prevPushScheduled > 0)
                        totPushEventGap += curTick() - prevPushScheduled;
                    prevPushScheduled = curTick();
                    dram_pkt->schedulePushEvents(curTick());
                } else {
                    assert(destroyTrialPkt(trial_pkt));
                }
            }
        }

        // frfcfs for pack pkts.
        for (auto i=queue.begin(); i!=queue.end(); ++i) {
            DRAMPacket* dram_pkt = *i;
            if (dram_pkt->isPack && dram_pkt->packReady()) {
                queue.erase(i);
                queue.push_front(dram_pkt);
                found_packet = true;
                found_id = dram_pkt->id;
                break;
            }
        }
    } else {
        // All wide accesses.
        if (memSchedPolicy == Enums::fcfs) {
            // check if there is a packet going to a free rank
            for (auto i = queue.begin(); i != queue.end() ; ++i) {
                DRAMPacket* dram_pkt = *i;
                if (ranks[dram_pkt->rank]->isAvailable()) {
                    queue.erase(i);
                    queue.push_front(dram_pkt);
                    found_packet = true;
                    found_id = dram_pkt->id;
                    break;
                }
            }
        } else if (memSchedPolicy == Enums::frfcfs) {
            found_packet = reorderQueue(queue, extra_col_delay);
        } else
            panic("No scheduling policy chosen\n");
    }
    if (isVMCMode) {
        if (found_packet)
            DPRINTF(VMC, "Pkt %d found\n", found_id);
        else
            DPRINTF(VMC, "Pkt not found\n");
    }
    return found_packet;
}

bool
DRAMCtrl::reorderQueue(std::deque<DRAMPacket*>& queue, Tick extra_col_delay)
{
    // Only determine this if needed
    uint64_t earliest_banks = 0;
    bool hidden_bank_prep = false;

    // search for seamless row hits first, if no seamless row hit is
    // found then determine if there are other packets that can be issued
    // without incurring additional bus delay due to bank timing
    // Will select closed rows first to enable more open row possibilies
    // in future selections
    bool found_hidden_bank = false;

    // remember if we found a row hit, not seamless, but bank prepped
    // and ready
    bool found_prepped_pkt = false;

    // if we have no row hit, prepped or not, and no seamless packet,
    // just go for the earliest possible
    bool found_earliest_pkt = false;

    auto selected_pkt_it = queue.end();

    // time we need to issue a column command to be seamless
    const Tick min_col_at = std::max(busBusyUntil - tCL + extra_col_delay,
                                     curTick());

    for (auto i = queue.begin(); i != queue.end() ; ++i) {
        DRAMPacket* dram_pkt = *i;
        const Bank& bank = *(dram_pkt->rankRef.banks[dram_pkt->bank]);

        // check if rank is available, if not, jump to the next packet
        if (dram_pkt->rankRef.isAvailable()) {
            // check if it is a row hit
            if (bank.openRow == dram_pkt->row) {
                // no additional rank-to-rank or same bank-group
                // delays, or we switched read/write and might as well
                // go for the row hit
                if (bank.colAllowedAt <= min_col_at) {
                    // FCFS within the hits, giving priority to
                    // commands that can issue seamlessly, without
                    // additional delay, such as same rank accesses
                    // and/or different bank-group accesses
                    DPRINTF(DRAM, "Seamless row buffer hit\n");
                    selected_pkt_it = i;
                    // no need to look through the remaining queue entries
                    break;
                } else if (!found_hidden_bank && !found_prepped_pkt) {
                    // if we did not find a packet to a closed row that can
                    // issue the bank commands without incurring delay, and
                    // did not yet find a packet to a prepped row, remember
                    // the current one
                    selected_pkt_it = i;
                    found_prepped_pkt = true;
                    DPRINTF(DRAM, "Prepped row buffer hit\n");
                }
            } else if (!found_earliest_pkt) {
                // if we have not initialised the bank status, do it
                // now, and only once per scheduling decisions
                if (earliest_banks == 0) {
                    // determine entries with earliest bank delay
                    pair<uint64_t, bool> bankStatus =
                        minBankPrep(queue, min_col_at);
                    earliest_banks = bankStatus.first;
                    hidden_bank_prep = bankStatus.second;
                }

                // bank is amongst first available banks
                // minBankPrep will give priority to packets that can
                // issue seamlessly
                if (bits(earliest_banks, dram_pkt->bankId, dram_pkt->bankId)) {
                    found_earliest_pkt = true;
                    found_hidden_bank = hidden_bank_prep;

                    // give priority to packets that can issue
                    // bank commands 'behind the scenes'
                    // any additional delay if any will be due to
                    // col-to-col command requirements
                    if (hidden_bank_prep || !found_prepped_pkt)
                        selected_pkt_it = i;
                }
            }
        }
    }

    if (selected_pkt_it != queue.end()) {
        DRAMPacket* selected_pkt = *selected_pkt_it;
        queue.erase(selected_pkt_it);
        queue.push_front(selected_pkt);
        return true;
    }

    return false;
}

void
DRAMCtrl::accessAndRespond(PacketPtr pkt, Tick static_latency)
{
    if (pkt->req->hasContextId())
        DPRINTF(MemCtx, "to %d\n", pkt->req->contextId());
    DPRINTF(DRAM, "Responding to address %#08x..\n", pkt->getAddr());
    if (isVMCMode)
        DPRINTF(VMC, "Responding to address %#08x..\n", pkt->getAddr());

    bool needsResponse = pkt->needsResponse();
    // do the actual memory access which also turns the packet into a
    // response
    access(pkt);

    // turn packet around to go back to requester if response expected
    if (needsResponse) {
        // access already turned the packet into a response
        assert(pkt->isResponse());
        // response_time consumes the static latency and is charged also
        // with headerDelay that takes into account the delay provided by
        // the xbar and also the payloadDelay that takes into account the
        // number of data beats.
        Tick response_time = curTick() + static_latency + pkt->headerDelay +
                             pkt->payloadDelay;
        // Here we reset the timing of the packet before sending it out.
        pkt->headerDelay = pkt->payloadDelay = 0;

        // queue the packet in the response queue to be sent out after
        // the static latency has passed
        port.schedTimingResp(pkt, response_time, true);
    } else {
        // @todo the packet is going to be deleted, and the DRAMPacket
        // is still having a pointer to it
        pendingDelete.reset(pkt);
    }

    DPRINTF(DRAM, "Done\n");

    return;
}

void
DRAMCtrl::activateBank(Rank& rankRef, Bank& bankRef,
                       Tick act_tick, uint32_t row)
{
    assert(!isVMCMode);
    assert(rankRef.actTicks.size() == activationLimit);

    // update the open row
    assert(bankRef.openRow == Bank::NO_ROW);
    bankRef.openRow = row;

    // start counting anew, this covers both the case when we
    // auto-precharged, and when this access is forced to
    // precharge
    bankRef.bytesAccessed = 0;
    bankRef.rowAccesses = 0;

    ++rankRef.numBanksActive;
    assert(rankRef.numBanksActive <= banksPerRank);

    DPRINTF(DRAM, "Activate bank %d, rank %d at tick %lld, now got %d active\n",
            bankRef.bank, rankRef.rank, act_tick,
            rankRef.numBanksActive);

    rankRef.cmdList.push_back(Command(MemCommand::ACT, bankRef.bank,
                               act_tick));
    ++rankRef.actCmds;
    for (auto d : rankRef.devices)
        ++rankRef.devActCmds[d->device];

    DPRINTF(DRAMPower, "%llu,ACT,%d,%d\n", divCeil(act_tick, tCK) -
            timeStampOffset, bankRef.bank, rankRef.rank);

    // The next access has to respect tRAS for this bank
    bankRef.preAllowedAt = act_tick + tRAS;

    // Respect the row-to-column command delay
    bankRef.colAllowedAt = std::max(act_tick + tRCD, bankRef.colAllowedAt);

    // start by enforcing tRRD
    for (int i = 0; i < banksPerRank; i++) {
        // next activate to any bank in this rank must not happen
        // before tRRD
        if (bankGroupArch && (bankRef.bankgr == rankRef.banks[i]->bankgr)) {
            // bank group architecture requires longer delays between
            // ACT commands within the same bank group.  Use tRRD_L
            // in this case
            rankRef.banks[i]->actAllowedAt = std::max(act_tick + tRRD_L,
                                             rankRef.banks[i]->actAllowedAt);
        } else {
            // use shorter tRRD value when either
            // 1) bank group architecture is not supportted
            // 2) bank is in a different bank group
            rankRef.banks[i]->actAllowedAt = std::max(act_tick + tRRD,
                                             rankRef.banks[i]->actAllowedAt);
        }
    }

    // next, we deal with tXAW, if the activation limit is disabled
    // then we directly schedule an activate power event
    if (!rankRef.actTicks.empty()) {
        // sanity check
        if (rankRef.actTicks.back() &&
           (act_tick - rankRef.actTicks.back()) < tXAW) {
            panic("Got %d activates in window %d (%llu - %llu) which "
                  "is smaller than %llu\n", activationLimit, act_tick -
                  rankRef.actTicks.back(), act_tick,
                  rankRef.actTicks.back(), tXAW);
        }

        // shift the times used for the book keeping, the last element
        // (highest index) is the oldest one and hence the lowest value
        rankRef.actTicks.pop_back();

        // record an new activation (in the future)
        rankRef.actTicks.push_front(act_tick);

        // cannot activate more than X times in time window tXAW, push the
        // next one (the X + 1'st activate) to be tXAW away from the
        // oldest in our window of X
        if (rankRef.actTicks.back() &&
           (act_tick - rankRef.actTicks.back()) < tXAW) {
            DPRINTF(DRAM, "Enforcing tXAW with X = %d, next activate "
                    "no earlier than %llu\n", activationLimit,
                    rankRef.actTicks.back() + tXAW);
            for (int j = 0; j < banksPerRank; j++)
                // next activate must not happen before end of window
                rankRef.banks[j]->actAllowedAt =
                    std::max(rankRef.actTicks.back() + tXAW,
                             rankRef.banks[j]->actAllowedAt);
        }
    }

    // at the point when this activate takes place, make sure we
    // transition to the active power state
    if (!rankRef.activateEvent.scheduled())
        schedule(rankRef.activateEvent, act_tick);
    else if (rankRef.activateEvent.when() > act_tick)
        // move it sooner in time
        reschedule(rankRef.activateEvent, act_tick);
}

void
DRAMCtrl::activateBank(Device& deviceRef, Bank& bankRef,
                       Tick act_tick, uint32_t row)
{
    assert(deviceRef.actTicks.size() == activationLimit);

    // update the open row
    assert(bankRef.openRow == Bank::NO_ROW);
    bankRef.openRow = row;

    // start counting anew, this covers both the case when we
    // auto-precharged, and when this access is forced to
    // precharge
    bankRef.bytesAccessed = 0;
    bankRef.rowAccesses = 0;

    ++deviceRef.numBanksActive;
    assert(deviceRef.numBanksActive <= banksPerRank);

    DPRINTF(DRAM, "Activate bank %d, device %d_%d at tick %lld, now got %d active\n",
            bankRef.bank, deviceRef.rank, deviceRef.device, act_tick, deviceRef.numBanksActive);

    deviceRef.cmdList.push_back(Command(MemCommand::ACT, bankRef.bank,
                               act_tick));
    ++deviceRef.rankRef.devActCmds[deviceRef.device];

    DPRINTF(DRAMPower, "%llu,ACT,%d,%d\n", divCeil(act_tick, tCK) -
            timeStampOffset, bankRef.bank, deviceRef.device);

    // The next access has to respect tRAS for this bank
    bankRef.preAllowedAt = act_tick + tRAS;

    // Respect the row-to-column command delay
    bankRef.colAllowedAt = std::max(act_tick + tRCD, bankRef.colAllowedAt);

    // start by enforcing tRRD
    for (int i = 0; i < banksPerRank; i++) {
        // next activate to any bank in this rank must not happen
        // before tRRD
        if (bankGroupArch && (bankRef.bankgr == deviceRef.banks[i]->bankgr)) {
            // bank group architecture requires longer delays between
            // ACT commands within the same bank group.  Use tRRD_L
            // in this case
            deviceRef.banks[i]->actAllowedAt = std::max(act_tick + tRRD_L,
                                             deviceRef.banks[i]->actAllowedAt);
        } else {
            // use shorter tRRD value when either
            // 1) bank group architecture is not supportted
            // 2) bank is in a different bank group
            deviceRef.banks[i]->actAllowedAt = std::max(act_tick + tRRD,
                                             deviceRef.banks[i]->actAllowedAt);
        }
    }

    // next, we deal with tXAW, if the activation limit is disabled
    // then we directly schedule an activate power event
    if (!deviceRef.actTicks.empty()) {
        // sanity check
        if (deviceRef.actTicks.back() &&
           (act_tick - deviceRef.actTicks.back()) < tXAW) {
            panic("Got %d activates in window %d (%llu - %llu) which "
                  "is smaller than %llu\n", activationLimit, act_tick -
                  deviceRef.actTicks.back(), act_tick,
                  deviceRef.actTicks.back(), tXAW);
        }

        // shift the times used for the book keeping, the last element
        // (highest index) is the oldest one and hence the lowest value
        deviceRef.actTicks.pop_back();

        // record an new activation (in the future)
        deviceRef.actTicks.push_front(act_tick);

        // cannot activate more than X times in time window tXAW, push the
        // next one (the X + 1'st activate) to be tXAW away from the
        // oldest in our window of X
        if (deviceRef.actTicks.back() &&
           (act_tick - deviceRef.actTicks.back()) < tXAW) {
            DPRINTF(DRAM, "Enforcing tXAW with X = %d, next activate "
                    "no earlier than %llu\n", activationLimit,
                    deviceRef.actTicks.back() + tXAW);
            for (int j = 0; j < banksPerRank; j++)
                // next activate must not happen before end of window
                deviceRef.banks[j]->actAllowedAt =
                    std::max(deviceRef.actTicks.back() + tXAW,
                             deviceRef.banks[j]->actAllowedAt);
        }
    }

    // at the point when this activate takes place, make sure we
    // transition to the active power state
    if (!deviceRef.activateEvent.scheduled())
        schedule(deviceRef.activateEvent, act_tick);
    else if (deviceRef.activateEvent.when() > act_tick)
        // move it sooner in time
        reschedule(deviceRef.activateEvent, act_tick);
}

void
DRAMCtrl::prechargeBank(Rank& rankRef, Bank& bankRef, Tick pre_at, bool trace)
{
    assert(!isVMCMode);
    // make sure the bank has an open row
    assert(bankRef.openRow != Bank::NO_ROW);

    // sample the bytes per activate here since we are closing
    // the page
    bytesPerActivate.sample(bankRef.bytesAccessed);

    bankRef.openRow = Bank::NO_ROW;

    // no precharge allowed before this one
    bankRef.preAllowedAt = pre_at;

    Tick pre_done_at = pre_at + tRP;

    bankRef.actAllowedAt = std::max(bankRef.actAllowedAt, pre_done_at);

    assert(rankRef.numBanksActive != 0);
    --rankRef.numBanksActive;

    DPRINTF(DRAM, "Precharging rank%d_bank%d at tick %lld, now got "
            "%d active\n", rankRef.rank, bankRef.bank, pre_at,
            rankRef.numBanksActive);

    if (trace) {

        rankRef.cmdList.push_back(Command(MemCommand::PRE, bankRef.bank,
                                   pre_at));
        DPRINTF(DRAMPower, "%llu,PRE,%d,%d\n", divCeil(pre_at, tCK) -
                timeStampOffset, bankRef.bank, rankRef.rank);
    }
    // if we look at the current number of active banks we might be
    // tempted to think the DRAM is now idle, however this can be
    // undone by an activate that is scheduled to happen before we
    // would have reached the idle state, so schedule an event and
    // rather check once we actually make it to the point in time when
    // the (last) precharge takes place
    if (!rankRef.prechargeEvent.scheduled()) {
        schedule(rankRef.prechargeEvent, pre_done_at);
        // New event, increment count
        ++rankRef.outstandingEvents;
        std::ostringstream oss;
        oss << "precharge";
        dPrintEventCount(rankRef.rank, true, oss.str());
    } else if (rankRef.prechargeEvent.when() < pre_done_at) {
        reschedule(rankRef.prechargeEvent, pre_done_at);
    }
}

void
DRAMCtrl::prechargeBank(Device& deviceRef, Bank& bankRef, Tick pre_at, bool trace)
{
    // make sure the bank has an open row
    assert(bankRef.openRow != Bank::NO_ROW);

    // sample the bytes per activate here since we are closing
    // the page
    bytesPerActivate.sample(bankRef.bytesAccessed);

    bankRef.openRow = Bank::NO_ROW;

    // no precharge allowed before this one
    bankRef.preAllowedAt = pre_at;

    Tick pre_done_at = pre_at + tRP;

    bankRef.actAllowedAt = std::max(bankRef.actAllowedAt, pre_done_at);

    assert(deviceRef.numBanksActive != 0);
    --deviceRef.numBanksActive;

    DPRINTF(VMC, "Precharging rank%d_device%d_bank%d at tick %lld, now got "
            "%d active\n", deviceRef.rank, deviceRef.device, bankRef.bank,
            pre_at, deviceRef.numBanksActive);

    if (trace) {

        deviceRef.cmdList.push_back(Command(MemCommand::PRE, bankRef.bank,
                                   pre_at));
        DPRINTF(DRAMPower, "%llu,PRE,%d,%d\n", divCeil(pre_at, tCK) -
                timeStampOffset, bankRef.bank, deviceRef.device);
    }
    // if we look at the current number of active banks we might be
    // tempted to think the DRAM is now idle, however this can be
    // undone by an activate that is scheduled to happen before we
    // would have reached the idle state, so schedule an event and
    // rather check once we actually make it to the point in time when
    // the (last) precharge takes place
    if (!deviceRef.prechargeEvent.scheduled()) {
        schedule(deviceRef.prechargeEvent, pre_done_at);
        // New event, increment count
        ++deviceRef.outstandingEvents;
        DPRINTF(DeviceState, "Device %d_%d schedule prechargeEvent, outstadingEvents: %d\n",
                deviceRef.rank, deviceRef.device, deviceRef.outstandingEvents);
    } else if (deviceRef.prechargeEvent.when() < pre_done_at) {
        reschedule(deviceRef.prechargeEvent, pre_done_at);
    }
}

Tick
DRAMCtrl::doBankAccess(DRAMPacket* dram_pkt, Tick busBusyUntil) {
    if (dram_pkt->isPack) {
        assert(isVMCMode);
        DPRINTF(VMC, "Access rank%d for size %d\n",
                dram_pkt->rank, dram_pkt->size);
        return doMultiBankAccess(dram_pkt, busBusyUntil);
    } else {
        return doSingleBankAccess(dram_pkt, busBusyUntil);
    }
}

Tick
DRAMCtrl::doSingleBankAccess(DRAMPacket* dram_pkt, Tick busBusyUntil) {
    assert(!isVMCMode);
    assert(!dram_pkt->isPack);
    assert(dram_pkt->size <= burstSize);
    // Get the rank.
    Rank& rankRef = dram_pkt->rankRef;
    // Get the bank, note that this is not the device bank.
    Bank& bankRef = *(dram_pkt->rankRef.banks[dram_pkt->bank]);

    // are we in or transitioning to a low-power state and have not scheduled
    // a power-up event?
    // if so, wake up from power down to issue RD/WR burst
    if (rankRef.inLowPowerState) {
        assert(rankRef.pwrState != PWR_SREF);
        rankRef.scheduleWakeUpEvent(tXP);
    }

    // for the state we need to track if it is a row hit or not
    bool row_hit = true;

    // respect any constraints on the command (e.g. tRCD or tCCD)
    Tick cmd_at = std::max(bankRef.colAllowedAt, curTick());

    // Determine the access latency and update the bank state
    if (bankRef.openRow == dram_pkt->row) {
        // nothing to do
    } else {
        row_hit = false;

        // If there is a page open, precharge it.
        if (bankRef.openRow != Bank::NO_ROW) {
            prechargeBank(rankRef, bankRef, std::max(bankRef.preAllowedAt, curTick()));
        }

        // next we need to account for the delay in activating the
        // page
        Tick act_tick = std::max(bankRef.actAllowedAt, curTick());

        // Record the activation and deal with all the global timing
        // constraints caused be a new activation (tRRD and tXAW)
        activateBank(rankRef, bankRef, act_tick, dram_pkt->row);

        // issue the command as early as possible
        cmd_at = bankRef.colAllowedAt;
    }

    // we need to wait until the bus is available before we can issue
    // the command
    cmd_at = std::max(cmd_at, busBusyUntil - tCL);

    // update the packet ready time
    dram_pkt->readyTime = cmd_at + tCL + tBURST;

    // only one burst can use the bus at any one point in time
    assert(dram_pkt->readyTime - busBusyUntil >= tBURST);

    // update the time for the next read/write burst for each
    // bank (add a max with tCCD/tCCD_L here)
    Tick cmd_dly;
    for (int j = 0; j < ranksPerChannel; j++) {
        for (int i = 0; i < banksPerRank; i++) {
            // next burst to same bank group in this rank must not happen
            // before tCCD_L.  Different bank group timing requirement is
            // tBURST; Add tCS for different ranks
            if (dram_pkt->rank == j) {
                if (bankGroupArch &&
                   (bankRef.bankgr == ranks[j]->banks[i]->bankgr)) {
                    // bank group architecture requires longer delays between
                    // RD/WR burst commands to the same bank group.
                    // Use tCCD_L in this case
                    cmd_dly = tCCD_L;
                } else {
                    // use tBURST (equivalent to tCCD_S), the shorter
                    // cas-to-cas delay value, when either:
                    // 1) bank group architecture is not supportted
                    // 2) bank is in a different bank group
                    cmd_dly = tBURST;
                }
            } else {
                // different rank is by default in a different bank group
                // use tBURST (equivalent to tCCD_S), which is the shorter
                // cas-to-cas delay in this case
                // Add tCS to account for rank-to-rank bus delay requirements
                cmd_dly = tBURST + tCS;
            }
            ranks[j]->banks[i]->colAllowedAt = std::max(cmd_at + cmd_dly,
                                             ranks[j]->banks[i]->colAllowedAt);
        }
    }

    // Save rank of current access
    activeRank = dram_pkt->rank;

    // If this is a write, we also need to respect the write recovery
    // time before a precharge, in the case of a read, respect the
    // read to precharge constraint
    bankRef.preAllowedAt = std::max(bankRef.preAllowedAt,
                                 dram_pkt->isRead ? cmd_at + tRTP :
                                 dram_pkt->readyTime + tWR);

    // increment the bytes accessed and the accesses per row
    bankRef.bytesAccessed += burstSize;
    ++bankRef.rowAccesses;

    // if we reached the max, then issue with an auto-precharge
    bool auto_precharge = pageMgmt == Enums::close ||
        bankRef.rowAccesses == maxAccessesPerRow;

    // if we did not hit the limit, we might still want to
    // auto-precharge
    if (!auto_precharge &&
        (pageMgmt == Enums::open_adaptive ||
         pageMgmt == Enums::close_adaptive)) {
        // a twist on the open and close page policies:
        // 1) open_adaptive page policy does not blindly keep the
        // page open, but close it if there are no row hits, and there
        // are bank conflicts in the queue
        // 2) close_adaptive page policy does not blindly close the
        // page, but closes it only if there are no row hits in the queue.
        // In this case, only force an auto precharge when there
        // are no same page hits in the queue
        bool got_more_hits = false;
        bool got_bank_conflict = false;

        // either look at the read queue or write queue
        const deque<DRAMPacket*>& queue = dram_pkt->isRead ? readQueue :
            writeQueue;
        auto p = queue.begin();
        // make sure we are not considering the packet that we are
        // currently dealing with (which is the head of the queue)
        ++p;

        // keep on looking until we find a hit or reach the end of the queue
        // 1) if a hit is found, then both open and close adaptive policies keep
        // the page open
        // 2) if no hit is found, got_bank_conflict is set to true if a bank
        // conflict request is waiting in the queue
        while (!got_more_hits && p != queue.end()) {
            bool same_rank_bank = (dram_pkt->rank == (*p)->rank) &&
                (dram_pkt->bank == (*p)->bank);
            bool same_row = dram_pkt->row == (*p)->row;
            got_more_hits |= same_rank_bank && same_row;
            got_bank_conflict |= same_rank_bank && !same_row;
            ++p;
        }

        // auto pre-charge when either
        // 1) open_adaptive policy, we have not got any more hits, and
        //    have a bank conflict
        // 2) close_adaptive policy and we have not got any more hits
        auto_precharge = !got_more_hits &&
            (got_bank_conflict || pageMgmt == Enums::close_adaptive);
    }

    // DRAMPower trace command to be written
    std::string mem_cmd = dram_pkt->isRead ? "RD" : "WR";

    // MemCommand required for DRAMPower library
    MemCommand::cmds command = (mem_cmd == "RD") ? MemCommand::RD :
                                                   MemCommand::WR;

    // Update bus state
    busBusyUntil = dram_pkt->readyTime;

    DPRINTF(DRAM, "Single access to %#08x, ready at %lld busBusyUntil %d cycles later.\n",
            dram_pkt->addr, dram_pkt->readyTime, (busBusyUntil - curTick())/tCK);

    dram_pkt->rankRef.cmdList.push_back(Command(command, dram_pkt->bank,
                                        cmd_at));

    DPRINTF(DRAMPower, "%llu,%s,%d,%d\n", divCeil(cmd_at, tCK) -
            timeStampOffset, mem_cmd, dram_pkt->bank, dram_pkt->rank);

    // if this access should use auto-precharge, then we are
    // closing the row after the read/write burst
    if (auto_precharge) {
        // if auto-precharge push a PRE command at the correct tick to the
        // list used by DRAMPower library to calculate power
        prechargeBank(rankRef, bankRef, std::max(curTick(), bankRef.preAllowedAt));

        DPRINTF(DRAM, "Auto-precharged bank: %d\n", dram_pkt->bankId);
    }

    // Update the stats and schedule the next request
    if (dram_pkt->isRead) {
        ++readsThisTime;
        if (row_hit)
            readRowHits++;
        if (isVMCMode)
            bytesReadDRAM += dram_pkt->size;
        else
            bytesReadDRAM += burstSize;
        perBankRdBursts[dram_pkt->bankId]++;

        // Update latency stats
        totMemAccLat += dram_pkt->readyTime - dram_pkt->entryTime;
        totBusLat += tBURST;
        totQLat += cmd_at - dram_pkt->entryTime;
    } else {
        ++writesThisTime;
        if (row_hit)
            writeRowHits++;
        if (isVMCMode)
            bytesWritten += dram_pkt->size;
        else
            bytesWritten += burstSize;
        perBankWrBursts[dram_pkt->bankId]++;
    }

    return busBusyUntil;
}

Tick
DRAMCtrl::doMultiBankAccess(DRAMPacket* dram_pkt, Tick busBusyUntil) {
    assert(isVMCMode);
    assert(dram_pkt->isPack);
    assert(dram_pkt->size <= burstSize);
    bool got_row_hit = false;
    Tick dram_pkt_cmd_at = curTick();
    Tick pushed_dram_pkt_cmd_at = curTick();
    bool issuePre[dram_pkt->packHelper->pkt_cnt] = { false };

    for (int i=0; i<dram_pkt->packHelper->pkt_cnt; ++i) {
        DPRINTF(VMC, "Processing pkt %d/%d\n", i+1, dram_pkt->packHelper->pkt_cnt);
        DRAMPacket* cur_pkt = (*(dram_pkt->packHelper))[i];
        Device& device = cur_pkt->deviceRef;
        Bank& bank = cur_pkt->bankRef;
        DPRINTF(VMC, "Pkt (%d) accessing rank%d_device%d_bank%d\n",
                dram_pkt->id, device.rank, device.device, bank.bank);
    
        // are we in or transitioning to a low-power state and have not scheduled
        // a power-up event?
        // if so, wake up from power down to issue RD/WR burst
        if (device.inLowPowerState) {
            assert(device.pwrState != PWR_SREF);
            device.scheduleWakeUpEvent(tXP);
        }

        // for the state we need to track if it is a row hit or not
        bool row_hit = true;

        // respect any constraints on the command (e.g. tRCD or tCCD)
        Tick cmd_at = std::max(bank.colAllowedAt, curTick());

        // Determine the access latency and update the bank state
        if (bank.openRow == cur_pkt->row) {
            // nothing to do
        } else {
            row_hit = false;

            // If there is a page open, precharge it.
            if (bank.openRow != Bank::NO_ROW) {
                prechargeBank(device, bank, std::max(bank.preAllowedAt, curTick()));
            }

            // next we need to account for the delay in activating the
            // page
            Tick act_tick = std::max(bank.actAllowedAt, curTick());

            // Record the activation and deal with all the global timing
            // constraints caused be a new activation (tRRD and tXAW)
            activateBank(device, bank, act_tick, cur_pkt->row);

            // issue the command as early as possible
            cmd_at = bank.colAllowedAt;
        }

        // we need to wait until the bus is available before we can issue
        // the command
        cmd_at = std::max(cmd_at, busBusyUntil - tCL);

        if (cmd_at > dram_pkt_cmd_at)
            dram_pkt_cmd_at = cmd_at;

        // weil0ng: we also need to wait until the addr is ready.
        cmd_at = std::max(cmd_at, cur_pkt->addrReadyAt);

        if (cmd_at > pushed_dram_pkt_cmd_at)
            pushed_dram_pkt_cmd_at = cmd_at;
    
        // update the packet ready time
        cur_pkt->readyTime = cmd_at + tCL + tBURST;

        // only one burst can use the bus at any one point in time
        assert(cur_pkt->readyTime - busBusyUntil >= tBURST);

        // update the time for the next read/write burst for each
        // bank within the device (add a max with tCCD/tCCD_L here)
        Tick cmd_dly;
        for (int j = 0; j < ranksPerChannel; j++) {
            for (int i = 0; i < banksPerRank; i++) {
                // next burst to same bank group in this rank must not happen
                // before tCCD_L.  Different bank group timing requirement is
                // tBURST; Add tCS for different ranks
                if (cur_pkt->rank == j) {
                    if (bankGroupArch &&
                            (bank.bankgr == ranks[j]->banks[i]->bankgr)) {
                        // bank group architecture requires longer delays between
                        // RD/WR burst commands to the same bank group.
                        // Use tCCD_L in this case
                        cmd_dly = tCCD_L;
                    } else {
                        // use tBURST (equivalent to tCCD_S), the shorter
                        // cas-to-cas delay value, when either:
                        // 1) bank group architecture is not supportted
                        // 2) bank is in a different bank group
                        cmd_dly = tBURST;
                    }
                } else {
                    // different rank is by default in a different bank group
                    // use tBURST (equivalent to tCCD_S), which is the shorter
                    // cas-to-cas delay in this case
                    // Add tCS to account for rank-to-rank bus delay requirements
                    cmd_dly = tBURST + tCS;
                }
                // weil0ng: TODO: do we add a universal bubble here for push?
                // cmd_dlt += tBURST/2;
                // totPushDelay += tBURST/2;
                ranks[j]->devices[device.device]->banks[i]->colAllowedAt =
                    std::max(cmd_at + cmd_dly,
                            ranks[j]->devices[device.device]->banks[i]->colAllowedAt);
            }
        }

        // Save rank of current access
        activeRank = cur_pkt->rank;

        // If this is a write, we also need to respect the write recovery
        // time before a precharge, in the case of a read, respect the
        // read to precharge constraint
        bank.preAllowedAt = std::max(bank.preAllowedAt,
                cur_pkt->isRead ? cmd_at + tRTP :
                cur_pkt->readyTime + tWR);

        // increment the bytes accessed and the accesses per row
        bank.bytesAccessed += burstSize;
        ++bank.rowAccesses;

        // if we reached the max, then issue with an auto-precharge
        bool auto_precharge = pageMgmt == Enums::close ||
            bank.rowAccesses == maxAccessesPerRow;

        // if we did not hit the limit, we might still want to
        // auto-precharge
        if (!auto_precharge &&
                (pageMgmt == Enums::open_adaptive ||
                 pageMgmt == Enums::close_adaptive)) {
            // a twist on the open and close page policies:
            // 1) open_adaptive page policy does not blindly keep the
            // page open, but close it if there are no row hits, and there
            // are bank conflicts in the queue
            // 2) close_adaptive page policy does not blindly close the
            // page, but closes it only if there are no row hits in the queue.
            // In this case, only force an auto precharge when there
            // are no same page hits in the queue
            bool got_more_hits = false;
            bool got_bank_conflict = false;

            // either look at the read queue or write queue
            const deque<DRAMPacket*>& queue = cur_pkt->isRead ?
                readQueue : writeQueue;
            auto p = queue.begin();
            // make sure we are not considering the packet that we are
            // currently dealing with (which is the head of the queue)
            ++p;

            // keep on looking until we find a hit or reach the end of the queue
            // 1) if a hit is found, then both open and close adaptive policies keep
            // the page open
            // 2) if no hit is found, got_bank_conflict is set to true if a bank
            // conflict request is waiting in the queue
            while (!got_more_hits && p != queue.end()) {
                bool same_rank = (cur_pkt->rank == (*p)->rank);
                bool same_device = true;
                if ((*p)->isPack) {
                    same_device = false;
                    for (int i=0; i<(*p)->packHelper->pkt_cnt; ++i) {
                        DRAMPacket* fut_pkt = (*((*p)->packHelper))[i];
                        same_device |= (fut_pkt->device == cur_pkt->device);
                    }   
                }
                bool same_bank = (cur_pkt->bank == (*p)->bank);
                bool same_row = cur_pkt->row == (*p)->row;
                got_more_hits |= same_rank && same_device && same_bank && same_row;
                got_bank_conflict |= same_rank && same_device && same_bank && !same_row;
                ++p;
            }

            // auto pre-charge when either
            // 1) open_adaptive policy, we have not got any more hits, and
            //    have a bank conflict
            // 2) close_adaptive policy and we have not got any more hits
            auto_precharge = !got_more_hits &&
                (got_bank_conflict || pageMgmt == Enums::close_adaptive);
        }

        // DRAMPower trace command to be written
        std::string mem_cmd = dram_pkt->isRead ? "RD" : "WR";

        // MemCommand required for DRAMPower library
        MemCommand::cmds command = (mem_cmd == "RD") ?
            MemCommand::RD : MemCommand::WR;

        DPRINTF(VMC, "Pkt (%d) accesses %#08x, ready at %lld\n",
                cur_pkt->id, cur_pkt->addr, cur_pkt->readyTime);

        device.cmdList.push_back(Command(command, cur_pkt->bank,
                    cmd_at));

        DPRINTF(DRAMPower, "%llu,%s,%d,%d\n", divCeil(cmd_at, tCK) -
                timeStampOffset, mem_cmd, cur_pkt->bank, cur_pkt->rank);

        // Remember that we need to precharge this device_bank.
        // Issue precharge all together at the end.
        issuePre[i] = auto_precharge;

        got_row_hit |= row_hit;

        device.bytesAccessed[bank.bank] += burstSize / devicesPerRank;
        device.totalBytesAccessed += burstSize / devicesPerRank;
    }

    if (pushed_dram_pkt_cmd_at > dram_pkt_cmd_at)
        totPushDelay += pushed_dram_pkt_cmd_at - dram_pkt_cmd_at;

    // Clear the addrRegs.
    assert(dram_pkt_cmd_at >= curTick());
    DPRINTF(Push, "clearEvent from (%d) scheduled at %lld (%d)\n",
            dram_pkt->id, pushed_dram_pkt_cmd_at,
            (pushed_dram_pkt_cmd_at - curTick())/tCK);
    dram_pkt->prepareClear();
    dram_pkt->scheduleClearEvents(pushed_dram_pkt_cmd_at);

    // Update bus state
    for (int i=0; i<dram_pkt->packHelper->pkt_cnt; ++i) {
        DRAMPacket* cur_pkt = (*(dram_pkt->packHelper))[i];
        if (cur_pkt->readyTime > busBusyUntil)
            busBusyUntil = cur_pkt->readyTime;
    }
    dram_pkt->readyTime = busBusyUntil;
    DPRINTF(VMC, "Access (%d) done at %lld for pack pkt (%d), busBusyUntil %d cycles later.\n",
            dram_pkt->id, dram_pkt->readyTime, dram_pkt->id, (busBusyUntil - curTick())/tCK);

    for (int i=0; i<dram_pkt->packHelper->pkt_cnt; ++i) {
        DRAMPacket* cur_pkt = (*(dram_pkt->packHelper))[i];
        Device& device = cur_pkt->deviceRef;
        Bank& bank = cur_pkt->bankRef;
        // weil0ng: even if cur_pkt finishes early, next precharge
        // can only be issued when all pkts are done. This is because
        // we can only handle responseEvent one at a time, so all pkts
        // must be processed in one go, meaning outstadingEvents should
        // only decrease when all pkts are done, limiting precharge time.
        bank.preAllowedAt = std::max(bank.preAllowedAt,
                dram_pkt->readyTime + tWR);
        if (issuePre[i]) {
            prechargeBank(device, bank, std::max(curTick(), bank.preAllowedAt));
            DPRINTF(VMC, "Auto-precharged bank: %d_%d_%d\n",
                    cur_pkt->rank, cur_pkt->device, cur_pkt->bank);
        }
    }

    // Update the stats
    if (dram_pkt->isRead) {
        ++readsThisTime;
        if (got_row_hit)
            readRowHits++;
        if (isVMCMode)
            bytesReadDRAM += dram_pkt->size;
        else
            bytesReadDRAM += burstSize;
        perBankRdBursts[dram_pkt->bankId]++;

        // Update latency stats
        totMemAccLat += dram_pkt->readyTime - dram_pkt->entryTime;
        totBusLat += tBURST;
        totQLat += pushed_dram_pkt_cmd_at - dram_pkt->entryTime;
    } else {
        ++writesThisTime;
        if (got_row_hit)
            writeRowHits++;
        if (isVMCMode)
            bytesWritten += dram_pkt->size;
        else
            bytesWritten += burstSize;
        perBankWrBursts[dram_pkt->bankId]++;
    }

    return busBusyUntil;
}

void
DRAMCtrl::doDRAMAccess(DRAMPacket* dram_pkt)
{
    DPRINTF(DRAM, "Timing access to addr %#08x, rank/bank/row %d %d %d\n",
            dram_pkt->addr, dram_pkt->rank, dram_pkt->bank, dram_pkt->row);

    // Update bus state
    busBusyUntil = doBankAccess(dram_pkt, busBusyUntil);

    // Update the minimum timing between the requests, this is a
    // conservative estimate of when we have to schedule the next
    // request to not introduce any unecessary bubbles. In most cases
    // we will wake up sooner than we have to.
    nextReqTime = busBusyUntil - (tRP + tRCD + tCL);
}

void
DRAMCtrl::processNextReqEvent()
{
    DPRINTF(Progress, "nextReq\n");
    if (!isVMCMode) {
        int busyRanks = 0;
        for (auto r : ranks) {
            if (!r->isAvailable()) {
                if (r->pwrState != PWR_SREF) {
                    // rank is busy refreshing
                    DPRINTF(DRAMState, "Rank %d is not available\n", r->rank);
                    busyRanks++;

                    // let the rank know that if it was waiting to drain, it
                    // is now done and ready to proceed
                    r->checkDrainDone();
                }

                // check if we were in self-refresh and haven't started
                // to transition out
                if ((r->pwrState == PWR_SREF) && r->inLowPowerState) {
                    DPRINTF(DRAMState, "Rank %d is in self-refresh\n", r->rank);
                    // if we have commands queued to this rank and we don't have
                    // a minimum number of active commands enqueued,
                    // exit self-refresh
                    if (r->forceSelfRefreshExit()) {
                        DPRINTF(DRAMState, "rank %d was in self refresh and"
                                " should wake up\n", r->rank);
                        //wake up from self-refresh
                        r->scheduleWakeUpEvent(tXS);
                        // things are brought back into action once a refresh is
                        // performed after self-refresh
                        // continue with selection for other ranks
                    }
                }
            }
        }

        if (busyRanks == ranksPerChannel) {
            // if all ranks are refreshing wait for them to finish
            // and stall this state machine without taking any further
            // action, and do not schedule a new nextReqEvent
            return;
        }
    } else {
        int busyDevices = 0;
        for (auto r : ranks) {
            for (auto d : r->devices) {
                if (!d->isAvailable()) {
                    if (d->pwrState != PWR_SREF) {
                        DPRINTF(DeviceState, "Rank%d_device%d is not available\n",
                                d->rank, d->device);
                        ++busyDevices;
                        d->checkDrainDone();
                    }
                    if ((d->pwrState == PWR_SREF) && d->inLowPowerState) {
                        DPRINTF(DeviceState, "Rank%d_device%d is self-refreshing\n",
                                d->rank, d->device);
                        if (d->forceSelfRefreshExit()) {
                            d->scheduleWakeUpEvent(tXS);
                        }
                    }
                }
            }
        }
        if (busyDevices == ranksPerChannel * devicesPerRank) {
            DPRINTF(VMC, "All devices busy\n");
            return;
        }
    }
    // pre-emptively set to false.  Overwrite if in transitioning to
    // a new state
    bool switched_cmd_type = false;
    if (busState != busStateNext) {
        if (isVMCMode)
            DPRINTF(VMC, "Bus Turnaround from %s to %s\n",
                    busState == WRITE?"WRITE":"READ",
                    busStateNext == WRITE?"WRITE":"READ");
        if (busState == READ) {
            DPRINTF(DRAM, "Switching to writes after %d reads with %d reads "
                    "waiting\n", readsThisTime, readQueue.size());

            // sample and reset the read-related stats as we are now
            // transitioning to writes, and all reads are done
            rdPerTurnAround.sample(readsThisTime);
            readsThisTime = 0;

            // now proceed to do the actual writes
            switched_cmd_type = true;

            ++busRd2Wr;
        } else {
            DPRINTF(DRAM, "Switching to reads after %d writes with %d writes "
                    "waiting\n", writesThisTime, writeQueue.size());

            wrPerTurnAround.sample(writesThisTime);
            writesThisTime = 0;

            switched_cmd_type = true;

            ++busWr2Rd;
        }
        // update busState to match next state until next transition
        busState = busStateNext;
    }

    // weil0ng: sanity check bus state
    if (packRdDrain)
        assert(busState == READ);
    if (packWrDrain)
        assert(busState == WRITE);

    // when we get here it is either a read or a write
    if (busState == READ) {

        // track if we should switch or not
        bool switch_to_writes = false;

        if (readQueue.empty()) {
            // In the case there is no read request to go next,
            // trigger writes if we have passed the low threshold (or
            // if we are draining)
            if (!writeQueue.empty() &&
                (drainState() == DrainState::Draining ||
                 writeQueue.size() > writeLowThreshold)) {
                switch_to_writes = true;
            } else {
                // check if we are drained
                // not done draining until in PWR_IDLE state
                // ensuring all banks are closed and
                // have exited low power states
                if (drainState() == DrainState::Draining &&
                    respQueue.empty() && allRanksDrained()) {

                    DPRINTF(Drain, "DRAM controller done draining\n");
                    signalDrainDone();
                }

                // nothing to do, not even any point in scheduling an
                // event for the next request
                return;
            }
        } else {
            // bool to check if there is a read to a free rank
            bool found_read = false;

            // Figure out which read request goes next, and move it to the
            // front of the read queue
            // If we are changing command type, incorporate the minimum
            // bus turnaround delay which will be tCS (different rank) case
            if (isVMCMode)
                DPRINTF(VMC, "ChooseNext from read queue\n");
            found_read = chooseNext(readQueue,
                             switched_cmd_type ? tCS : 0);

            // if no read to an available rank is found then return
            // at this point. There could be writes to the available ranks
            // which are above the required threshold. However, to
            // avoid adding more complexity to the code, return and wait
            // for a refresh event to kick things into action again.
            if (!found_read)
                return;

            DRAMPacket* dram_pkt = readQueue.front();
            
            // Sanity checks.
            if (!dram_pkt->isPack)
                assert(dram_pkt->rankRef.isAvailable());
            else
                assert(dram_pkt->packReady());

            // here we get a bit creative and shift the bus busy time not
            // just the tWTR, but also a CAS latency to capture the fact
            // that we are allowed to prepare a new bank, but not issue a
            // read command until after tWTR, in essence we capture a
            // bubble on the data bus that is tWTR + tCL
            if (switched_cmd_type && dram_pkt->rank == activeRank) {
                busBusyUntil += tWTR + tCL;
            }

            doDRAMAccess(dram_pkt);

            // At this point we're done dealing with the request
            readQueue.pop_front();

            // Every respQueue which will generate an event, increment count
            if (!dram_pkt->isPack) {
                ++dram_pkt->rankRef.outstandingEvents;
            } else {
                for(int i=0; i<dram_pkt->packHelper->pkt_cnt; ++i) {
                    DRAMPacket* cur_pkt = (*(dram_pkt->packHelper))[i];
                    ++cur_pkt->deviceRef.outstandingEvents;
                    //DPRINTF(VMC, "Device %d_%d waits for read response, outstandingEvents: %d\n",
                    //       cur_pkt->deviceRef.rank, cur_pkt->deviceRef.device,
                    //       cur_pkt->deviceRef.outstandingEvents);
                }
            }

            // sanity check
            assert(dram_pkt->size <= burstSize);
            assert(dram_pkt->readyTime >= curTick());

            // Insert into response queue. It will be sent back to the
            // requestor at its readyTime
            if (respQueue.empty()) {
                assert(!respondEvent.scheduled());
                schedule(respondEvent, dram_pkt->readyTime);
            } else {
                assert(respQueue.back()->readyTime <= dram_pkt->readyTime);
                assert(respondEvent.scheduled());
            }

            respQueue.push_back(dram_pkt);

            bool switchCondSatisfied = writeQueue.size() > writeHighThreshold;
            // weil0ng: before switch to WRITE, check if some pack read has been pushed.
            /**
            if (isVMCMode) {
                int pushed_reads = 0;
                for (auto r : ranks) {
                    for (auto d : r->devices) {
                        for (auto b: d->banks) {
                            if (!((b->addrRegs.empty() && !b->pushEvent.scheduled()) ||
                                        (b->addrRegs.size() == 1 &&
                                        b->clearEvent.scheduled()))) {
                                DPRINTF(Push, "Pushed addr found on rank%d_device%d_bank%d\n",
                                        r->rank, d->device, b->bank);
                                ++pushed_reads;
                            }
                        }
                    }
                }
                if (pushed_reads > 0) {
                    int pendingReqs = 0;
                    DPRINTF(Push, "Still has %d pushed reads\n", pushed_reads);
                    for (auto i=readQueue.begin(); i!=readQueue.end(); ++i) {
                        DRAMPacket* cur_pkt = *i;
                        if (cur_pkt->isPack) {
                            pendingReqs += cur_pkt->packHelper->pkt_cnt;
                            for(int i=0; i<cur_pkt->packHelper->pkt_cnt; ++i) {
                                DRAMPacket* p_pkt = (*(cur_pkt->packHelper))[i];
                                DPRINTF(Push, "Pending read to rank%d_device%d\n",
                                        p_pkt->rank, p_pkt->device);
                            }
                        }
                    }
                    DPRINTF(Push, "Still has %d pending micro reads\n", pendingReqs);
                    assert(pushed_reads <= pendingReqs);
                    if (!packRdDrain && switchCondSatisfied) {
                        DPRINTF(Push, "Start draining packRead pkts\n");
                        packRdDrain = true;
                        packRdDrainStartTick = curTick();
                    }
                } else {
                    if (packRdDrain) {
                        packRdDrain = false;
                        totPackRdDrainTime += (curTick() - packRdDrainStartTick);
                    }
                }
            } else {
                assert(!packRdDrain);
            }
            

            // we have so many writes that we have to transition
            if (!packRdDrain && switchCondSatisfied) {
            **/
            if (switchCondSatisfied) {
                switch_to_writes = true;
            }
        }

        // switching to writes, either because the read queue is empty
        // and the writes have passed the low threshold (or we are
        // draining), or because the writes hit the hight threshold
        if (switch_to_writes) {
            // transition to writing
            busStateNext = WRITE;
        }
    } else {
        // bool to check if write to free rank is found
        bool found_write = false;

        // If we are changing command type, incorporate the minimum
        // bus turnaround delay
        if (isVMCMode)
            DPRINTF(VMC, "ChooseNext from write queue\n");

        found_write = chooseNext(writeQueue,
                                 switched_cmd_type ? std::min(tRTW, tCS) : 0);

        // if no writes to an available rank are found then return.
        // There could be reads to the available ranks. However, to avoid
        // adding more complexity to the code, return at this point and wait
        // for a refresh event to kick things into action again.
        if (!found_write)
            return;

        DRAMPacket* dram_pkt = writeQueue.front();

        // sanity check
        if (!dram_pkt->isPack)
            assert(dram_pkt->rankRef.isAvailable());
        else
            assert(dram_pkt->packReady());

        assert(dram_pkt->size <= burstSize);

        // add a bubble to the data bus, as defined by the
        // tRTW when access is to the same rank as previous burst
        // Different rank timing is handled with tCS, which is
        // applied to colAllowedAt
        if (switched_cmd_type && dram_pkt->rank == activeRank) {
            busBusyUntil += tRTW;
        }

        doDRAMAccess(dram_pkt);

        writeQueue.pop_front();

        // removed write from queue, decrement count
        if (!dram_pkt->isPack)
            --dram_pkt->rankRef.writeEntries;
        else {
            for(int i=0; i<dram_pkt->packHelper->pkt_cnt; ++i) {
                DRAMPacket* cur_pkt = (*(dram_pkt->packHelper))[i];
                --cur_pkt->deviceRef.writeEntries;
            }
        }

        // Schedule write done event to decrement event count
        // after the readyTime has been reached
        // Only schedule latest write event to minimize events
        // required; only need to ensure that final event scheduled covers
        // the time that writes are outstanding and bus is active
        // to holdoff power-down entry events
        if (!dram_pkt->isPack) {
            if (!dram_pkt->rankRef.writeDoneEvent.scheduled()) {
                schedule(dram_pkt->rankRef.writeDoneEvent, dram_pkt->readyTime);
                // New event, increment count
                ++dram_pkt->rankRef.outstandingEvents;
            } else if (dram_pkt->rankRef.writeDoneEvent.when() <
                    dram_pkt-> readyTime) {
                reschedule(dram_pkt->rankRef.writeDoneEvent, dram_pkt->readyTime);
            }
        } else {
            for(int i=0; i<dram_pkt->packHelper->pkt_cnt; ++i) {
                DRAMPacket* cur_pkt = (*(dram_pkt->packHelper))[i];
                if (!cur_pkt->deviceRef.writeDoneEvent.scheduled()) {
                    // weil0ng: TODO, we keep all short pkts sync atm.
                    schedule(cur_pkt->deviceRef.writeDoneEvent, dram_pkt->readyTime);
                    ++cur_pkt->deviceRef.outstandingEvents;
                    //DPRINTF(VMC, "Device %d_%d writeDone scheduled, outstandingEvents: %d\n",
                    //        cur_pkt->deviceRef.rank, cur_pkt->deviceRef.device,
                    //        cur_pkt->deviceRef.outstandingEvents);
                } else if (cur_pkt->deviceRef.writeDoneEvent.when() <
                        dram_pkt->readyTime) {
                    reschedule(cur_pkt->deviceRef.writeDoneEvent, dram_pkt->readyTime);
                }
            }
        }

        if (dram_pkt->isPack) {
            for(int i=0; i<dram_pkt->packHelper->pkt_cnt; ++i) {
                DRAMPacket* cur_pkt = (*(dram_pkt->packHelper))[i];
                isInWriteQueue.erase(devBurstAlign(cur_pkt->addr));
            }
        } else {
            isInWriteQueue.erase(burstAlign(dram_pkt->addr));
        }


        bool switchCondSatisfied = (writeQueue.empty() ||
                (writeQueue.size() + minWritesPerSwitch < writeLowThreshold &&
                 drainState() != DrainState::Draining) ||
                (!readQueue.empty() && writesThisTime >= minWritesPerSwitch)); 
        // weil0ng: before switch to READ, check if some pack write has been pushed.
        /**
        if (isVMCMode) {
            int pushed_writes = 0;
            for (auto r : ranks) {
                for (auto d : r->devices) {
                    for (auto b : d->banks) {
                        if (!((b->addrRegs.empty() && !b->pushEvent.scheduled()) ||
                                    (b->addrRegs.size() == 1 &&
                                     b->clearEvent.scheduled()))) {
                            DPRINTF(Push, "Pushed addr found on rank%d_device%d_bank%d\n",
                                    r->rank, d->device, b->bank);
                            ++pushed_writes;
                        }
                    }
                }
            }
            // weil0ng: if there's pushed pkts, check how many short pkts are still pending.
            if (pushed_writes > 0) {
                int pendingReqs = 0;        
                DPRINTF(Push, "Still has %d pushed writes\n", pushed_writes);
                for (auto i=writeQueue.begin(); i!=writeQueue.end(); ++i) {
                    DRAMPacket* cur_pkt = *i;
                    if (cur_pkt->isPack)
                        pendingReqs += cur_pkt->packHelper->pkt_cnt;
                }
                DPRINTF(Push, "Still has %d pending micro writes\n", pendingReqs);
                assert(pushed_writes <= pendingReqs);
                if (!packWrDrain && switchCondSatisfied) {
                    DPRINTF(Push, "Start draining packWrite pkts\n");
                    packWrDrain = true;
                    packWrDrainStartTick = curTick();
                }
            } else {
                if (packWrDrain) {
                    packWrDrain = false;
                    totPackWrDrainTime += (curTick() - packWrDrain);
                }
            }
        } else {
            assert(!packWrDrain);
        }

        // If we emptied the write queue, or got sufficiently below the
        // threshold (using the minWritesPerSwitch as the hysteresis) and
        // are not draining, or we have reads waiting and have done enough
        // writes, then switch to reads.
        if (!packWrDrain && switchCondSatisfied) {
        **/
        if (switchCondSatisfied) {
            // turn the bus back around for reads again
            busStateNext = READ;

            // note that the we switch back to reads also in the idle
            // case, which eventually will check for any draining and
            // also pause any further scheduling if there is really
            // nothing to do
        }
        assert(dram_pkt != NULL);
        // weilong: ninja bug?
        // Shouldn't destroy actual pkts here, clearEvent is scheduled earlier, destroy pkts there!
        if (dram_pkt->isPack) {
            delete dram_pkt->pkt->req;
            delete dram_pkt->pkt;
            delete dram_pkt->packHelper;
        }
        if (dram_pkt->burstHelper)
            delete dram_pkt->burstHelper;
        delete dram_pkt;
    }

    // It is possible that a refresh to another rank kicks things back into
    // action before reaching this point.
    if (!nextReqEvent.scheduled()) {
        if (isVMCMode)
            DPRINTF(VMC, "Next request scheduled at %lld (%ld)\n",
                    std::max(nextReqTime, curTick()),
                    std::max(nextReqTime, curTick()) - curTick());
        schedule(nextReqEvent, std::max(nextReqTime, curTick()));
    }

    // If there is space available and we have writes waiting then let
    // them retry. This is done here to ensure that the retry does not
    // cause a nextReqEvent to be scheduled before we do so as part of
    // the next request processing
    if (retryWrReq && writeQueue.size() < writeBufferSize) {
        retryWrReq = false;
        port.sendRetryReq();
    }
}

pair<uint64_t, bool>
DRAMCtrl::minBankPrep(const deque<DRAMPacket*>& queue,
                      Tick min_col_at) const
{
    uint64_t bank_mask = 0;
    Tick min_act_at = MaxTick;

    // latest Tick for which ACT can occur without incurring additoinal
    // delay on the data bus
    const Tick hidden_act_max = std::max(min_col_at - tRCD, curTick());

    // Flag condition when burst can issue back-to-back with previous burst
    bool found_seamless_bank = false;

    // Flag condition when bank can be opened without incurring additional
    // delay on the data bus
    bool hidden_bank_prep = false;

    // determine if we have queued transactions targetting the
    // bank in question
    vector<bool> got_waiting(ranksPerChannel * banksPerRank, false);
    for (const auto& p : queue) {
        if (p->rankRef.isAvailable())
            got_waiting[p->bankId] = true;
    }

    // Find command with optimal bank timing
    // Will prioritize commands that can issue seamlessly.
    for (int i = 0; i < ranksPerChannel; i++) {
        for (int j = 0; j < banksPerRank; j++) {
            uint16_t bank_id = i * banksPerRank + j;

            // if we have waiting requests for the bank, and it is
            // amongst the first available, update the mask
            if (got_waiting[bank_id]) {
                // make sure this rank is not currently refreshing.
                assert(ranks[i]->isAvailable());
                // simplistic approximation of when the bank can issue
                // an activate, ignoring any rank-to-rank switching
                // cost in this calculation
                Tick act_at = ranks[i]->banks[j]->openRow == Bank::NO_ROW ?
                    std::max(ranks[i]->banks[j]->actAllowedAt, curTick()) :
                    std::max(ranks[i]->banks[j]->preAllowedAt, curTick()) + tRP;

                // When is the earliest the R/W burst can issue?
                Tick col_at = std::max(ranks[i]->banks[j]->colAllowedAt,
                                       act_at + tRCD);

                // bank can issue burst back-to-back (seamlessly) with
                // previous burst
                bool new_seamless_bank = col_at <= min_col_at;

                // if we found a new seamless bank or we have no
                // seamless banks, and got a bank with an earlier
                // activate time, it should be added to the bit mask
                if (new_seamless_bank ||
                    (!found_seamless_bank && act_at <= min_act_at)) {
                    // if we did not have a seamless bank before, and
                    // we do now, reset the bank mask, also reset it
                    // if we have not yet found a seamless bank and
                    // the activate time is smaller than what we have
                    // seen so far
                    if (!found_seamless_bank &&
                        (new_seamless_bank || act_at < min_act_at)) {
                        bank_mask = 0;
                    }

                    found_seamless_bank |= new_seamless_bank;

                    // ACT can occur 'behind the scenes'
                    hidden_bank_prep = act_at <= hidden_act_max;

                    // set the bit corresponding to the available bank
                    replaceBits(bank_mask, bank_id, bank_id, 1);
                    min_act_at = act_at;
                }
            }
        }
    }

    return make_pair(bank_mask, hidden_bank_prep);
}

DRAMCtrl::Bank::Bank(DRAMCtrl& _memory, uint8_t _rank = 0, uint8_t _device = 0)
    : EventManager(&_memory), memory(_memory), openRow(NO_ROW),
      rank(_rank), device(_device), bank(0), bankgr(0), colAllowedAt(0),
      preAllowedAt(0), actAllowedAt(0), rowAccesses(0), bytesAccessed(0),
      pendingRdPushPkt(NULL), pendingRdClearPkt(NULL), pendingWrPushPkt(NULL),
      pendingWrClearPkt(NULL), prevPushTick(0), rdPushEvent(*this),
      wrPushEvent(*this), rdClearEvent(*this), wrClearEvent(*this)
{
    rdAddrRegs.clear();
    wrAddrRegs.clear();
}

DRAMCtrl::Device::Device(DRAMCtrl& _memory, Rank& _rank,
        const DRAMCtrlParams* _p, const uint8_t _device)
    : EventManager(&_memory), memory(_memory), rankRef(_rank),
      pwrStateTrans(PWR_IDLE), pwrStatePostRefresh(PWR_IDLE),
      pwrStateTick(0), refreshDueAt(0), pwrState(PWR_IDLE),
      refreshState(REF_IDLE), inLowPowerState(false), rank(_rank.rank), device(_device),
      readEntries(0), writeEntries(0), outstandingEvents(0),
      wakeUpAllowedAt(0), pushAllowedAt(0), power(_p, false), numBanksActive(0),
      writeDoneEvent(*this), activateEvent(*this), prechargeEvent(*this),
      refreshEvent(*this), powerEvent(*this), wakeUpEvent(*this)
{
    actTicks.resize(memory.activationLimit, 0);
    for (int b=0; b<memory.banksPerRank; ++b) {
        banks.push_back(new Bank(_memory, rank, device));
        banks[b]->bank = b;
        if (memory.bankGroupArch) {
            banks[b]->bankgr = b % memory.bankGroupsPerRank;
        } else {
            banks[b]->bankgr = b;
        }
    }
}

DRAMCtrl::Rank::Rank(DRAMCtrl& _memory, const DRAMCtrlParams* _p, const uint8_t _rank)
    : EventManager(&_memory), memory(_memory),
      pwrStateTrans(PWR_IDLE), pwrStatePostRefresh(PWR_IDLE),
      pwrStateTick(0), refreshDueAt(0), pwrState(PWR_IDLE),
      refreshState(REF_IDLE), inLowPowerState(false), rank(_rank),
      readEntries(0), writeEntries(0), outstandingEvents(0),
      wakeUpAllowedAt(0), power(_p, false), numBanksActive(0),
      writeDoneEvent(*this), activateEvent(*this), prechargeEvent(*this),
      refreshEvent(*this), powerEvent(*this), wakeUpEvent(*this)
{
    for (int i=0; i<memory.devicesPerRank; ++i) {
        Device* device = new Device(_memory, *this, _p, i);
        devices.push_back(device);
    }
}

void
DRAMCtrl::Rank::startup(Tick ref_tick)
{
    assert(ref_tick > curTick());

    pwrStateTick = curTick();

    // kick off the refresh, and give ourselves enough time to
    // precharge
    assert(!memory.isVMCMode);
    schedule(refreshEvent, ref_tick);
}

void
DRAMCtrl::Rank::suspend()
{
    assert(!memory.isVMCMode);
    deschedule(refreshEvent);

    // Update the stats
    updatePowerStats();

    // don't automatically transition back to LP state after next REF
    pwrStatePostRefresh = PWR_IDLE;
}

bool
DRAMCtrl::Rank::lowPowerEntryReady() const
{
    assert(!memory.isVMCMode);
    bool no_queued_cmds = ((memory.busStateNext == READ) && (readEntries == 0))
                          || ((memory.busStateNext == WRITE) &&
                              (writeEntries == 0));

    if (refreshState == REF_RUN) {
       // have not decremented outstandingEvents for refresh command
       // still check if there are no commands queued to force PD
       // entry after refresh completes
       return no_queued_cmds;
    } else {
       // ensure no commands in Q and no commands scheduled
       return (no_queued_cmds && (outstandingEvents == 0));
    }
}

bool
DRAMCtrl::Device::lowPowerEntryReady() const
{
    bool no_queued_cmds = ((memory.busStateNext == READ) && (readEntries == 0)) ||
        ((memory.busStateNext == WRITE) && (writeEntries == 0));
    if (refreshState == REF_RUN) {
        return no_queued_cmds;
    } else {
        return (no_queued_cmds && (outstandingEvents == 0));
    }
}

void
DRAMCtrl::Rank::checkDrainDone()
{
    assert(!memory.isVMCMode);
    // if this rank was waiting to drain it is now able to proceed to
    // precharge
    if (refreshState == REF_DRAIN) {
        DPRINTF(DRAM, "Refresh drain done, now precharging\n");

        refreshState = REF_PD_EXIT;

        // hand control back to the refresh event loop
        schedule(refreshEvent, curTick());
    }
}

void
DRAMCtrl::Device::checkDrainDone()
{
    assert(memory.isVMCMode);
    if (refreshState == REF_DRAIN) {
        refreshState = REF_PD_EXIT;
        schedule(refreshEvent, curTick());
    }
}

void
DRAMCtrl::Rank::flushCmdList()
{
    // at the moment sort the list of commands and update the counters
    // for DRAMPower libray when doing a refresh
    sort(cmdList.begin(), cmdList.end(), DRAMCtrl::sortTime);

    auto next_iter = cmdList.begin();
    // push to commands to DRAMPower
    for ( ; next_iter != cmdList.end() ; ++next_iter) {
         Command cmd = *next_iter;
         if (cmd.timeStamp <= curTick()) {
             // Move all commands at or before curTick to DRAMPower
             power.powerlib.doCommand(cmd.type, cmd.bank,
                                      divCeil(cmd.timeStamp, memory.tCK) -
                                      memory.timeStampOffset);
         } else {
             // done - found all commands at or before curTick()
             // next_iter references the 1st command after curTick
             break;
         }
    }
    // reset cmdList to only contain commands after curTick
    // if there are no commands after curTick, updated cmdList will be empty
    // in this case, next_iter is cmdList.end()
    cmdList.assign(next_iter, cmdList.end());
}

void
DRAMCtrl::Device::flushCmdList() {
    sort(cmdList.begin(), cmdList.end(), DRAMCtrl::sortTime);

    auto next_iter = cmdList.begin();
    for (; next_iter != cmdList.end(); ++next_iter) {
        Command cmd = *next_iter;
        if (cmd.timeStamp <= curTick()) {
            power.powerlib.doCommand(cmd.type, cmd.bank,
                    divCeil(cmd.timeStamp, memory.tCK) -
                    memory.timeStampOffset);
        } else {
            break;
        }
    }
    cmdList.assign(next_iter, cmdList.end());
}

void
DRAMCtrl::Rank::processActivateEvent()
{
    assert(!memory.isVMCMode);
    // we should transition to the active state as soon as any bank is active
    if (pwrState != PWR_ACT)
        // note that at this point numBanksActive could be back at
        // zero again due to a precharge scheduled in the future
        schedulePowerEvent(PWR_ACT, curTick());
}

void
DRAMCtrl::Device::processActivateEvent()
{
    if (pwrState != PWR_ACT)
        schedulePowerEvent(PWR_ACT, curTick());
}

void
DRAMCtrl::Rank::processPrechargeEvent()
{
    assert(!memory.isVMCMode);
    // counter should at least indicate one outstanding request
    // for this precharge
    assert(outstandingEvents > 0);
    // precharge complete, decrement count
    --outstandingEvents;
    std::ostringstream oss;
    oss << "precharge done";
    memory.dPrintEventCount(rank, false, oss.str());

    // if we reached zero, then special conditions apply as we track
    // if all banks are precharged for the power models
    if (numBanksActive == 0) {
        // no reads to this rank in the Q and no pending
        // RD/WR or refresh commands
        if (lowPowerEntryReady()) {
            // should still be in ACT state since bank still open
            assert(pwrState == PWR_ACT);

            // All banks closed - switch to precharge power down state.
            DPRINTF(DRAMState, "Rank %d sleep at tick %d\n",
                    rank, curTick());
            powerDownSleep(PWR_PRE_PDN, curTick());
        } else {
            // we should transition to the idle state when the last bank
            // is precharged
            schedulePowerEvent(PWR_IDLE, curTick());
        }
    }
}

void
DRAMCtrl::Device::processPrechargeEvent()
{
    assert(outstandingEvents > 0);
    --outstandingEvents;
    DPRINTF(DeviceState, "Device %d_%d hits prechargeEvent, outstandingEvents: %d\n",
            rank, device, outstandingEvents);
    if (numBanksActive == 0) {
        if (lowPowerEntryReady()) {
            assert(pwrState == PWR_ACT);
            DPRINTF(DeviceState, "Rank %d device %d goes to sleep\n", rank, device);
            powerDownSleep(PWR_PRE_PDN, curTick());
        } else {
            schedulePowerEvent(PWR_IDLE, curTick());
        }
    }
}

void
DRAMCtrl::Rank::processWriteDoneEvent()
{
    assert(!memory.isVMCMode);
    // counter should at least indicate one outstanding request
    // for this write
    assert(outstandingEvents > 0);
    // Write transfer on bus has completed
    // decrement per rank counter
    --outstandingEvents;
}

void
DRAMCtrl::Device::processWriteDoneEvent()
{
    assert(outstandingEvents > 0);
    --outstandingEvents;
}

void
DRAMCtrl::Rank::processRefreshEvent()
{
    if (memory.isVMCMode) {
        // if we are in VMC mode, this refresh has been scheduled
        // before entering VMC mode.
        // Transition on clean state.
        assert(refreshState == REF_IDLE);
        for (auto d : devices) {
            assert(!d->refreshEvent.scheduled());
            schedule(d->refreshEvent, curTick());
        }
        return;
    }
    // when first preparing the refresh, remember when it was due
    if ((refreshState == REF_IDLE) || (refreshState == REF_SREF_EXIT)) {
        // remember when the refresh is due
        refreshDueAt = curTick();

        // proceed to drain
        refreshState = REF_DRAIN;

        // make nonzero while refresh is pending to ensure
        // power down and self-refresh are not entered
        ++outstandingEvents;

        DPRINTF(DRAM, "Refresh due\n");
    }

    // let any scheduled read or write to the same rank go ahead,
    // after which it will
    // hand control back to this event loop
    if (refreshState == REF_DRAIN) {
        // if a request is at the moment being handled and this request is
        // accessing the current rank then wait for it to finish
        if ((rank == memory.activeRank)
            && (memory.nextReqEvent.scheduled())) {
            // hand control over to the request loop until it is
            // evaluated next
            DPRINTF(DRAM, "Refresh awaiting draining\n");

            return;
        } else {
            refreshState = REF_PD_EXIT;
        }
    }

    // at this point, ensure that rank is not in a power-down state
    if (refreshState == REF_PD_EXIT) {
        // if rank was sleeping and we have't started exit process,
        // wake-up for refresh
        if (inLowPowerState) {
            DPRINTF(DRAM, "Wake Up for refresh\n");
            // save state and return after refresh completes
            scheduleWakeUpEvent(memory.tXP);
            return;
        } else {
            refreshState = REF_PRE;
        }
    }

    // at this point, ensure that all banks are precharged
    if (refreshState == REF_PRE) {
        // precharge any active bank
        if (numBanksActive != 0) {
            // at the moment, we use a precharge all even if there is
            // only a single bank open
            DPRINTF(DRAM, "Precharging all\n");

            // first determine when we can precharge
            Tick pre_at = curTick();

            for (auto b : banks) {
                // respect both causality and any existing bank
                // constraints, some banks could already have a
                // (auto) precharge scheduled
                pre_at = std::max(b->preAllowedAt, pre_at);
            }

            // make sure all banks per rank are precharged, and for those that
            // already are, update their availability
            Tick act_allowed_at = pre_at + memory.tRP;

            for (auto b : banks) {
                if (b->openRow != Bank::NO_ROW) {
                    memory.prechargeBank(*this, *b, pre_at, false);
                } else {
                    b->actAllowedAt = std::max(b->actAllowedAt, act_allowed_at);
                    b->preAllowedAt = std::max(b->preAllowedAt, pre_at);
                }
            }

            // precharge all banks in rank
            cmdList.push_back(Command(MemCommand::PREA, 0, pre_at));

            DPRINTF(DRAMPower, "%llu,PREA,0,%d\n",
                    divCeil(pre_at, memory.tCK) -
                            memory.timeStampOffset, rank);
        } else if ((pwrState == PWR_IDLE) && (outstandingEvents == 1))  {
            // Banks are closed, have transitioned to IDLE state, and
            // no outstanding ACT,RD/WR,Auto-PRE sequence scheduled
            DPRINTF(DRAM, "All banks already precharged, starting refresh\n");

            // go ahead and kick the power state machine into gear since
            // we are already idle
            schedulePowerEvent(PWR_REF, curTick());
        } else {
            // banks state is closed but haven't transitioned pwrState to IDLE
            // or have outstanding ACT,RD/WR,Auto-PRE sequence scheduled
            // should have outstanding precharge event in this case
            assert(prechargeEvent.scheduled());
            // will start refresh when pwrState transitions to IDLE
        }

        assert(numBanksActive == 0);

        // wait for all banks to be precharged, at which point the
        // power state machine will transition to the idle state, and
        // automatically move to a refresh, at that point it will also
        // call this method to get the refresh event loop going again
        return;
    }

    // last but not least we perform the actual refresh
    if (refreshState == REF_START) {
        // should never get here with any banks active
        assert(numBanksActive == 0);
        assert(pwrState == PWR_REF);

        Tick ref_done_at = curTick() + memory.tRFC;

        for (auto b : banks) {
            b->actAllowedAt = ref_done_at;
        }

        // at the moment this affects all ranks
        cmdList.push_back(Command(MemCommand::REF, 0, curTick()));

        // Update the stats
        updatePowerStats();

        DPRINTF(DRAMPower, "%llu,REF,0,%d\n", divCeil(curTick(), memory.tCK) -
                memory.timeStampOffset, rank);

        // Update for next refresh
        refreshDueAt += memory.tREFI;

        // make sure we did not wait so long that we cannot make up
        // for it
        if (refreshDueAt < ref_done_at) {
            fatal("Refresh was delayed so long we cannot catch up\n");
        }

        // Run the refresh and schedule event to transition power states
        // when refresh completes
        refreshState = REF_RUN;
        schedule(refreshEvent, ref_done_at);
        return;
    }

    if (refreshState == REF_RUN) {
        // should never get here with any banks active
        assert(numBanksActive == 0);
        assert(pwrState == PWR_REF);

        assert(!powerEvent.scheduled());

        if ((memory.drainState() == DrainState::Draining) ||
            (memory.drainState() == DrainState::Drained)) {
            // if draining, do not re-enter low-power mode.
            // simply go to IDLE and wait
            schedulePowerEvent(PWR_IDLE, curTick());
        } else {
            // At the moment, we sleep when the refresh ends and wait to be
            // woken up again if previously in a low-power state.
            if (pwrStatePostRefresh != PWR_IDLE) {
                // power State should be power Refresh
                assert(pwrState == PWR_REF);
                DPRINTF(DRAMState, "Rank %d sleeping after refresh and was in "
                        "power state %d before refreshing\n", rank,
                        pwrStatePostRefresh);
                powerDownSleep(pwrState, curTick());

            // Force PRE power-down if there are no outstanding commands
            // in Q after refresh.
            } else if (lowPowerEntryReady()) {
                DPRINTF(DRAMState, "Rank %d sleeping after refresh but was NOT"
                        " in a low power state before refreshing\n", rank);
                powerDownSleep(PWR_PRE_PDN, curTick());

            } else {
                // move to the idle power state once the refresh is done, this
                // will also move the refresh state machine to the refresh
                // idle state
                schedulePowerEvent(PWR_IDLE, curTick());
            }
        }

        // if transitioning to self refresh do not schedule a new refresh;
        // when waking from self refresh, a refresh is scheduled again.
        if (pwrStateTrans != PWR_SREF) {
            // compensate for the delay in actually performing the refresh
            // when scheduling the next one
            schedule(refreshEvent, refreshDueAt - memory.tRP);

            DPRINTF(DRAMState, "Refresh done at %llu and next refresh"
                    " at %llu\n", curTick(), refreshDueAt);
        }
    }
}

void
DRAMCtrl::Device::processRefreshEvent()
{
    if (!memory.isVMCMode) {
        // this has been scheduled before exiting VMC mode.
        assert(refreshState == REF_IDLE);
        if (!this->rankRef.refreshEvent.scheduled())
            schedule(this->rankRef.refreshEvent, curTick());
        return;
    }
    DPRINTF(DeviceState, "Device %d_%d hits refreshEvent at ref_state %d pwr_state %d\n",
            this->rank, this->device, this->refreshState, this->pwrState);
    if ((refreshState == REF_IDLE) || (refreshState == REF_SREF_EXIT)) {
        refreshDueAt = curTick();
        refreshState = REF_DRAIN;
        ++outstandingEvents;
    }
    
    if (refreshState == REF_DRAIN) {
        //if ((rank == memory.activeRank) &&
        //        (memory.nextReqEvent.scheduled())) {
        // weil0ng: bug fix? activeRank may have already switched before
        // the actual access is done, so instead, we check if this device
        // has other pending events than refresh.
        if (outstandingEvents > 1) {
            return;
        } else {
            refreshState = REF_PD_EXIT;
        }
    }

    if (refreshState == REF_PD_EXIT) {
        if (inLowPowerState) {
            scheduleWakeUpEvent(memory.tXP);
            return;
        } else {
            refreshState = REF_PRE;
        }
    }

    if (refreshState == REF_PRE) {
        if (numBanksActive != 0) {
            Tick pre_at = curTick();
            for (auto b : banks) {
                pre_at = std::max(b->preAllowedAt, pre_at);
            }
            Tick act_allowed_at = pre_at + memory.tRP;
            for (auto b : banks) {
                if (b->openRow != Bank::NO_ROW) {
                    memory.prechargeBank(*this, *b, pre_at, false);
                } else {
                    b->actAllowedAt = std::max(b->actAllowedAt, act_allowed_at);
                    b->preAllowedAt = std::max(b->preAllowedAt, pre_at);
                }
            }
            cmdList.push_back(Command(MemCommand::PREA, 0, pre_at));
        } else if ((pwrState == PWR_IDLE) && (outstandingEvents == 1)) {
            schedulePowerEvent(PWR_REF, curTick());
        } else {
            assert(prechargeEvent.scheduled());
        }
        assert(numBanksActive == 0);
        return;
    }

    if (refreshState == REF_START) {
        assert(numBanksActive == 0);
        assert(pwrState == PWR_REF);

        Tick ref_done_at = curTick() + memory.tRFC;
        DPRINTF(DeviceState, "Ref done at tick %llu\n", ref_done_at);

        for (auto b : banks) {
            b->actAllowedAt = ref_done_at;
        }
        cmdList.push_back(Command(MemCommand::REF, 0, curTick()));

        updatePowerStats();

        refreshDueAt += memory.tREFI;

        if (refreshDueAt < ref_done_at)
            fatal("Refresh was delayed so long we cannot catch up\n");

        refreshState = REF_RUN;
        schedule(refreshEvent, ref_done_at);
        return;
    }

    if (refreshState == REF_RUN) {
        assert(numBanksActive == 0);
        assert(pwrState == PWR_REF);
        assert(!powerEvent.scheduled());
        if ((memory.drainState() == DrainState::Draining) ||
                memory.drainState() == DrainState::Drained) {
            schedulePowerEvent(PWR_IDLE, curTick());
        } else {
            if (pwrStatePostRefresh != PWR_IDLE) {
                assert(pwrState == PWR_REF);
                powerDownSleep(pwrState, curTick());
            } else if (lowPowerEntryReady()) {
                powerDownSleep(PWR_PRE_PDN, curTick());
            } else {
                schedulePowerEvent(PWR_IDLE, curTick());
            }
        }
        
        if (pwrStateTrans != PWR_SREF) {
            schedule(refreshEvent, refreshDueAt - memory.tRP);
        }
    }
}

void
DRAMCtrl::Rank::schedulePowerEvent(PowerState pwr_state, Tick tick)
{
    assert(!memory.isVMCMode);
    // respect causality
    assert(tick >= curTick());

    if (!powerEvent.scheduled()) {
        DPRINTF(DRAMState, "Scheduling power event at %llu to state %d\n",
                tick, pwr_state);

        // insert the new transition
        pwrStateTrans = pwr_state;

        schedule(powerEvent, tick);
    } else {
        panic("Scheduled power event at %llu to state %d, "
              "with scheduled event at %llu to %d\n", tick, pwr_state,
              powerEvent.when(), pwrStateTrans);
    }
}

void
DRAMCtrl::Device::schedulePowerEvent(PowerState pwr_state, Tick tick)
{
    assert(tick >= curTick());
    if (!powerEvent.scheduled()) {
        pwrStateTrans = pwr_state;
        schedule(powerEvent, tick);
    } else {
        panic("Scheduled power event at %llu to stat %d, "
                "with scheduled event at %llu to %d\n",
                tick, pwr_state, powerEvent.when(), pwrStateTrans);
    }
}

void
DRAMCtrl::Rank::powerDownSleep(PowerState pwr_state, Tick tick)
{
    assert(!memory.isVMCMode);
    // if low power state is active low, schedule to active low power state.
    // in reality tCKE is needed to enter active low power. This is neglected
    // here and could be added in the future.
    if (pwr_state == PWR_ACT_PDN) {
        schedulePowerEvent(pwr_state, tick);
        // push command to DRAMPower
        cmdList.push_back(Command(MemCommand::PDN_F_ACT, 0, tick));
        DPRINTF(DRAMPower, "%llu,PDN_F_ACT,0,%d\n", divCeil(tick,
                memory.tCK) - memory.timeStampOffset, rank);
        ++pdnCmds;
        for (auto d : devices)
            ++devPdnCmds[d->device];
    } else if (pwr_state == PWR_PRE_PDN) {
        // if low power state is precharge low, schedule to precharge low
        // power state. In reality tCKE is needed to enter active low power.
        // This is neglected here.
        schedulePowerEvent(pwr_state, tick);
        //push Command to DRAMPower
        cmdList.push_back(Command(MemCommand::PDN_F_PRE, 0, tick));
        DPRINTF(DRAMPower, "%llu,PDN_F_PRE,0,%d\n", divCeil(tick,
                memory.tCK) - memory.timeStampOffset, rank);
        ++pdnCmds;
        for (auto d : devices)
            ++devPdnCmds[d->device];
    } else if (pwr_state == PWR_REF) {
        // if a refresh just occured
        // transition to PRE_PDN now that all banks are closed
        // do not transition to SREF if commands are in Q; stay in PRE_PDN
        if (pwrStatePostRefresh == PWR_ACT_PDN || !lowPowerEntryReady()) {
            // prechage power down requires tCKE to enter. For simplicity
            // this is not considered.
            schedulePowerEvent(PWR_PRE_PDN, tick);
            //push Command to DRAMPower
            cmdList.push_back(Command(MemCommand::PDN_F_PRE, 0, tick));
            DPRINTF(DRAMPower, "%llu,PDN_F_PRE,0,%d\n", divCeil(tick,
                    memory.tCK) - memory.timeStampOffset, rank);
            ++pdnCmds;
            for (auto d : devices)
                ++devPdnCmds[d->device];
        } else {
            // last low power State was power precharge
            assert(pwrStatePostRefresh == PWR_PRE_PDN);
            // self refresh requires time tCKESR to enter. For simplicity,
            // this is not considered.
            schedulePowerEvent(PWR_SREF, tick);
            // push Command to DRAMPower
            cmdList.push_back(Command(MemCommand::SREN, 0, tick));
            DPRINTF(DRAMPower, "%llu,SREN,0,%d\n", divCeil(tick,
                    memory.tCK) - memory.timeStampOffset, rank);
            ++selfRefCmds;
            for (auto d : devices)
                ++devSelfRefCmds[d->device];
        }
    }
    // Ensure that we don't power-down and back up in same tick
    // Once we commit to PD entry, do it and wait for at least 1tCK
    // This could be replaced with tCKE if/when that is added to the model
    wakeUpAllowedAt = tick + memory.tCK;

    // Transitioning to a low power state, set flag
    inLowPowerState = true;
}

void
DRAMCtrl::Device::powerDownSleep(PowerState pwr_state, Tick tick)
{
    if (pwr_state == PWR_ACT_PDN) {
        schedulePowerEvent(pwr_state, tick);
        cmdList.push_back(Command(MemCommand::PDN_F_ACT, 0, tick));
        ++rankRef.devPdnCmds[device];
    } else if (pwr_state == PWR_PRE_PDN) {
        schedulePowerEvent(pwr_state, tick);
        cmdList.push_back(Command(MemCommand::PDN_F_PRE, 0, tick));
        ++rankRef.devPdnCmds[device];
    } else if (pwr_state == PWR_REF) {
        if (pwrStatePostRefresh == PWR_ACT_PDN || !lowPowerEntryReady()) {
            schedulePowerEvent(PWR_PRE_PDN, tick);
            cmdList.push_back(Command(MemCommand::PDN_F_PRE, 0, tick));
            ++rankRef.devPdnCmds[device];
        } else {
            assert(pwrStatePostRefresh == PWR_PRE_PDN);
            schedulePowerEvent(PWR_SREF, tick);
            cmdList.push_back(Command(MemCommand::SREN, 0, tick));
            ++rankRef.devSelfRefCmds[device];
        }
    }
    wakeUpAllowedAt = tick + memory.tCK;
    inLowPowerState = true;
}

void
DRAMCtrl::Rank::scheduleWakeUpEvent(Tick exit_delay)
{
    assert(!memory.isVMCMode);
    Tick wake_up_tick = std::max(curTick(), wakeUpAllowedAt);

    DPRINTF(DRAMState, "Scheduling wake-up for rank %d at tick %d\n",
            rank, wake_up_tick);

    // if waking for refresh, hold previous state
    // else reset state back to IDLE
    if (refreshState == REF_PD_EXIT) {
        pwrStatePostRefresh = pwrState;
    } else {
        // don't automatically transition back to LP state after next REF
        pwrStatePostRefresh = PWR_IDLE;
    }

    // schedule wake-up with event to ensure entry has completed before
    // we try to wake-up
    schedule(wakeUpEvent, wake_up_tick);

    for (auto b : banks) {
        // respect both causality and any existing bank
        // constraints, some banks could already have a
        // (auto) precharge scheduled
        b->colAllowedAt = std::max(wake_up_tick + exit_delay, b->colAllowedAt);
        b->preAllowedAt = std::max(wake_up_tick + exit_delay, b->preAllowedAt);
        b->actAllowedAt = std::max(wake_up_tick + exit_delay, b->actAllowedAt);
    }
    // Transitioning out of low power state, clear flag
    inLowPowerState = false;

    // push to DRAMPower
    // use pwrStateTrans for cases where we have a power event scheduled
    // to enter low power that has not yet been processed
    if (pwrStateTrans == PWR_ACT_PDN) {
        cmdList.push_back(Command(MemCommand::PUP_ACT, 0, wake_up_tick));
        DPRINTF(DRAMPower, "%llu,PUP_ACT,0,%d\n", divCeil(wake_up_tick,
                memory.tCK) - memory.timeStampOffset, rank);

    } else if (pwrStateTrans == PWR_PRE_PDN) {
        cmdList.push_back(Command(MemCommand::PUP_PRE, 0, wake_up_tick));
        DPRINTF(DRAMPower, "%llu,PUP_PRE,0,%d\n", divCeil(wake_up_tick,
                memory.tCK) - memory.timeStampOffset, rank);
    } else if (pwrStateTrans == PWR_SREF) {
        cmdList.push_back(Command(MemCommand::SREX, 0, wake_up_tick));
        DPRINTF(DRAMPower, "%llu,SREX,0,%d\n", divCeil(wake_up_tick,
                memory.tCK) - memory.timeStampOffset, rank);
    }
}

void
DRAMCtrl::Device::scheduleWakeUpEvent(Tick exit_delay)
{
    Tick wake_up_tick = std::max(curTick(), wakeUpAllowedAt);

    if (refreshState == REF_PD_EXIT) {
        pwrStatePostRefresh = pwrState;
    } else {
        pwrStatePostRefresh = PWR_IDLE;
    }

    schedule(wakeUpEvent, wake_up_tick);

    for (auto b : banks) {
        b->colAllowedAt = std::max(wake_up_tick + exit_delay, b->colAllowedAt);
        b->preAllowedAt = std::max(wake_up_tick + exit_delay, b->preAllowedAt);
        b->actAllowedAt = std::max(wake_up_tick + exit_delay, b->actAllowedAt);
    }
    inLowPowerState = false;

    if (pwrStateTrans == PWR_ACT_PDN) {
        cmdList.push_back(Command(MemCommand::PUP_ACT, 0, wake_up_tick));
    } else if (pwrStateTrans == PWR_PRE_PDN) {
        cmdList.push_back(Command(MemCommand::PUP_PRE, 0, wake_up_tick));
    } else if (pwrStateTrans == PWR_SREF) {
        cmdList.push_back((Command(MemCommand::SREX, 0, wake_up_tick)));
    }
}

void
DRAMCtrl::Rank::processWakeUpEvent()
{
    assert(!memory.isVMCMode);
    // Should be in a power-down or self-refresh state
    assert((pwrState == PWR_ACT_PDN) || (pwrState == PWR_PRE_PDN) ||
           (pwrState == PWR_SREF));

    // Check current state to determine transition state
    if (pwrState == PWR_ACT_PDN) {
        // banks still open, transition to PWR_ACT
        schedulePowerEvent(PWR_ACT, curTick());
    } else {
        // transitioning from a precharge power-down or self-refresh state
        // banks are closed - transition to PWR_IDLE
        schedulePowerEvent(PWR_IDLE, curTick());
    }
}

void
DRAMCtrl::Device::processWakeUpEvent()
{
    assert(pwrState == PWR_ACT_PDN || pwrState == PWR_PRE_PDN ||
            pwrState == PWR_SREF);
    if (pwrState == PWR_ACT_PDN)
        schedulePowerEvent(PWR_ACT, curTick());
    else
        schedulePowerEvent(PWR_IDLE, curTick());
}

void
DRAMCtrl::Rank::processPowerEvent()
{
    assert(!memory.isVMCMode);
    assert(curTick() >= pwrStateTick);
    // remember where we were, and for how long
    Tick duration = curTick() - pwrStateTick;
    PowerState prev_state = pwrState;

    // update the accounting
    pwrStateTime[prev_state] += duration;

    // track to total idle time
    if ((prev_state == PWR_PRE_PDN) || (prev_state == PWR_ACT_PDN) ||
        (prev_state == PWR_SREF)) {
        totalIdleTime += duration;
    }

    pwrState = pwrStateTrans;
    pwrStateTick = curTick();

    // if rank was refreshing, make sure to start scheduling requests again
    if (prev_state == PWR_REF) {
        // bus IDLED prior to REF
        // counter should be one for refresh command only
        assert(outstandingEvents == 1);
        // REF complete, decrement count
        --outstandingEvents;
        std::ostringstream oss;
        oss << "refresh done";
        memory.dPrintEventCount(rank, false, oss.str());

        DPRINTF(DRAMState, "Was refreshing for %llu ticks\n", duration);
        // if sleeping after refresh
        if (pwrState != PWR_IDLE) {
            assert((pwrState == PWR_PRE_PDN) || (pwrState == PWR_SREF));
            DPRINTF(DRAMState, "Switching to power down state after refreshing"
                    " rank %d at %llu tick\n", rank, curTick());
        }
        if (pwrState != PWR_SREF) {
            // rank is not available in SREF
            // don't transition to IDLE in this case
            refreshState = REF_IDLE;
        }
        // a request event could be already scheduled by the state
        // machine of the other rank
        if (!memory.nextReqEvent.scheduled()) {
            DPRINTF(DRAM, "Scheduling next request after refreshing rank %d\n",
                    rank);
            schedule(memory.nextReqEvent, curTick());
        }
    } else if (pwrState == PWR_ACT) {
        if (refreshState == REF_PD_EXIT) {
            // kick the refresh event loop into action again
            assert(prev_state == PWR_ACT_PDN);

            // go back to REF event and close banks
            refreshState = REF_PRE;
            schedule(refreshEvent, curTick());
        }
    } else if (pwrState == PWR_IDLE) {
        DPRINTF(DRAMState, "All banks precharged\n");
        if (prev_state == PWR_SREF) {
            // set refresh state to REF_SREF_EXIT, ensuring isAvailable
            // continues to return false during tXS after SREF exit
            // Schedule a refresh which kicks things back into action
            // when it finishes
            refreshState = REF_SREF_EXIT;
            schedule(refreshEvent, curTick() + memory.tXS);
        } else {
            // if we have a pending refresh, and are now moving to
            // the idle state, directly transition to a refresh
            if ((refreshState == REF_PRE) || (refreshState == REF_PD_EXIT)) {
                // ensure refresh is restarted only after final PRE command.
                // do not restart refresh if controller is in an intermediate
                // state, after PRE_PDN exit, when banks are IDLE but an
                // ACT is scheduled.
                if (!activateEvent.scheduled()) {
                    // there should be nothing waiting at this point
                    assert(!powerEvent.scheduled());
                    // update the state in zero time and proceed below
                    pwrState = PWR_REF;
                } else {
                    // must have PRE scheduled to transition back to IDLE
                    // and re-kick off refresh
                    assert(prechargeEvent.scheduled());
                }
            }
       }
    }

    // we transition to the refresh state, let the refresh state
    // machine know of this state update and let it deal with the
    // scheduling of the next power state transition as well as the
    // following refresh
    if (pwrState == PWR_REF) {
        assert(refreshState == REF_PRE || refreshState == REF_PD_EXIT);
        DPRINTF(DRAMState, "Refreshing\n");

        // kick the refresh event loop into action again, and that
        // in turn will schedule a transition to the idle power
        // state once the refresh is done
        if (refreshState == REF_PD_EXIT) {
            // Wait for PD exit timing to complete before issuing REF
            schedule(refreshEvent, curTick() + memory.tXP);
        } else {
            schedule(refreshEvent, curTick());
        }
        // Banks transitioned to IDLE, start REF
        refreshState = REF_START;
    }
}

void
DRAMCtrl::Device::processPowerEvent()
{
    DPRINTF(DeviceState, "Device %d_%d power trans from state %d to state %d\n",
            this->rank, this->device, this->pwrState, this->pwrStateTrans);
    assert(curTick() >= pwrStateTick);
    Tick duration = curTick() - pwrStateTick;
    PowerState prev_state = pwrState;
    pwrStateTime[prev_state] += duration;

    if ((prev_state == PWR_PRE_PDN) || (prev_state == PWR_ACT_PDN) || (prev_state == PWR_SREF)) {
        totalIdleTime += duration;
    }

    pwrState = pwrStateTrans;
    pwrStateTick = curTick();

    if (prev_state == PWR_REF) {
        assert(outstandingEvents == 1);
        --outstandingEvents;
        if (pwrState != PWR_IDLE) {
            assert((pwrState == PWR_PRE_PDN) || (pwrState == PWR_SREF));
        }
        if (pwrState != PWR_SREF) {
            refreshState = REF_IDLE;
        }
        if (!memory.nextReqEvent.scheduled()) {
            schedule(memory.nextReqEvent, curTick());
        }
    } else if (pwrState == PWR_ACT) {
        if (refreshState == REF_PD_EXIT) {
            assert(prev_state == PWR_ACT_PDN);
            refreshState = REF_PRE;
            schedule(refreshEvent, curTick());
        }
    } else if (pwrState == PWR_IDLE) {
        if (prev_state == PWR_SREF) {
            refreshState = REF_SREF_EXIT;
            schedule(refreshEvent, curTick() + memory.tXS);
        } else {
            if ((refreshState == REF_PRE) || (refreshState == REF_PD_EXIT)) {
                if (!activateEvent.scheduled()) {
                    assert(!powerEvent.scheduled());
                    pwrState = PWR_REF;
                } else {
                    assert(prechargeEvent.scheduled());
                }
            }
        }
    }
    
    if (pwrState == PWR_REF) {
        assert(refreshState == REF_PRE || refreshState == REF_PD_EXIT);
        if (refreshState == REF_PD_EXIT)
            schedule(refreshEvent, curTick() + memory.tRP);
        else
            schedule(refreshEvent, curTick());
        refreshState = REF_START;
    }
}

void
DRAMCtrl::Bank::processClearEvent(bool isRead)
{
    DPRINTF(Push, "%s clearEvent on rank%d_device%d_bank%d, busBusyUntil %lld\n",
            isRead?"Rd":"Wr", rank, device, bank, memory.busBusyUntil);
    // There must be a thing to push.
    DRAMPacket*& pendingClearPkt = isRead?pendingRdClearPkt:pendingWrClearPkt;

    // Sanity check.
    assert(pendingClearPkt);
    std::deque<DRAMPacket*>& addrRegs = isRead?rdAddrRegs:wrAddrRegs;
    assert(!pendingClearPkt->isPack);
    assert(addrRegs.size() <= memory.addrRegsPerDevice);
    // It has to be pushed already if we are to clear it.
    assert(std::find(addrRegs.begin(), addrRegs.end(), pendingClearPkt) !=
            addrRegs.end());
    // Clear.
    addrRegs.erase(std::find(addrRegs.begin(), addrRegs.end(), pendingClearPkt));
    // Delete write pkts since we no longer need it.
    // DESTROY actual write pkt.
    if (!pendingClearPkt->isRead) {
        DPRINTF(Push, "Destroy pkt %d\n", pendingClearPkt->id);
        delete pendingClearPkt;
    }
    // Reset.
    pendingClearPkt = NULL;
    memory.dPrintAddrRegs(rank, device, isRead);
}

void
DRAMCtrl::Bank::processRdClearEvent()
{
    processClearEvent(true);
}

void
DRAMCtrl::Bank::processWrClearEvent()
{
    processClearEvent(false);
}

void
DRAMCtrl::Bank::processPushEvent(bool isRead)
{
    DPRINTF(Push, "%s pushEvent on rank%d_device%d_bank%d, busBusyUntil %lld\n",
            isRead?"Rd":"Wr", rank, device, bank, memory.busBusyUntil);
    // There must be a thing to push.
    DRAMPacket*& pendingPushPkt = isRead?pendingRdPushPkt:pendingWrPushPkt;
    assert(pendingPushPkt);
    std::deque<DRAMPacket*>& addrRegs = isRead?rdAddrRegs:wrAddrRegs;
    DPRINTF(Push, "pkt %d to be pushed\n", pendingPushPkt->id);
    // We must have not pushed the pkt already.
    assert(pendingPushPkt->addrReadyAt == 0);
    ++memory.pushReqs;
    // When we are here, there gotta be an empty space in addrRegs.
    assert(addrRegs.size() < memory.addrRegsPerDevice);
    // Update stats.
    if (prevPushTick > 0)
        memory.totPushGap += curTick() - prevPushTick;
    prevPushTick = curTick();
    // Push.
    addrRegs.push_back(pendingPushPkt);
    // There's enough time on data bus for addr transfer.
    assert((curTick() + memory.pushDelay <= memory.busBusyUntil - memory.tBURST) ||
            curTick() >= memory.busBusyUntil);
    // Update addrReadyAt.
    pendingPushPkt->addrReadyAt = curTick() + memory.pushDelay;
    DPRINTF(Push, "Addr for (%d) ready at %lld (%d cycles later) on rank%d_device%d_bank%d\n",
            pendingPushPkt->id, pendingPushPkt->addrReadyAt,
            (pendingPushPkt->addrReadyAt - curTick())/memory.tCK,
            rank, device, bank);
    if (!memory.nextReqEvent.scheduled()) {
        DPRINTF(VMC, "Request scheduled immediately after push\n");
        //memory.schedule(memory.nextReqEvent, pendingPushPkt->addrReadyAt);
        memory.schedule(memory.nextReqEvent, curTick());
    }
    // Reset.
    pendingPushPkt = NULL;
    memory.dPrintAddrRegs(rank, device, isRead);
}

void
DRAMCtrl::Bank::processRdPushEvent()
{
    processPushEvent(true);
}

void
DRAMCtrl::Bank::processWrPushEvent()
{
    processPushEvent(false);
}

void
DRAMCtrl::Rank::updatePowerStats()
{
    assert(!memory.isVMCMode);
    // All commands up to refresh have completed
    // flush cmdList to DRAMPower
    flushCmdList();

    // update the counters for DRAMPower, passing false to
    // indicate that this is not the last command in the
    // list. DRAMPower requires this information for the
    // correct calculation of the background energy at the end
    // of the simulation. Ideally we would want to call this
    // function with true once at the end of the
    // simulation. However, the discarded energy is extremly
    // small and does not effect the final results.
    power.powerlib.updateCounters(false);

    // call the energy function
    power.powerlib.calcEnergy();

    // Get the energy and power from DRAMPower
    Data::MemoryPowerModel::Energy energy =
        power.powerlib.getEnergy();
    Data::MemoryPowerModel::Power rank_power =
        power.powerlib.getPower();

    actEnergy = energy.act_energy * memory.devicesPerRank;
    preEnergy = energy.pre_energy * memory.devicesPerRank;
    readEnergy = energy.read_energy * memory.devicesPerRank;
    writeEnergy = energy.write_energy * memory.devicesPerRank;
    refreshEnergy = energy.ref_energy * memory.devicesPerRank;
    actBackEnergy = energy.act_stdby_energy * memory.devicesPerRank;
    preBackEnergy = energy.pre_stdby_energy * memory.devicesPerRank;
    actPowerDownEnergy = energy.f_act_pd_energy * memory.devicesPerRank;
    prePowerDownEnergy = energy.f_pre_pd_energy * memory.devicesPerRank;
    selfRefreshEnergy = energy.sref_energy * memory.devicesPerRank;
    totalEnergy = energy.total_energy * memory.devicesPerRank;
    averagePower = rank_power.average_power * memory.devicesPerRank;
}

void
DRAMCtrl::Device::updatePowerStats()
{
    assert(memory.isVMCMode);
    flushCmdList();
    power.powerlib.updateCounters(false);
    power.powerlib.calcEnergy();
    // weil0ng: note that energy and power is computed for the entire rank.
    // So when we split the energy/power evenly across all chips to derive
    // energy/power per chip given its own cmdList.
    Data::MemoryPowerModel::Energy energy = power.powerlib.getEnergy();
    Data::MemoryPowerModel::Power device_power = power.powerlib.getPower();

    actEnergy = energy.act_energy / memory.devicesPerRank;
    rankRef.devActEnergy[device] = energy.act_energy / memory.devicesPerRank;
    preEnergy = energy.pre_energy / memory.devicesPerRank;
    rankRef.devPreEnergy[device] = energy.pre_energy / memory.devicesPerRank;
    readEnergy = energy.read_energy / memory.devicesPerRank;
    rankRef.devReadEnergy[device] = energy.read_energy / memory.devicesPerRank;
    writeEnergy = energy.write_energy / memory.devicesPerRank;
    rankRef.devWriteEnergy[device] = energy.write_energy / memory.devicesPerRank;
    refreshEnergy = energy.ref_energy / memory.devicesPerRank;
    rankRef.devRefreshEnergy[device] = energy.ref_energy / memory.devicesPerRank;
    actBackEnergy = energy.act_stdby_energy / memory.devicesPerRank;
    rankRef.devActBackEnergy[device] = energy.act_stdby_energy / memory.devicesPerRank;
    preBackEnergy = energy.pre_stdby_energy / memory.devicesPerRank;
    rankRef.devPreBackEnergy[device] = energy.pre_stdby_energy / memory.devicesPerRank;
    actPowerDownEnergy = energy.f_act_pd_energy / memory.devicesPerRank;
    rankRef.devActPowerDownEnergy[device] = energy.f_act_pd_energy / memory.devicesPerRank;
    prePowerDownEnergy = energy.f_pre_pd_energy / memory.devicesPerRank;
    rankRef.devPrePowerDownEnergy[device] = energy.f_pre_pd_energy / memory.devicesPerRank;
    selfRefreshEnergy = energy.sref_energy / memory.devicesPerRank;
    rankRef.devSelfRefreshEnergy[device] = energy.sref_energy / memory.devicesPerRank;
    totalEnergy = energy.total_energy / memory.devicesPerRank;
    rankRef.devTotalEnergy[device] = energy.total_energy / memory.devicesPerRank;
    averagePower = device_power.average_power/memory.devicesPerRank;
    rankRef.devAveragePower[device] = device_power.average_power/memory.devicesPerRank;

    // final update of power state times
    pwrStateTime[pwrState] += (curTick() - pwrStateTick);
    pwrStateTick = curTick();
}

void
DRAMCtrl::Rank::computeStats()
{
    DPRINTF(DRAM,"Computing final stats\n");

    // Force DRAM power to update counters based on time spent in
    // current state up to curTick()
    cmdList.push_back(Command(MemCommand::NOP, 0, curTick()));

    // Update the stats.
    if (!memory.isVMCMode) {
        updatePowerStats();
    } else {
        for (auto d : devices) {
            d->updatePowerStats();
        }
    }

    // final update of power state times
    pwrStateTime[pwrState] += (curTick() - pwrStateTick);
    pwrStateTick = curTick();

}

void
DRAMCtrl::Rank::regStats()
{
    using namespace Stats;

    pwrStateTime
        .init(6)
        .name(name() + ".memoryStateTime")
        .desc("Time in different power states");
    pwrStateTime.subname(0, "IDLE");
    pwrStateTime.subname(1, "REF");
    pwrStateTime.subname(2, "SREF");
    pwrStateTime.subname(3, "PRE_PDN");
    pwrStateTime.subname(4, "ACT");
    pwrStateTime.subname(5, "ACT_PDN");

    actEnergy
        .name(name() + ".actEnergy")
        .desc("Energy for activate commands per rank (pJ)");

    devActEnergy
        .init(memory.devicesPerRank)
        .name(name() + ".devActEnergy")
        .desc("Energy for activate per device (pJ)");

    accActEnergy
        .name(name() + ".accActEnergy")
        .desc("Energy for activate collected from devices (pJ)")
        .precision(2);

    accActEnergy = sum(devActEnergy);

    preEnergy
        .name(name() + ".preEnergy")
        .desc("Energy for precharge commands per rank (pJ)");

    devPreEnergy
        .init(memory.devicesPerRank)
        .name(name() + ".devPreEnergy")
        .desc("Energy for precharge per device (pJ)");

    accPreEnergy
        .name(name() + ".accPreEnergy")
        .desc("Energy for precharge collected from devices (pJ)")
        .precision(2);

    accPreEnergy = sum(devPreEnergy);

    readEnergy
        .name(name() + ".readEnergy")
        .desc("Energy for read commands per rank (pJ)");

    devReadEnergy
        .init(memory.devicesPerRank)
        .name(name() + ".devReadEnergy")
        .desc("Energy for read per device (pJ)");

    accReadEnergy
        .name(name() + ".accReadEnergy")
        .desc("Energy for read collected from devices (pJ)")
        .precision(2);

    accReadEnergy = sum(devReadEnergy);

    writeEnergy
        .name(name() + ".writeEnergy")
        .desc("Energy for write commands per rank (pJ)");

    devWriteEnergy
        .init(memory.devicesPerRank)
        .name(name() + ".devWriteEnergy")
        .desc("Energy for write per device (pJ)");

    accWriteEnergy
        .name(name() + ".accWriteEnergy")
        .desc("Energy for write collected from devices (pJ)")
        .precision(2);

    accWriteEnergy = sum(devWriteEnergy);

    refreshEnergy
        .name(name() + ".refreshEnergy")
        .desc("Energy for refresh commands per rank (pJ)");

    devRefreshEnergy
        .init(memory.devicesPerRank)
        .name(name() + ".devRefreshEnergy")
        .desc("Energy for refresh per device (pJ)");

    accRefreshEnergy
        .name(name() + ".accRefreshEnergy")
        .desc("Energy for refresh collected from devices (pJ)")
        .precision(2);

    accRefreshEnergy = sum(devRefreshEnergy);

    actBackEnergy
        .name(name() + ".actBackEnergy")
        .desc("Energy for active background per rank (pJ)");

    devActBackEnergy
        .init(memory.devicesPerRank)
        .name(name() + ".devActBackEnergy")
        .desc("Energy for active background per device (pJ)");

    accActBackEnergy
        .name(name() + ".accActBackEnergy")
        .desc("Energy for active background collected from devices (pJ)")
        .precision(2);

    accActBackEnergy = sum(devActBackEnergy);

    preBackEnergy
        .name(name() + ".preBackEnergy")
        .desc("Energy for precharge background per rank (pJ)");

    devPreBackEnergy
        .init(memory.devicesPerRank)
        .name(name() + ".devPreBackEnergy")
        .desc("Energy for precharge background per device (pJ)");

    accPreBackEnergy
        .name(name() + ".accPreBackEnergy")
        .desc("Energy for precharge background collected from devices (pJ)")
        .precision(2);

    accPreBackEnergy = sum(devPreBackEnergy);

    actPowerDownEnergy
        .name(name() + ".actPowerDownEnergy")
        .desc("Energy for active power-down per rank (pJ)");

    devActPowerDownEnergy
        .init(memory.devicesPerRank)
        .name(name() + ".devActPowerDownEnergy")
        .desc("Energy for active power-down per device (pJ)");

    accActPowerDownEnergy
        .name(name() + ".accActPowerDownEnergy")
        .desc("Energy for active power-down collected from devices (pJ)")
        .precision(2);

    accActPowerDownEnergy = sum(devActPowerDownEnergy);

    prePowerDownEnergy
        .name(name() + ".prePowerDownEnergy")
        .desc("Energy for precharge power-down per rank (pJ)");

    devPrePowerDownEnergy
        .init(memory.devicesPerRank)
        .name(name() + ".devPrePowerDownEnergy")
        .desc("Energy for precharge power-down per device (pJ)");

    accPrePowerDownEnergy
        .name(name() + ".accPrePowerDownEnergy")
        .desc("Energy for precharge power-down collected from devices (pJ)")
        .precision(2);

    accPrePowerDownEnergy = sum(devPrePowerDownEnergy);

    selfRefreshEnergy
        .name(name() + ".selfRefreshEnergy")
        .desc("Energy for self refresh per rank (pJ)");

    devSelfRefreshEnergy
        .init(memory.devicesPerRank)
        .name(name() + ".devSelfRefreshEnergy")
        .desc("Energy for self refresh per device (pJ)");

    accSelfRefreshEnergy
        .name(name() + ".accSelfRefreshEnergy")
        .desc("Energy for self refresh collected from devices (pJ)")
        .precision(2);

    accSelfRefreshEnergy = sum(devSelfRefreshEnergy);

    totalEnergy
        .name(name() + ".totalEnergy")
        .desc("Total energy per rank (pJ)");

    devTotalEnergy
        .init(memory.devicesPerRank)
        .name(name() + ".devTotalEnergy")
        .desc("Total energy per device (pJ)");

    accTotalEnergy
        .name(name() + ".accTotalEnergy")
        .desc("Total energy collected from devices (pJ)")
        .precision(2);

    accTotalEnergy = sum(devTotalEnergy);

    activationEnergy
        .name(name() + ".activationEnergy")
        .desc("Total activation energy per rank (pJ)")
        .precision(2);

    activationEnergy = actEnergy + preEnergy;

    accActivationEnergy
        .name(name() + ".accActivationEnergy")
        .desc("Total activation energy collected from devices (pJ)")
        .precision(2);

    accActivationEnergy = accActEnergy + accPreEnergy;

    dynamicEnergy
        .name(name() + ".dynamicEnergy")
        .desc("Total dynamic energy per rank (pJ)")
        .precision(2);

    dynamicEnergy = actEnergy + preEnergy + readEnergy + writeEnergy;

    accDynamicEnergy
        .name(name() + ".accDynamicEnergy")
        .desc("Total dynamic energy collected from devices (pJ)")
        .precision(2);

    accDynamicEnergy = accActEnergy + accPreEnergy + accReadEnergy + accWriteEnergy;

    backgroundEnergy
        .name(name() + ".backgroundEnergy")
        .desc("Total background energy per rank (pJ)")
        .precision(2);

    backgroundEnergy = refreshEnergy + actBackEnergy +
        preBackEnergy + actPowerDownEnergy +
        prePowerDownEnergy + selfRefreshEnergy;

    accBackgroundEnergy
        .name(name() + ".accBackgroundEnergy")
        .desc("Total background energy collected from devices (pJ)")
        .precision(2);

    accBackgroundEnergy = accRefreshEnergy + accActBackEnergy +
        accPreBackEnergy + accActPowerDownEnergy +
        accPrePowerDownEnergy + accSelfRefreshEnergy;

    averagePower
        .name(name() + ".averagePower")
        .desc("Core power per rank (mW)");

    devAveragePower
        .init(memory.devicesPerRank)
        .name(name() + ".devAveragePower")
        .desc("Core power per device (mW)");

    accAveragePower
        .name(name() + ".accAveragePower")
        .desc("Core power averaged across devices (mW)")
        .precision(2);

    accAveragePower = sum(devAveragePower);

    actCmds
        .name(name() + ".actCmds")
        .desc("Number of activate cmds per rank");

    devActCmds
        .init(memory.devicesPerRank)
        .name(name() + ".devActCmds")
        .desc("Number of activate cmds per device");

    avgDevActCmds
        .name(name() + ".avgDevActCmds")
        .desc("Average number of activate cmds per device")
        .precision(2);

    avgDevActCmds = sum(devActCmds) / memory.devicesPerRank;

    preCmds
        .name(name() + ".preCmds")
        .desc("Number of precharge cmds per rank");

    devPreCmds
        .init(memory.devicesPerRank)
        .name(name() + ".devPreCmds")
        .desc("Number of precharge cmds per device");

    avgDevPreCmds
        .name(name() + ".avgDevPreCmds")
        .desc("Average number of activate cmds per device")
        .precision(2);

    avgDevPreCmds = sum(devPreCmds) / memory.devicesPerRank;

    refCmds
        .name(name() + ".refCmds")
        .desc("Number of refresh cmds per rank");

    devRefCmds
        .init(memory.devicesPerRank)
        .name(name() + ".devRefCmds")
        .desc("Number of refresh cmds per device");

    avgDevRefCmds
        .name(name() + ".avgDevRefCmds")
        .desc("Average number of refresh cmds per device")
        .precision(2);

    avgDevRefCmds = sum(devRefCmds) / memory.devicesPerRank;

    pdnCmds
        .name(name() + ".pdnCmds")
        .desc("Number of power-down cmds per rank");

    devPdnCmds
        .init(memory.devicesPerRank)
        .name(name() + ".devPdnCmds")
        .desc("Number of power-down cmds per device");

    avgDevPdnCmds
        .name(name() + ".avgDevPdnCmds")
        .desc("Average number of power-down cmds per device")
        .precision(2);

    avgDevPdnCmds = sum(devPdnCmds) / memory.devicesPerRank;

    selfRefCmds
        .name(name() + ".selfRefCmds")
        .desc("Number of self-refresh cmds per rank");

    devSelfRefCmds
        .init(memory.devicesPerRank)
        .name(name() + ".devSelfRefCmds")
        .desc("Number of self-refresh cmds per device");

    avgDevSelfRefCmds
        .name(name() + ".avgDevSelfRefCmds")
        .desc("Average number of self-refresh cmds per device")
        .precision(2);

    avgDevSelfRefCmds = sum(devSelfRefCmds) / memory.devicesPerRank;

    totalIdleTime
        .name(name() + ".totalIdleTime")
        .desc("Total Idle time Per DRAM Rank");

    registerDumpCallback(new RankDumpCallback(this));
}

void
DRAMCtrl::Device::regStats()
{
    using namespace Stats;

    pwrStateTime
        .init(6)
        .name(name() + ".memoryStateTime")
        .desc("Time in different power states");
    pwrStateTime.subname(0, "IDLE");
    pwrStateTime.subname(1, "REF");
    pwrStateTime.subname(2, "SREF");
    pwrStateTime.subname(3, "PRE_PDN");
    pwrStateTime.subname(4, "ACT");
    pwrStateTime.subname(5, "ACT_PDN");

    actEnergy
        .name(name() + ".actEnergy")
        .desc("Energy for activate commands per device (pJ)");

    preEnergy
        .name(name() + ".preEnergy")
        .desc("Energy for precharge commands per device (pJ)");

    readEnergy
        .name(name() + ".readEnergy")
        .desc("Energy for read commands per device (pJ)");

    writeEnergy
        .name(name() + ".writeEnergy")
        .desc("Energy for write commands per device (pJ)");

    refreshEnergy
        .name(name() + ".refreshEnergy")
        .desc("Energy for refresh commands per device (pJ)");

    actBackEnergy
        .name(name() + ".actBackEnergy")
        .desc("Energy for active background per device (pJ)");

    preBackEnergy
        .name(name() + ".preBackEnergy")
        .desc("Energy for precharge background per device (pJ)");

    actPowerDownEnergy
        .name(name() + ".actPowerDownEnergy")
        .desc("Energy for active power-down per device (pJ)");

    prePowerDownEnergy
        .name(name() + ".prePowerDownEnergy")
        .desc("Energy for precharge power-down per device (pJ)");

    selfRefreshEnergy
        .name(name() + ".selfRefreshEnergy")
        .desc("Energy for self refresh per device (pJ)");

    totalEnergy
        .name(name() + ".totalEnergy")
        .desc("Total energy per device (pJ)");

    averagePower
        .name(name() + ".averagePower")
        .desc("Core power per device (mW)");

    totalIdleTime
        .name(name() + ".totalIdleTime")
        .desc("Total Idle time Per DRAM device");

    bytesAccessed
        .name(name() + ".bytesAccessed")
        .desc("Bytes accessed during VMC mode per bank")
        .init(memory.banksPerRank);

    totalBytesAccessed
        .name(name() + ".totalBytesAccessed")
        .desc("Total bytes accessed during VMC mode per dev");
    
    devBW
        .name(name() + ".devBW")
        .desc("BW for the device during VMC mode in MB")
        .precision(2);

    devBW = (totalBytesAccessed / (1024*1024)) / simSeconds;
}

void
DRAMCtrl::regStats()
{
    using namespace Stats;

    AbstractMemory::regStats();

    for (auto r : ranks) {
        r->regStats();
        for (auto d : r->devices) {
            d->regStats();
        }
    }

    readReqs
        .name(name() + ".readReqs")
        .desc("Number of read requests accepted");

    writeReqs
        .name(name() + ".writeReqs")
        .desc("Number of write requests accepted");

    pushReqs
        .name(name() + ".pushReqs")
        .desc("Number of push requests dispatched");

    packRdReqs
        .name(name() + ".packRdReqs")
        .desc("Number of packRd requests dispatched");

    packWrReqs
        .name(name() + ".packWrReqs")
        .desc("Number of packWr requests dispatched");

    packReqs
        .name(name() + ".packReqs")
        .desc("Number of pack requests dispatched")
        .precision(2);

    packReqs = packRdReqs + packWrReqs;

    perCoreReqs
        .name(name() + ".perCoreReqs")
        .desc("Number of reqs per core received by memctrl")
        .init(system()->numContexts());

    readBursts
        .name(name() + ".readBursts")
        .desc("Number of DRAM read bursts, "
              "including those serviced by the write queue");

    devReadBursts
        .name(name() + ".devReadBursts")
        .desc("Number of DRAM read bursts, "
                "including those serviced by the dev write queue");

    writeBursts
        .name(name() + ".writeBursts")
        .desc("Number of DRAM write bursts, "
              "including those merged in the write queue");

    servicedByWrQ
        .name(name() + ".servicedByWrQ")
        .desc("Number of DRAM read bursts serviced by the write queue");

    servicedByDevWrQ
        .name(name() + ".servicedByDevWrQ")
        .desc("Number of DRAM dev read bursts serviced by the dev write queue");

    mergedWrBursts
        .name(name() + ".mergedWrBursts")
        .desc("Number of DRAM write bursts merged with an existing one");

    neitherReadNorWrite
        .name(name() + ".neitherReadNorWriteReqs")
        .desc("Number of requests that are neither read nor write");

    perBankRdBursts
        .init(banksPerRank * ranksPerChannel)
        .name(name() + ".perBankRdBursts")
        .desc("Per bank write bursts");

    perBankWrBursts
        .init(banksPerRank * ranksPerChannel)
        .name(name() + ".perBankWrBursts")
        .desc("Per bank write bursts");

    avgRdQLen
        .name(name() + ".avgRdQLen")
        .desc("Average read queue length when enqueuing")
        .precision(2);

    avgWrQLen
        .name(name() + ".avgWrQLen")
        .desc("Average write queue length when enqueuing")
        .precision(2);

    rdRetry
        .name(name() + ".rdRetry")
        .desc("Total number of read retry due to read queue full in VMC mode");

    wrRetry
        .name(name() + ".wrRetry")
        .desc("Total number of write retry due to write queue full in VMC mode");

    totalRetry
        .name(name() + ".totalRetry")
        .desc("Total number of retry due to queue full in VMC mode")
        .precision(2);

    totalRetry = rdRetry + wrRetry;

    rdDispatchFail
        .name(name() + ".rdDispatchFail")
        .desc("Times of a read dispatch failure due to read queue full");

    wrDispatchFail
        .name(name() + ".wrDispatchFail")
        .desc("Times of a write dispatch failure due to write queue full");

    dispatchFail
        .name(name() + ".dispatchFail")
        .desc("Times of dispatch failure due to queue full")
        .precision(2);

    dispatchFail = rdDispatchFail + wrDispatchFail;

    totQLat
        .name(name() + ".totQLat")
        .desc("Total ticks spent queuing");

    totDevQLat
        .name(name() + ".totDevQLat")
        .desc("Total ticks spent queuing in dev queues");

    totBusLat
        .name(name() + ".totBusLat")
        .desc("Total ticks spent in databus transfers");

    totMemAccLat
        .name(name() + ".totMemAccLat")
        .desc("Total ticks spent from burst creation until serviced "
              "by the DRAM");

    avgQLat
        .name(name() + ".avgQLat")
        .desc("Average queueing delay per DRAM burst")
        .precision(2);

    avgQLat = totQLat / (readBursts - servicedByWrQ);

    avgDevQLat
        .name(name() + ".avgDevQLat")
        .desc("Average queueing delay per pack req")
        .precision(2);

    avgDevQLat = totDevQLat / (devReadBursts - servicedByDevWrQ);

    avgBusLat
        .name(name() + ".avgBusLat")
        .desc("Average bus latency per DRAM burst")
        .precision(2);

    avgBusLat = totBusLat / (readBursts - servicedByWrQ);

    avgMemAccLat
        .name(name() + ".avgMemAccLat")
        .desc("Average memory access latency per DRAM burst")
        .precision(2);

    avgMemAccLat = totMemAccLat / (readBursts - servicedByWrQ);

    numRdRetry
        .name(name() + ".numRdRetry")
        .desc("Number of times read queue was full causing retry");

    numWrRetry
        .name(name() + ".numWrRetry")
        .desc("Number of times write queue was full causing retry");

    readRowHits
        .name(name() + ".readRowHits")
        .desc("Number of row buffer hits during reads");

    writeRowHits
        .name(name() + ".writeRowHits")
        .desc("Number of row buffer hits during writes");

    readRowHitRate
        .name(name() + ".readRowHitRate")
        .desc("Row buffer hit rate for reads")
        .precision(2);

    readRowHitRate = (readRowHits / (readBursts - servicedByWrQ)) * 100;

    writeRowHitRate
        .name(name() + ".writeRowHitRate")
        .desc("Row buffer hit rate for writes")
        .precision(2);

    writeRowHitRate = (writeRowHits / (writeBursts - mergedWrBursts)) * 100;

    readPktSize
        .init(ceilLog2(burstSize) + 1)
        .name(name() + ".readPktSize")
        .desc("Read request sizes (log2)");

     writePktSize
        .init(ceilLog2(burstSize) + 1)
        .name(name() + ".writePktSize")
        .desc("Write request sizes (log2)");

     rdQLenPdf
        .init(readBufferSize * devicesPerRank)
        .name(name() + ".rdQLenPdf")
        .desc("What read queue length does an incoming req see");

     wrQLenPdf
        .init(writeBufferSize * devicesPerRank)
        .name(name() + ".wrQLenPdf")
        .desc("What write queue length does an incoming req see");

     devRdQLen
         .init(ranksPerChannel * devicesPerRank)
         .name(name() + ".devRdQLen")
         .desc("Accumulative dev read queue size per device");

     devWrQLen
         .init(ranksPerChannel * devicesPerRank)
         .name(name() + ".devWrQLen")
         .desc("Accumulative dev write queue size per device");

     avgDevRdQLen
        .name(name() + ".avgDevRdQLen")
        .desc("Average device read queue length")
        .precision(2);
     
     avgDevRdQLen = sum(devRdQLen) / (ranksPerChannel * devicesPerRank);

     avgDevWrQLen
        .name(name() + ".avgDevWrQLen")
        .desc("Average device write queue length")
        .precision(2);
     
     avgDevWrQLen = sum(devWrQLen) / (ranksPerChannel * devicesPerRank);

     bytesPerActivate
         .init(maxAccessesPerRow)
         .name(name() + ".bytesPerActivate")
         .desc("Bytes accessed per row activation")
         .flags(nozero);

     rdPerTurnAround
         .init(readBufferSize * devicesPerRank)
         .name(name() + ".rdPerTurnAround")
         .desc("Reads before turning the bus around for writes")
         .flags(nozero);

     pckRdLength
         .init(devicesPerRank * devicesPerRank)
         .name(name() + ".pckRdLength")
         .desc("# of read pkts packed once")
         .flags(nozero);

     pckWrLength
         .init(devicesPerRank)
         .name(name() + ".pckWrLength")
         .desc("# of write pkts packed once")
         .flags(nozero);

     pckLength
         .init(devicesPerRank)
         .name(name() + ".pckLength")
         .desc("# of pkts packed once")
         .flags(nozero);

     totPackRdDrainTime
         .name(name() + ".totPackRdDrainTime")
         .desc("Total time spent draining pack rd reqs");

     totPackWrDrainTime
         .name(name() + ".totPackWrDrainTime")
         .desc("Total time spent draining pack wr reqs");

     wrPerTurnAround
         .init(writeBufferSize)
         .name(name() + ".wrPerTurnAround")
         .desc("Writes before turning the bus around for reads")
         .flags(nozero);

    bytesReadDRAM
        .name(name() + ".bytesReadDRAM")
        .desc("Total number of bytes read from DRAM");

    bytesReadWrQ
        .name(name() + ".bytesReadWrQ")
        .desc("Total number of bytes read from write queue");

    bytesWritten
        .name(name() + ".bytesWritten")
        .desc("Total number of bytes written to DRAM");

    bytesReadSys
        .name(name() + ".bytesReadSys")
        .desc("Total read bytes from the system interface side");

    bytesWrittenSys
        .name(name() + ".bytesWrittenSys")
        .desc("Total written bytes from the system interface side");

    avgRdBW
        .name(name() + ".avgRdBW")
        .desc("Average DRAM read bandwidth in MiByte/s")
        .precision(2);

    avgRdBW = (bytesReadDRAM / (1024*1024)) / simSeconds;

    avgWrBW
        .name(name() + ".avgWrBW")
        .desc("Average achieved write bandwidth in MiByte/s")
        .precision(2);

    avgWrBW = (bytesWritten / (1024*1024)) / simSeconds;

    avgRdBWSys
        .name(name() + ".avgRdBWSys")
        .desc("Average system read bandwidth in MiByte/s")
        .precision(2);

    avgRdBWSys = (bytesReadSys / (1024*1024)) / simSeconds;

    avgWrBWSys
        .name(name() + ".avgWrBWSys")
        .desc("Average system write bandwidth in MiByte/s")
        .precision(2);

    avgWrBWSys = (bytesWrittenSys / (1024*1024)) / simSeconds;

    peakBW
        .name(name() + ".peakBW")
        .desc("Theoretical peak bandwidth in MiByte/s")
        .precision(2);

    peakBW = (SimClock::Frequency / tBURST) * burstSize / (1024*1024);

    busUtil
        .name(name() + ".busUtil")
        .desc("Data bus utilization in percentage")
        .precision(2);
    busUtil = (avgRdBW + avgWrBW) / peakBW * 100;

    totGap
        .name(name() + ".totGap")
        .desc("Total gap between requests");

    totPackGap
        .name(name() + ".totPackGap")
        .desc("Total gap between pack requests");

    totPushGap
        .name(name() + ".totPushGap")
        .desc("Total gap between address push");

    totPushEventGap
        .name(name() + ".totPushEventGap")
        .desc("Total gap between push events");

    totPushDelay
        .name(name() + ".totPushDelay")
        .desc("Total additional delays inserted due to push.");

    avgGap
        .name(name() + ".avgGap")
        .desc("Average gap between requests")
        .precision(2);

    avgGap = totGap / (readReqs + writeReqs);

    avgPackGap
        .name(name() + ".avgPackGap")
        .desc("Average gap between pack requests")
        .precision(2);

    avgPackGap = totPackGap / (packRdReqs + packWrReqs);

    avgPushGap
        .name(name() + ".avgPushGap")
        .desc("Average gap between address push")
        .precision(2);

    avgPushGap = totPushGap / (pushReqs);

    // Stats for DRAM Power calculation based on Micron datasheet
    busUtilRead
        .name(name() + ".busUtilRead")
        .desc("Data bus utilization in percentage for reads")
        .precision(2);

    busUtilRead = avgRdBW / peakBW * 100;

    busUtilWrite
        .name(name() + ".busUtilWrite")
        .desc("Data bus utilization in percentage for writes")
        .precision(2);

    busUtilWrite = avgWrBW / peakBW * 100;

    busTraffic
        .name(name() + ".busTraffic")
        .desc("Total traffic in GB trasfered over bus")
        .precision(2);

    busTraffic = (bytesReadDRAM + bytesWritten) / (1024 * 1024 * 1024);

    busRd2Wr
        .name(name() + ".busRd2Wr")
        .desc("Times of DBUS turn from Rd to Wr");

    busWr2Rd
        .name(name() + ".busWr2Rd")
        .desc("Times of DBUS turn from Wr to Rd");

    busTurnaround
        .name(name() + ".busTurnaround")
        .desc("Times of DBUS turnarounds")
        .precision(2);

    busTurnaround = busRd2Wr + busWr2Rd;

    pageHitRate
        .name(name() + ".pageHitRate")
        .desc("Row buffer hit rate, read and write combined")
        .precision(2);

    pageHitRate = (writeRowHits + readRowHits) /
        (writeBursts - mergedWrBursts + readBursts - servicedByWrQ) * 100;
}

void
DRAMCtrl::recvFunctional(PacketPtr pkt)
{
    // rely on the abstract memory
    functionalAccess(pkt);
}

BaseSlavePort&
DRAMCtrl::getSlavePort(const string &if_name, PortID idx)
{
    if (if_name != "port") {
        return MemObject::getSlavePort(if_name, idx);
    } else {
        return port;
    }
}

DrainState
DRAMCtrl::drain()
{
    DPRINTF(VMC, "DRAM_ctrl trying to drain: read %d, write %d, resp %d\n",
            readQueue.size(), writeQueue.size(), respQueue.size());
    // if there is anything in any of our internal queues, keep track
    // of that as well
    if (!(writeQueue.empty() && readQueue.empty() && respQueue.empty() &&
          allRanksDrained())) {

        DPRINTF(Drain, "DRAM controller not drained, write: %d, read: %d,"
                " resp: %d\n", writeQueue.size(), readQueue.size(),
                respQueue.size());

        // the only queue that is not drained automatically over time
        // is the write queue, thus kick things into action if needed
        if (!writeQueue.empty() && !nextReqEvent.scheduled()) {
            schedule(nextReqEvent, curTick());
        }

        // also need to kick off events to exit self-refresh
        for (auto r : ranks) {
            // force self-refresh exit, which in turn will issue auto-refresh
            if (r->pwrState == PWR_SREF) {
                DPRINTF(DRAM,"Rank%d: Forcing self-refresh wakeup in drain\n",
                        r->rank);
                r->scheduleWakeUpEvent(tXS);
            }
        }

        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

bool
DRAMCtrl::allRanksDrained() const
{
    // true until proven false
    bool all_ranks_drained = true;
    for (auto r : ranks) {
        // then verify that the power state is IDLE
        // ensuring all banks are closed and rank is not in a low power state
        // weil0ng: also mark as drained if in PWR_PRE_PDN state.
        all_ranks_drained = (r->inPwrIdleState() || r->pwrState == PWR_PRE_PDN) && all_ranks_drained;
    }
    return all_ranks_drained;
}

void
DRAMCtrl::drainResume()
{
    if (!isTimingMode && system()->isTimingMode()) {
        // if we switched to timing mode, kick things into action,
        // and behave as if we restored from a checkpoint
        startup();
    } else if (isTimingMode && !system()->isTimingMode()) {
        // if we switch from timing mode, stop the refresh events to
        // not cause issues with KVM
        for (auto r : ranks) {
            r->suspend();
        }
    }

    // update the mode
    isTimingMode = system()->isTimingMode();
}

DRAMCtrl::MemoryPort::MemoryPort(const std::string& name, DRAMCtrl& _memory)
    : QueuedSlavePort(name, &_memory, queue), queue(_memory, *this),
      memory(_memory)
{ }

AddrRangeList
DRAMCtrl::MemoryPort::getAddrRanges() const
{
    AddrRangeList ranges;
    ranges.push_back(memory.getAddrRange());
    return ranges;
}

void
DRAMCtrl::MemoryPort::recvFunctional(PacketPtr pkt)
{
    pkt->pushLabel(memory.name());

    if (!queue.checkFunctional(pkt)) {
        // Default implementation of SimpleTimingPort::recvFunctional()
        // calls recvAtomic() and throws away the latency; we can save a
        // little here by just not calculating the latency.
        memory.recvFunctional(pkt);
    }

    pkt->popLabel();
}

Tick
DRAMCtrl::MemoryPort::recvAtomic(PacketPtr pkt)
{
    return memory.recvAtomic(pkt);
}

bool
DRAMCtrl::MemoryPort::recvTimingReq(PacketPtr pkt)
{
    // pass it to the memory controller
    return memory.recvTimingReq(pkt);
}

DRAMCtrl*
DRAMCtrlParams::create()
{
    return new DRAMCtrl(this);
}
