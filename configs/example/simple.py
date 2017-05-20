import optparse
import sys
import os

import m5
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../')

from common import Options
from common import Simulation

parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addAccOptions(parser)
(options, args) = parser.parse_args()

def setup_workload(system):
    process = Process()
    process.cmd = ['/home/weilong/stream/stream_c.exe']
    system.cpu[0].workload = process
    system.cpu[0].createThreads()

def cmd_print(s):
    print '==================\n{}\n=================='.format(s)

def sanity_check(options):
    if options.use_graph_accelerator:
        assert (options.cpu_type == 'TimingSimpleCPU' or
                options.cpu_type == 'AtomicSimpleCPU'), (
                        'graph_accelerator can only used with '
                        'TimingSimpleCPU or AtomicSimpleCPU')

sanity_check(options)

# Get the cpu class and system memory mode. Note that if mem_mode is atomic,
# system.mem_ctrl has no effect?
(CPUClass, mem_mode, FutureClass) = Simulation.setCPUClass(options)

if options.use_graph_accelerator:

# Always use single-threaded core.
CPUClass.numThreads = 1
np = options.num_cpus
system = System(cpu=[CPUClass(cpu_id=i) for i in xrange(np)],
                mem_mode=mem_mode,
                mem_ranges=[AddrRange(options.mem_size)],
                cache_line_size=options.cacheline_size)

system_info = 'System:\n\tnum_core: {}\n\tcpu: {}\n\tmem_mode: {}'.format(
        np, options.cpu_type, system.mem_mode)
cmd_print(system_info)

system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '2GHz'
system.clk_domain.voltage_domain = VoltageDomain()

system.membus = SystemXBar()

for i in xrange(np):
    cpu = system.cpu[i]
    cpu.icache_port = system.membus.slave
    cpu.dcache_port = system.membus.slave

    cpu.createInterruptController()
    cpu.interrupts[0].pio = system.membus.master
    cpu.interrupts[0].int_master = system.membus.slave
    cpu.interrupts[0].int_slave = system.membus.master

system.system_port = system.membus.slave

system.mem_ctrl = DDR4_2400_8x8()
system.mem_ctrl.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.master

setup_workload(system)

root = Root(full_system=False, system=system)
m5.instantiate()

print 'Begin simulation...'
exit_event = m5.simulate()
print 'Exiting @ tick {} because {}'.format(m5.curTick(), exit_event.getCause())
