# Copyright (c) 2012-2013 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2006-2008 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Steve Reinhardt

# Simple test script
#
# "m5 test.py"

import optparse
import sys
import copy

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../common')
addToPath('../ruby')
addToPath('../topologies')

import Options
import Ruby
import Simulation
import CacheConfig
import MemConfig
from Caches import *
from cpu2000 import *

binary_dir = '../SPEC_CPU2006/{0}/build/build_base_i386-m32-gcc42-nn.0000/'
data_dir = '../SPEC_CPU2006/{0}/data/test/input/'

name = "401.bzip2"
bzip2 = LiveProcess()
bzip2.executable =  binary_dir.format( name ) + 'bzip2'
data= '../SPEC_CPU2006/401.bzip2/data/ref/input/liberty.jpg'
bzip2.cmd = [bzip2.executable] + [data, '30']
bzip2.output = 'bzip2.ref.liberty.out'


name = '429.mcf'
mcf = LiveProcess()
mcf.executable =  binary_dir.format( name ) + 'mcf'
data='../SPEC_CPU2006/429.mcf/data/ref/input/inp.in'
mcf.cmd = [mcf.executable] + [data]
mcf.output = 'inp.out'

# lbm 3000 reference.dat 0 0 100_100_130_ldc.of > lbm.ref.out 2> lbm.ref.err 
name = '470.lbm'
lbm = LiveProcess()
lbm.executable =  binary_dir.format( name ) + 'lbm'
lbm.cmd = [lbm.executable]+['3000', 'reference.dat', '0', '0', '../SPEC_CPU2006/470.lbm/data/ref/input/100_100_130_ldc.of']
lbm.output = 'lbm.out'

name = '433.milc'
milc=LiveProcess()
milc.executable = binary_dir.format( name ) +'milc'
stdin = '../SPEC_CPU2006/433.milc/data/ref/input/su3imp.in'
milc.cmd = [milc.executable]
milc.input=stdin
milc.output='su3imp.out'

name = '437.leslie3d'
leslie3d = LiveProcess()
leslie3d.executable = binary_dir.format( name ) + 'leslie3d'
stdin = '../SPEC_CPU2006/437.leslie3d/data/ref/input/leslie3d.in'
leslie3d.cmd = [leslie3d.executable]
leslie3d.input=stdin
leslie3d.output='leslie3d.ref.out'




name = '459.GemsFDTD'
GemsFDTD=LiveProcess()
GemsFDTD.executable =  binary_dir.format( name ) +'GemsFDTD'
GemsFDTD.cmd = [GemsFDTD.executable]
GemsFDTD.output = 'test.log'

name = '437.leslie3d'
leslie3d=LiveProcess()
leslie3d.executable =  binary_dir.format( name ) +'leslie3d'
stdin = data_dir.format( name ) +'leslie3d.in'
leslie3d.cmd = [leslie3d.executable]
leslie3d.input=stdin
leslie3d.output='leslie3d.stdout'

name = '473.astar'
astar=LiveProcess()
astar.executable =  binary_dir.format( name ) +'astar'
astar.cmd = [astar.executable]+['rivers.cfg']
astar.output = 'rivers.out'

name = '450.soplex'
soplex = LiveProcess()
soplex.executable =  binary_dir.format( name ) +'soplex'
data = '../SPEC_CPU2006/450.soplex/data/ref/input/pds-50.mps'
soplex.cmd = [soplex.executable]+['-s1 -e -m45000',data]
soplex.output = 'soplex.ref.pds-50.mps.out'

name = '434.zeusmp'
zeusmp=LiveProcess()
zeusmp.executable = binary_dir.format( name ) +'zeusmp'
zeusmp.cmd = [zeusmp.executable]
zeusmp.output = 'zeusmp.stdout'

name = '471.omnetpp'
omnetpp=LiveProcess()
omnetpp.executable = binary_dir.format( name ) +'omnetpp'
#data = '../SPEC_CPU2006/471.omnetpp/data/ref/input/omnetpp.ini'
omnetpp.cmd = [omnetpp.executable]+['omnetpp.ini']
omnetpp.output = 'omnetpp.log'

name = '410.bwaves'
bwaves = LiveProcess()
bwaves.executable =  binary_dir.format( name ) +'bwaves'
bwaves.cmd = [bwaves.executable]

name = '436.cactusADM'
cactusADM = LiveProcess()
cactusADM.executable =  binary_dir.format( name ) +'cactusADM'
data= data_dir.format( name ) +'benchADM.par'
cactusADM.cmd = [cactusADM.executable] + [data]
cactusADM.output = 'benchADM.out'

#482.sphinx
name = '482.sphinx3'
sphinx3 = LiveProcess()
sphinx3.executable = binary_dir.format( name ) + 'sphinx_livepretend'
sphinx3.cmd = [ sphinx3.executable ] + [ 'ctlfile', '.', 'args.an4' ]
sphinx3.output = 'an4.out'

#xalancbmk
name = "483.xalancbmk"
xalancbmk = LiveProcess()
xalancbmk.executable = binary_dir.format( name ) + 'Xalan'
xalancbmk.cmd = [ xalancbmk.executable ] + ['-v ../SPEC_CPU2006/483.xalancbmk/data/ref/input/t5.xml ../SPEC_CPU2006/483.xalancbmk/data/ref/input/xalanc.xsl']
xalancbmk.output = 'xalancbmk.ref.out'

#hmmer
name = "456.hmmer"
hmmer = LiveProcess()
hmmer.executable = binary_dir.format( name ) + 'hmmer'
hmmer.cmd = [ hmmer.executable ] + ['../SPEC_CPU2006/456.hmmer/data/ref/input/nph3.hmm','../SPEC_CPU2006/456.hmmer/data/ref/input/swiss41']
hmmer.output = 'hmmer.ref.nph3.out'

#libquantum
name = "462.libquantum"
libquantum = LiveProcess()
libquantum.executable = binary_dir.format( name ) + 'libquantum'
libquantum.cmd = [ libquantum.executable ] + ['1397','8']
libquantum.output = 'libquantum.ref.out'

#454.calculix
name = "454.calculix"
calculix=LiveProcess()
calculix.executable =  binary_dir.format( name ) + 'calculix'
calculix.cmd = [calculix.executable] + ['-i', 'hyperviscoplastic']
calculix.output = 'hyperviscoplastic.out'


def get_processes(options):
    """Interprets provided options and returns a list of processes"""

    multiprocesses = []
    inputs = []
    outputs = []
    errouts = []
    pargs = []

    workloads = options.cmd.split(';')
    if workloads[ 0 ] == "":
        workloads = []
    if options.input != "":
        inputs = options.input.split(';')
    if options.output != "":
        outputs = options.output.split(';')
    if options.errout != "":
        errouts = options.errout.split(';')
    if options.options != "":
        pargs = options.options.split(';')

    idx = 0
    for wrkld in workloads:
        process = LiveProcess()
        process.executable = wrkld

        if len(pargs) > idx:
            process.cmd = [wrkld] + pargs[idx].split()
        else:
            process.cmd = [wrkld]

        if len(inputs) > idx:
            process.input = inputs[idx]
        if len(outputs) > idx:
            process.output = outputs[idx]
        if len(errouts) > idx:
            process.errout = errouts[idx]

        multiprocesses.append(process)
        idx += 1

    if options.benchmark != None:
        benchmarks = options.benchmark.split( ';' )
        print '{}: {}'.format(options.benchmark, benchmarks)
    else:
        benchmarks = []
    for benchmark in benchmarks:
        if benchmark == 'perlbench':
            process = perlbench
        elif benchmark == 'bzip2':
            process = bzip2
        elif benchmark == 'gcc':
            process = gcc
        elif benchmark == 'bwaves':
            process = bwaves
        elif benchmark == 'gamess':
            process = gamess
        elif benchmark == 'mcf':
            process = mcf
        elif benchmark == 'milc':
            process = milc
        elif benchmark == 'zeusmp':
            process = zeusmp
        elif benchmark == 'gromacs':
            process = gromacs
        elif benchmark == 'cactusADM':
            process = cactusADM
        elif benchmark == 'leslie3d':
            process = leslie3d
        elif benchmark == 'namd':
            process = namd
        elif benchmark == 'gobmk':
            process = gobmk;
        elif benchmark == 'dealII':
            process = dealII
        elif benchmark == 'soplex':
            process = soplex
        elif benchmark == 'povray':
            process = povray
        elif benchmark == 'calculix':
            process = calculix
        elif benchmark == 'hmmer':
            process = hmmer
        elif benchmark == 'sjeng':
            process = sjeng
        elif benchmark == 'GemsFDTD':
            process = GemsFDTD
        elif benchmark == 'libquantum':
            process = libquantum
        elif benchmark == 'h264ref':
            process = h264ref
        elif benchmark == 'tonto':
            process = tonto
        elif benchmark == 'lbm':
            process = lbm
        elif benchmark == 'omnetpp':
            process = omnetpp
        elif benchmark == 'astar':
            process = astar
        elif benchmark == 'wrf':
            process = wrf
        elif benchmark == 'sphinx3':
            process = sphinx3
        elif benchmark == 'xalancbmk':
            process = xalancbmk
        elif benchmark == 'specrand_i':
            process = specrand_i
        elif benchmark == 'specrand_f':
            process = specrand_f
        new_process = LiveProcess()
        new_process.executable = process.executable
        new_process.cmd = process.cmd
        new_process.input=process.input
        new_process.output=process.output.replace( '.', str(idx) + '.' )
        multiprocesses.append(new_process)
        idx += 1

    if options.smt:
        assert(options.cpu_type == "detailed" or options.cpu_type == "inorder")
        return multiprocesses, idx
    else:
        return multiprocesses, 1


parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addSEOptions(parser)
m5.disableAllListeners()

if '--ruby' in sys.argv:
    Ruby.define_options(parser)

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

multiprocesses = []
numThreads = 1

if options.bench:
    apps = options.bench.split("-")
    if len(apps) != options.num_cpus:
        print "number of benchmarks not equal to set num_cpus!"
        sys.exit(1)

    for app in apps:
        try:
            if buildEnv['TARGET_ISA'] == 'alpha':
                exec("workload = %s('alpha', 'tru64', 'ref')" % app)
            else:
                exec("workload = %s(buildEnv['TARGET_ISA'], 'linux', 'ref')" % app)
            multiprocesses.append(workload.makeLiveProcess())
        except:
            print >>sys.stderr, "Unable to find workload for %s: %s" % (buildEnv['TARGET_ISA'], app)
            sys.exit(1)
elif options.cmd or options.benchmark:
    multiprocesses, numThreads = get_processes(options)
else:
    print >> sys.stderr, "No workload specified. Exiting!\n"
    sys.exit(1)


(CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)
CPUClass.numThreads = numThreads

MemClass = Simulation.setMemClass(options)

# Check -- do not allow SMT with multiple CPUs
if options.smt and options.num_cpus > 1:
    fatal("You cannot use SMT with multiple CPUs!")

np = options.num_cpus
system = System(cpu = [CPUClass(cpu_id=i) for i in xrange(np)],
                mem_mode = test_mem_mode,
                mem_ranges = [AddrRange(options.mem_size)],
                cache_line_size = options.cacheline_size)

# Create a top-level voltage domain
system.voltage_domain = VoltageDomain(voltage = options.sys_voltage)

# Create a source clock for the system and set the clock period
system.clk_domain = SrcClockDomain(clock =  options.sys_clock,
                                   voltage_domain = system.voltage_domain)

# Create a CPU voltage domain
system.cpu_voltage_domain = VoltageDomain()

# Create a separate clock domain for the CPUs
system.cpu_clk_domain = SrcClockDomain(clock = options.cpu_clock,
                                       voltage_domain =
                                       system.cpu_voltage_domain)

# All cpus belong to a common cpu_clk_domain, therefore running at a common
# frequency.
for cpu in system.cpu:
    cpu.clk_domain = system.cpu_clk_domain

# Sanity check
if options.fastmem:
    if CPUClass != AtomicSimpleCPU:
        fatal("Fastmem can only be used with atomic CPU!")
    if (options.caches or options.l2cache):
        fatal("You cannot use fastmem in combination with caches!")

if options.simpoint_profile:
    if not options.fastmem:
        # Atomic CPU checked with fastmem option already
        fatal("SimPoint generation should be done with atomic cpu and fastmem")
    if np > 1:
        fatal("SimPoint generation not supported with more than one CPUs")

for i in xrange(np):
    if options.smt:
        system.cpu[i].workload = multiprocesses
    elif len(multiprocesses) == 1:
        system.cpu[i].workload = multiprocesses[0]
    else:
        system.cpu[i].workload = multiprocesses[i]

    if options.fastmem:
        system.cpu[i].fastmem = True

    if options.simpoint_profile:
        system.cpu[i].simpoint_profile = True
        system.cpu[i].simpoint_interval = options.simpoint_interval

    if options.checker:
        system.cpu[i].addCheckerCpu()

    system.cpu[i].createThreads()

if options.ruby:
    if not (options.cpu_type == "detailed" or options.cpu_type == "timing"):
        print >> sys.stderr, "Ruby requires TimingSimpleCPU or O3CPU!!"
        sys.exit(1)

    # Set the option for physmem so that it is not allocated any space
    system.physmem = MemClass(range=AddrRange(options.mem_size),
                              null = True)

    options.use_map = True
    Ruby.create_system(options, False, system)
    assert(options.num_cpus == len(system.ruby._cpu_ports))

    for i in xrange(np):
        ruby_port = system.ruby._cpu_ports[i]

        # Create the interrupt controller and connect its ports to Ruby
        # Note that the interrupt controller is always present but only
        # in x86 does it have message ports that need to be connected
        system.cpu[i].createInterruptController()

        # Connect the cpu's cache ports to Ruby
        system.cpu[i].icache_port = ruby_port.slave
        system.cpu[i].dcache_port = ruby_port.slave
        if buildEnv['TARGET_ISA'] == 'x86':
            system.cpu[i].interrupts.pio = ruby_port.master
            system.cpu[i].interrupts.int_master = ruby_port.slave
            system.cpu[i].interrupts.int_slave = ruby_port.master
            system.cpu[i].itb.walker.port = ruby_port.slave
            system.cpu[i].dtb.walker.port = ruby_port.slave
else:
    #system.membus = CoherentBus(width = 64)
    MemClass = Simulation.setMemClass(options)
    system.membus = SystemXBar()
    system.system_port = system.membus.slave
    CacheConfig.config_cache(options, system)
    MemConfig.config_mem(options, system)

root = Root(full_system = False, system = system)
Simulation.run(options, root, system, FutureClass)
