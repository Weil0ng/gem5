time build/X86/gem5.opt --debug-flags=WorkItems configs/example/spec.py -n 1 --cpu-type=AtomicSimpleCPU --mem-type=DDR4_2400_x64 --mem-size=8GB --benchmark=ssca2 --work-begin-checkpoint-count=1 --work-end-exit-count=1 --checkpoint-dir=ckpts