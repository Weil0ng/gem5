This version treats address pushing as a normal dram write pkt.

Each pack pkt A has a leading push pkt B which has all the addresses needed for A.
B is then treated as a normal dram write pkt.

Perfomrace sucks because, in this way, we have a shit ton of dram bus turnaround (tWTR).
