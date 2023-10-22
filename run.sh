#!/usr/bin/zsh

# L2C no  + LLC no  : IPC=0.4368
# L2C no  + LLC spp : IPC=0.3786
# L2C spp + LLC no  : IPC=0.4133
# LLC no  + LLC next_line : IPC=0.3917

warmup_instructions=$(expr 3000 \* 10000)
simulation_instructions=$(expr 5000 \* 10000)
trace=./traces/429.mcf-22B.champsimtrace.xz


time ./bin/champsim --warmup-instructions $warmup_instructions --simulation-instructions $simulation_instructions $trace 

