# riscVProcessor
Roaa Bahaa 900203054
Seifeldin Elshabshiri

## Single cycle approach assumptions
1. Big Endian approach to byte based data memory, for example if we have a lw instruction: lw x5, 0(x0) and the memory contents are mem[0] = 17, mem[1] = 0, mem[2] = 0, mem[3] = 0, then the data comming out of the memory will be ordered as follows {mem[3], mem[2], mem[1], mem[0]} and the register x5 will contain the decimal value of 17

## Instructions Not Supported
Fence ecall and ebreak are not yet supported in this milesone and will be implemented later

## Errors
Currently the only error discovered was that the i type instructions don't account for negative numbers, for example if we do addi x3, x0, -3, then 3 will be stored in 3 and not -3
