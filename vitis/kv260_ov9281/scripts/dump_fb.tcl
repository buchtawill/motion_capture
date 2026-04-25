connect 
targets -set -filter {name =~ "Cortex-A53 #0"}
mrd -bin -file ./mem_dump.bin 0x38800 256000
