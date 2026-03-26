# Dump framebuffer memory from DDR via JTAG

connect
targets -set -filter {name =~ "Cortex-A9 MPCore #0"}
mrd -bin -file ./mem_dump.bin 0x36800 256000
