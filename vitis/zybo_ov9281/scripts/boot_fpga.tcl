# Program bitstream to PL and initialize PS — does not download or run ELF

set script_dir [file dirname [info script]]
set script_dir [file normalize $script_dir]

set project_name  "zybo_ov9281"
set app_name      ${project_name}_app

set workspace_dir  [file normalize "$script_dir/../workspace"]
set bitstream_path [file normalize "$script_dir/../../../vivado/$project_name/${project_name}_proj/${project_name}_proj.bit"]
set ps7_init_path  [file normalize "$workspace_dir/$app_name/_ide/psinit/ps7_init.tcl"]

puts "INFO \[boot_fpga.tcl\] Connecting to hardware"
connect

puts "INFO \[boot_fpga.tcl\] Programming PL with bitstream: $bitstream_path"
targets -set -filter {name =~ "xc7z020"}
fpga -file $bitstream_path

# Zynq fsbl
puts "INFO \[boot_fpga.tcl\] Downloading zynq fsbl and waiting 5 seconds"
dow "$workspace_dir/$platform_name/zynq_fsbl/build/fsbl.elf"
con
after 5000
stop

puts "INFO \[boot_fpga.tcl\] Selecting Cortex-A9 Core 0 and initializing PS"
targets -set -filter {name =~ "ARM Cortex-A9 MPCore #0"}
rst -processor -clear-registers
source $ps7_init_path
ps7_init

puts "INFO \[boot_fpga.tcl\] PL programmed and PS initialized"
