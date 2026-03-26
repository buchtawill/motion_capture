# Program bitstream to PL and run application ELF on Cortex-A9

set script_dir [file dirname [info script]]
set script_dir [file normalize $script_dir]

set project_name  "zybo_ov9281"
set platform_name ${project_name}_platform
set app_name      ${project_name}_app

set workspace_dir  [file normalize "$script_dir/../workspace"]
set bitstream_path [file normalize "$script_dir/../../../vivado/$project_name/${project_name}_proj/${project_name}_proj.bit"]
set ps7_init_path  [file normalize "$workspace_dir/$app_name/_ide/psinit/ps7_init.tcl"]
set elf_path       [file normalize "$workspace_dir/$app_name/build/$app_name.elf"]

puts "INFO \[program_and_boot.tcl\] Connecting to hardware"
connect

puts "INFO \[program_and_boot.tcl\] Programming PL with bitstream: $bitstream_path"
targets -set -filter {name =~ "xc7z020"}
fpga -file $bitstream_path

puts "INFO \[program_and_boot.tcl\] Selecting Cortex-A9 Core 0"
targets -set -filter {name =~ "ARM Cortex-A9 MPCore #0"}
rst -processor -clear-registers

puts "INFO \[program_and_boot.tcl\] Running ps7_init"
source $ps7_init_path
ps7_init

puts "INFO \[program_and_boot.tcl\] Downloading application ELF"
dow $elf_path
after 500
con
puts "INFO \[program_and_boot.tcl\] Boot sequence complete! Application running"
