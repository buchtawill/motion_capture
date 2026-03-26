# Download and run application ELF — assumes bitstream already loaded

set script_dir [file dirname [info script]]
set script_dir [file normalize $script_dir]

set project_name  "zybo_ov9281"
set app_name      ${project_name}_app

set workspace_dir [file normalize "$script_dir/../workspace"]
set ps7_init_path [file normalize "$workspace_dir/$app_name/_ide/psinit/ps7_init.tcl"]
set elf_path      [file normalize "$workspace_dir/$app_name/build/$app_name.elf"]

puts "INFO \[boot_fw.tcl\] Connecting to hardware"
connect

puts "INFO \[boot_fw.tcl\] Selecting Cortex-A9 Core 0"
targets -set -filter {name =~ "ARM Cortex-A9 MPCore #0"}
rst -processor -clear-registers

puts "INFO \[boot_fw.tcl\] Running ps7_init"
source $ps7_init_path
ps7_init

puts "INFO \[boot_fw.tcl\] Downloading application ELF"
dow $elf_path
after 500
con
puts "INFO \[boot_fw.tcl\] Boot sequence complete! Application running"
