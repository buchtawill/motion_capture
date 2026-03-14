# Run application elf on core

set vitis_workspace_dir [file dirname [info script]]
set vitis_workspace_dir "$vitis_workspace_dir/../workspace"

set project_name  "kv260_ov9281"
set platform_name $project_name\_platform
set app_name      $project_name\_app

puts "INFO \[boot_kv260_jtag.tcl\] Connecting to hardware"
connect

# Select A53 Core 0
puts "INFO \[boot_kv260_jtag.tcl\] Selecting A53 Core 0 and downloading FSBL"
targets -set -filter {name =~ "Cortex-A53 #0"}
rst -processor -clear-registers
dow $vitis_workspace_dir/$platform_name/export/$platform_name/sw/boot/fsbl.elf
con
puts "INFO \[boot_kv260_jtag.tcl\] Waiting 5 seconds for FSBL to complete"
after 5000
stop

puts "INFO \[boot_kv260_jtag.tcl\] Downloading application elf"
dow $vitis_workspace_dir/$app_name/build/$app_name.elf
after 500
con
puts "INFO \[boot_kv260_jtag.tcl\] Boot sequence complete! Application running"