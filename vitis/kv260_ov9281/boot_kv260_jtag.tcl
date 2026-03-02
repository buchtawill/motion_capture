set vitis_workspace_dir [file dirname [info script]]
set vitis_workspace_dir "$vitis_workspace_dir/workspace"

set project_name  "kv260_ov9281"
set platform_name $project_name\_platform
set app_name      $project_name\_app

proc boot_jtag { } {
    ############################
    # Switch to JTAG boot mode #
    ############################
    puts "INFO \[boot_kv260_jtag.tcl\] Configuring JTAG boot mode"
    targets -set -filter {name =~ "PSU"}
    # update multiboot to ZERO
    mwr 0xffca0010 0x0
    # change boot mode to JTAG
    mwr 0xff5e0200 0x0100
    # reset
    puts "INFO \[boot_kv260_jtag.tcl\] Resetting system"
    rst -system
}

puts "INFO \[boot_kv260_jtag.tcl\] Connecting to hardware"
connect
puts "INFO \[boot_kv260_jtag.tcl\] Applying JTAG boot sequence"
boot_jtag

puts "INFO \[boot_kv260_jtag.tcl\] Waiting for boot mode change"
after 2000
targets -set -filter {name =~ "PSU"}
puts "INFO \[boot_kv260_jtag.tcl\] Loading FPGA bitstream"
fpga "$vitis_workspace_dir/$platform_name/export/$platform_name/hw/sdt/kv260_ov9281_proj.bit"
mwr 0xffca0038 0x1FF

# Download pmufw.elf
puts "INFO \[boot_kv260_jtag.tcl\] Downloading PMU firmware"
targets -set -filter {name =~ "MicroBlaze PMU"}
after 500
dow $vitis_workspace_dir/$platform_name/export/$platform_name/sw/qemu/pmufw.elf
con
after 500

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