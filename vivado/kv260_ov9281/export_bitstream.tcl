# Run this script from the vivado directory. NOT from the project directory.
# This script will open the project, validate the block design, run synthesis and implementation, and generate the bitstream.
# The hardware platform will be saved to the project directory.
# Run with the following command: vivado -mode batch -source example_script.tcl
# Author: Will Buchta Dec 2024

set project_name "kv260_ov9281_proj"
set project_dir "./$project_name"
set bitstream_dir "./bitstreams"

# Lower for RAM constrained machines
set max_jobs 4 

open_project "$project_dir/$project_name.xpr"

open_bd_design "$project_dir/$project_name.srcs/sources_1/bd/design_1/design_1.bd"
if {[catch {save_bd_design} result]} {
    puts "Error saving block design: $result"
    exit 1
}
if {[catch {validate_bd_design -force} result]} {
    puts "Error validating block design: $result"
    exit 1
}
if {[catch {save_bd_design} result]} {
    puts "Error saving block design: $result"
    exit 1
}

make_wrapper -files [get_files "$project_dir/$project_name.srcs/sources_1/bd/design_1/design_1.bd"] -top
update_compile_order -fileset sources_1

# Run synthesis
reset_run synth_1
launch_runs synth_1 -jobs $max_jobs
if {[catch {wait_on_run synth_1} result]} {
    puts "Error during synthesis: $result"
    exit 1
}

reset_run impl_1
launch_runs impl_1 -to_step write_bitstream -jobs $max_jobs
if {[catch {wait_on_run impl_1} result]} {
    puts "Error during implementation: $result"
    exit 1
}

if {[catch {write_hw_platform -fixed -include_bit -force -file "$project_dir/$project_name.xsa"} result]} {
    puts "Error writing hardware platform: $result"
    exit 1
}

puts "Opening impl1 run"
open_run impl_1
puts "opened impl1 run"

# Write the bitstream and export it to ./bitstreams
write_bitstream -force "$project_dir/$project_name.bit"

set xsa_path "$project_dir/${project_name}.xsa"
if {[catch {write_hw_platform -fixed -include_bit -force -file $xsa_path} result]} {
    puts "Error writing XSA: $result"
    exit 1
}
puts "Exported XSA to: $xsa_path"

close_project
exit 0