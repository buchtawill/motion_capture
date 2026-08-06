# Open the freshly-generated project, run synthesis then implementation
# (place + route + timing), and report utilization/timing. No bitstream.
set project_dir "kv260_ov9281_proj"
set project_name "kv260_ov9281_proj"
open_project "$project_dir/$project_name.xpr"

set max_jobs 8

# ---- Confirm the mocap_wrapper instance is sized 64/64 -------------------------
puts "==== mocap_wrapper instance parameters ===="
catch {
    open_bd_design [get_files *.bd]
    puts "MAX_BLOBS        = [get_property CONFIG.MAX_BLOBS        [get_bd_cells /mocap_wrapper_0]]"
    puts "MAX_RUNS_PER_ROW = [get_property CONFIG.MAX_RUNS_PER_ROW [get_bd_cells /mocap_wrapper_0]]"
    close_bd_design [current_bd_design]
}

# ---- Synthesis ---------------------------------------------------------------
reset_run synth_1
launch_runs synth_1 -jobs $max_jobs
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    error "SYNTHESIS FAILED"
}
puts "==== SYNTHESIS COMPLETE ===="

# ---- Implementation (place + route, no bitstream) ----------------------------
reset_run impl_1
launch_runs impl_1 -to_step route_design -jobs $max_jobs
wait_on_run impl_1
set impl_progress [get_property PROGRESS [get_runs impl_1]]
puts "==== IMPL PROGRESS: $impl_progress ===="

open_run impl_1
puts "==== POST-ROUTE UTILIZATION ===="
report_utilization
puts "==== POST-ROUTE TIMING SUMMARY ===="
report_timing_summary -max_paths 1 -report_unconstrained
puts "==== WNS ===="
set wns [get_property STATS.WNS [get_runs impl_1]]
puts "WNS = $wns"
puts "==== BUILD DONE ===="
