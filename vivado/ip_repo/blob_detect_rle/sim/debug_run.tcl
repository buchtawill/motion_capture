log_wave -r /tb_blob_detect_rle/*

# Run until first few rows are processed
run 100 us
puts "=== DEBUG at 100us ==="
puts "top state: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/state]"
puts "re state: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_run_ext/state]"
puts "re pix_row: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_run_ext/pix_row]"
puts "re pix_col: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_run_ext/pix_col]"
puts "re run_valid: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_run_ext/run_valid]"
puts "re run_ready: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/re_run_ready]"
puts "re enable: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/re_enable]"
puts "re s_valid: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_run_ext/s_valid]"
puts "re s_ready: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_run_ext/s_ready]"
puts "rm state: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_row_merger/state]"
puts "rm enable: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/rm_enable]"
puts "rm next_blob_id: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_row_merger/next_blob_id]"
puts "rm prev_count: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_row_merger/prev_count]"
puts "rm curr_count: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_row_merger/curr_count]"
puts "rm last_seen_row: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_row_merger/last_seen_row]"
puts "rm overflow: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_row_merger/overflow]"
puts "bt state: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_blob_table/state]"

run 400 us
puts "=== DEBUG at 500us ==="
puts "top state: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/state]"
puts "re pix_row: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_run_ext/pix_row]"
puts "rm next_blob_id: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_row_merger/next_blob_id]"
puts "rm prev_count: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_row_merger/prev_count]"
puts "rm curr_count: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_row_merger/curr_count]"
puts "rm last_seen_row: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_row_merger/last_seen_row]"
puts "rm overflow: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_row_merger/overflow]"
puts "bt blob_count_r0: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_blob_table/blob_count_r\[0\]]"
puts "bt blob_count_r1: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_blob_table/blob_count_r\[1\]]"

run all
puts "=== DEBUG at end ==="
puts "bt state: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_blob_table/state]"
puts "bt flat_out_id: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_blob_table/flat_out_id]"
puts "bt blob_count: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_blob_table/blob_count]"
puts "bt blob_count_r0: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_blob_table/blob_count_r\[0\]]"
puts "bt parent0: [get_value /tb_blob_detect_rle/dut/u_blob_detect_rle_top/u_blob_table/parent\[0\]]"

quit
