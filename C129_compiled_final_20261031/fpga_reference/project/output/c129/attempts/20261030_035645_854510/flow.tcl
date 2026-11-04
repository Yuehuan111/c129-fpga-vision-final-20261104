proc checked {args} {
 puts "C128_STEP_BEGIN $args"; flush stdout
 if {[catch {uplevel 1 $args} result]} {puts stderr "C128_FAILED $args : $result";exit 1}
 puts "C128_STEP_DONE $args";flush stdout
}
checked open_project {D:/FPGA/C129_extended_20261030/fpga_reference/project/board/build_cmos1/camera_board50.pds}
checked compile -force_to_run -top_module camera_board50_top
checked synthesize -force_to_run -ads -selected_syn_tool_opt 2
checked dev_map -force_to_run
checked pnr -force_to_run -gplace_seed 19 -mode performance -optimize_multi_corner_timing -fix_hold_violation -fix_hold_violation_threshold 100 -fix_hold_violation_in_route TRUE -fix_hold_violation_threshold_for_clk_tree 200
checked report_timing -force_to_run -report_max_skew
checked gen_netlist -force_to_run
checked gen_bit_stream -force_to_run -master_configuration_clock_frequency {40M}
puts "C129_FULL_FLOW_COMPLETED"
exit
