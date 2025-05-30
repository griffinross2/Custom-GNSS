
    create_project -name .temp -force

    set part "xc7s50csga324-1"
    set brd_part "digilentinc.com:arty-s7-50:part0:1.1"

    set obj [get_projects .temp]
    set_property "default_lib" "xil_defaultlib" $obj
    set_property "part" $part $obj
    set_property "board_part" $brd_part $obj

    set_property XPM_LIBRARIES XPM_MEMORY [current_project]

    add_files -fileset sources_1 [ glob ./include/*.vh ]
    add_files -fileset sim_1 [ glob ./include/*.vh ]
    add_files -fileset constrs_1 [ glob ./constraints/*.xdc ]
    add_files -fileset constrs_1 ./IP/mig_7series_0/mig_7series_0/user_design/constraints/mig_7series_0.xdc

    # Get the list of .vh files in the target directory
    set vh_files [glob -nocomplain -directory ./include *.vh]

    # Loop through each .vh file and set the file_type property
    foreach file $vh_files {
        set file_obj [get_files $file]
        if {$file_obj ne ""} {
            set_property file_type "Verilog Header" $file_obj
            puts "Set file_type to 'Verilog Header' for: $file"
        } else {
            puts "Warning: File not found in project: $file"
        }
    }

    # read_ip ./IP/mig_7series_0/mig_7series_0.xci
    # read_ip ./IP/clk_wiz_0/clk_wiz_0.xci
    # read_ip ./IP/clk_wiz_1/clk_wiz_1.xci
    add_files -fileset sources_1 [ glob ./IP/mig_7series_0/mig_7series_0/user_design/rtl/axi/*.v ]
    add_files -fileset sources_1 [ glob ./IP/mig_7series_0/mig_7series_0/user_design/rtl/clocking/*.v ]
    add_files -fileset sources_1 [ glob ./IP/mig_7series_0/mig_7series_0/user_design/rtl/controller/*.v ]
    add_files -fileset sources_1 [ glob ./IP/mig_7series_0/mig_7series_0/user_design/rtl/ecc/*.v ]
    add_files -fileset sources_1 [ glob ./IP/mig_7series_0/mig_7series_0/user_design/rtl/ip_top/*.v ]
    add_files -fileset sources_1 [ glob ./IP/mig_7series_0/mig_7series_0/user_design/rtl/phy/*.v ]
    add_files -fileset sources_1 [ glob ./IP/mig_7series_0/mig_7series_0/user_design/rtl/ui/*.v ]
    add_files -fileset sources_1 [ glob ./IP/mig_7series_0/mig_7series_0/user_design/rtl/*.v ]
    add_files -fileset sources_1 [ glob ./IP/mig_7series_0/mig_7series_0/example_design/sim/*.sv ]
    add_files -fileset sources_1 [ glob ./IP/mig_7series_0/mig_7series_0/example_design/sim/*.v ]
    add_files -fileset sources_1 [ glob ./IP/mig_7series_0/mig_7series_0/example_design/sim/*.vh ]
    add_files -fileset sources_1 [ glob ./source/*.sv ]
    add_files -fileset sim_1 ./testbench/axi_dram_test_tb.sv
    
    add_files -fileset sim_1 -norecurse ./waveforms/axi_dram_test_tb.wcfg
    
    set_property include_dirs ./include [current_fileset]

    check_syntax -fileset sources_1
    check_syntax -fileset sim_1

    update_compile_order -fileset sources_1
    update_compile_order -fileset sim_1

    set_property top axi_dram_test_tb [get_fileset sim_1]
    set_property -name {xsim.simulate.log_all_signals} -value {true} -objects [get_filesets sim_1]
    
    set_property xsim.view { ./waveforms/axi_dram_test_tb.wcfg } [get_filesets sim_1]
    
    
    launch_simulation -mode behavioral
    log_wave -r /
    run all
    