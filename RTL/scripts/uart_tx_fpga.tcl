
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

    add_files -fileset sources_1 [ glob ./source/*.sv ]
    add_files -fileset sources_1 [ glob ./testbench/uart_tx_fpga.sv ]
    
    set_property include_dirs ./include [current_fileset]

    check_syntax -fileset sources_1

    update_compile_order -fileset sources_1
    
    set_property top uart_tx_fpga [get_fileset sources_1]
    # set_property -name {xsim.simulate.log_all_signals} -value {true} -objects [get_filesets sim_1]
    
    # set_property xsim.view { ./waveforms/uart_tx_fpga.wcfg } [get_filesets sources_1]
    