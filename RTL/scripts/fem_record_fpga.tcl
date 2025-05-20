
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
    add_files -fileset sources_1 [ glob ./testbench/fem_record_fpga.sv ]
    
    set_property include_dirs ./include [current_fileset]

    check_syntax -fileset sources_1

    update_compile_order -fileset sources_1
    
    set_property top fem_record_fpga [get_fileset sources_1]

    synth_design -top fem_record_fpga -part $part

    opt_design
    place_design
    route_design

    write_bitstream -force -file fem_record_fpga.bit
    write_cfgmem -format mcs -size 16 -interface SPIx4 -loadbit "up 0x0 fem_record_fpga.bit" -file fem_record_fpga.mcs

    open_hw_manager
    connect_hw_server -url localhost:3121
    open_hw_target
    set_property PROGRAM.FILE {fem_record_fpga.bit} [current_hw_device]
    program_hw_devices