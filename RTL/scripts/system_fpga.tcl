
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
    add_files -fileset sources_1 [ glob ./testbench/system_fpga.sv ]

    add_files -fileset sources_1 -norecurse ./bootloader.mem

    set_property include_dirs ./include [current_fileset]

    check_syntax -fileset sources_1

    set_property top system_fpga [get_fileset sources_1]
    update_compile_order -fileset sources_1

    # synth_design

    # opt_design
    # place_design
    # route_design

    # write_bitstream -force -file system_fpga.bit
    # write_cfgmem -force -format mcs -size 16 -interface SPIx4 -loadbit "up 0x0 system_fpga.bit" -loaddata "up 0x00800000 program.bin" -file system_fpga.mcs

    # # Open Device
    # open_hw_manager
    # connect_hw_server -url localhost:3121
    # open_hw_target

    # # Create Configuration Memory Device
    # create_hw_cfgmem -hw_device [lindex [get_hw_devices xc7s50_0] 0] [lindex [get_cfgmem_parts {s25fl128sxxxxxx0-spi-x1_x2_x4}] 0]
    # set_property PROGRAM.BLANK_CHECK  0 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.ERASE  1 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.CFG_PROGRAM  1 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.VERIFY  1 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.CHECKSUM  0 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # refresh_hw_device [lindex [get_hw_devices xc7s50_0] 0]

    # # Program the configuration memory device
    # set_property PROGRAM.ADDRESS_RANGE  {use_file} [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.FILES [list "D:/github_repos/RISC-V-Core/RTL/system_fpga.mcs" ] [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.PRM_FILE {} [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.UNUSED_PIN_TERMINATION {pull-none} [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.BLANK_CHECK  0 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.ERASE  1 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.CFG_PROGRAM  1 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.VERIFY  1 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # set_property PROGRAM.CHECKSUM  0 [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # startgroup 
    # create_hw_bitstream -hw_device [lindex [get_hw_devices xc7s50_0] 0] [get_property PROGRAM.HW_CFGMEM_BITFILE [ lindex [get_hw_devices xc7s50_0] 0]]; program_hw_devices [lindex [get_hw_devices xc7s50_0] 0]; refresh_hw_device [lindex [get_hw_devices xc7s50_0] 0];
    # program_hw_cfgmem -hw_cfgmem [ get_property PROGRAM.HW_CFGMEM [lindex [get_hw_devices xc7s50_0] 0]]
    # endgroup

    # set_property PROGRAM.FILE {system_fpga.bit} [current_hw_device]
    # program_hw_devices