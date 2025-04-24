import os
import sys
import subprocess

# def generate_tcl(toplevel, synthesize=False):
#     add_waveform = False
#     if os.path.exists("./waveforms/{toplevel}.wcfg".format(toplevel=toplevel)):
#         add_waveform = True
#         print("Waveform file found. Adding waveform to simulation.")
#     string = """
#     create_project -name .temp -force

#     set part "xc7s50csga324-1"
#     set brd_part "digilentinc.com:arty-s7-50:part0:1.1"

#     set obj [get_projects .temp]
#     set_property "default_lib" "xil_defaultlib" $obj
#     set_property "part" $part $obj
#     set_property "board_part" $brd_part $obj

#     set_property XPM_LIBRARIES XPM_MEMORY [current_project]

#     add_files -fileset sources_1 [ glob ./include/*.vh ]
#     add_files -fileset sim_1 [ glob ./include/*.vh ]
#     add_files -fileset constrs_1 [ glob ./constraints/*.xdc ]

#     # Get the list of .vh files in the target directory
#     set vh_files [glob -nocomplain -directory ./include *.vh]

#     # Loop through each .vh file and set the file_type property
#     foreach file $vh_files {
#         set file_obj [get_files $file]
#         if {$file_obj ne ""} {
#             set_property file_type "Verilog Header" $file_obj
#             puts "Set file_type to 'Verilog Header' for: $file"
#         } else {
#             puts "Warning: File not found in project: $file"
#         }
#     }

#     add_files -fileset sources_1 [ glob ./source/*.sv ]
#     add_files -fileset sim_1 [ glob ./testbench/*.sv ]
#     """ + ("""
#     add_files -fileset sim_1 -norecurse ./waveforms/{toplevel}.wcfg
#     """.format(toplevel=toplevel) if add_waveform else "") + """
#     set_property include_dirs ./include [current_fileset]

#     check_syntax -fileset sources_1
#     check_syntax -fileset sim_1

#     update_compile_order -fileset sources_1
#     update_compile_order -fileset sim_1

#     set_property top {toplevel} [get_fileset sim_1]
#     set_property -name {{xsim.simulate.log_all_signals}} -value {{true}} -objects [get_filesets sim_1]
#     """.format(toplevel=toplevel) + ("""
#     set_property xsim.view {{ ./waveforms/{toplevel}.wcfg }} [get_filesets sim_1]
#     """.format(toplevel=toplevel) if add_waveform else "") + ("""
#     set_param general.maxThreads 12
#     synth_design -top {toplevel} -part xc7s50csga324-2 -flatten_hierarchy rebuilt -include_dirs {{../../include}} -mode out_of_context
#     create_clock -period 10 [get_nets clk] -name clk
#     """.format(toplevel=(toplevel.replace("_tb", ""))) if synthesize else "") + """
    
#     launch_simulation -mode """ + ("post-synthesis -type functional" if synthesize else "behavioral") + """
#     log_wave -r /
#     run all
#     """
#     return string

if __name__ == "__main__":
    if(len(sys.argv) < 2):
        print("Usage: python simulate.py <toplevel>")
        print("Options:")
        print("-s : Synthesize before simulating")
        print("-g : GUI mode")
        sys.exit(1)
    args = sys.argv[1:]

    # GUI mode?
    if "-g" in args:
        gui = True
        args.remove("-g")
    else:
        gui = False

    toplevel = args[0]
    
    # Find TCL script
    if not os.path.exists("scripts/{}.tcl".format(toplevel)):
        print("TCL file not found!")
        sys.exit(1)

    # Run Vivado
    cmd_1 = "call \"settings64.bat\""
    cmd_2 = "vivado -mode {mode} -source scripts/{toplevel}.tcl".format(mode="gui" if gui else "batch", toplevel=toplevel)

    subprocess.run(cmd_1 + " && " + cmd_2, shell=True, check=True)
    print("Simulation complete.")
