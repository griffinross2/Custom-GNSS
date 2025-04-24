# Simulate using Verilator
import sys
import subprocess

if __name__ == "__main__":
    if(len(sys.argv) < 2):
        print("Usage: python simulate_verilator.py <action>")
        print("Actions:")
        print("  build <top_level>: Build the Verilator simulation")
        print("  run: Run the Verilator simulation")
        sys.exit(1)

    action = sys.argv[1]
    if action == "build":
        if(len(sys.argv) < 3):
            print("Usage: python simulate_verilator.py build <top_level>")
            sys.exit(1)
        subprocess.run("wsl -e verilator --cc --binary {top} --trace --threads 4 -o sim -Isource -Itestbench -Iinclude -sv --clk clk -DSIMULATOR".format(top=sys.argv[2]), shell=True, check=True)
    elif action == "run":
        subprocess.run("wsl -e obj_dir/sim", shell=True, check=True)
    else:
        print("Invalid action: {}".format(action))
        sys.exit(1)
    
