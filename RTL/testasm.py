# Script to automatically simulate the CPU against a specified test case or cases

import os
import sys
import subprocess
import difflib

def find_asm_files(prompt):
    asm_dir = "Assembly"

    # Get list of assembly files
    asm_files = []
    for file in os.listdir(asm_dir):
        if file.endswith(".asm"):
            asm_files.append(file)

    # For each assembly see if it starts with the prompt
    matching_files = []
    for file in asm_files:
        if file.startswith(prompt):
            matching_files.append(file)

    return matching_files

def diff_files(file1, file2):
    f1 = open(file1, 'r')
    f2 = open(file2, 'r')

    f1_lines = f1.readlines()
    f2_lines = f2.readlines()

    diff = difflib.ndiff(f1_lines, f2_lines)
    diff_str = ''.join(x for x in diff if x.startswith('- ') or x.startswith('+ '))
    
    f1.close()
    f2.close()

    # Log diff to file
    with open("diff.log", 'w') as file:
        file.write(diff_str)

    if diff_str == "":
        return True
    
    return False


def run_test():
    if(len(sys.argv) < 2):
        prompt = ""
    else:
        prompt = sys.argv[1]
    matching_files = find_asm_files(prompt)
    if len(matching_files) == 0:
        print("No assembly files found for search term \"{}\"".format(prompt))
        sys.exit(1)

    num_files = len(matching_files)

    for idx, file in enumerate(matching_files):
        print("Running test case ({}/{}): {}".format(idx+1, num_files, file).ljust(60), end='')

        # Assemble the file
        try:
            subprocess.run("python assemble.py \"Assembly/{}\"".format(file), shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            print("Error assembling file: {}".format(e))
            sys.exit(1)

        # Emulate the file
        try:
            subprocess.run("python emulate.py \"Assembly/{}.asm\"".format('.'.join(file.split('.')[:-1])), shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            print("Error emulating file: {}".format(e))
            sys.exit(1)

        # Simulate the CPU
        try:
            subprocess.run("python simulate_verilator.py run".format('.'.join(file.split('.')[:-1])), shell=True, check=True, stdout=subprocess.DEVNULL)
        except Exception as e:
            print("Error simulating file: {}".format(e))
            sys.exit(1)

        # Compare the output
        success = diff_files("memsim.hex", "ramcpu.hex")

        if success:
            print("\x1b[32mPASSED\x1b[0m")
        else:
            print("\x1b[31mFAILED\x1b[0m")
        
        # Divider
        print("-"*66)

if __name__ == "__main__":
    run_test()