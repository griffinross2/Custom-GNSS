# Script to assemble the assembly file and convert a memory file for the fpga

import os
import subprocess
import sys

def convert_intel_hex_to_vivado_mem(file_path):
    memsize = 16384
    hex_strings = []

    # Fill with zeros
    for i in range(memsize):
        hex_strings.append('00000000')

    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith(':'):
                hex_data = line[1:].strip()

                record_type = int(hex_data[6:8], 16)
                if record_type != 0:
                    # Only process data records
                    continue

                data_size = int(hex_data[0:2], 16)
                record_addr = int(hex_data[2:6], 16)

                # Loop through each byte in the record
                for i in range(data_size):
                    # Get the hex for that byte
                    data = hex_data[8 + i*2:8 + i*2 + 2]

                    addr = (record_addr + i)

                    # Write to the row floor(addr/4) at position (6-2*(addr%4)):(8-2*(addr%4))
                    hex_strings[addr//4] = hex_strings[addr//4][0:(6-2*(addr%4))] + data + hex_strings[addr//4][(8-2*(addr%4)):]

    return '\n'.join(hex_strings)

def assemble(asm_file):
    march = "rv32im_zicsr"
    subprocess.run("riscv-none-elf-as -march={march} -mabi=ilp32 -o {asm_file_start}.o {asm_file}".format(march=march, asm_file_start='.'.join(sys.argv[1].split('.')[:-1]), asm_file=asm_file), shell=True)
    subprocess.run("riscv-none-elf-ld -T linkerscript.ld -o {asm_file_start}.elf {asm_file_start}.o".format(asm_file_start='.'.join(sys.argv[1].split('.')[:-1])), shell=True)
    subprocess.run("riscv-none-elf-objcopy -O ihex {asm_file_start}.elf {asm_file_start}.hex".format(asm_file_start='.'.join(sys.argv[1].split('.')[:-1])), shell=True)
    # os.system("wsl -e /opt/riscv/bin/riscv32-unknown-elf-as -march={march} -mabi=ilp32d -o {asm_file_start}.o {asm_file}".format(march=march, asm_file_start='.'.join(sys.argv[1].split('.')[:-1]), asm_file=asm_file))
    # os.system("wsl -e /opt/riscv/bin/riscv32-unknown-elf-objcopy -O ihex {asm_file_start}.o {asm_file_start}.hex".format(asm_file_start='.'.join(sys.argv[1].split('.')[:-1])))

if __name__ == "__main__":
    if(len(sys.argv) < 2):
        print("Usage: python assemble.py <asm_file>")
        sys.exit(1)
    
    if not os.path.exists(sys.argv[1]):
        print("Assembly file \"{}\" does not exist".format(sys.argv[1]))
        sys.exit(1)

    assemble(sys.argv[1])
    viv_mem = convert_intel_hex_to_vivado_mem('.'.join(sys.argv[1].split('.')[:-1]) + ".hex")
    with open("raminit.mem", 'w') as file:
        file.write(viv_mem)