import subprocess
import sys
import os

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

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python compile.py <source_dir>")
        sys.exit(1)

    src_dir = sys.argv[1]
    c_files_in_dir = [x for x in os.listdir(src_dir) if x.endswith('.c')]
    c_files_out = [x + ".o" for x in c_files_in_dir]
    S_files_in_dir = [x for x in os.listdir(src_dir) if x.endswith('.S')]
    S_files_out = [x + ".o" for x in S_files_in_dir]

    # Empty the output directory
    build_files = [x for x in os.listdir("build")]
    for f in build_files:
        os.remove(f"build/{f}")

    # Generate object files from C source files
    for c_in, c_out in zip(c_files_in_dir, c_files_out):
        subprocess.run(f"riscv-none-elf-gcc -march=rv32im_zicsr -mabi=ilp32 -c -o build/{c_out} {src_dir}/{c_in}", shell=True)

    # Generate object files from ASM source files
    for S_in, S_out in zip(S_files_in_dir, S_files_out):
        subprocess.run(f"riscv-none-elf-as -march=rv32im_zicsr -mabi=ilp32 -o build/{S_out} {src_dir}/{S_in}", shell=True)
    
    # Generate object files from assembly startup file
    subprocess.run("riscv-none-elf-as -march=rv32im_zicsr -mabi=ilp32 -o build/startup.o startup.S", shell=True)

    subprocess.run(f"riscv-none-elf-gcc -Wl,--print-memory-usage -nostartfiles -T linkerscript.ld {" ".join(["build/" + x for x in c_files_out])} {" ".join(["build/" + x for x in S_files_out])} build/startup.o -o build/program.elf", shell=True)
    subprocess.run("riscv-none-elf-objdump -D build/program.elf > build/program.S", shell=True)
    subprocess.run("riscv-none-elf-objcopy -O ihex build/program.elf build/program.hex", shell=True)

    mem = convert_intel_hex_to_vivado_mem('build/program.hex')
    with open('build/raminit.mem', 'w') as file:
        file.write(mem)