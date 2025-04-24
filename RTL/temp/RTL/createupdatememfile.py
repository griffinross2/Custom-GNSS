def create_mem():
    diff_lines = []

    with open('raminit.mem', 'r') as f:
        lines_old = f.readlines()

    with open('ramnew.mem', 'r') as f:
        lines_new = f.readlines()

    with open('ramupd.mem', 'w') as f:
        for idx, line in enumerate(lines_new):
            if line.strip() != lines_old[idx].strip():
                diff_lines.append((idx, line.strip()))

                # If this line starts at an address non-contiguous to the previous set, pop those and write to the final mem file
                if len(diff_lines) >= 2 and diff_lines[-2][0] + 1 != diff_lines[-1][0]:
                    f.write("@{:08X}\n".format(diff_lines[0][0]*4))
                    for i in range(len(diff_lines) - 1):
                        f.write("{} ".format(diff_lines[i][1]))
                    f.write("\n")
                    diff_lines = [diff_lines[-1]]

 
        # Write any remaining differences
        f.write("@{:08X}\n".format(diff_lines[0][0]*4))
        for i in range(len(diff_lines)):
            f.write("{} ".format(diff_lines[i][1]))
        f.write("\n")

if __name__ == "__main__":
    create_mem()
    print("Memory file created from differences.")