import numpy as np

verilog = ""

# Total number of full adders and half adders
num_fa = 0
num_ha = 0

# The matrix should consist of the partial products in their proper places
# To fill in the blank spots, we can use empty tuples
def generate_initial_matrix(n):
    # Generate the initial matrix
    matrix = np.empty((n, 2*n), dtype=object)

    # Fill with none
    matrix = fill_matrix_with_none(matrix)

    for i in range(n):
        for j in range(n):
            matrix[i, i+j] = (i, j)
    return matrix

def print_matrix(matrix):
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            element = matrix[i, j]
            print(format_element(element) if element != (None, None) else "", end=",")
        print()

def format_element(element):
    if(type(element) == str):
        return element
    else:
        return str(element[0]) + "_" + str(element[1])

def fill_matrix_with_none(matrix):
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            matrix[i, j] = (None, None)
    return matrix

def find_first_none_row(matrix, j):
    for i in range(matrix.shape[0]):
        if (None, None) == matrix[i, j]:
            return i
    return -1

def find_last_filled_row_at_or_above(matrix, i_above, j):
    for i in range(i_above, -1, -1):
        if (None, None) != matrix[i, j]:
            return i
    return -1

def find_longest_column(matrix):
    longest = 0
    longest_idx = 0
    for j in range(matrix.shape[1]):
        length = 0
        for i in range(matrix.shape[0]):
            if matrix[i, j] != (None, None):
                length += 1
        if length > longest:
            longest = length
            longest_idx = j

    return longest_idx

def verilog_add_variables(reduction, matrix):
    global verilog
    # Create a hint vector stating whether the weight is more than 1 bit long
    variable_len_hint = np.zeros(matrix.shape[1])

    # For each column
    for j in range(matrix.shape[1]):
        # Find the length of the column
        length = 0
        for i in range(matrix.shape[0]):
            if matrix[i, j] != (None, None):
                length += 1
        # And create a variable for that # of lines in the weight
        if length == 1:
            verilog += "logic weight_{}_{};\n".format(reduction, j)
        elif length > 1:
            verilog += "logic [{}:0] weight_{}_{};\n".format(length-1, reduction, j)
            variable_len_hint[j] = 1
    return verilog, variable_len_hint

# Connect the multiplier inputs to the first partial products
def verilog_add_input_connections(matrix, variable_len_hint):
    global verilog

    verilog += "logic [{:.0f}:0] a, b;\n".format(matrix.shape[0]-1)
    verilog += "assign a = multiplier_if.a;\n"
    verilog += "assign b = multiplier_if.b;\n"

    connections = verilog_generate_connections_vector(matrix)
    for j in range(matrix.shape[1]):
        for i in range(matrix.shape[0]):
            if matrix[i, j] != (None, None):
                verilog += "assign weight_{:.0f}_{:.0f}{} = a[{:.0f}] & b[{:.0f}];\n".format(0, j, "[{:.0f}]".format(connections[j]) if variable_len_hint[j] > 0 else "", matrix[i, j][0], matrix[i, j][1])
                connections[j] += 1
    return verilog

# Connect the output connections to the final sum
def verilog_add_output_connections(matrix, reduction, variable_len_hint_input):
    global verilog
    verilog += "logic [{:.0f}:0] final_sum_a;\n".format(matrix.shape[1]-1)
    verilog += "logic [{:.0f}:0] final_sum_b;\n".format(matrix.shape[1]-1)
    
    # Top row connections
    verilog += "assign final_sum_a = {"
    top_row = ["weight_{:.0f}_{:.0f}{}".format(reduction, j, "[0]" if variable_len_hint_input[j] else "") for j in range(matrix.shape[1]-1, -1, -1)]
    verilog += ", ".join(top_row)
    verilog += "};\n"

    # Top row connections
    verilog += "assign final_sum_b = {"
    bottom_row = [("weight_{:.0f}_{:.0f}{}".format(reduction, j, "[1]") if variable_len_hint_input[j] else "1'b0") for j in range(matrix.shape[1]-1, -1, -1)]
    verilog += ", ".join(bottom_row)
    verilog += "};\n"

    verilog += "assign multiplier_if.out = final_sum_a + final_sum_b;\n"

    return verilog

# Creates a vector where each element counts the number of
# connections that have been made to that weight bus
def verilog_generate_connections_vector(matrix):
    return np.zeros(matrix.shape[1], dtype=int)

# Adds a 1, 2, or 3 input adder to the verilog code
# connections is the connections vector
# connections_next is the connections vector for the next stage
# variable_len_hint_input is the hint vector for the length of the input wires
# variable_len_hint_output is the hint vector for the length of the output wires
# reduction is the current reduction stage
# n_inputs is the number of inputs to the adder
# src1, src2, src3 are the source indexes of the matrix in the form (i, j)
# dest is the destination index of the matrix in the form (i, j)
def verilog_add_adder(connections, connections_next, variable_len_hint_input, variable_len_hint_output, reduction, n_inputs, src1, src2, src3):
    global verilog
    global num_fa
    global num_ha
    if n_inputs == 3:
        verilog += "full_adder fa{}(.a(weight_{:.0f}_{:.0f}{}), .b(weight_{:.0f}_{:.0f}{}), .cin(weight_{:.0f}_{:.0f}{}), .sum(weight_{:.0f}_{:.0f}{}), .cout(weight_{:.0f}_{:.0f}{}));\n".format(num_fa, \
                    reduction, src1[1], "[" + str(connections[src1[1]]) + "]" if variable_len_hint_input[src1[1]] else "", \
                    reduction, src2[1], "[" + str(connections[src2[1]]+1) + "]" if variable_len_hint_input[src2[1]] else "", \
                    reduction, src3[1], "[" + str(connections[src3[1]]+2) + "]" if variable_len_hint_input[src3[1]] else "", \
                    reduction+1, src1[1], "[" + str(connections_next[src1[1]]) + "]" if variable_len_hint_output[src1[1]] else "", \
                    reduction+1, src1[1]+1, "[" + str(connections_next[src1[1]+1]) + "]" if variable_len_hint_output[src1[1]+1] else "")
        # Count connections to the source (actually all the same column)
        connections[src1[1]] += 1
        connections[src2[1]] += 1
        connections[src3[1]] += 1
        # Count connections to the destination (source column and next column)
        connections_next[src1[1]] += 1
        connections_next[src1[1]+1] += 1
        # Number of full adders
        num_fa += 1
    elif n_inputs == 2:
        verilog += "half_adder ha{}(.a(weight_{:.0f}_{:.0f}{}), .b(weight_{:.0f}_{:.0f}{}), .sum(weight_{:.0f}_{:.0f}{}), .cout(weight_{:.0f}_{:.0f}{}));\n".format(num_ha, \
                    reduction, src1[1], "[" + str(connections[src1[1]]) + "]" if variable_len_hint_input[src1[1]] else "", \
                    reduction, src2[1], "[" + str(connections[src2[1]]+1) + "]" if variable_len_hint_input[src2[1]] else "", \
                    reduction+1, src1[1], "[" + str(connections_next[src1[1]]) + "]" if variable_len_hint_output[src1[1]] else "", \
                    reduction+1, src1[1]+1, "[" + str(connections_next[src1[1]+1]) + "]" if variable_len_hint_output[src1[1]+1] else "")
        # Count connections to the source (actually all the same column)
        connections[src1[1]] += 1
        connections[src2[1]] += 1
        # Count connections to the destination (source column and next column)
        connections_next[src1[1]] += 1
        connections_next[src1[1]+1] += 1
        # Number of half adders
        num_ha += 1
    elif n_inputs == 1:
        verilog += "assign weight_{:.0f}_{:.0f}{} = weight_{:.0f}_{:.0f}{};\n".format(reduction+1, src1[1], "[" + str(connections_next[src1[1]]) + "]" if variable_len_hint_output[src1[1]] else "", reduction, src1[1], "[" + str(connections[src1[1]]) + "]" if variable_len_hint_input[src1[1]] else "")
        # Count connections to the source
        connections[src1[1]] += 1
        # Count connections to the destination (next column)
        connections_next[src1[1]] += 1

    return (connections, connections_next)

def reduce_matrix(matrix):
    out_matrix = np.empty((n, 2*n), dtype=object)
    # Fill with none
    out_matrix = fill_matrix_with_none(out_matrix)

    # Reduce the matrix
    i = 0
    while True:
        # For each row, reduce vertical groups up to 3 by either:
        #   if 1, just pass through
        #   if 2, use a half adder
        #   if 3, use a full adder
        if i+2 >= matrix.shape[0]:
            break

        for j in range(out_matrix.shape[1]):
            if matrix[i, j] != (None, None) and matrix[i+1, j] != (None, None) and matrix[i+2, j] != (None, None):
                # Use a full adder
                first_avail_j = find_first_none_row(out_matrix, j)
                first_avail_j_next = find_first_none_row(out_matrix, j+1)
                out_matrix[first_avail_j, j] = "S(" + format_element(matrix[i, j]) + "+" + format_element(matrix[i+1, j]) + "+" + format_element(matrix[i+2, j]) + ")"
                out_matrix[first_avail_j_next, j+1] = "C(" + format_element(matrix[i, j]) + "+" + format_element(matrix[i+1, j]) + "+" + format_element(matrix[i+2, j]) + ")"
            elif matrix[i, j] != (None, None) and matrix[i+1, j] != (None, None):
                # Skip if the next column is >= 2n
                if j+1 >= out_matrix.shape[1]:
                    continue
                # Use a half adder (top aligned)
                first_avail_j = find_first_none_row(out_matrix, j)
                first_avail_j_next = find_first_none_row(out_matrix, j+1)
                out_matrix[first_avail_j, j] = "S(" + format_element(matrix[i, j]) + "+" + format_element(matrix[i+1, j]) + ")"
                out_matrix[first_avail_j_next, j+1] = "C(" + format_element(matrix[i, j]) + "+" + format_element(matrix[i+1, j]) + ")"
            elif matrix[i+1, j] != (None, None) and matrix[i+2, j] != (None, None):
                # Skip if the next column is >= 2n
                if j+1 >= out_matrix.shape[1]:
                    continue
                # Use a half adder (bottom aligned)
                first_avail_j = find_first_none_row(out_matrix, j)
                first_avail_j_next = find_first_none_row(out_matrix, j+1)
                out_matrix[first_avail_j, j] = "S(" + format_element(matrix[i+1, j]) + "+" + format_element(matrix[i+2, j]) + ")"
                out_matrix[first_avail_j_next, j+1] = "C(" + format_element(matrix[i+1, j]) + "+" + format_element(matrix[i+2, j]) + ")"
            else:
                # Pass through any non-none row
                for k in range(i, i+3):
                    if matrix[k, j] != (None, None):
                        first_avail_j = find_first_none_row(out_matrix, j)
                        out_matrix[first_avail_j, j] = format_element(matrix[k, j])
        
        i += 3

    # Next, pass through any remaining rows
    while i < matrix.shape[0]:
        for j in range(out_matrix.shape[1]):
            if matrix[i, j] != (None, None):
                first_avail_j = find_first_none_row(out_matrix, j)
                out_matrix[first_avail_j, j] = format_element(matrix[i, j])
        i += 1

    # Next, strip out remaining empty rows
    i = 0
    while i < out_matrix.shape[0]:
        for j in range(out_matrix.shape[1]):
            if out_matrix[i, j] != (None, None):
                break
        else:
            out_matrix = np.delete(out_matrix, i, axis=0)
            i-=1
        i+=1

    # Finally, it is necessary to move any columns that are to the right of the longest column down so that
    # the bottom of the matrix is flat past this point
    flat_start_j = find_longest_column(out_matrix) + 1
    for j in range(flat_start_j, out_matrix.shape[1]):
        for i in range(out_matrix.shape[0]-1, -1, -1):
            # If the cell is empty...
            if out_matrix[i, j] == (None, None):
                # Find the last filled row
                last_filled_i = find_last_filled_row_at_or_above(out_matrix, i, j)
                if(last_filled_i == -1):
                    break
                # Move the last filled row down to the empty cell
                out_matrix[i, j] = out_matrix[last_filled_i, j]
                out_matrix[last_filled_i, j] = (None, None)

    return out_matrix

# Must be done after a pass of reduce_matrix so that the output variables are known
def reduce_matrix_adding_verilog(matrix, variable_len_hint_input, variable_len_hint_output):
    # Connections vector
    connections = verilog_generate_connections_vector(matrix)
    connections_next = verilog_generate_connections_vector(matrix)

    # Reduce the matrix
    i = 0
    while True:
        # For each row, reduce vertical groups up to 3 by either:
        #   if 1, just pass through
        #   if 2, use a half adder
        #   if 3, use a full adder
        if i+2 >= matrix.shape[0]:
            break

        for j in range(matrix.shape[1]):
            if matrix[i, j] != (None, None) and matrix[i+1, j] != (None, None) and matrix[i+2, j] != (None, None):
                # Use a full adder
                # Add verilog
                connections, connections_next = verilog_add_adder(connections, connections_next, variable_len_hint_input, variable_len_hint_output, reduction-1, 3, (i, j), (i+1, j), (i+2, j))
            elif matrix[i, j] != (None, None) and matrix[i+1, j] != (None, None):
                # Skip if the next column is >= 2n
                if j+1 >= matrix.shape[1]:
                    continue
                # Use a half adder (top aligned)
                # Add verilog
                connections, connections_next = verilog_add_adder(connections, connections_next, variable_len_hint_input, variable_len_hint_output, reduction-1, 2, (i, j), (i+1, j), (None, None))
                pass
            elif matrix[i+1, j] != (None, None) and matrix[i+2, j] != (None, None):
                # Skip if the next column is >= 2n
                if j+1 >= matrix.shape[1]:
                    continue
                # Use a half adder (bottom aligned)
                # Add verilog
                connections, connections_next = verilog_add_adder(connections, connections_next, variable_len_hint_input, variable_len_hint_output, reduction-1, 2, (i+1, j), (i+2, j), (None, None))
                pass
            else:
                # Pass through any non-none row
                for k in range(i, i+3):
                    if matrix[k, j] != (None, None):
                        # Add verilog
                        connections, connections_next = verilog_add_adder(connections, connections_next, variable_len_hint_input, variable_len_hint_output, reduction-1, 1, (k, j), (None, None), (None, None))
                pass
        
        i += 3

    # Next, pass through any remaining rows
    while i < matrix.shape[0]:
        for j in range(matrix.shape[1]):
            if matrix[i, j] != (None, None):
                # Add verilog
                connections, connections_next = verilog_add_adder(connections, connections_next, variable_len_hint_input, variable_len_hint_output, reduction-1, 1, (i, j), (None, None), (None, None))
        i += 1

if __name__ == "__main__":
    n = 32
    reduction = 0

    # Generate the initial matrix
    matrix = generate_initial_matrix(n)

    # Add variables for the initial matrix
    _, variable_len_hint_input = verilog_add_variables(reduction, matrix)

    # Add input connections for the initial matrix
    verilog_add_input_connections(matrix, variable_len_hint_input)

    # print_matrix(matrix)

    while matrix.shape[0] > 2:
        # print()

        reduction += 1

        # First reduce the matrix
        new_matrix = reduce_matrix(matrix)

        # Add variables for the new matrix
        _, variable_len_hint_output = verilog_add_variables(reduction, new_matrix)

        # Add verilog for the new matrix
        reduce_matrix_adding_verilog(matrix, variable_len_hint_input, variable_len_hint_output)

        # Update the matrix
        matrix = new_matrix
        variable_len_hint_input = variable_len_hint_output
        
        # print_matrix(matrix)

    verilog_add_output_connections(matrix, reduction, variable_len_hint_input)

    print(verilog)