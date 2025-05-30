/**************************************/
/* Fake Flash Memory Simulation Model */
/**************************************/
`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;

module flash_model (
  input logic clk,
  input logic nrst,
  input logic [3:0] clk_div,
  input logic cs,
  inout logic [3:0] dq
);
    localparam logic [2:0] NUM_DUMMY_CYCLES = 4;

    // Flash clock divider
    logic flash_clk;
    logic flash_clk_div;
    logic [3:0] clk_cnt;
    always_ff @(posedge clk) begin
        if (~nrst) begin
            clk_cnt <= 4'b0000;
            flash_clk_div <= 1'b0;
        end else if (clk_cnt == (clk_div - 4'd1)) begin
            clk_cnt <= 4'b0000;
            flash_clk_div <= ~flash_clk_div;
        end else begin
            clk_cnt <= clk_cnt + 4'd1;
        end
    end

    always_comb begin
        // Bypass the flash clock divider if clk_div is 0
        if (clk_div == 4'd0) begin
            flash_clk = clk;
        end else begin
            flash_clk = flash_clk_div;
        end
    end

    // Flash array
    logic [7:0] flash_array [0:8388607]; // 8MB
    logic [31:0] addr, next_addr;
    logic [2:0] cycle_count, next_cycle_count;
    logic tristate;
    logic [3:0] dq_out;
    logic [3:0] dq_in;
    typedef logic [22:0] addr_t;

    // Initialize flash memory
    integer fd;
    initial begin
        // Zero fill
        for (int i = 0; i < 8388608; i++) begin
            flash_array[i] = 8'h00;
        end

`ifndef SIMULATOR
        fd = $fopen("../../../../program.bin", "rb");
`else
        fd = $fopen("program.bin", "rb");
`endif
        if (fd == 0) begin
            $fatal("Error: Could not open program.bin");
            $finish;
        end

        // Initialize flash memory with some values
        for (int i = 0; i < 8388608; i++) begin
            if ($feof(fd)) begin
                break;
            end
            flash_array[i] = $fgetc(fd);
        end

        $fclose(fd);
    end

    // States
    typedef enum logic [2:0] {
        IDLE,
        INST,
        ADDR,
        MODE,
        DUMMY,
        DATA
    } state_t;   

    state_t state, next_state;

    // Next state logic
    always_comb begin
        next_state = state;
        case (state)
            IDLE: begin
                if (!cs) begin
                    next_state = INST;
                end
            end

            INST: begin
                if (cycle_count == 3'd7) begin
                    next_state = ADDR;
                end
            end

            ADDR: begin
                if (cycle_count == 3'd7) begin
                    next_state = MODE;
                end
            end

            MODE: begin
                if (cycle_count == 3'd1) begin
                    next_state = DUMMY;
                end
            end

            DUMMY: begin
                if (cycle_count == (NUM_DUMMY_CYCLES - 3'd1)) begin
                    next_state = DATA;
                end
            end

            DATA: begin
                if (cycle_count == 3'd7) begin
                    next_state = INST;
                end
            end

            default: begin
                next_state = INST;
            end
        endcase
    end

    // State output logic
    always_comb begin
        next_addr = addr;
        next_cycle_count = cycle_count + 3'd1;
        tristate = 1'b1;
        dq_out = '0;

        case (state)
            IDLE: begin
                next_cycle_count = 3'd0;
            end

            INST: begin
                next_addr = '0;
                if (cycle_count == 3'd7) begin
                    next_cycle_count = 3'd0;
                end
            end

            ADDR: begin
                case (cycle_count)
                    3'd0: next_addr = {dq_in, addr[27:0]};
                    3'd1: next_addr = {addr[31:28], dq_in, addr[23:0]};
                    3'd2: next_addr = {addr[31:24], dq_in, addr[19:0]};
                    3'd3: next_addr = {addr[31:20], dq_in, addr[15:0]};
                    3'd4: next_addr = {addr[31:16], dq_in, addr[11:0]};
                    3'd5: next_addr = {addr[31:12], dq_in, addr[7:0]};
                    3'd6: next_addr = {addr[31:8], dq_in, addr[3:0]};
                    3'd7: next_addr = {addr[31:4], dq_in};
                    default: next_addr = '0;
                endcase
                if (cycle_count == 3'd7) begin
                    next_cycle_count = 3'd0;
                end
            end

            MODE: begin
                if (cycle_count == 3'd1) begin
                    next_cycle_count = 3'd0;
                end
            end

            DUMMY: begin
                if (cycle_count == (NUM_DUMMY_CYCLES - 3'd1)) begin
                    next_cycle_count = 3'd0;
                end
            end

            DATA: begin
                tristate = 1'b0;
                case (cycle_count)
                    3'd0: dq_out = flash_array[addr_t'(addr-32'h0080_0000)][7:4];
                    3'd1: dq_out = flash_array[addr_t'(addr-32'h0080_0000)][3:0];
                    3'd2: dq_out = flash_array[addr_t'(addr-32'h0080_0000 + 32'd1)][7:4];
                    3'd3: dq_out = flash_array[addr_t'(addr-32'h0080_0000 + 32'd1)][3:0];
                    3'd4: dq_out = flash_array[addr_t'(addr-32'h0080_0000 + 32'd2)][7:4];
                    3'd5: dq_out = flash_array[addr_t'(addr-32'h0080_0000 + 32'd2)][3:0];
                    3'd6: dq_out = flash_array[addr_t'(addr-32'h0080_0000 + 32'd3)][7:4];
                    3'd7: dq_out = flash_array[addr_t'(addr-32'h0080_0000 + 32'd3)][3:0];
                    default: dq_out = '0;
                endcase
                if (cycle_count == 3'd7) begin
                    next_cycle_count = 3'd0;
                end
            end

            default: begin
            end
        endcase
    end

    always_ff @(negedge flash_clk, negedge cs) begin
        if (cs) begin
            state <= IDLE;
            cycle_count <= '0;
        end else begin
            state <= next_state;
            cycle_count <= next_cycle_count;
        end
    end

    always_ff @(posedge flash_clk) begin
        if (cs) begin
            addr <= '0;
        end else begin
            addr <= next_addr;
        end
    end

    // Tristate logic
    IOBUF flash_dq0_buf (
        .IO(dq[0]),
        .O(dq_in[0]),
        .I(dq_out[0]),
        .T(tristate)
    );
    
    IOBUF flash_dq1_buf (
        .IO(dq[1]),
        .O(dq_in[1]),
        .I(dq_out[1]),
        .T(tristate)
    );

    IOBUF flash_dq2_buf (
        .IO(dq[2]),
        .O(dq_in[2]),
        .I(dq_out[2]),
        .T(tristate)
    );

    IOBUF flash_dq3_buf (
        .IO(dq[3]),
        .O(dq_in[3]),
        .I(dq_out[3]),
        .T(tristate)
    );

endmodule