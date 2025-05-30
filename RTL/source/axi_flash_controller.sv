/********************************/
/*  Flash Read-Only Controller  */
/********************************/
`timescale 1ns/1ns

`include "axi_bus_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module axi_flash_controller (
    input logic clk, nrst,
    axi_bus_if.satellite_to_mux abif,
    input logic [3:0] clk_div,
    output logic flash_cs,
    inout wire [3:0] flash_dq
);
    localparam logic [2:0] QIO_READ_LAT_CYCLES = 3'd4;

    // Flash clock - sent external through STARTUPE2
    logic flash_clk;
    logic flash_clk_en, next_flash_clk_en, next_next_flash_clk_en;

    // Flash clock divider
    logic flash_clk_div;
    logic flash_clk_strobe;
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
        flash_clk_strobe = 1'b0;
    
        // Bypass the flash clock divider if clk_div is 0
        if (clk_div == 4'd0) begin
            flash_clk = clk;
            flash_clk_strobe = 1'b1;
        end else begin
            flash_clk = flash_clk_div;

            if (!flash_clk_div && (clk_cnt == (clk_div - 4'd1))) begin
                flash_clk_strobe = 1'b1;
            end
        end
    end

    // Flash clock output
    STARTUPE2 #(
        .PROG_USR("FALSE"),
        .SIM_CCLK_FREQ(0)
    ) startupe2_inst (
        .CFGCLK(),
        .CFGMCLK(),
        .CLK(1'b0),
        .EOS(),
        .GSR(1'b0),
        .GTS(1'b0),
        .KEYCLEARB(1'b1),
        .PACK(1'b0),
        .PREQ(),
        .USRCCLKO(flash_clk & flash_clk_en),
        .USRCCLKTS(1'b0),
        .USRDONEO(1'b1),
        .USRDONETS(1'b0)
    );

    // Flash state machine
    typedef enum logic [2:0] {
        FLASH_IDLE,
        FLASH_READ_INST,
        FLASH_READ_ADDR,
        FLASH_READ_MODE,
        FLASH_READ_DUMMY,
        FLASH_READ_DATA
    } flash_state_t;

    flash_state_t state, next_state;
    logic [2:0] cycle_counter, next_cycle_counter;

    logic flash_read_start;
    logic flash_read_done;
    word_t flash_read_addr;
    word_t flash_read_data, next_flash_read_data;

    logic next_flash_cs, next_next_flash_cs;

    // Flash bidirectional buffer
    logic flash_tristate;   // 1 -> high-Z
    logic [3:0] flash_dq_o, next_flash_dq_o;
    logic [3:0] flash_dq_i;

    IOBUF flash_dq0_buf (
        .IO(flash_dq[0]),
        .O(flash_dq_i[0]),
        .I(flash_dq_o[0]),
        .T(flash_tristate)
    );
    
    IOBUF flash_dq1_buf (
        .IO(flash_dq[1]),
        .O(flash_dq_i[1]),
        .I(flash_dq_o[1]),
        .T(flash_tristate)
    );
    
    IOBUF flash_dq2_buf (
        .IO(flash_dq[2]),
        .O(flash_dq_i[2]),
        .I(flash_dq_o[2]),
        .T(flash_tristate)
    );
    
    IOBUF flash_dq3_buf (
        .IO(flash_dq[3]),
        .O(flash_dq_i[3]),
        .I(flash_dq_o[3]),
        .T(flash_tristate)
    );

    // Flash control signals
    always_ff @(posedge clk) begin
        if (~nrst) begin
            state <= FLASH_IDLE;
            next_flash_cs <= 1'b1;
            next_flash_clk_en <= 1'b0;
            cycle_counter <= 3'b000;
            flash_read_data <= '0;
        end else if (flash_clk_strobe) begin
            state <= next_state;
            next_flash_cs <= next_next_flash_cs;
            next_flash_clk_en <= next_next_flash_clk_en;
            cycle_counter <= next_cycle_counter;
            flash_read_data <= next_flash_read_data;
        end
    end

    // Flash data output
    always_ff @(negedge flash_clk, negedge nrst) begin
        if (~nrst) begin
            flash_dq_o <= 4'b0000;
            flash_cs <= 1'b1;
            flash_clk_en <= 1'b0;
        end else begin
            flash_dq_o <= next_flash_dq_o;
            flash_cs <= next_flash_cs;
            flash_clk_en <= next_flash_clk_en;
        end
    end

    // Next state logic
    always_comb begin
        next_state = state;

        case (state)
            FLASH_IDLE: begin
                if (flash_read_start) begin
                    next_state = FLASH_READ_INST;
                end
            end

            FLASH_READ_INST: begin
                if (cycle_counter == 3'd7) begin
                    next_state = FLASH_READ_ADDR;
                end
            end

            FLASH_READ_ADDR: begin
                if (cycle_counter == 3'd7) begin
                    next_state = FLASH_READ_MODE;
                end
            end
            
            FLASH_READ_MODE: begin
                if (cycle_counter == 3'd1) begin
                    next_state = FLASH_READ_DUMMY;
                end
            end

            FLASH_READ_DUMMY: begin
                if (cycle_counter == (QIO_READ_LAT_CYCLES - 3'd1)) begin
                    next_state = FLASH_READ_DATA;
                end
            end

            FLASH_READ_DATA: begin
                if (cycle_counter == 3'd7) begin
                    next_state = FLASH_IDLE;
                end
            end

            default: begin
            end
        endcase
    end

    // State output logic
    always_comb begin
        next_cycle_counter = cycle_counter;
        next_next_flash_cs = next_flash_cs;
        next_next_flash_clk_en = next_flash_clk_en;
        next_flash_dq_o = flash_dq_o;
        next_flash_read_data = flash_read_data;

        flash_read_done = 1'b0;

        // High-Z by default
        flash_tristate = 1'b1;

        case (state)
            FLASH_IDLE: begin
                if (next_state == FLASH_READ_INST) begin
                    next_next_flash_cs = 1'b0;
                    next_next_flash_clk_en = 1'b1;
                    next_cycle_counter = 3'b000;
                end
            end

            FLASH_READ_INST: begin
                // Enable flash data output
                flash_tristate = 1'b0;

                // Inc cycle counter
                next_cycle_counter = cycle_counter + 3'd1;

                // Next bit of instruction or address
                case (cycle_counter)
                    3'd0: next_flash_dq_o = 4'b0001;
                    3'd1: next_flash_dq_o = 4'b0001;
                    3'd2: next_flash_dq_o = 4'b0001;
                    3'd3: next_flash_dq_o = 4'b0000;
                    3'd4: next_flash_dq_o = 4'b0001;
                    3'd5: next_flash_dq_o = 4'b0001;
                    3'd6: next_flash_dq_o = 4'b0000;
                    3'd7: next_flash_dq_o = 4'b0000;
                    default: next_flash_dq_o = 4'b0000;
                endcase

                if (next_state == FLASH_READ_ADDR) begin
                    next_cycle_counter = '0;
                end
            end

            FLASH_READ_ADDR: begin
                // Enable flash data output
                flash_tristate = 1'b0;

                // Inc cycle counter
                next_cycle_counter = cycle_counter + 3'd1;

                // Next bit of address or mode
                case (cycle_counter)
                    3'd0: next_flash_dq_o = flash_read_addr[31:28];
                    3'd1: next_flash_dq_o = flash_read_addr[27:24];
                    3'd2: next_flash_dq_o = flash_read_addr[23:20];
                    3'd3: next_flash_dq_o = flash_read_addr[19:16];
                    3'd4: next_flash_dq_o = flash_read_addr[15:12];
                    3'd5: next_flash_dq_o = flash_read_addr[11:8];
                    3'd6: next_flash_dq_o = flash_read_addr[7:4];
                    3'd7: next_flash_dq_o = flash_read_addr[3:0];
                    default: next_flash_dq_o = 4'b0000;
                endcase

                if (next_state == FLASH_READ_MODE) begin
                    next_cycle_counter = '0;
                end
            end
            
            FLASH_READ_MODE: begin
                // Enable flash data output
                flash_tristate = 1'b0;

                // Inc cycle counter
                next_cycle_counter = cycle_counter + 3'd1;

                // Next bit of mode
                next_flash_dq_o = 4'b0000;

                if (next_state == FLASH_READ_DUMMY) begin
                    next_cycle_counter = '0;
                end
            end

            FLASH_READ_DUMMY: begin
                // Inc cycle counter
                next_cycle_counter = cycle_counter + 3'd1;

                if (next_state == FLASH_READ_DATA) begin
                    next_cycle_counter = '0;
                end
            end

            FLASH_READ_DATA: begin
                // Inc cycle counter
                next_cycle_counter = cycle_counter + 3'd1;

                next_flash_read_data = flash_read_data;
                case (cycle_counter)
                    3'd0: next_flash_read_data[7:4] = flash_dq_i;   // Top of first byte
                    3'd1: next_flash_read_data[3:0] = flash_dq_i;   // Bottom of first byte
                    3'd2: next_flash_read_data[15:12] = flash_dq_i; // Top of second byte
                    3'd3: next_flash_read_data[11:8] = flash_dq_i;  // Bottom of second byte
                    3'd4: next_flash_read_data[23:20] = flash_dq_i; // Top of third byte
                    3'd5: next_flash_read_data[19:16] = flash_dq_i; // Bottom of third byte
                    3'd6: next_flash_read_data[31:28] = flash_dq_i; // Top of fourth byte
                    3'd7: begin 
                        next_flash_read_data[27:24] = flash_dq_i; // Bottom of fourth byte
                        next_next_flash_cs = 1'b1; // Deassert chip select
                        next_next_flash_clk_en = 1'b0; // Disable flash clock
                        
                        // Only on the clk cycle where flash clock is going high,
                        // ensure that the AXI side doesn't see this twice
                        if (flash_clk_strobe) begin
                            flash_read_done = 1'b1; // Indicate read done
                        end
                    end
                    default begin end
                endcase
            end

            default: begin
            end
        endcase
    end

    // AXI bus interface

    // AXI states
    typedef enum logic [1:0] {
        AXI_IDLE,
        AXI_READ_WAIT,
        AXI_READ_DATA
    } axi_state_t;

    axi_state_t axi_state, next_axi_state;

    word_t next_flash_read_addr;
    logic [3:0] read_id, next_read_id;
    logic rvalid, next_rvalid;

    always_ff @(posedge clk) begin
        if (~nrst) begin
            axi_state <= AXI_IDLE;
            flash_read_addr <= '0;
            read_id <= '0;
            rvalid <= 1'b0;
        end else begin
            axi_state <= next_axi_state;
            flash_read_addr <= next_flash_read_addr;
            read_id <= next_read_id;
            rvalid <= next_rvalid;
        end
    end

    // Next state logic
    always_comb begin
        next_axi_state = axi_state;

        case (axi_state)
            AXI_IDLE: begin
                if (abif.arvalid && abif.arready) begin
                    next_axi_state = AXI_READ_WAIT;
                end
            end

            AXI_READ_WAIT: begin
                if (flash_read_done) begin
                    next_axi_state = AXI_READ_DATA;
                end
            end

            AXI_READ_DATA: begin
                if (abif.rready) begin
                    next_axi_state = AXI_IDLE;
                end
            end

            default: begin
            end
        endcase
    end

    // State output logic
    always_comb begin
        next_flash_read_addr = flash_read_addr;
        next_read_id = read_id;
        next_rvalid = rvalid;

        abif.awready = 1'b0;
        
        abif.wready = 1'b0;

        abif.bid = '0;
        abif.bvalid = 1'b0;
        abif.bresp = 2'b00;

        abif.arready = 1'b0;

        abif.rid = read_id;
        abif.rdata = flash_read_data;
        abif.rvalid = rvalid;
        abif.rresp = 2'b00;
        abif.rlast = 1'b1;

        flash_read_start = 1'b0;

        case (axi_state)
            AXI_IDLE: begin
                // Only proceed on the rising edge of the flash clock
                // Makes sure the flash_read_start signal takes effect
                if (flash_clk_strobe) begin
                    abif.arready = 1'b1;
                end
                
                if (abif.arvalid && abif.arready) begin
                    next_read_id = abif.arid;
                    next_flash_read_addr = {abif.araddr[31:2], 2'b00};  // Align
                    flash_read_start = 1'b1;
                end
            end

            AXI_READ_WAIT: begin
                if (flash_read_done) begin
                    next_rvalid = 1'b1;
                end
            end

            AXI_READ_DATA: begin
                if (abif.rready) begin
                    next_rvalid = 1'b0;
                    next_flash_read_addr = '0;
                    next_read_id = '0;
                end
            end

            default: begin
            end
        endcase
    end

endmodule