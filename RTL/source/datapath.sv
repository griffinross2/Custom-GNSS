`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;

// interfaces (module)
`include "control_unit_if.vh"
`include "alu_if.vh"
`include "multiplier_if.vh"
`include "divider_if.vh"
`include "register_file_if.vh"
`include "hazard_unit_if.vh"
`include "exception_unit_if.vh"
`include "forward_unit_if.vh"
`include "branch_unit_if.vh"
`include "csr_if.vh"
`include "ahb_controller_if.vh"
`include "ahb_bus_if.vh"

// interface (pipeline)
`include "fetch_to_decode_if.vh"
`include "decode_to_execute_if.vh"
`include "execute_to_memory_if.vh"
`include "memory_to_writeback_if.vh"

module datapath #(
  parameter PC_INIT = 0
)(
  input logic clk, nrst,
  input logic [31:0] interrupt_in_sync,
  output logic halt,
  ahb_bus_if.controller_to_mux abif
);
  parameter NOP = 32'h00000013;

  (* mark_debug = "true" *) word_t pc;

  word_t pc_n;
  word_t forwarded_rdat1;
  word_t forwarded_rdat2;

  // Interface instantiation
  control_unit_if ctrlif();
  alu_if aluif();
  multiplier_if mulif();
  divider_if divif();
  register_file_if rfif();
  hazard_unit_if hazif();
  forward_unit_if fuif();
  branch_unit_if buif();
  csr_if csrif();
  exception_unit_if euif();
  ahb_controller_if amif();

  // Module instantiation
  (* keep_hierarchy = "yes" *) control_unit ctrl0(ctrlif);
  (* keep_hierarchy = "yes" *) alu alu0(aluif);
  (* keep_hierarchy = "yes" *) multiplier mul0(clk, nrst, mulif);
  (* keep_hierarchy = "yes" *) divider div0(clk, nrst, divif);
  (* keep_hierarchy = "yes" *) register_file rf0(clk, nrst, rfif);
  (* keep_hierarchy = "yes" *) hazard_unit haz0(hazif);
  (* keep_hierarchy = "yes" *) forward_unit for0(fuif);
  (* keep_hierarchy = "yes" *) branch_unit bu0(clk, nrst, buif);
  (* keep_hierarchy = "yes" *) csr csr0(clk, nrst, csrif);
  (* keep_hierarchy = "yes" *) exception_unit eu0(clk, nrst, euif);
  (* keep_hierarchy = "yes" *) ahb_controller am0(clk, nrst, amif, abif);

  fetch_to_decode_if      f2dif();
  decode_to_execute_if    d2eif();
  execute_to_memory_if    e2mif();
  memory_to_writeback_if  m2wif();

  // Hazard and Exception Unit to Pipeline
  always_comb begin
    // Enable signals
    f2dif.en = hazif.f2dif_en;
    d2eif.en = hazif.d2eif_en;
    e2mif.en = hazif.e2mif_en;
    m2wif.en = hazif.m2wif_en;

    // Flush signals, from hazard or exception units
    f2dif.flush = hazif.f2dif_flush | euif.f2dif_flush;
    d2eif.flush = hazif.d2eif_flush | euif.d2eif_flush;
    e2mif.flush = hazif.e2mif_flush | euif.e2mif_flush;
    m2wif.flush = hazif.m2wif_flush | euif.m2wif_flush;
    
    // Signals from other modules to the hazard unit
    hazif.halt = m2wif.halt;
    hazif.dread = e2mif.dread;
    hazif.dwrite = |e2mif.dwrite;
    hazif.ihit = amif.ihit;
    hazif.dhit = amif.dhit;
    hazif.d2eif_dread = d2eif.dread;
    hazif.f2dif_rs1 = ctrlif.rs1;
    hazif.f2dif_rs2 = ctrlif.rs2;
    hazif.d2eif_rd = d2eif.rd;
    hazif.d2eif_mult = d2eif.mult;
    hazif.mult_ready = mulif.ready;
    hazif.d2eif_div = d2eif.div;
    hazif.div_ready = divif.ready;
    hazif.branch_flush = buif.mem_flush;
    
    hazif.ex_csr = d2eif.csr_write;
    hazif.mem_csr = e2mif.csr_write;
    hazif.wb_csr = m2wif.csr_write;

    // Signals from other modules to the exception unit
    euif.illegal_inst = e2mif.illegal_inst;
    euif.interrupt_in_sync = interrupt_in_sync & {32{e2mif.en}};  // Allow through when mem is finishing
  end

  // Forwarding Unit to Pipeline
  always_comb begin
    fuif.ex_rsel1 = d2eif.rs1;
    fuif.ex_rsel2 = d2eif.rs2;
    fuif.mem_rd = e2mif.rd;
    fuif.wb_rd = m2wif.rd;
  end

  // Command and Status Registers to Exception Unit
  always_comb begin
    // Tell the CSR details about the exception/interrupt
    csrif.csr_exception = euif.exception;
    csrif.csr_exception_cause = euif.exception_cause;
    csrif.csr_exception_pc = euif.exception_pc;
    csrif.csr_mret = (e2mif.pc_ctrl == 3'd4); // MRET instruction restores MIE

    // Tell the exception unit about whether interrupts are enabled
    euif.interrupt_en = csrif.csr_interrupt_en;
    euif.mie = csrif.csr_mie;
    euif.mtvec_mode = csrif.csr_mtvec_mode;
    euif.mtvec_base = {csrif.csr_mtvec_base, 2'b0};
  end

  // PC
  always_ff @(posedge clk) begin
    if(~nrst) begin
      pc <= PC_INIT;
    end else begin
      pc <= pc_n;
    end
  end

  // STAGE 1: FETCH
  always_comb begin
    amif.iread = 1'b1;
    if((e2mif.dread | |e2mif.dwrite) & amif.ihit & ~amif.dhit) begin
      // If we already got the next instruction, don't assert iren (let dread/write thru)
      amif.iread = 1'b0;
    end
    // If branch is predicted, fetch from the predicted target address
    buif.fetch_pc = pc;
    amif.iaddr = pc;
  end

  // STAGE 1 => STAGE 2: FETCH => DECODE
  always_ff @(posedge clk) begin
    if(~nrst) begin
      f2dif.pc <= PC_INIT;
      f2dif.branch_predict <= 0;
      f2dif.branch_target <= 0;
      f2dif.inst_latch <= 0;
    end else if (f2dif.en & f2dif.flush) begin
      f2dif.pc <= f2dif.pc;
      f2dif.branch_predict <= 0;
      f2dif.branch_target <= 0;
      f2dif.inst_latch <= 0;
    end else if (f2dif.en) begin
      f2dif.pc <= pc;
      f2dif.branch_predict <= buif.fetch_predict;
      f2dif.branch_target <= buif.fetch_target;
      f2dif.inst_latch <= 1;
    end else begin
      if(amif.ihit | f2dif.flush) begin
        f2dif.inst_latch <= 0;
      end
    end
  end

  // STAGE 2: DECODE

  // Latch the instruction if the pipeline is stalled after the AHB-Lite is ready
  // so that it won't be lost.
  word_t inst;
  always_ff @(posedge clk) begin
    if(~nrst) begin
      inst <= NOP;
    end else if (f2dif.flush) begin
      inst <= NOP;
    end else if (f2dif.inst_latch) begin
      inst <= amif.iload;
    end
  end

  always_comb begin
    ctrlif.inst = NOP;  // Default
    if(f2dif.inst_latch) begin
      // Get instruction from AHB bus
      ctrlif.inst = amif.iload;
    end else begin
      // The pipeline is stalled, so we need to use the instruction
      // saved from the bus. That or we flushed.
      ctrlif.inst = inst;
    end
  end

  always_comb begin
    rfif.rsel1 = ctrlif.rs1;
    rfif.rsel2 = ctrlif.rs2;
  end

  // STAGE 2 => STAGE 3: DECODE => EXECUTE
  logic mult_en_strobe;
  always_ff @(posedge clk) begin
    if(~nrst) begin
      d2eif.pc <= PC_INIT;
      d2eif.halt <= '0;
      d2eif.alu_op <= ALU_ADD;
      d2eif.rd <= '0;
      d2eif.rs1 <= '0;
      d2eif.rs2 <= '0;
      d2eif.dwrite <= '0;
      d2eif.dread <= '0;
      d2eif.immediate <= '0;
      d2eif.alu_src1 <= '0;
      d2eif.alu_src2 <= '0;
      d2eif.reg_wr_src <= '0;
      d2eif.reg_wr_mem <= '0;
      d2eif.reg_wr_mem_signed <= '0;
      d2eif.branch_pol <= '0;
      d2eif.pc_ctrl <= '0;
      d2eif.rdat1 <= '0;
      d2eif.rdat2 <= '0;
      d2eif.mult <= '0;
      d2eif.mult_half <= '0;
      d2eif.mult_signed_a <= '0;
      d2eif.mult_signed_b <= '0;
      d2eif.div <= '0;
      d2eif.div_rem <= '0;
      d2eif.div_signed <= '0;
      d2eif.branch_predict <= 0;
      d2eif.branch_target <= 0;
      d2eif.csr_write <= '0;
      d2eif.csr_waddr <= '0;
      d2eif.csr_wr_op <= '0;
      d2eif.csr_wr_imm <= '0;
      d2eif.illegal_inst <= '0;
      mult_en_strobe <= 1'b1;
    end else if (d2eif.en & d2eif.flush) begin
      d2eif.pc <= d2eif.pc;
      d2eif.halt <= '0;
      d2eif.alu_op <= ALU_ADD;
      d2eif.rd <= '0;
      d2eif.rs1 <= '0;
      d2eif.rs2 <= '0;
      d2eif.dwrite <= '0;
      d2eif.dread <= '0;
      d2eif.immediate <= '0;
      d2eif.alu_src1 <= '0;
      d2eif.alu_src2 <= '0;
      d2eif.reg_wr_src <= '0;
      d2eif.reg_wr_mem <= '0;
      d2eif.reg_wr_mem_signed <= '0;
      d2eif.branch_pol <= '0;
      d2eif.pc_ctrl <= '0;
      d2eif.rdat1 <= '0;
      d2eif.rdat2 <= '0;
      d2eif.mult <= '0;
      d2eif.mult_half <= '0;
      d2eif.mult_signed_a <= '0;
      d2eif.mult_signed_b <= '0;
      d2eif.div <= '0;
      d2eif.div_rem <= '0;
      d2eif.div_signed <= '0;
      d2eif.branch_predict <= 0;
      d2eif.branch_target <= 0;
      d2eif.csr_write <= '0;
      d2eif.csr_waddr <= '0;
      d2eif.csr_wr_op <= '0;
      d2eif.csr_wr_imm <= '0;
      d2eif.illegal_inst <= '0;
      mult_en_strobe <= 1'b1;
    end else if (d2eif.en) begin
      d2eif.pc <= f2dif.pc;
      d2eif.halt <= ctrlif.halt;
      d2eif.alu_op <= ctrlif.alu_op;
      d2eif.rd <= ctrlif.rd;
      d2eif.rs1 <= ctrlif.rs1;
      d2eif.rs2 <= ctrlif.rs2;
      d2eif.dwrite <= ctrlif.dwrite;
      d2eif.dread <= ctrlif.dread;
      d2eif.immediate <= ctrlif.immediate;
      d2eif.alu_src1 <= ctrlif.alu_src1;
      d2eif.alu_src2 <= ctrlif.alu_src2;
      d2eif.reg_wr_src <= ctrlif.reg_wr_src;
      d2eif.reg_wr_mem <= ctrlif.reg_wr_mem;
      d2eif.reg_wr_mem_signed <= ctrlif.reg_wr_mem_signed;
      d2eif.branch_pol <= ctrlif.branch_pol;
      d2eif.pc_ctrl <= ctrlif.pc_ctrl;
      d2eif.rdat1 <= rfif.rdat1;
      d2eif.rdat2 <= rfif.rdat2;
      d2eif.mult <= ctrlif.mult;
      d2eif.mult_half <= ctrlif.mult_half;
      d2eif.mult_signed_a <= ctrlif.mult_signed_a;
      d2eif.mult_signed_b <= ctrlif.mult_signed_b;
      d2eif.div <= ctrlif.div;
      d2eif.div_rem <= ctrlif.div_rem;
      d2eif.div_signed <= ctrlif.div_signed;
      d2eif.branch_predict <= f2dif.branch_predict;
      d2eif.branch_target <= f2dif.branch_target;
      d2eif.csr_write <= ctrlif.csr_write;
      d2eif.csr_waddr <= ctrlif.csr_waddr;
      d2eif.csr_wr_op <= ctrlif.csr_wr_op;
      d2eif.csr_wr_imm <= ctrlif.csr_wr_imm;
      d2eif.illegal_inst <= ctrlif.illegal_inst;
      mult_en_strobe <= 1'b1;
    end else begin
      mult_en_strobe <= 1'b0;
    end
  end

  // STAGE 3: EXECUTE
  word_t execute_alu_out;
  always_comb begin
    // forwarding unit a (0 - rdat1, 1 - e2m alu_out, 2 - m2w alu_out)
    casez (fuif.forward_a_to_ex)
      2'd0: forwarded_rdat1 = d2eif.rdat1;
      2'd1: begin
        casez(e2mif.reg_wr_src)
          2'd2: forwarded_rdat1 = e2mif.pc + 32'd4;
          default: forwarded_rdat1 = e2mif.alu_out;
        endcase
      end
      2'd2: begin
        casez(m2wif.reg_wr_src)
          2'd1: begin
            // Memory writeback
            casez(m2wif.reg_wr_mem)
              2'd0: begin
                // LB/LBU
                if(m2wif.reg_wr_mem_signed) begin
                  casez(m2wif.alu_out[1:0])
                    // Alignment
                    2'b00: forwarded_rdat1 = {{24{m2wif.dload[7]}}, m2wif.dload[7:0]};
                    2'b01: forwarded_rdat1 = {{24{m2wif.dload[15]}}, m2wif.dload[15:8]};
                    2'b10: forwarded_rdat1 = {{24{m2wif.dload[23]}}, m2wif.dload[23:16]};
                    2'b11: forwarded_rdat1 = {{24{m2wif.dload[31]}}, m2wif.dload[31:24]};
                  endcase
                end else begin
                  casez(m2wif.alu_out[1:0])
                    // Alignment
                    2'b00: forwarded_rdat1 = {24'b0, m2wif.dload[7:0]};
                    2'b01: forwarded_rdat1 = {24'b0, m2wif.dload[15:8]};
                    2'b10: forwarded_rdat1 = {24'b0, m2wif.dload[23:16]};
                    2'b11: forwarded_rdat1 = {24'b0, m2wif.dload[31:24]};
                  endcase
                end
              end
              2'd1: begin
                // LH/LHU
                if(m2wif.reg_wr_mem_signed) begin
                  casez(m2wif.alu_out[1])
                    // Alignment
                    1'b0: forwarded_rdat1 = {{16{m2wif.dload[15]}}, m2wif.dload[15:0]};
                    1'b1: forwarded_rdat1 = {{16{m2wif.dload[31]}}, m2wif.dload[31:16]};
                  endcase
                end else begin
                  casez(m2wif.alu_out[1])
                    // Alignment
                    1'b0: forwarded_rdat1 = {16'b0, m2wif.dload[15:0]};
                    1'b1: forwarded_rdat1 = {16'b0, m2wif.dload[31:16]};
                  endcase
                end
              end
              // LW
              2'd2: forwarded_rdat1 = m2wif.dload[31:0];
              default: forwarded_rdat1 = m2wif.alu_out;
            endcase
          end
          2'd2: forwarded_rdat1 = m2wif.pc + 32'd4;
          default: forwarded_rdat1 = m2wif.alu_out;
        endcase
      end
      default: forwarded_rdat1 = d2eif.rdat1;
    endcase
    // forwarding unit b (0 - rdat2, 1 - e2m alu_out, 2 - m2w alu_out)
    casez (fuif.forward_b_to_ex)
      2'd0: forwarded_rdat2 = d2eif.rdat2;
      2'd1: begin
        casez(e2mif.reg_wr_src)
          2'd2: forwarded_rdat2 = e2mif.pc + 32'd4;
          default: forwarded_rdat2 = e2mif.alu_out;
        endcase
      end
      2'd2: begin
        casez(m2wif.reg_wr_src)
          2'd1: begin
            // Memory writeback
            casez(m2wif.reg_wr_mem)
              2'd0: begin
                // LB/LBU
                if(m2wif.reg_wr_mem_signed) begin
                  casez(m2wif.alu_out[1:0])
                    // Alignment
                    2'b00: forwarded_rdat2 = {{24{m2wif.dload[7]}}, m2wif.dload[7:0]};
                    2'b01: forwarded_rdat2 = {{24{m2wif.dload[15]}}, m2wif.dload[15:8]};
                    2'b10: forwarded_rdat2 = {{24{m2wif.dload[23]}}, m2wif.dload[23:16]};
                    2'b11: forwarded_rdat2 = {{24{m2wif.dload[31]}}, m2wif.dload[31:24]};
                  endcase
                end else begin
                  casez(m2wif.alu_out[1:0])
                    // Alignment
                    2'b00: forwarded_rdat2 = {24'b0, m2wif.dload[7:0]};
                    2'b01: forwarded_rdat2 = {24'b0, m2wif.dload[15:8]};
                    2'b10: forwarded_rdat2 = {24'b0, m2wif.dload[23:16]};
                    2'b11: forwarded_rdat2 = {24'b0, m2wif.dload[31:24]};
                  endcase
                end
              end
              2'd1: begin
                // LH/LHU
                if(m2wif.reg_wr_mem_signed) begin
                  casez(m2wif.alu_out[1])
                    // Alignment
                    1'b0: forwarded_rdat2 = {{16{m2wif.dload[15]}}, m2wif.dload[15:0]};
                    1'b1: forwarded_rdat2 = {{16{m2wif.dload[31]}}, m2wif.dload[31:16]};
                  endcase
                end else begin
                  casez(m2wif.alu_out[1])
                    // Alignment
                    1'b0: forwarded_rdat2 = {16'b0, m2wif.dload[15:0]};
                    1'b1: forwarded_rdat2 = {16'b0, m2wif.dload[31:16]};
                  endcase
                end 
              end
              // LW
              2'd2: forwarded_rdat2 = m2wif.dload[31:0];
              default: forwarded_rdat2 = m2wif.alu_out;
            endcase
          end
          2'd2: forwarded_rdat2 = m2wif.pc + 32'd4;
          default: forwarded_rdat2 = m2wif.alu_out;
        endcase
      end
      default: forwarded_rdat2 = d2eif.rdat2;
    endcase

    // ALU source 1 mux (0 - rs1, 1 - pc)
    aluif.a = d2eif.alu_src1 ? d2eif.pc : forwarded_rdat1;
    // ALU source 2 mux (0 - rs2, 1 - immediate)
    aluif.b = d2eif.alu_src2 ? d2eif.immediate : forwarded_rdat2;
    // ALU control
    aluif.op = d2eif.alu_op;

    // Multiplier Unit
    mulif.a = forwarded_rdat1;
    mulif.b = forwarded_rdat2;
    mulif.en = d2eif.mult & mult_en_strobe;
    mulif.is_signed_a = d2eif.mult_signed_a;
    mulif.is_signed_b = d2eif.mult_signed_b;

    // Divider Unit
    divif.a = forwarded_rdat1;
    divif.b = forwarded_rdat2;
    divif.en = d2eif.div & mult_en_strobe;
    divif.is_signed = d2eif.div_signed;

    execute_alu_out = aluif.out;
    if(d2eif.mult) execute_alu_out = (d2eif.mult_half ? mulif.out[63:32] : mulif.out[31:0]);
    if(d2eif.div) execute_alu_out = (d2eif.div_rem ? divif.r : divif.q);
  end

  // STAGE 3 => STAGE 4: EXECUTE => MEMORY
  always_ff @(posedge clk) begin
    if(~nrst) begin
      e2mif.pc <= PC_INIT;
      e2mif.halt <= '0;
      e2mif.rd <= '0;
      e2mif.rs1 <= '0;
      e2mif.dwrite <= '0;
      e2mif.dread <= '0;
      e2mif.reg_wr_src <= '0;
      e2mif.reg_wr_mem <= '0;
      e2mif.reg_wr_mem_signed <= '0;
      e2mif.branch_pol <= '0;
      e2mif.pc_ctrl <= '0;
      e2mif.rdat1 <= '0;
      e2mif.rdat2 <= '0;
      e2mif.pc_plus_imm <= '0;
      e2mif.alu_out <= '0;
      e2mif.alu_zero <= '0;
      e2mif.branch_predict <= 0;
      e2mif.branch_target <= 0;
      e2mif.csr_write <= '0;
      e2mif.csr_waddr <= '0;
      e2mif.csr_wr_op <= '0;
      e2mif.csr_wr_imm <= '0;
      e2mif.illegal_inst <= '0;
    end else if (e2mif.en & e2mif.flush) begin
      e2mif.pc <= d2eif.pc;
      e2mif.halt <= '0;
      e2mif.rd <= '0;
      e2mif.rs1 <= '0;
      e2mif.dwrite <= '0;
      e2mif.dread <= '0;
      e2mif.reg_wr_src <= '0;
      e2mif.reg_wr_mem <= '0;
      e2mif.reg_wr_mem_signed <= '0;
      e2mif.branch_pol <= '0;
      e2mif.pc_ctrl <= '0;
      e2mif.rdat1 <= '0;
      e2mif.rdat2 <= '0;
      e2mif.pc_plus_imm <= '0;
      e2mif.alu_out <= '0;
      e2mif.alu_zero <= '0;
      e2mif.branch_predict <= 0;
      e2mif.branch_target <= 0;
      e2mif.csr_write <= '0;
      e2mif.csr_waddr <= '0;
      e2mif.csr_wr_op <= '0;
      e2mif.csr_wr_imm <= '0;
      e2mif.illegal_inst <= '0;
    end else if (e2mif.en) begin
      e2mif.pc <= d2eif.pc;
      e2mif.halt <= d2eif.halt;
      e2mif.rd <= d2eif.rd;
      e2mif.rs1 <= d2eif.rs1;
      e2mif.dwrite <= d2eif.dwrite;
      e2mif.dread <= d2eif.dread;
      e2mif.dwrite_short <= d2eif.dwrite;
      e2mif.dread_short <= d2eif.dread;
      e2mif.reg_wr_src <= d2eif.reg_wr_src;
      e2mif.reg_wr_mem <= d2eif.reg_wr_mem;
      e2mif.reg_wr_mem_signed <= d2eif.reg_wr_mem_signed;
      e2mif.branch_pol <= d2eif.branch_pol;
      e2mif.pc_ctrl <= d2eif.pc_ctrl;
      e2mif.rdat1 <= forwarded_rdat1;
      e2mif.rdat2 <= forwarded_rdat2;
      e2mif.pc_plus_imm <= (d2eif.pc + {d2eif.immediate[31:1], 1'b0});
      e2mif.alu_out <= execute_alu_out;
      e2mif.alu_zero <= aluif.zero;
      e2mif.branch_predict <= d2eif.branch_predict;
      e2mif.branch_target <= d2eif.branch_target;
      e2mif.csr_write <= d2eif.csr_write;
      e2mif.csr_waddr <= d2eif.csr_waddr;
      e2mif.csr_wr_op <= d2eif.csr_wr_op;
      e2mif.csr_wr_imm <= d2eif.csr_wr_imm;
      e2mif.illegal_inst <= d2eif.illegal_inst;
    end else if (amif.dhit) begin
      // Ensure local memory signals only cause one transaction
      e2mif.dwrite_short <= '0;
      e2mif.dread_short <= '0;
    end
  end

  // STAGE 4: MEMORY
  always_comb begin
    // Memory address for ld/st is the sum from ALU
    amif.daddr = e2mif.alu_out;
    
    // Memory data to store is from rs2:
    // Align the data to the proper part of the word
    amif.dwrite = e2mif.dwrite_short;
    casez(e2mif.dwrite_short)
      // No store
      2'b00: begin
        amif.dstore = e2mif.rdat2;
      end
      // Byte store
      2'b01: begin
        // Alignment
        casez(amif.daddr[1:0])
          2'b00: begin
            amif.dstore = {24'd0, e2mif.rdat2[7:0]};
          end
          2'b01: begin
            amif.dstore = {16'd0, e2mif.rdat2[7:0], 8'd0};
          end
          2'b10: begin
            amif.dstore = {8'd0, e2mif.rdat2[7:0], 16'd0};
          end
          2'b11: begin
            amif.dstore = {e2mif.rdat2[7:0], 24'd0};
          end
        endcase
      end
      // Halfword store
      2'b10: begin
        // Alignment
        casez(amif.daddr[1:0])
          2'b00, 2'b01: begin
            amif.dstore = {16'd0, e2mif.rdat2[15:0]};
          end
          2'b10, 2'b11: begin
            amif.dstore = {e2mif.rdat2[15:0], 16'd0};
          end
        endcase
      end
      // Word store
      2'b11: begin
        amif.dstore = e2mif.rdat2;
      end
      default: begin
        amif.dstore = e2mif.rdat2;
      end
    endcase

    amif.dread = e2mif.dread_short;
  end

  // STAGE 4 => STAGE 5: MEMORY => WRITEBACK
  always_ff @(posedge clk) begin
    if(~nrst) begin
      m2wif.pc <= PC_INIT;
      m2wif.halt <= '0;
      m2wif.rd <= '0;
      m2wif.rs1 <= '0;
      m2wif.reg_wr_src <= '0;
      m2wif.reg_wr_mem <= '0;
      m2wif.reg_wr_mem_signed <= '0;
      m2wif.alu_out <= '0;
      m2wif.dload <= '0;
      m2wif.rdat1 <= '0;
      m2wif.csr_write <= '0;
      m2wif.csr_waddr <= '0;
      m2wif.csr_wr_op <= '0;
      m2wif.csr_wr_imm <= '0;
    end else if (m2wif.en & m2wif.flush) begin
        m2wif.pc <= e2mif.pc;
        m2wif.halt <= '0;
        m2wif.rd <= '0;
        m2wif.rs1 <= '0;
        m2wif.reg_wr_src <= '0;
        m2wif.reg_wr_mem <= '0;
        m2wif.reg_wr_mem_signed <= '0;
        m2wif.alu_out <= '0;
        m2wif.dload <= '0;
        m2wif.rdat1 <= '0;
        m2wif.csr_write <= '0;
        m2wif.csr_waddr <= '0;
        m2wif.csr_wr_op <= '0;
        m2wif.csr_wr_imm <= '0;
    end else if (m2wif.en) begin
        m2wif.pc <= e2mif.pc;
        m2wif.halt <= e2mif.halt;
        m2wif.rd <= e2mif.rd;
        m2wif.rs1 <= e2mif.rs1;
        m2wif.reg_wr_src <= e2mif.reg_wr_src;
        m2wif.reg_wr_mem <= e2mif.reg_wr_mem;
        m2wif.reg_wr_mem_signed <= e2mif.reg_wr_mem_signed;
        m2wif.alu_out <= e2mif.csr_write ? csrif.csr_rdata : e2mif.alu_out;
        m2wif.dload <= amif.dload;
        m2wif.rdat1 <= e2mif.rdat1;
        m2wif.csr_write <= e2mif.csr_write;
        m2wif.csr_waddr <= e2mif.csr_waddr;
        m2wif.csr_wr_op <= e2mif.csr_wr_op;
        m2wif.csr_wr_imm <= e2mif.csr_wr_imm;
    end
  end

  // STAGE 5: WRITEBACK
  always_comb begin
    // CSR calculation
    csrif.csr_raddr = m2wif.csr_waddr;
    casez(m2wif.csr_wr_op)
      2'd0: begin
        // Move (either rs1 or immediate)
        csrif.csr_wdata = m2wif.csr_wr_imm ? {27'd0, m2wif.rs1} : m2wif.rdat1;
      end
      2'd1: begin
        // Set (either rs1 or immediate)
        csrif.csr_wdata = csrif.csr_rdata | (m2wif.csr_wr_imm ? {27'd0, m2wif.rs1} : m2wif.rdat1);
      end
      2'd2: begin
        // Clear (either rs1 or immediate)
        csrif.csr_wdata = csrif.csr_rdata & ~(m2wif.csr_wr_imm ? {27'd0, m2wif.rs1} : m2wif.rdat1);
      end
      default: begin
        // Don't modify
        csrif.csr_wdata = csrif.csr_rdata;
      end
    endcase

    // Also writeback CSR here
    csrif.csr_waddr = m2wif.csr_waddr;
    csrif.csr_write = m2wif.csr_write & m2wif.en;

    // Register File writeback

    rfif.wsel = m2wif.rd;
    rfif.wen = 1; // If not writing, rd will be 0 anyway
    // Writeback source mux (0 - alu, 1 - memory, 2 - pc + 4)
    casez(m2wif.reg_wr_src)
      2'd0: rfif.wdat = m2wif.csr_write ? csrif.csr_rdata : m2wif.alu_out;
      2'd1: begin
        // Memory writeback
        casez(m2wif.reg_wr_mem)
          2'd0: begin
            // LB/LBU
            if(m2wif.reg_wr_mem_signed) begin
              casez(m2wif.alu_out[1:0])
                // Alignment
                2'b00: rfif.wdat = {{24{m2wif.dload[7]}}, m2wif.dload[7:0]};
                2'b01: rfif.wdat = {{24{m2wif.dload[15]}}, m2wif.dload[15:8]};
                2'b10: rfif.wdat = {{24{m2wif.dload[23]}}, m2wif.dload[23:16]};
                2'b11: rfif.wdat = {{24{m2wif.dload[31]}}, m2wif.dload[31:24]};
              endcase
            end else begin
              casez(m2wif.alu_out[1:0])
                // Alignment
                2'b00: rfif.wdat = {24'b0, m2wif.dload[7:0]};
                2'b01: rfif.wdat = {24'b0, m2wif.dload[15:8]};
                2'b10: rfif.wdat = {24'b0, m2wif.dload[23:16]};
                2'b11: rfif.wdat = {24'b0, m2wif.dload[31:24]};
              endcase
            end
          end
          2'd1: begin
            // LH/LHU
            if(m2wif.reg_wr_mem_signed) begin
              casez(m2wif.alu_out[1])
                // Alignment
                1'b0: rfif.wdat = {{16{m2wif.dload[15]}}, m2wif.dload[15:0]};
                1'b1: rfif.wdat = {{16{m2wif.dload[31]}}, m2wif.dload[31:16]};
              endcase
            end else begin
              casez(m2wif.alu_out[1])
                // Alignment
                1'b0: rfif.wdat = {16'b0, m2wif.dload[15:0]};
                1'b1: rfif.wdat = {16'b0, m2wif.dload[31:16]};
              endcase
            end
          end
          // LW
          2'd2: rfif.wdat = m2wif.dload[31:0];
          default: rfif.wdat = m2wif.alu_out;
        endcase
      end
      2'd2: rfif.wdat = m2wif.pc + 32'd4;
      default: rfif.wdat = m2wif.alu_out;
    endcase
  end

  // Program Counter Control
  always_comb begin
    pc_n = pc;

    // Give prior assumptions to branch unit
    buif.mem_predict = e2mif.branch_predict;
    buif.mem_target = e2mif.branch_target;
    buif.mem_pc = e2mif.pc;

    // Default to telling BU no branch
    buif.mem_branch = 1'b0;
    buif.mem_taken = 1'b0;
    buif.mem_target_res = e2mif.pc + 32'd4;

    // Signal to handle this branch instruction
    if(e2mif.en & |e2mif.pc_ctrl) begin
      buif.mem_branch = 1'b1;
    end
    
    // Branch resolution
    casez(e2mif.pc_ctrl)
      3'b001: begin
        buif.mem_target_res = e2mif.pc_plus_imm;
        if(e2mif.branch_pol ^ e2mif.alu_zero) begin
          // Resolve to taken
          buif.mem_taken = 1'b1;
        end
      end
      3'b010: begin
        // Unconditional
        buif.mem_taken = 1'b1;
        buif.mem_target_res = e2mif.pc_plus_imm;
      end
      3'b011: begin
        // Unconditional
        buif.mem_taken = 1'b1;
        buif.mem_target_res = e2mif.alu_out;
      end
      3'b100: begin
        // Unconditional
        buif.mem_taken = 1'b1;
        buif.mem_target_res = csrif.csr_mepc;
      end
      default: begin
        // Default to telling BU no branch
        buif.mem_taken = 1'b0;
        buif.mem_target_res = e2mif.pc + 32'd4;
      end
    endcase

    // Create the next PC state when the fetch stage is ready to proceed
    if (f2dif.en) begin
      // The branch predictor thought we shouldn't branch but we need to
      if (~e2mif.branch_predict & buif.mem_branch_miss) begin
        // We may have to jump to the resolved target address
        casez(e2mif.pc_ctrl)
          3'b001: begin
            pc_n = e2mif.pc_plus_imm;
          end
          3'b010: begin
            pc_n = e2mif.pc_plus_imm;
          end
          3'b011: begin
            pc_n = e2mif.alu_out;
          end
          3'b100: begin
            pc_n = csrif.csr_mepc;
          end
          default: begin
            pc_n = pc + 32'd4;
          end
        endcase
      end else if (e2mif.branch_predict & buif.mem_branch_miss) begin
        // The branch predictor either predicted a false branch, or the wrong destination
        casez(e2mif.pc_ctrl)
          3'b001: begin
            if(e2mif.branch_pol ^ e2mif.alu_zero) begin
              pc_n = e2mif.pc_plus_imm;
            end else begin
              // False branch
              pc_n = e2mif.pc + 32'd4;
            end
          end
          3'b010: begin
            pc_n = e2mif.pc_plus_imm;
          end
          3'b011: begin
            pc_n = e2mif.alu_out;
          end
          3'b100: begin
            pc_n = csrif.csr_mepc;
          end
          default: begin
            // False branch: go to the next instruction after the original branch
            pc_n = e2mif.pc + 32'd4;
          end
        endcase
      end else if (buif.fetch_predict) begin
        // If a flush isn't occuring, but we are making a prediction, we have to
        // update the PC to the predicted target address
        pc_n = buif.fetch_target;
      end else begin
        // The branch predictor was correct, just continue execution
        pc_n = pc + 32'd4;
      end
    end

    // Give exception cause PC
    euif.e2mif_pc = pc_n;

    // If an exception occured, jump to the exception handler
    if(euif.exception) begin
      pc_n = euif.exception_target;
    end
  end

  always_ff @(posedge clk) begin
    if(~nrst) begin
      halt <= '0;
    end else begin
      halt <= m2wif.halt | halt;
    end
  end
endmodule
