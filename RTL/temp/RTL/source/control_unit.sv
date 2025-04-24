// Control Unit (instruction decode)
`timescale 1ns/1ns

`include "control_unit_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module control_unit (
    control_unit_if.control_unit ctrlif
);

// verilator lint_off UNUSEDSIGNAL
j_t j_inst;
u_t u_inst;
b_t b_inst;
s_t s_inst;
i_t i_inst;
r_t r_inst;
// verilator lint_on UNUSEDSIGNAL

always_comb begin
    j_inst = 32'h0;
    u_inst = 32'h0;
    b_inst = 32'h0;
    s_inst = 32'h0;
    i_inst = 32'h0;
    r_inst = 32'h0;
    ctrlif.alu_op = ALU_ADD;
    ctrlif.rs1 = '0;
    ctrlif.rs2 = '0;
    ctrlif.rd = '0;
    ctrlif.halt = 1'b0;
    ctrlif.dread = 1'b0;
    ctrlif.dwrite = 2'b0;
    ctrlif.immediate = 32'd0;
    ctrlif.alu_src1 = 1'd0;
    ctrlif.alu_src2 = 1'd0;
    ctrlif.reg_wr_src = 2'd0;
    ctrlif.reg_wr_mem = 2'b00;
    ctrlif.reg_wr_mem_signed = 1'b0;
    ctrlif.pc_ctrl = 3'd0;
    ctrlif.branch_pol = 1'b0;
    ctrlif.mult = 1'b0;
    ctrlif.mult_signed_a = 1'b0;
    ctrlif.mult_signed_b = 1'b0;
    ctrlif.mult_half = 1'b0;
    ctrlif.div = 1'b0;
    ctrlif.div_rem = 1'b0;
    ctrlif.div_signed = 1'b0;
    ctrlif.csr_write = 1'b0;
    ctrlif.csr_waddr = 12'd0;
    ctrlif.csr_wr_op = 2'b0;
    ctrlif.csr_wr_imm = 1'b0;
    ctrlif.illegal_inst = 1'b0;

    casez(ctrlif.inst[OP_W-1:0])
        RTYPE:
        begin
            // R-type instruction
            r_inst = r_t'(ctrlif.inst);
            
            ctrlif.rs1 = r_inst.rs1;
            ctrlif.rs2 = r_inst.rs2;
            ctrlif.rd = r_inst.rd;

            casez({r_inst.funct3, r_inst.funct7})
                {ADD_SUB_MUL, ADD_SRL}:     ctrlif.alu_op = ALU_ADD;    // ADD
                {ADD_SUB_MUL, SUB_SRA}:     ctrlif.alu_op = ALU_SUB;    // SUB
                {SLL_MULH, 7'd0}:           ctrlif.alu_op = ALU_SLL;    // SLL
                {SRL_SRA_DIVU, SUB_SRA}:    ctrlif.alu_op = ALU_SRA;    // SRA
                {SRL_SRA_DIVU, ADD_SRL}:    ctrlif.alu_op = ALU_SRL;    // SRL
                {AND_REMU, 7'd0}:           ctrlif.alu_op = ALU_AND;    // AND
                {OR_REM, 7'd0}:             ctrlif.alu_op = ALU_OR;     // OR
                {XOR_DIV, 7'd0}:            ctrlif.alu_op = ALU_XOR;    // XOR
                {SLT_MULHSU, 7'd0}:         ctrlif.alu_op = ALU_SLT;    // SLT
                {SLTU_MULHU, 7'd0}:         ctrlif.alu_op = ALU_SLTU;   // SLTU
                {ADD_SUB_MUL, MUL_DIV}: begin                   // MUL                 
                    ctrlif.mult = 1'b1;
                    ctrlif.mult_signed_a = 1'b1;
                    ctrlif.mult_signed_b = 1'b1;
                    ctrlif.mult_half = 1'b0;
                end
                {SLTU_MULHU, MUL_DIV}: begin                    // MULHU
                    ctrlif.mult = 1'b1;
                    ctrlif.mult_signed_a = 1'b0;
                    ctrlif.mult_signed_b = 1'b0;
                    ctrlif.mult_half = 1'b1;
                end
                {SLL_MULH, MUL_DIV}: begin                      // MULH
                    ctrlif.mult = 1'b1;
                    ctrlif.mult_signed_a = 1'b1;
                    ctrlif.mult_signed_b = 1'b1;
                    ctrlif.mult_half = 1'b1;
                end
                {SLT_MULHSU, MUL_DIV}: begin                    // MULHSU
                    ctrlif.mult = 1'b1;
                    ctrlif.mult_signed_a = 1'b1;
                    ctrlif.mult_signed_b = 1'b0;
                    ctrlif.mult_half = 1'b1;
                end
                {XOR_DIV, MUL_DIV}: begin                       // DIV
                    ctrlif.div = 1'b1;
                    ctrlif.div_signed = 1'b1;
                    ctrlif.div_rem = 1'b0;
                end
                {SRL_SRA_DIVU, MUL_DIV}: begin                  // DIVU
                    ctrlif.div = 1'b1;
                    ctrlif.div_signed = 1'b0;
                    ctrlif.div_rem = 1'b0;
                end
                {OR_REM, MUL_DIV}: begin                        // REM
                    ctrlif.div = 1'b1;
                    ctrlif.div_signed = 1'b1;
                    ctrlif.div_rem = 1'b1;
                end
                {AND_REMU, MUL_DIV}: begin                      // REMU
                    ctrlif.div = 1'b1;
                    ctrlif.div_signed = 1'b0;
                    ctrlif.div_rem = 1'b1;
                end
                default: begin
                    // Illegal instruction
                    ctrlif.illegal_inst = 1'b1;
                    ctrlif.halt = 1'b1;
                end
            endcase
        end
        ITYPE:
        begin
            // I-type instruction
            i_inst = i_t'(ctrlif.inst);
            ctrlif.rs1 = i_inst.rs1;
            ctrlif.rd = i_inst.rd;

            casez(i_inst.funct3)
                ADDI:           ctrlif.alu_op = ALU_ADD;    // ADD
                XORI:           ctrlif.alu_op = ALU_XOR;    // XOR
                ORI:            ctrlif.alu_op = ALU_OR;     // OR
                ANDI:           ctrlif.alu_op = ALU_AND;    // AND
                SLLI:           ctrlif.alu_op = ALU_SLL;    // SLL
                SRLI_SRAI: begin
                    if(i_inst.imm[10] == 1'b1) begin
                                ctrlif.alu_op = ALU_SRA;    // SRA
                    end else begin
                                ctrlif.alu_op = ALU_SRL;    // SRL
                    end
                end
                SLTI:           ctrlif.alu_op = ALU_SLT;    // SLT
                SLTIU:          ctrlif.alu_op = ALU_SLTU;   // SLTU
                default: begin
                    // Illegal instruction
                    ctrlif.halt = 1'b1;
                end
            endcase
            // I-type uses a sign-extended 12-bit immediate
            ctrlif.immediate = {{20{i_inst.imm[11]}}, i_inst.imm};
            ctrlif.alu_src2 = 1'd1;  // Second operand comes from immediate
        end
        ITYPE_LW:
        begin
            // I-type instruction
            i_inst = i_t'(ctrlif.inst);
            ctrlif.rs1 = i_inst.rs1;
            ctrlif.rd = i_inst.rd;

            case (funct3_ld_i_t'(i_inst.funct3))
                LW: begin
                    // Read a word and don't need to sign-extend
                    ctrlif.reg_wr_mem = 2'b10;
                    ctrlif.reg_wr_mem_signed = 1'b0;
                end 
                LH: begin
                    // Read a halfword and sign-extend
                    ctrlif.reg_wr_mem = 2'b01;
                    ctrlif.reg_wr_mem_signed = 1'b1;
                end
                LB: begin
                    // Read a byte and sign-extend
                    ctrlif.reg_wr_mem = 2'b00;
                    ctrlif.reg_wr_mem_signed = 1'b1;
                end
                LBU: begin
                    // Read a byte and don't need to sign-extend
                    ctrlif.reg_wr_mem = 2'b00;
                    ctrlif.reg_wr_mem_signed = 1'b0;
                end
                LHU: begin
                    // Read a halfword and don't need to sign-extend
                    ctrlif.reg_wr_mem = 2'b01;
                    ctrlif.reg_wr_mem_signed = 1'b0;
                end
                default: begin
                    // Illegal instruction
                    ctrlif.illegal_inst = 1'b1;
                    ctrlif.halt = 1'b1;
                end
            endcase

            ctrlif.alu_op = ALU_ADD;    // Add the offset

            // I-type uses a sign-extended 12-bit immediate
            ctrlif.immediate = {{20{i_inst.imm[11]}}, i_inst.imm};
            ctrlif.alu_src2 = 1'd1;  // Second operand comes from immediate

            // Tell memory to read data
            ctrlif.dread = 1'b1;

            // A load should write back from the memory
            ctrlif.reg_wr_src = 2'd1;
        end
        JALR: begin
            // I-type instruction
            i_inst = i_t'(ctrlif.inst);
            ctrlif.pc_ctrl = 3'd3;
            ctrlif.rs1 = i_inst.rs1;
            ctrlif.rd = i_inst.rd;

            ctrlif.alu_op = ALU_ADD;    // Add base to offset

            // I-type uses a sign-extended 12-bit immediate
            ctrlif.immediate = {{20{i_inst.imm[11]}}, i_inst.imm};
            ctrlif.alu_src2 = 1'd1;  // Second operand comes from immediate

            // Write PC + 4 back to Reg File
            ctrlif.reg_wr_src = 2'd2;
        end
        STYPE:
        begin
            // S-type instruction
            s_inst = s_t'(ctrlif.inst);
            ctrlif.rs1 = s_inst.rs1;
            ctrlif.rs2 = s_inst.rs2;
                        
            // Only handle SW, not other types

            ctrlif.alu_op = ALU_ADD;    // Add the offset

            // S-type uses a sign-extended 12-bit immediate (split in two)
            ctrlif.immediate = {{20{s_inst.imm2[6]}}, s_inst.imm2, s_inst.imm1};
            ctrlif.alu_src2 = 1'd1;  // Second operand comes from immediate

            // Tell memory to write data
            casez(s_inst.funct3)
                SW: ctrlif.dwrite = 2'b11;
                SH: ctrlif.dwrite = 2'b10;
                SB: ctrlif.dwrite = 2'b01;
                default: begin
                    // Illegal instruction
                    ctrlif.illegal_inst = 1'b1;
                    ctrlif.halt = 1'b1;
                end
            endcase
        end
        LUI:
        begin
            // U-type instruction
            u_inst = u_t'(ctrlif.inst);
            ctrlif.rd = u_inst.rd;

            ctrlif.alu_op = ALU_ADD;    // Will add the immediate with x0 and place in rd

            // U-type uses a 20-bit immediate
            ctrlif.immediate = {u_inst.imm, 12'b0};
            ctrlif.alu_src2 = 1'd1;  // Second operand comes from immediate
        end
        AUIPC:
        begin
            // U-type instruction
            u_inst = u_t'(ctrlif.inst);
            ctrlif.rd = u_inst.rd;

            ctrlif.alu_op = ALU_ADD;    // Will add the immediate with PC and place in rd

            // U-type uses a 20-bit immediate
            ctrlif.immediate = {u_inst.imm, 12'b0};
            ctrlif.alu_src1 = 1'd1; // First operand comes from PC
            ctrlif.alu_src2 = 1'd1; // Second operand comes from immediate
        end
        BTYPE:
        begin
            // SB-type instruction
            b_inst = b_t'(ctrlif.inst);
            ctrlif.pc_ctrl = 3'd1;
            ctrlif.rs1 = b_inst.rs1;
            ctrlif.rs2 = b_inst.rs2;

            casez({b_inst.funct3})
                BEQ: begin
                    // If rs1 - rs2 -> zero = 1, branch
                    ctrlif.alu_op = ALU_SUB;   
                    ctrlif.branch_pol = 1'b0;
                end
                BNE: begin
                    // If rs1 - rs2 -> zero = 0, branch
                    ctrlif.alu_op = ALU_SUB;
                    ctrlif.branch_pol = 1'b1;
                end
                BLT: begin
                    // If rs1 < rs2 -> zero = 0, branch
                    ctrlif.alu_op = ALU_SLT;
                    ctrlif.branch_pol = 1'b1;
                end
                BGE: begin
                    // If rs1 < rs2 -> zero = 1, branch
                    ctrlif.alu_op = ALU_SLT;
                    ctrlif.branch_pol = 1'b0;
                end
                BLTU: begin
                    // If unsigned rs1 < rs2 -> zero = 0, branch
                    ctrlif.alu_op = ALU_SLTU;
                    ctrlif.branch_pol = 1'b1;
                end
                BGEU: begin
                    // If unsigned rs1 < rs2 -> zero = 1, branch
                    ctrlif.alu_op = ALU_SLTU;
                    ctrlif.branch_pol = 1'b0;
                end
                default: begin
                    // Illegal instruction
                    ctrlif.illegal_inst = 1'b1;
                    ctrlif.halt = 1'b1;
                end
            endcase

            // B-type uses a sign-extended weird 12-bit immediate that is split in two
            ctrlif.immediate = {{19{b_inst.imm2[6]}}, b_inst.imm2[6], b_inst.imm1[0], b_inst.imm2[5:0], b_inst.imm1[4:1], 1'b0};
        end
        JAL:
        begin
            // UJ-type instruction
            j_inst = j_t'(ctrlif.inst);
            ctrlif.pc_ctrl = 3'd2;
            ctrlif.rd = j_inst.rd;

            // UJ-type uses a sign-extended weird 20-bit immediate
            ctrlif.immediate = {{11{j_inst.imm[19]}}, j_inst.imm[19], j_inst.imm[7:0], j_inst.imm[8], j_inst.imm[18:9], 1'b0};

            // Write PC + 4 back to the destination register
            ctrlif.reg_wr_src = 2'd2;
        end
        SYSTEM:
        begin
            // I-type instruction
            i_inst = i_t'(ctrlif.inst);
            casez(funct3_system_i_t'(i_inst.funct3))
                ENV_CALL_BREAK_MRET: begin
                    casez(imm_system_i_t'(i_inst.imm))
                        // NOP
                        ECALL: begin end
                        // Halt
                        EBREAK: ctrlif.halt = 1'b1;
                        // MRET
                        MRET: ctrlif.pc_ctrl = 3'd4; // exception return
                        // Illegal instruction
                        default: begin
                            ctrlif.illegal_inst = 1'b1;
                            ctrlif.halt = 1'b1;
                        end
                    endcase
                end
                CSRRW: begin
                    ctrlif.csr_write = 1'b1;
                    ctrlif.csr_waddr = i_inst.imm;
                    ctrlif.csr_wr_op = 2'd0;
                    ctrlif.csr_wr_imm = 1'b0;
                    ctrlif.rs1 = i_inst.rs1;
                    ctrlif.rd = i_inst.rd;

                    // ALU is not used
                end
                CSRRS: begin
                    ctrlif.csr_write = 1'b1;
                    ctrlif.csr_waddr = i_inst.imm;
                    ctrlif.csr_wr_op = 2'd1;
                    ctrlif.csr_wr_imm = 1'b0;
                    ctrlif.rs1 = i_inst.rs1;
                    ctrlif.rd = i_inst.rd;

                    // ALU is not used
                end
                CSRRC: begin
                    ctrlif.csr_write = 1'b1;
                    ctrlif.csr_waddr = i_inst.imm;
                    ctrlif.csr_wr_op = 2'd2;
                    ctrlif.csr_wr_imm = 1'b0;
                    ctrlif.rs1 = i_inst.rs1;
                    ctrlif.rd = i_inst.rd;

                    // ALU is not used
                end
                CSRRWI: begin
                    ctrlif.csr_write = 1'b1;
                    ctrlif.csr_waddr = i_inst.imm;
                    ctrlif.csr_wr_op = 2'd0;
                    ctrlif.csr_wr_imm = 1'b1;
                    ctrlif.rs1 = i_inst.rs1;
                    ctrlif.rd = i_inst.rd;

                    // ALU is not used
                end
                CSRRSI: begin
                    ctrlif.csr_write = 1'b1;
                    ctrlif.csr_waddr = i_inst.imm;
                    ctrlif.csr_wr_op = 2'd1;
                    ctrlif.csr_wr_imm = 1'b1;
                    ctrlif.rs1 = i_inst.rs1;
                    ctrlif.rd = i_inst.rd;

                    // ALU is not used
                end
                CSRRCI: begin
                    ctrlif.csr_write = 1'b1;
                    ctrlif.csr_waddr = i_inst.imm;
                    ctrlif.csr_wr_op = 2'd2;
                    ctrlif.csr_wr_imm = 1'b1;
                    ctrlif.rs1 = i_inst.rs1;
                    ctrlif.rd = i_inst.rd;

                    // ALU is not used
                end
                // Illegal instruction
                default: begin
                    ctrlif.illegal_inst = 1'b1;
                    ctrlif.halt = 1'b1;
                end
            endcase
        end
        default: begin
            // Illegal instruction
            ctrlif.illegal_inst = 1'b1;
            ctrlif.halt = 1'b1;
        end
    endcase
end

endmodule
