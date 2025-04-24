// Hazard Unit
`timescale 1ns/1ns

`include "hazard_unit_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module hazard_unit (
    hazard_unit_if.hazard_unit hazif
);

    always_comb begin
        hazif.f2dif_en = 1;
        hazif.d2eif_en = 1;
        hazif.e2mif_en = 1;
        hazif.m2wif_en = 1;
        hazif.f2dif_flush = 0;
        hazif.d2eif_flush = 0;
        hazif.e2mif_flush = 0;
        hazif.m2wif_flush = 0;

        // Stop the pipeline when HALTed
        if(hazif.halt) begin
            hazif.f2dif_en = 0;
            hazif.d2eif_en = 0;
            hazif.e2mif_en = 0;
            hazif.m2wif_en = 0;
        end

        // Structural hazard - stall the pipeline to wait for a data write/read
        if((hazif.dwrite | hazif.dread) & (~hazif.dhit | ~hazif.ihit)) begin
            hazif.f2dif_en = 0;
            hazif.d2eif_en = 0;
            hazif.e2mif_en = 0;
            hazif.m2wif_en = 0;
        end
        // Stall to keep the pipeline from running away from the instruction reads
        else if(~hazif.dread & ~hazif.dwrite & ~hazif.ihit) begin
            hazif.f2dif_en = 0;
            hazif.d2eif_en = 0;
            hazif.e2mif_en = 0;
            hazif.m2wif_en = 0;
        end
        // Needed because during ld/st it has to wait for D access AND I access
        // Also gate with branch signal, if the branch signal is high
        // we don't need to worry about the stall anyway because we
        // will be about to flush those instructions.
        // else if(~hazif.ihit & ~hazif.branch_flush) begin
        //     // Give NOP to FETCH/DECODE
        //     hazif.f2dif_en = 0;
        //     hazif.d2eif_en = 1;
        //     hazif.d2eif_flush = 1;
        // end

        /*******************/
        /* Load Use Hazard */
        /*******************/

        // When the instruction following a load depends on the result of the load, we need to avoid
        // a load-use hazard. Data dependencies can be solved by forwarding because their result is
        // available at the end of the EX stage, right as the following instruction enters EX. However,
        // with a load use hazard, the result is only available at the end of the MEM stage. To resolve
        // it, we just need to stall one cycle, then we can forward the result to the next instruction.
        
        // The hazard unit will implement the one cycle delay, and the forwarding unit will take care of
        // the forward.

        // To implement the delay, we need to detect when a load instruction is in the EX stage, and the
        // instruction currently in the DECODE stage uses its result. If this occurs, we will stall the
        // DECODE and FETCH stages to let the load instruction get to MEM. Now, after the memory operation
        // completes, the load instruction will pass MEM, and its result will be forwarded to the
        // instruction entering EX, so it can use the result.
        
        // If (DECODE/EX.dread == 1) - Checks if a load instruction
        // Also gate with branch signal, if the branch signal is high
        // we don't need to worry about the load-use anyway because we
        // will be about to flush those instructions.
        else if(hazif.d2eif_dread & ~hazif.branch_flush) begin
            // If nonzero destination from load equals rs1 or rs2
            if(hazif.d2eif_rd != 0 && (hazif.d2eif_rd == hazif.f2dif_rs1 || hazif.d2eif_rd == hazif.f2dif_rs2)) begin
                // Stall fetch to decode
                hazif.f2dif_en = 0;
                // Give execute a bubble 
                hazif.d2eif_en = 1;
                hazif.d2eif_flush = 1;
            end
        end
        // Multiplier delay, wait to finish the multiplication
        else if(hazif.d2eif_mult & ~hazif.mult_ready) begin
            // Stall fetch to decode
            hazif.f2dif_en = 0;
            // Stall decode to execute
            hazif.d2eif_en = 0;
            // Give memory a bubble
            hazif.e2mif_en = 1;
            hazif.e2mif_flush = 1;
        end
        // Divider delay, wait to finish the division
        else if(hazif.d2eif_div & ~hazif.div_ready) begin
            // Stall fetch to decode
            hazif.f2dif_en = 0;
            // Stall decode to execute
            hazif.d2eif_en = 0;
            // Give memory a bubble
            hazif.e2mif_en = 1;
            hazif.e2mif_flush = 1;
        end
        // CSR instruction should be completed before execute continues
        // Don't care on flush because execute will be flushed anyway
        else if((hazif.ex_csr | hazif.mem_csr | hazif.wb_csr) & ~hazif.branch_flush) begin
            // Stall fetch to decode
            hazif.f2dif_en = 0;
            // Give execute a bubble
            hazif.d2eif_en = 1;
            hazif.d2eif_flush = 1;
        end

        /*******************/
        /* Control Hazards */
        /*******************/

        // We need to flush when the branch_flush signal is asserted by the branch unit.

        // The signal to branch comes from the memory stage after the instruction has
        // passed the execute stage. Therefore, if we get a branch, assert the flush signal
        // for fetch2decode, decode2execute, and execute2memory to make sure they are clear on
        // the next cycle.
        if(hazif.branch_flush & hazif.ihit) begin
            hazif.f2dif_flush = 1;
            hazif.d2eif_flush = 1;
            hazif.e2mif_flush = 1;
        end
    end
endmodule
