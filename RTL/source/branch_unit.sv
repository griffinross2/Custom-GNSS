/****************************/
/*  Branch Prediction Unit  */
/* 2-bit Saturating Counter */
/****************************/
`timescale 1ns/1ns

`include "branch_unit_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

// Switch to always not taken for comparison
// `define ALWAYS_NOT_TAKEN

module branch_unit #(
    parameter BTB_BITS=5            // 32 entry buffers
) (
    input logic clk, nrst,
    branch_unit_if.branch_unit buif
);
    
    // Branch prediction buffer
    // Table of 2^BTB_BITS 2-bit entries
    branch_pred_t prediction_buffer [0:(1<<BTB_BITS)-1];
    branch_pred_t prediction_buffer_n [0:(1<<BTB_BITS)-1];

    // Branch target buffer
    // Table of 2^BTB_BITS 32-bit entries
    word_t target_buffer [0:(1<<BTB_BITS)-1];
    word_t target_buffer_n [0:(1<<BTB_BITS)-1];

    // State machine
    always_ff @(posedge clk) begin
        if (~nrst) begin
            prediction_buffer <= '{default: STRONG_NOT_TAKEN};
            target_buffer <= '{default: '0};
        end else begin
            prediction_buffer <= prediction_buffer_n;
            target_buffer <= target_buffer_n;
        end
    end

    
    always_comb begin
        // Next state
        // Default to no change
        prediction_buffer_n = prediction_buffer;
        target_buffer_n = target_buffer;
        buif.mem_flush = 0;

        // If the branch signal is high (branch resolved)
        // Update the prediction buffer and target buffer
        if (buif.mem_branch) begin
            // Update the prediction buffer
            case(prediction_buffer[buif.mem_pc[BTB_BITS+1:2]])
                STRONG_NOT_TAKEN: begin
                    if (buif.mem_taken) begin
                        prediction_buffer_n[buif.mem_pc[BTB_BITS+1:2]] = WEAK_NOT_TAKEN;
                    end
                end
                WEAK_NOT_TAKEN: begin
                    if (buif.mem_taken) begin
                        prediction_buffer_n[buif.mem_pc[BTB_BITS+1:2]] = WEAK_TAKEN;
                    end else begin
                        prediction_buffer_n[buif.mem_pc[BTB_BITS+1:2]] = STRONG_NOT_TAKEN;
                    end
                end
                WEAK_TAKEN: begin
                    if (buif.mem_taken) begin
                        prediction_buffer_n[buif.mem_pc[BTB_BITS+1:2]] = STRONG_TAKEN;
                    end else begin
                        prediction_buffer_n[buif.mem_pc[BTB_BITS+1:2]] = WEAK_NOT_TAKEN;
                    end
                end
                STRONG_TAKEN: begin
                    if (!buif.mem_taken) begin
                        prediction_buffer_n[buif.mem_pc[BTB_BITS+1:2]] = WEAK_TAKEN;
                    end
                end
            endcase

            // Update the target buffer
            target_buffer_n[buif.mem_pc[BTB_BITS+1:2]] = buif.mem_target_res;
        end

        // Output logic
        // An incorrect branch was taken
        // We predicted taken AND  (it was not taken    OR    we jumped to the wrong place)
        if (buif.mem_predict && (buif.mem_taken == 1'b0 || buif.mem_target_res != buif.mem_target)) begin
            // Flush the pipeline
            buif.mem_flush = 1;
            buif.mem_branch_miss = 1;
        // We predicted not taken      AND    it was taken
        end else if (~buif.mem_predict && buif.mem_taken == 1'b1) begin
            // Flush the pipeline
            buif.mem_flush = 1;
            buif.mem_branch_miss = 1;
        end else begin
            // Correct prediction, continue
            buif.mem_flush = 0;
            buif.mem_branch_miss = 0;
        end

        // Get target and prediction for the fetch stage
        buif.fetch_target = target_buffer[buif.fetch_pc[BTB_BITS+1:2]];
        `ifndef ALWAYS_NOT_TAKEN
        case (prediction_buffer[buif.fetch_pc[BTB_BITS+1:2]])
            STRONG_TAKEN, WEAK_TAKEN: begin
                // If the prediction is taken, fetch from the target buffer
                buif.fetch_predict = 1;
            end
            STRONG_NOT_TAKEN, WEAK_NOT_TAKEN: begin
                // Predict not taken
                buif.fetch_predict = 0;
            end
        endcase
        `endif
        `ifdef ALWAYS_NOT_TAKEN
        buif.fetch_predict = 0;
        `endif
    end

endmodule