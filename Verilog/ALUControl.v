`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/01/2023 12:57:20 PM
// Design Name: 
// Module Name: ALUControl
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`include "defines.v"


module ALUControl(
    input[1:0] op,
    input[31:0] inst,
    output reg[3:0] sel
);

    always@(*) begin
        if(op == 0) sel = `ALU_ADD;
        if(op == 1) sel = `ALU_SUB;
        if(op == 2)begin
            if(inst[`IR_funct3] == `F3_ADD)begin
                if(inst[30] == 0) sel = `ALU_ADD;
                else sel = `ALU_SUB;
            end
            if(inst[14:12] == `F3_OR)begin
                if(inst[30] == 0) sel = `ALU_OR;
            end
            if(inst[14:12] == `F3_AND)begin
                if(inst[30] == 0) sel = `ALU_AND;
            end
            if(inst[14:12] == `F3_XOR)begin
                if(inst[30] == 0) sel = `ALU_XOR;
            end
            if(inst[14:12] == `F3_SRL)begin
               if(inst[30] == 0) sel = `ALU_SRL;
               else sel = `ALU_SRA;
            end
            if(inst[14:12]  == `F3_SLL)begin
               if(inst[30] == 0) sel = `ALU_SLL;
            end
            if(inst[14:12] == `F3_SLT)begin
               if(inst[30] == 0) sel = `ALU_SLT;
            end
            if(inst[14:12] == `F3_SLTU)begin
               if(inst[30] == 0) sel = `ALU_SLTU;
            if(inst[`IR_opcode] ==  `OPCODE_AUIPC) sel = `ALU_AUIPC;
            if(inst[`IR_opcode] ==  `OPCODE_LUI) sel = `ALU_LUI;

            end                
        end
    end
endmodule
