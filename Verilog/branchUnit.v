`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/13/2023 10:09:51 PM
// Design Name: 
// Module Name: branchUnit
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


module branchUnit(
input zf, cf, vf, sf, input[2:0] funct3,input[31:0] IR,
output reg flag
    );
    
    always@(*) begin
    if(`OPCODE == `OPCODE_JAL ||`OPCODE == `OPCODE_JALR  )  flag = 1'b1;
    else if(`OPCODE == `OPCODE_Branch)
    begin
        case(funct3)
            `BR_BEQ: if(zf == 1'b1) flag = 1'b1; else flag = 0;
            `BR_BNE: if(zf == 1'b0) flag = 1'b1; else flag = 0;
            `BR_BLT: if(sf != vf) flag = 1'b1; else flag = 0;
            `BR_BGE: if(sf == vf) flag = 1'b1; else flag = 0;
            `BR_BLTU: if(cf == 1'b1) flag = 1'b1; else flag = 0;
            `BR_BGEU: if(cf == 1'b0) flag = 1'b1; else flag = 0;
            default: flag = 1'b0;
        endcase
        end
    else flag = 0;
   end
            
endmodule
