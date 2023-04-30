`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/13/2023 09:58:13 PM
// Design Name: 
// Module Name: shifter
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


module shifter(
input [31:0] a, 
input [4:0] shamt, 
input [1:0] type, 
output reg [31:0] r
    );
    
    always@(*)begin
        case(type)
            2'b00:
                r = a<<shamt;
            2'b01:
                r = a<<<shamt;
            2'b01:
                r = a>>shamt;
        endcase
    end
endmodule
