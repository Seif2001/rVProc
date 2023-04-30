`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/21/2023 09:49:05 AM
// Design Name: 
// Module Name: mux2by1
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


module mux2by1#(parameter N=32)(
input[N-1:0] in1,
input [N-1:0]in2,
input sel,
output [N-1:0] out
    );
    assign out = (sel) ? in2:in1;
    
    
endmodule
