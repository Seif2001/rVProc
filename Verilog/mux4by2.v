`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/12/2023 01:44:50 PM
// Design Name: 
// Module Name: mux4by2
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


module mux4by2#(parameter N=32)(
input[N-1:0] i0,
input [N-1:0]i1,
input[N-1:0] i2,
input [N-1:0]i3,
input [1:0] s,
output[N-1:0] out
    );
    
    assign out = s[1]?(s[0]?i3:i2):(s[0]?i1:i0);
    
endmodule


