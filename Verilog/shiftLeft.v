`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/21/2023 10:04:33 AM
// Design Name: 
// Module Name: shiftLeft
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


module shiftLeft#(parameter N =32)(
input [N-1:0] inp,
output [N-1:0] out
    );
    assign out= {inp[N-2:0], 1'b0};
endmodule
