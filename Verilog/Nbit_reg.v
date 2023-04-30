`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/21/2023 09:08:17 AM
// Design Name: 
// Module Name: Nbit_reg
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


module Nbit_reg#(parameter N=32)(
input clk,
input load,
input reset,
input [N-1:0] data,
output reg [N-1:0] Q

    );
    always@(posedge(clk), posedge(reset)) begin
    if (reset==1'b1)
    Q<=0;
    else begin
    if (load== 1'b1)
    Q<=data;
    end
    end
endmodule
