`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/14/2023 09:05:08 AM
// Design Name: 
// Module Name: processTB
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


module processTB();
reg clk, reset;
Proccessor test(.clk(clk), .reset(reset));
initial begin
clk =0;
forever
#5 clk=~clk;
end

initial begin
reset = 1;
#10;
reset = 0;
end
endmodule
