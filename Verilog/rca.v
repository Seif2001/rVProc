`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/14/2023 08:54:28 AM
// Design Name: 
// Module Name: RCA
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


module RCA(
input [31:0] A, B,
output wire [32:0] Sum
    );
    
genvar i;
 
wire [32:0]cIn;

assign cIn[0]=1'b0;

generate
  for (i=0; i<=31; i=i+1) begin : genbit
     FullAdder inst(A[i],B[i],cIn[i],Sum[i],cIn[i+1]);
        
  end
  endgenerate
  
      assign Sum[32] = cIn[32];
endmodule
