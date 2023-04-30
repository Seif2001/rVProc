`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/28/2023 09:05:59 AM
// Design Name: 
// Module Name: N-bitReg
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


module NbitReg#(parameter N = 32)(
input clk, rst,
input[4:0] RS1,
input[4:0] RS2,
input[4:0] RD,
input[N-1:0] WData,
input regWrite,
output[N-1:0] R1,
output[N-1:0] R2
    );
    reg [N-1:0] RegFile[31:0];
    assign R1 = RegFile[RS1];
    assign R2 = RegFile[RS2];
    integer i;
    always@(posedge(clk), posedge (rst)) begin
    if(rst == 1'b1) begin
        
        //initialization
            for(i =0; i<32; i = i+1)begin : genbit
                 RegFile[i] = 0;
            end
            
     end
        //endgenerate
      else   if(regWrite && RD!=0) begin
             RegFile[RD] = WData;
        end
        
    end
      
    
    
endmodule
