`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/28/2023 10:33:19 AM
// Design Name: 
// Module Name: ALU
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


module ALU(input [31:0] A,B,
input [3:0]sel ,
output zeroF,
output reg[31:0] out

    );
    wire [31:0]sum;
    wire  [31:0]  Bcomp = sel [2]?  ~B+1 :B;
       RCA rca(.A(A),.B( Bcomp),.Sum(sum));
//     RCA(.A(A), .B(Bcomp), .sum(sum));

    always @(*) begin
    if (sel ==4'b0010) begin
    out =sum ;
    end
    else if (sel==4'b0110) begin
        out =sum ;
    end   
       else if (sel==4'b0000) begin
   out= A&B;
    end  
        else if (sel==4'b0001) begin
   out= A|B;
    end  
     
    end
    assign zeroF= ~|out;
endmodule
