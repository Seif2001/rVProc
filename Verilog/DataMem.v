`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/07/2023 09:13:01 AM
// Design Name: 
// Module Name: DataMem
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


module DataMem
 (input clk, input MemRead, input MemWrite,
 input [5:0] addr, input [31:0] data_in,input [3:0] read_write, output reg [31:0] data_out);
 reg [7:0] mem [0:63];
 initial begin
 {mem[3],mem[2],mem[1],mem[0]}=32'd17;
 mem[5] = 8'd36;
 //mem[7:4]=32'd9;
 //mem[11:8]=32'd6;
 //mem[15:12] = 32'd1;
 end 


 //assign data_out =(MemRead)? mem[addr]:0;//this can't be written inside an always block
 always @(*) begin
 if (MemRead)begin
    if(read_write==4'b1000)//lb
        data_out={{25{mem[addr][7]}},mem[addr][6:0]};
    else if(read_write==4'b1001)//lh
         data_out={{17{mem[addr+1][7]}},mem[addr+1][6:0],mem[addr]};
    else if(read_write==4'b1010)//lw
         data_out={mem[addr+3],mem[addr+2],mem[addr+1],mem[addr]};
    else if(read_write==4'b1100)//lbu
         data_out=mem[addr];
    else if(read_write==4'b1101)//lhu
              data_out={mem[addr+1],mem[addr]};
                  
     
    end
 end
 always @(posedge clk) begin
 if (MemWrite) begin
    if(read_write==4'b1011)//sb
        mem[addr]= data_in;
     else if(read_write==4'b1111)//sw
        {mem[addr+3], mem[addr+2], mem[addr+1], mem[addr]}= data_in;
     else if(read_write==4'b1110)//sh
               {mem[addr+1], mem[addr]}= data_in;
        
    

    end
 end
 
endmodule
