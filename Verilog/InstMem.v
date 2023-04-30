`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/07/2023 08:57:46 AM
// Design Name: 
// Module Name: InstMem
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


module InstMem (input [5:0] addr, output [31:0] data_out);
    reg [31:0] mem [0:63];
    assign data_out = mem[addr];
    initial begin

        $readmemh("C:/Users/elsha/Desktop/Arch_Project1/test_files/test1.hex", mem);

//        mem[0]=32'h00108093; //l1: addi x3, x1, 3
//        mem[1]=32'h00420213; //addi x5, x4, -3
//        mem[2]=32'h00500283; //bne x5, x3, l1
//        mem[3]=32'h001282b3; //aupc x9,32
//        mem[4]=32'h00108093; //jal x1, exit
//        mem[5]=32'hfe40cae3; //add x1, x3, x5
//        mem[6]=32'h00430333; //addi x6, x6,1
////       mem[7]=32'h004282b3; //jalr x0, 0(x1)

        
        
    end


endmodule