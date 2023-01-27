// ==============================================================
// Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.2 (64-bit)
// Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
// ==============================================================
`timescale 1 ns / 1 ps
module rt_imp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_PipelieOg (
address0, ce0, q0, address1, ce1, q1, address2, ce2, q2, address3, ce3, q3, reset,clk);

parameter DataWidth = 24;
parameter AddressWidth = 5;
parameter AddressRange = 32;

input[AddressWidth-1:0] address0;
input ce0;
output reg[DataWidth-1:0] q0;
input[AddressWidth-1:0] address1;
input ce1;
output reg[DataWidth-1:0] q1;
input[AddressWidth-1:0] address2;
input ce2;
output reg[DataWidth-1:0] q2;
input[AddressWidth-1:0] address3;
input ce3;
output reg[DataWidth-1:0] q3;
input reset;
input clk;

reg [DataWidth-1:0] ram0[0:AddressRange-1];
reg [DataWidth-1:0] ram1[0:AddressRange-1];

initial begin
    $readmemh("./rt_imp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_PipelieOg.dat", ram0);
    $readmemh("./rt_imp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_PipelieOg.dat", ram1);
end



always @(posedge clk)  
begin 
    if (ce0) 
    begin
        q0 <= ram0[address0];
    end
end



always @(posedge clk)  
begin 
    if (ce1) 
    begin
        q1 <= ram0[address1];
    end
end



always @(posedge clk)  
begin 
    if (ce2) 
    begin
        q2 <= ram1[address2];
    end
end



always @(posedge clk)  
begin 
    if (ce3) 
    begin
        q3 <= ram1[address3];
    end
end



endmodule

