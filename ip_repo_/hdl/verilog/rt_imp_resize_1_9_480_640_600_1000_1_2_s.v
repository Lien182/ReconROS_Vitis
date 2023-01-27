// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL v2021.2 (64-bit)
// Version: 2021.2
// Copyright (C) Copyright 1986-2021 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

`timescale 1 ns / 1 ps 

module rt_imp_resize_1_9_480_640_600_1000_1_2_s (
        ap_clk,
        ap_rst,
        ap_start,
        start_full_n,
        ap_done,
        ap_continue,
        ap_idle,
        ap_ready,
        start_out,
        start_write,
        in_mat_44_dout,
        in_mat_44_empty_n,
        in_mat_44_read,
        resized_mat_45_din,
        resized_mat_45_full_n,
        resized_mat_45_write
);

parameter    ap_ST_fsm_state1 = 2'd1;
parameter    ap_ST_fsm_state2 = 2'd2;

input   ap_clk;
input   ap_rst;
input   ap_start;
input   start_full_n;
output   ap_done;
input   ap_continue;
output   ap_idle;
output   ap_ready;
output   start_out;
output   start_write;
input  [23:0] in_mat_44_dout;
input   in_mat_44_empty_n;
output   in_mat_44_read;
output  [23:0] resized_mat_45_din;
input   resized_mat_45_full_n;
output   resized_mat_45_write;

reg ap_done;
reg ap_idle;
reg start_write;
reg in_mat_44_read;
reg resized_mat_45_write;

reg    real_start;
reg    start_once_reg;
reg    ap_done_reg;
(* fsm_encoding = "none" *) reg   [1:0] ap_CS_fsm;
wire    ap_CS_fsm_state1;
reg    internal_ap_ready;
wire    grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_start;
wire    grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_done;
wire    grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_idle;
wire    grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_ready;
wire    grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_in_mat_44_read;
wire   [23:0] grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_resized_mat_45_din;
wire    grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_resized_mat_45_write;
reg    grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_start_reg;
reg    ap_block_state1_ignore_call2;
wire    ap_CS_fsm_state2;
reg   [1:0] ap_NS_fsm;
reg    ap_block_state1;
reg    ap_ST_fsm_state1_blk;
reg    ap_ST_fsm_state2_blk;
wire    ap_ce_reg;

// power-on initialization
initial begin
#0 start_once_reg = 1'b0;
#0 ap_done_reg = 1'b0;
#0 ap_CS_fsm = 2'd1;
#0 grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_start_reg = 1'b0;
end

rt_imp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14(
    .ap_clk(ap_clk),
    .ap_rst(ap_rst),
    .ap_start(grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_start),
    .ap_done(grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_done),
    .ap_idle(grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_idle),
    .ap_ready(grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_ready),
    .in_mat_44_dout(in_mat_44_dout),
    .in_mat_44_empty_n(in_mat_44_empty_n),
    .in_mat_44_read(grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_in_mat_44_read),
    .resized_mat_45_din(grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_resized_mat_45_din),
    .resized_mat_45_full_n(resized_mat_45_full_n),
    .resized_mat_45_write(grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_resized_mat_45_write)
);

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        ap_CS_fsm <= ap_ST_fsm_state1;
    end else begin
        ap_CS_fsm <= ap_NS_fsm;
    end
end

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        ap_done_reg <= 1'b0;
    end else begin
        if ((ap_continue == 1'b1)) begin
            ap_done_reg <= 1'b0;
        end else if (((grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_done == 1'b1) & (1'b1 == ap_CS_fsm_state2))) begin
            ap_done_reg <= 1'b1;
        end
    end
end

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_start_reg <= 1'b0;
    end else begin
        if ((~((real_start == 1'b0) | (ap_done_reg == 1'b1)) & (1'b1 == ap_CS_fsm_state1))) begin
            grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_start_reg <= 1'b1;
        end else if ((grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_ready == 1'b1)) begin
            grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_start_reg <= 1'b0;
        end
    end
end

always @ (posedge ap_clk) begin
    if (ap_rst == 1'b1) begin
        start_once_reg <= 1'b0;
    end else begin
        if (((real_start == 1'b1) & (internal_ap_ready == 1'b0))) begin
            start_once_reg <= 1'b1;
        end else if ((internal_ap_ready == 1'b1)) begin
            start_once_reg <= 1'b0;
        end
    end
end

always @ (*) begin
    if (((real_start == 1'b0) | (ap_done_reg == 1'b1))) begin
        ap_ST_fsm_state1_blk = 1'b1;
    end else begin
        ap_ST_fsm_state1_blk = 1'b0;
    end
end

always @ (*) begin
    if ((grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_done == 1'b0)) begin
        ap_ST_fsm_state2_blk = 1'b1;
    end else begin
        ap_ST_fsm_state2_blk = 1'b0;
    end
end

always @ (*) begin
    if (((grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_done == 1'b1) & (1'b1 == ap_CS_fsm_state2))) begin
        ap_done = 1'b1;
    end else begin
        ap_done = ap_done_reg;
    end
end

always @ (*) begin
    if (((real_start == 1'b0) & (1'b1 == ap_CS_fsm_state1))) begin
        ap_idle = 1'b1;
    end else begin
        ap_idle = 1'b0;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state2)) begin
        in_mat_44_read = grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_in_mat_44_read;
    end else begin
        in_mat_44_read = 1'b0;
    end
end

always @ (*) begin
    if (((grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_done == 1'b1) & (1'b1 == ap_CS_fsm_state2))) begin
        internal_ap_ready = 1'b1;
    end else begin
        internal_ap_ready = 1'b0;
    end
end

always @ (*) begin
    if (((start_full_n == 1'b0) & (start_once_reg == 1'b0))) begin
        real_start = 1'b0;
    end else begin
        real_start = ap_start;
    end
end

always @ (*) begin
    if ((1'b1 == ap_CS_fsm_state2)) begin
        resized_mat_45_write = grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_resized_mat_45_write;
    end else begin
        resized_mat_45_write = 1'b0;
    end
end

always @ (*) begin
    if (((real_start == 1'b1) & (start_once_reg == 1'b0))) begin
        start_write = 1'b1;
    end else begin
        start_write = 1'b0;
    end
end

always @ (*) begin
    case (ap_CS_fsm)
        ap_ST_fsm_state1 : begin
            if ((~((real_start == 1'b0) | (ap_done_reg == 1'b1)) & (1'b1 == ap_CS_fsm_state1))) begin
                ap_NS_fsm = ap_ST_fsm_state2;
            end else begin
                ap_NS_fsm = ap_ST_fsm_state1;
            end
        end
        ap_ST_fsm_state2 : begin
            if (((grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_done == 1'b1) & (1'b1 == ap_CS_fsm_state2))) begin
                ap_NS_fsm = ap_ST_fsm_state1;
            end else begin
                ap_NS_fsm = ap_ST_fsm_state2;
            end
        end
        default : begin
            ap_NS_fsm = 'bx;
        end
    endcase
end

assign ap_CS_fsm_state1 = ap_CS_fsm[32'd0];

assign ap_CS_fsm_state2 = ap_CS_fsm[32'd1];

always @ (*) begin
    ap_block_state1 = ((real_start == 1'b0) | (ap_done_reg == 1'b1));
end

always @ (*) begin
    ap_block_state1_ignore_call2 = ((real_start == 1'b0) | (ap_done_reg == 1'b1));
end

assign ap_ready = internal_ap_ready;

assign grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_start = grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_ap_start_reg;

assign resized_mat_45_din = grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14_resized_mat_45_din;

assign start_out = real_start;

endmodule //rt_imp_resize_1_9_480_640_600_1000_1_2_s
