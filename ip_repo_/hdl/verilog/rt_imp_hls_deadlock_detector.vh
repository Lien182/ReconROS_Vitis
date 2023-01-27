
    wire dl_reset;
    wire dl_clock;
    assign dl_reset = ~ap_rst;
    assign dl_clock = ap_clk;
    wire [1:0] proc_0_data_FIFO_blk;
    wire [1:0] proc_0_data_PIPO_blk;
    wire [1:0] proc_0_start_FIFO_blk;
    wire [1:0] proc_0_TLF_FIFO_blk;
    wire [1:0] proc_0_input_sync_blk;
    wire [1:0] proc_0_output_sync_blk;
    wire [1:0] proc_dep_vld_vec_0;
    reg [1:0] proc_dep_vld_vec_0_reg;
    wire [1:0] in_chan_dep_vld_vec_0;
    wire [19:0] in_chan_dep_data_vec_0;
    wire [1:0] token_in_vec_0;
    wire [1:0] out_chan_dep_vld_vec_0;
    wire [9:0] out_chan_dep_data_0;
    wire [1:0] token_out_vec_0;
    wire dl_detect_out_0;
    wire dep_chan_vld_1_0;
    wire [9:0] dep_chan_data_1_0;
    wire token_1_0;
    wire dep_chan_vld_9_0;
    wire [9:0] dep_chan_data_9_0;
    wire token_9_0;
    wire [1:0] proc_1_data_FIFO_blk;
    wire [1:0] proc_1_data_PIPO_blk;
    wire [1:0] proc_1_start_FIFO_blk;
    wire [1:0] proc_1_TLF_FIFO_blk;
    wire [1:0] proc_1_input_sync_blk;
    wire [1:0] proc_1_output_sync_blk;
    wire [1:0] proc_dep_vld_vec_1;
    reg [1:0] proc_dep_vld_vec_1_reg;
    wire [1:0] in_chan_dep_vld_vec_1;
    wire [19:0] in_chan_dep_data_vec_1;
    wire [1:0] token_in_vec_1;
    wire [1:0] out_chan_dep_vld_vec_1;
    wire [9:0] out_chan_dep_data_1;
    wire [1:0] token_out_vec_1;
    wire dl_detect_out_1;
    wire dep_chan_vld_0_1;
    wire [9:0] dep_chan_data_0_1;
    wire token_0_1;
    wire dep_chan_vld_2_1;
    wire [9:0] dep_chan_data_2_1;
    wire token_2_1;
    wire [1:0] proc_2_data_FIFO_blk;
    wire [1:0] proc_2_data_PIPO_blk;
    wire [1:0] proc_2_start_FIFO_blk;
    wire [1:0] proc_2_TLF_FIFO_blk;
    wire [1:0] proc_2_input_sync_blk;
    wire [1:0] proc_2_output_sync_blk;
    wire [1:0] proc_dep_vld_vec_2;
    reg [1:0] proc_dep_vld_vec_2_reg;
    wire [1:0] in_chan_dep_vld_vec_2;
    wire [19:0] in_chan_dep_data_vec_2;
    wire [1:0] token_in_vec_2;
    wire [1:0] out_chan_dep_vld_vec_2;
    wire [9:0] out_chan_dep_data_2;
    wire [1:0] token_out_vec_2;
    wire dl_detect_out_2;
    wire dep_chan_vld_1_2;
    wire [9:0] dep_chan_data_1_2;
    wire token_1_2;
    wire dep_chan_vld_3_2;
    wire [9:0] dep_chan_data_3_2;
    wire token_3_2;
    wire [1:0] proc_3_data_FIFO_blk;
    wire [1:0] proc_3_data_PIPO_blk;
    wire [1:0] proc_3_start_FIFO_blk;
    wire [1:0] proc_3_TLF_FIFO_blk;
    wire [1:0] proc_3_input_sync_blk;
    wire [1:0] proc_3_output_sync_blk;
    wire [1:0] proc_dep_vld_vec_3;
    reg [1:0] proc_dep_vld_vec_3_reg;
    wire [1:0] in_chan_dep_vld_vec_3;
    wire [19:0] in_chan_dep_data_vec_3;
    wire [1:0] token_in_vec_3;
    wire [1:0] out_chan_dep_vld_vec_3;
    wire [9:0] out_chan_dep_data_3;
    wire [1:0] token_out_vec_3;
    wire dl_detect_out_3;
    wire dep_chan_vld_2_3;
    wire [9:0] dep_chan_data_2_3;
    wire token_2_3;
    wire dep_chan_vld_4_3;
    wire [9:0] dep_chan_data_4_3;
    wire token_4_3;
    wire [1:0] proc_4_data_FIFO_blk;
    wire [1:0] proc_4_data_PIPO_blk;
    wire [1:0] proc_4_start_FIFO_blk;
    wire [1:0] proc_4_TLF_FIFO_blk;
    wire [1:0] proc_4_input_sync_blk;
    wire [1:0] proc_4_output_sync_blk;
    wire [1:0] proc_dep_vld_vec_4;
    reg [1:0] proc_dep_vld_vec_4_reg;
    wire [1:0] in_chan_dep_vld_vec_4;
    wire [19:0] in_chan_dep_data_vec_4;
    wire [1:0] token_in_vec_4;
    wire [1:0] out_chan_dep_vld_vec_4;
    wire [9:0] out_chan_dep_data_4;
    wire [1:0] token_out_vec_4;
    wire dl_detect_out_4;
    wire dep_chan_vld_3_4;
    wire [9:0] dep_chan_data_3_4;
    wire token_3_4;
    wire dep_chan_vld_9_4;
    wire [9:0] dep_chan_data_9_4;
    wire token_9_4;
    wire [1:0] proc_5_data_FIFO_blk;
    wire [1:0] proc_5_data_PIPO_blk;
    wire [1:0] proc_5_start_FIFO_blk;
    wire [1:0] proc_5_TLF_FIFO_blk;
    wire [1:0] proc_5_input_sync_blk;
    wire [1:0] proc_5_output_sync_blk;
    wire [1:0] proc_dep_vld_vec_5;
    reg [1:0] proc_dep_vld_vec_5_reg;
    wire [1:0] in_chan_dep_vld_vec_5;
    wire [19:0] in_chan_dep_data_vec_5;
    wire [1:0] token_in_vec_5;
    wire [1:0] out_chan_dep_vld_vec_5;
    wire [9:0] out_chan_dep_data_5;
    wire [1:0] token_out_vec_5;
    wire dl_detect_out_5;
    wire dep_chan_vld_6_5;
    wire [9:0] dep_chan_data_6_5;
    wire token_6_5;
    wire dep_chan_vld_7_5;
    wire [9:0] dep_chan_data_7_5;
    wire token_7_5;
    wire [0:0] proc_6_data_FIFO_blk;
    wire [0:0] proc_6_data_PIPO_blk;
    wire [0:0] proc_6_start_FIFO_blk;
    wire [0:0] proc_6_TLF_FIFO_blk;
    wire [0:0] proc_6_input_sync_blk;
    wire [0:0] proc_6_output_sync_blk;
    wire [0:0] proc_dep_vld_vec_6;
    reg [0:0] proc_dep_vld_vec_6_reg;
    wire [1:0] in_chan_dep_vld_vec_6;
    wire [19:0] in_chan_dep_data_vec_6;
    wire [1:0] token_in_vec_6;
    wire [0:0] out_chan_dep_vld_vec_6;
    wire [9:0] out_chan_dep_data_6;
    wire [0:0] token_out_vec_6;
    wire dl_detect_out_6;
    wire dep_chan_vld_5_6;
    wire [9:0] dep_chan_data_5_6;
    wire token_5_6;
    wire dep_chan_vld_7_6;
    wire [9:0] dep_chan_data_7_6;
    wire token_7_6;
    wire [2:0] proc_7_data_FIFO_blk;
    wire [2:0] proc_7_data_PIPO_blk;
    wire [2:0] proc_7_start_FIFO_blk;
    wire [2:0] proc_7_TLF_FIFO_blk;
    wire [2:0] proc_7_input_sync_blk;
    wire [2:0] proc_7_output_sync_blk;
    wire [2:0] proc_dep_vld_vec_7;
    reg [2:0] proc_dep_vld_vec_7_reg;
    wire [1:0] in_chan_dep_vld_vec_7;
    wire [19:0] in_chan_dep_data_vec_7;
    wire [1:0] token_in_vec_7;
    wire [2:0] out_chan_dep_vld_vec_7;
    wire [9:0] out_chan_dep_data_7;
    wire [2:0] token_out_vec_7;
    wire dl_detect_out_7;
    wire dep_chan_vld_5_7;
    wire [9:0] dep_chan_data_5_7;
    wire token_5_7;
    wire dep_chan_vld_8_7;
    wire [9:0] dep_chan_data_8_7;
    wire token_8_7;
    wire [0:0] proc_8_data_FIFO_blk;
    wire [0:0] proc_8_data_PIPO_blk;
    wire [0:0] proc_8_start_FIFO_blk;
    wire [0:0] proc_8_TLF_FIFO_blk;
    wire [0:0] proc_8_input_sync_blk;
    wire [0:0] proc_8_output_sync_blk;
    wire [0:0] proc_dep_vld_vec_8;
    reg [0:0] proc_dep_vld_vec_8_reg;
    wire [0:0] in_chan_dep_vld_vec_8;
    wire [9:0] in_chan_dep_data_vec_8;
    wire [0:0] token_in_vec_8;
    wire [0:0] out_chan_dep_vld_vec_8;
    wire [9:0] out_chan_dep_data_8;
    wire [0:0] token_out_vec_8;
    wire dl_detect_out_8;
    wire dep_chan_vld_7_8;
    wire [9:0] dep_chan_data_7_8;
    wire token_7_8;
    wire [1:0] proc_9_data_FIFO_blk;
    wire [1:0] proc_9_data_PIPO_blk;
    wire [1:0] proc_9_start_FIFO_blk;
    wire [1:0] proc_9_TLF_FIFO_blk;
    wire [1:0] proc_9_input_sync_blk;
    wire [1:0] proc_9_output_sync_blk;
    wire [1:0] proc_dep_vld_vec_9;
    reg [1:0] proc_dep_vld_vec_9_reg;
    wire [1:0] in_chan_dep_vld_vec_9;
    wire [19:0] in_chan_dep_data_vec_9;
    wire [1:0] token_in_vec_9;
    wire [1:0] out_chan_dep_vld_vec_9;
    wire [9:0] out_chan_dep_data_9;
    wire [1:0] token_out_vec_9;
    wire dl_detect_out_9;
    wire dep_chan_vld_0_9;
    wire [9:0] dep_chan_data_0_9;
    wire token_0_9;
    wire dep_chan_vld_4_9;
    wire [9:0] dep_chan_data_4_9;
    wire token_4_9;
    wire [9:0] dl_in_vec;
    wire dl_detect_out;
    wire token_clear;
    reg [9:0] origin;

    reg ap_done_reg_0;// for module grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            ap_done_reg_0 <= 'b0;
        end
        else begin
            ap_done_reg_0 <= grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.ap_done & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.ap_continue;
        end
    end

    reg ap_done_reg_1;// for module grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            ap_done_reg_1 <= 'b0;
        end
        else begin
            ap_done_reg_1 <= grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0.ap_done & ~grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0.ap_continue;
        end
    end

    reg ap_done_reg_2;// for module grp_proc_fu_347.proc_Loop_loop_write_proc8_U0
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            ap_done_reg_2 <= 'b0;
        end
        else begin
            ap_done_reg_2 <= grp_proc_fu_347.proc_Loop_loop_write_proc8_U0.ap_done & ~grp_proc_fu_347.proc_Loop_loop_write_proc8_U0.ap_continue;
        end
    end

reg [15:0] trans_in_cnt_0;// for process grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_in_cnt_0 <= 16'h0;
    end
    else if (grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0.start_write == 1'b1) begin
        trans_in_cnt_0 <= trans_in_cnt_0 + 16'h1;
    end
    else begin
        trans_in_cnt_0 <= trans_in_cnt_0;
    end
end

reg [15:0] trans_out_cnt_0;// for process grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_out_cnt_0 <= 16'h0;
    end
    else if (grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0.ap_done == 1'b1 && grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0.ap_continue == 1'b1) begin
        trans_out_cnt_0 <= trans_out_cnt_0 + 16'h1;
    end
    else begin
        trans_out_cnt_0 <= trans_out_cnt_0;
    end
end

reg [15:0] trans_in_cnt_1;// for process grp_proc_fu_347.proc_Loop_loop_read_proc6_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_in_cnt_1 <= 16'h0;
    end
    else if (grp_proc_fu_347.proc_Loop_loop_read_proc6_U0.start_write == 1'b1) begin
        trans_in_cnt_1 <= trans_in_cnt_1 + 16'h1;
    end
    else begin
        trans_in_cnt_1 <= trans_in_cnt_1;
    end
end

reg [15:0] trans_out_cnt_1;// for process grp_proc_fu_347.proc_Loop_loop_read_proc6_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_out_cnt_1 <= 16'h0;
    end
    else if (grp_proc_fu_347.proc_Loop_loop_read_proc6_U0.ap_done == 1'b1 && grp_proc_fu_347.proc_Loop_loop_read_proc6_U0.ap_continue == 1'b1) begin
        trans_out_cnt_1 <= trans_out_cnt_1 + 16'h1;
    end
    else begin
        trans_out_cnt_1 <= trans_out_cnt_1;
    end
end

reg [15:0] trans_in_cnt_2;// for process grp_proc_fu_347.resize_1_9_480_640_600_1000_1_2_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_in_cnt_2 <= 16'h0;
    end
    else if (grp_proc_fu_347.resize_1_9_480_640_600_1000_1_2_U0.start_write == 1'b1) begin
        trans_in_cnt_2 <= trans_in_cnt_2 + 16'h1;
    end
    else begin
        trans_in_cnt_2 <= trans_in_cnt_2;
    end
end

reg [15:0] trans_out_cnt_2;// for process grp_proc_fu_347.resize_1_9_480_640_600_1000_1_2_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_out_cnt_2 <= 16'h0;
    end
    else if (grp_proc_fu_347.resize_1_9_480_640_600_1000_1_2_U0.ap_done == 1'b1 && grp_proc_fu_347.resize_1_9_480_640_600_1000_1_2_U0.ap_continue == 1'b1) begin
        trans_out_cnt_2 <= trans_out_cnt_2 + 16'h1;
    end
    else begin
        trans_out_cnt_2 <= trans_out_cnt_2;
    end
end

reg [15:0] trans_in_cnt_3;// for process grp_proc_fu_347.proc_Loop_VITIS_LOOP_132_3_proc7_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_in_cnt_3 <= 16'h0;
    end
    else if (grp_proc_fu_347.proc_Loop_VITIS_LOOP_132_3_proc7_U0.start_write == 1'b1) begin
        trans_in_cnt_3 <= trans_in_cnt_3 + 16'h1;
    end
    else begin
        trans_in_cnt_3 <= trans_in_cnt_3;
    end
end

reg [15:0] trans_out_cnt_3;// for process grp_proc_fu_347.proc_Loop_VITIS_LOOP_132_3_proc7_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_out_cnt_3 <= 16'h0;
    end
    else if (grp_proc_fu_347.proc_Loop_VITIS_LOOP_132_3_proc7_U0.ap_done == 1'b1 && grp_proc_fu_347.proc_Loop_VITIS_LOOP_132_3_proc7_U0.ap_continue == 1'b1) begin
        trans_out_cnt_3 <= trans_out_cnt_3 + 16'h1;
    end
    else begin
        trans_out_cnt_3 <= trans_out_cnt_3;
    end
end

reg [15:0] trans_in_cnt_4;// for process grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_in_cnt_4 <= 16'h0;
    end
    else if (grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.start_write == 1'b1) begin
        trans_in_cnt_4 <= trans_in_cnt_4 + 16'h1;
    end
    else begin
        trans_in_cnt_4 <= trans_in_cnt_4;
    end
end

reg [15:0] trans_out_cnt_4;// for process grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_out_cnt_4 <= 16'h0;
    end
    else if (grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.ap_done == 1'b1 && grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.ap_continue == 1'b1) begin
        trans_out_cnt_4 <= trans_out_cnt_4 + 16'h1;
    end
    else begin
        trans_out_cnt_4 <= trans_out_cnt_4;
    end
end

reg [15:0] trans_in_cnt_5;// for process grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_in_cnt_5 <= 16'h0;
    end
    else if (grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.start_write == 1'b1) begin
        trans_in_cnt_5 <= trans_in_cnt_5 + 16'h1;
    end
    else begin
        trans_in_cnt_5 <= trans_in_cnt_5;
    end
end

reg [15:0] trans_out_cnt_5;// for process grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0
always @(negedge dl_reset or posedge dl_clock) begin
    if (~dl_reset) begin
         trans_out_cnt_5 <= 16'h0;
    end
    else if (grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.ap_done == 1'b1 && grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.ap_continue == 1'b1) begin
        trans_out_cnt_5 <= trans_out_cnt_5 + 16'h1;
    end
    else begin
        trans_out_cnt_5 <= trans_out_cnt_5;
    end
end

    // Process: grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0
    rt_imp_hls_deadlock_detect_unit #(10, 0, 2, 2) rt_imp_hls_deadlock_detect_unit_0 (
        .reset(dl_reset),
        .clock(dl_clock),
        .proc_dep_vld_vec(proc_dep_vld_vec_0),
        .in_chan_dep_vld_vec(in_chan_dep_vld_vec_0),
        .in_chan_dep_data_vec(in_chan_dep_data_vec_0),
        .token_in_vec(token_in_vec_0),
        .dl_detect_in(dl_detect_out),
        .origin(origin[0]),
        .token_clear(token_clear),
        .out_chan_dep_vld_vec(out_chan_dep_vld_vec_0),
        .out_chan_dep_data(out_chan_dep_data_0),
        .token_out_vec(token_out_vec_0),
        .dl_detect_out(dl_in_vec[0]));

    assign proc_0_data_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0.grp_proc_Loop_VITIS_LOOP_78_1_proc5_Pipeline_VITIS_LOOP_78_2_fu_66.ram_in_V_blk_n);
    assign proc_0_data_PIPO_blk[0] = 1'b0;
    assign proc_0_start_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.start_for_proc_Loop_loop_read_proc6_U0_U.if_full_n & grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0.ap_start & ~grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0.real_start & (trans_in_cnt_0 == trans_out_cnt_0) & ~grp_proc_fu_347.start_for_proc_Loop_loop_read_proc6_U0_U.if_read);
    assign proc_0_TLF_FIFO_blk[0] = 1'b0;
    assign proc_0_input_sync_blk[0] = 1'b0;
    assign proc_0_output_sync_blk[0] = 1'b0;
    assign proc_dep_vld_vec_0[0] = dl_detect_out ? proc_dep_vld_vec_0_reg[0] : (proc_0_data_FIFO_blk[0] | proc_0_data_PIPO_blk[0] | proc_0_start_FIFO_blk[0] | proc_0_TLF_FIFO_blk[0] | proc_0_input_sync_blk[0] | proc_0_output_sync_blk[0]);
    assign proc_0_data_FIFO_blk[1] = 1'b0;
    assign proc_0_data_PIPO_blk[1] = 1'b0;
    assign proc_0_start_FIFO_blk[1] = 1'b0;
    assign proc_0_TLF_FIFO_blk[1] = 1'b0;
    assign proc_0_input_sync_blk[1] = 1'b0;
    assign proc_0_output_sync_blk[1] = 1'b0 | (ap_done_reg_1 & grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0.ap_done & ~grp_proc_fu_347.proc_Loop_loop_write_proc8_U0.ap_done);
    assign proc_dep_vld_vec_0[1] = dl_detect_out ? proc_dep_vld_vec_0_reg[1] : (proc_0_data_FIFO_blk[1] | proc_0_data_PIPO_blk[1] | proc_0_start_FIFO_blk[1] | proc_0_TLF_FIFO_blk[1] | proc_0_input_sync_blk[1] | proc_0_output_sync_blk[1]);
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            proc_dep_vld_vec_0_reg <= 'b0;
        end
        else begin
            proc_dep_vld_vec_0_reg <= proc_dep_vld_vec_0;
        end
    end
    assign in_chan_dep_vld_vec_0[0] = dep_chan_vld_1_0;
    assign in_chan_dep_data_vec_0[9 : 0] = dep_chan_data_1_0;
    assign token_in_vec_0[0] = token_1_0;
    assign in_chan_dep_vld_vec_0[1] = dep_chan_vld_9_0;
    assign in_chan_dep_data_vec_0[19 : 10] = dep_chan_data_9_0;
    assign token_in_vec_0[1] = token_9_0;
    assign dep_chan_vld_0_1 = out_chan_dep_vld_vec_0[0];
    assign dep_chan_data_0_1 = out_chan_dep_data_0;
    assign token_0_1 = token_out_vec_0[0];
    assign dep_chan_vld_0_9 = out_chan_dep_vld_vec_0[1];
    assign dep_chan_data_0_9 = out_chan_dep_data_0;
    assign token_0_9 = token_out_vec_0[1];

    // Process: grp_proc_fu_347.proc_Loop_loop_read_proc6_U0
    rt_imp_hls_deadlock_detect_unit #(10, 1, 2, 2) rt_imp_hls_deadlock_detect_unit_1 (
        .reset(dl_reset),
        .clock(dl_clock),
        .proc_dep_vld_vec(proc_dep_vld_vec_1),
        .in_chan_dep_vld_vec(in_chan_dep_vld_vec_1),
        .in_chan_dep_data_vec(in_chan_dep_data_vec_1),
        .token_in_vec(token_in_vec_1),
        .dl_detect_in(dl_detect_out),
        .origin(origin[1]),
        .token_clear(token_clear),
        .out_chan_dep_vld_vec(out_chan_dep_vld_vec_1),
        .out_chan_dep_data(out_chan_dep_data_1),
        .token_out_vec(token_out_vec_1),
        .dl_detect_out(dl_in_vec[1]));

    assign proc_1_data_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.proc_Loop_loop_read_proc6_U0.ram_in_V_blk_n);
    assign proc_1_data_PIPO_blk[0] = 1'b0;
    assign proc_1_start_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.start_for_proc_Loop_loop_read_proc6_U0_U.if_empty_n & grp_proc_fu_347.proc_Loop_loop_read_proc6_U0.ap_idle & ~grp_proc_fu_347.start_for_proc_Loop_loop_read_proc6_U0_U.if_write);
    assign proc_1_TLF_FIFO_blk[0] = 1'b0;
    assign proc_1_input_sync_blk[0] = 1'b0;
    assign proc_1_output_sync_blk[0] = 1'b0;
    assign proc_dep_vld_vec_1[0] = dl_detect_out ? proc_dep_vld_vec_1_reg[0] : (proc_1_data_FIFO_blk[0] | proc_1_data_PIPO_blk[0] | proc_1_start_FIFO_blk[0] | proc_1_TLF_FIFO_blk[0] | proc_1_input_sync_blk[0] | proc_1_output_sync_blk[0]);
    assign proc_1_data_FIFO_blk[1] = 1'b0 | (~grp_proc_fu_347.proc_Loop_loop_read_proc6_U0.in_mat_data_blk_n);
    assign proc_1_data_PIPO_blk[1] = 1'b0;
    assign proc_1_start_FIFO_blk[1] = 1'b0 | (~grp_proc_fu_347.start_for_resize_1_9_480_640_600_1000_1_2_U0_U.if_full_n & grp_proc_fu_347.proc_Loop_loop_read_proc6_U0.ap_start & ~grp_proc_fu_347.proc_Loop_loop_read_proc6_U0.real_start & (trans_in_cnt_1 == trans_out_cnt_1) & ~grp_proc_fu_347.start_for_resize_1_9_480_640_600_1000_1_2_U0_U.if_read);
    assign proc_1_TLF_FIFO_blk[1] = 1'b0;
    assign proc_1_input_sync_blk[1] = 1'b0;
    assign proc_1_output_sync_blk[1] = 1'b0;
    assign proc_dep_vld_vec_1[1] = dl_detect_out ? proc_dep_vld_vec_1_reg[1] : (proc_1_data_FIFO_blk[1] | proc_1_data_PIPO_blk[1] | proc_1_start_FIFO_blk[1] | proc_1_TLF_FIFO_blk[1] | proc_1_input_sync_blk[1] | proc_1_output_sync_blk[1]);
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            proc_dep_vld_vec_1_reg <= 'b0;
        end
        else begin
            proc_dep_vld_vec_1_reg <= proc_dep_vld_vec_1;
        end
    end
    assign in_chan_dep_vld_vec_1[0] = dep_chan_vld_0_1;
    assign in_chan_dep_data_vec_1[9 : 0] = dep_chan_data_0_1;
    assign token_in_vec_1[0] = token_0_1;
    assign in_chan_dep_vld_vec_1[1] = dep_chan_vld_2_1;
    assign in_chan_dep_data_vec_1[19 : 10] = dep_chan_data_2_1;
    assign token_in_vec_1[1] = token_2_1;
    assign dep_chan_vld_1_0 = out_chan_dep_vld_vec_1[0];
    assign dep_chan_data_1_0 = out_chan_dep_data_1;
    assign token_1_0 = token_out_vec_1[0];
    assign dep_chan_vld_1_2 = out_chan_dep_vld_vec_1[1];
    assign dep_chan_data_1_2 = out_chan_dep_data_1;
    assign token_1_2 = token_out_vec_1[1];

    // Process: grp_proc_fu_347.resize_1_9_480_640_600_1000_1_2_U0
    rt_imp_hls_deadlock_detect_unit #(10, 2, 2, 2) rt_imp_hls_deadlock_detect_unit_2 (
        .reset(dl_reset),
        .clock(dl_clock),
        .proc_dep_vld_vec(proc_dep_vld_vec_2),
        .in_chan_dep_vld_vec(in_chan_dep_vld_vec_2),
        .in_chan_dep_data_vec(in_chan_dep_data_vec_2),
        .token_in_vec(token_in_vec_2),
        .dl_detect_in(dl_detect_out),
        .origin(origin[2]),
        .token_clear(token_clear),
        .out_chan_dep_vld_vec(out_chan_dep_vld_vec_2),
        .out_chan_dep_data(out_chan_dep_data_2),
        .token_out_vec(token_out_vec_2),
        .dl_detect_out(dl_in_vec[2]));

    assign proc_2_data_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.resize_1_9_480_640_600_1000_1_2_U0.grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14.grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_Pipeline_VITIS_LOOP_331_1_VITIS_LOOP_336_2_fu_168.in_mat_44_blk_n);
    assign proc_2_data_PIPO_blk[0] = 1'b0;
    assign proc_2_start_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.start_for_resize_1_9_480_640_600_1000_1_2_U0_U.if_empty_n & grp_proc_fu_347.resize_1_9_480_640_600_1000_1_2_U0.ap_idle & ~grp_proc_fu_347.start_for_resize_1_9_480_640_600_1000_1_2_U0_U.if_write);
    assign proc_2_TLF_FIFO_blk[0] = 1'b0;
    assign proc_2_input_sync_blk[0] = 1'b0;
    assign proc_2_output_sync_blk[0] = 1'b0;
    assign proc_dep_vld_vec_2[0] = dl_detect_out ? proc_dep_vld_vec_2_reg[0] : (proc_2_data_FIFO_blk[0] | proc_2_data_PIPO_blk[0] | proc_2_start_FIFO_blk[0] | proc_2_TLF_FIFO_blk[0] | proc_2_input_sync_blk[0] | proc_2_output_sync_blk[0]);
    assign proc_2_data_FIFO_blk[1] = 1'b0 | (~grp_proc_fu_347.resize_1_9_480_640_600_1000_1_2_U0.grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_s_fu_14.grp_resizeNNBilinear_9_480_640_1_600_1000_1_2_Pipeline_VITIS_LOOP_388_5_fu_194.resized_mat_45_blk_n);
    assign proc_2_data_PIPO_blk[1] = 1'b0;
    assign proc_2_start_FIFO_blk[1] = 1'b0 | (~grp_proc_fu_347.start_for_proc_Loop_VITIS_LOOP_132_3_proc7_U0_U.if_full_n & grp_proc_fu_347.resize_1_9_480_640_600_1000_1_2_U0.ap_start & ~grp_proc_fu_347.resize_1_9_480_640_600_1000_1_2_U0.real_start & (trans_in_cnt_2 == trans_out_cnt_2) & ~grp_proc_fu_347.start_for_proc_Loop_VITIS_LOOP_132_3_proc7_U0_U.if_read);
    assign proc_2_TLF_FIFO_blk[1] = 1'b0;
    assign proc_2_input_sync_blk[1] = 1'b0;
    assign proc_2_output_sync_blk[1] = 1'b0;
    assign proc_dep_vld_vec_2[1] = dl_detect_out ? proc_dep_vld_vec_2_reg[1] : (proc_2_data_FIFO_blk[1] | proc_2_data_PIPO_blk[1] | proc_2_start_FIFO_blk[1] | proc_2_TLF_FIFO_blk[1] | proc_2_input_sync_blk[1] | proc_2_output_sync_blk[1]);
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            proc_dep_vld_vec_2_reg <= 'b0;
        end
        else begin
            proc_dep_vld_vec_2_reg <= proc_dep_vld_vec_2;
        end
    end
    assign in_chan_dep_vld_vec_2[0] = dep_chan_vld_1_2;
    assign in_chan_dep_data_vec_2[9 : 0] = dep_chan_data_1_2;
    assign token_in_vec_2[0] = token_1_2;
    assign in_chan_dep_vld_vec_2[1] = dep_chan_vld_3_2;
    assign in_chan_dep_data_vec_2[19 : 10] = dep_chan_data_3_2;
    assign token_in_vec_2[1] = token_3_2;
    assign dep_chan_vld_2_1 = out_chan_dep_vld_vec_2[0];
    assign dep_chan_data_2_1 = out_chan_dep_data_2;
    assign token_2_1 = token_out_vec_2[0];
    assign dep_chan_vld_2_3 = out_chan_dep_vld_vec_2[1];
    assign dep_chan_data_2_3 = out_chan_dep_data_2;
    assign token_2_3 = token_out_vec_2[1];

    // Process: grp_proc_fu_347.proc_Loop_VITIS_LOOP_132_3_proc7_U0
    rt_imp_hls_deadlock_detect_unit #(10, 3, 2, 2) rt_imp_hls_deadlock_detect_unit_3 (
        .reset(dl_reset),
        .clock(dl_clock),
        .proc_dep_vld_vec(proc_dep_vld_vec_3),
        .in_chan_dep_vld_vec(in_chan_dep_vld_vec_3),
        .in_chan_dep_data_vec(in_chan_dep_data_vec_3),
        .token_in_vec(token_in_vec_3),
        .dl_detect_in(dl_detect_out),
        .origin(origin[3]),
        .token_clear(token_clear),
        .out_chan_dep_vld_vec(out_chan_dep_vld_vec_3),
        .out_chan_dep_data(out_chan_dep_data_3),
        .token_out_vec(token_out_vec_3),
        .dl_detect_out(dl_in_vec[3]));

    assign proc_3_data_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.proc_Loop_VITIS_LOOP_132_3_proc7_U0.resized_mat_data_blk_n);
    assign proc_3_data_PIPO_blk[0] = 1'b0;
    assign proc_3_start_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.start_for_proc_Loop_VITIS_LOOP_132_3_proc7_U0_U.if_empty_n & grp_proc_fu_347.proc_Loop_VITIS_LOOP_132_3_proc7_U0.ap_idle & ~grp_proc_fu_347.start_for_proc_Loop_VITIS_LOOP_132_3_proc7_U0_U.if_write);
    assign proc_3_TLF_FIFO_blk[0] = 1'b0;
    assign proc_3_input_sync_blk[0] = 1'b0;
    assign proc_3_output_sync_blk[0] = 1'b0;
    assign proc_dep_vld_vec_3[0] = dl_detect_out ? proc_dep_vld_vec_3_reg[0] : (proc_3_data_FIFO_blk[0] | proc_3_data_PIPO_blk[0] | proc_3_start_FIFO_blk[0] | proc_3_TLF_FIFO_blk[0] | proc_3_input_sync_blk[0] | proc_3_output_sync_blk[0]);
    assign proc_3_data_FIFO_blk[1] = 1'b0 | (~grp_proc_fu_347.proc_Loop_VITIS_LOOP_132_3_proc7_U0.tmp_mat_data_blk_n);
    assign proc_3_data_PIPO_blk[1] = 1'b0;
    assign proc_3_start_FIFO_blk[1] = 1'b0 | (~grp_proc_fu_347.start_for_warpTransform_100_50_0_true_7_600_1000_1_false_U0_U.if_full_n & grp_proc_fu_347.proc_Loop_VITIS_LOOP_132_3_proc7_U0.ap_start & ~grp_proc_fu_347.proc_Loop_VITIS_LOOP_132_3_proc7_U0.real_start & (trans_in_cnt_3 == trans_out_cnt_3) & ~grp_proc_fu_347.start_for_warpTransform_100_50_0_true_7_600_1000_1_false_U0_U.if_read);
    assign proc_3_TLF_FIFO_blk[1] = 1'b0;
    assign proc_3_input_sync_blk[1] = 1'b0;
    assign proc_3_output_sync_blk[1] = 1'b0;
    assign proc_dep_vld_vec_3[1] = dl_detect_out ? proc_dep_vld_vec_3_reg[1] : (proc_3_data_FIFO_blk[1] | proc_3_data_PIPO_blk[1] | proc_3_start_FIFO_blk[1] | proc_3_TLF_FIFO_blk[1] | proc_3_input_sync_blk[1] | proc_3_output_sync_blk[1]);
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            proc_dep_vld_vec_3_reg <= 'b0;
        end
        else begin
            proc_dep_vld_vec_3_reg <= proc_dep_vld_vec_3;
        end
    end
    assign in_chan_dep_vld_vec_3[0] = dep_chan_vld_2_3;
    assign in_chan_dep_data_vec_3[9 : 0] = dep_chan_data_2_3;
    assign token_in_vec_3[0] = token_2_3;
    assign in_chan_dep_vld_vec_3[1] = dep_chan_vld_4_3;
    assign in_chan_dep_data_vec_3[19 : 10] = dep_chan_data_4_3;
    assign token_in_vec_3[1] = token_4_3;
    assign dep_chan_vld_3_2 = out_chan_dep_vld_vec_3[0];
    assign dep_chan_data_3_2 = out_chan_dep_data_3;
    assign token_3_2 = token_out_vec_3[0];
    assign dep_chan_vld_3_4 = out_chan_dep_vld_vec_3[1];
    assign dep_chan_data_3_4 = out_chan_dep_data_3;
    assign token_3_4 = token_out_vec_3[1];

    // Process: grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0
    rt_imp_hls_deadlock_detect_unit #(10, 4, 2, 2) rt_imp_hls_deadlock_detect_unit_4 (
        .reset(dl_reset),
        .clock(dl_clock),
        .proc_dep_vld_vec(proc_dep_vld_vec_4),
        .in_chan_dep_vld_vec(in_chan_dep_vld_vec_4),
        .in_chan_dep_data_vec(in_chan_dep_data_vec_4),
        .token_in_vec(token_in_vec_4),
        .dl_detect_in(dl_detect_out),
        .origin(origin[4]),
        .token_clear(token_clear),
        .out_chan_dep_vld_vec(out_chan_dep_vld_vec_4),
        .out_chan_dep_data(out_chan_dep_data_4),
        .token_out_vec(token_out_vec_4),
        .dl_detect_out(dl_in_vec[4]));

    assign proc_4_data_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_769_1_proc3_U0.tmp_mat_46_blk_n);
    assign proc_4_data_PIPO_blk[0] = 1'b0;
    assign proc_4_start_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.start_for_warpTransform_100_50_0_true_7_600_1000_1_false_U0_U.if_empty_n & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.ap_idle & ~grp_proc_fu_347.start_for_warpTransform_100_50_0_true_7_600_1000_1_false_U0_U.if_write);
    assign proc_4_TLF_FIFO_blk[0] = 1'b0;
    assign proc_4_input_sync_blk[0] = 1'b0;
    assign proc_4_output_sync_blk[0] = 1'b0;
    assign proc_dep_vld_vec_4[0] = dl_detect_out ? proc_dep_vld_vec_4_reg[0] : (proc_4_data_FIFO_blk[0] | proc_4_data_PIPO_blk[0] | proc_4_start_FIFO_blk[0] | proc_4_TLF_FIFO_blk[0] | proc_4_input_sync_blk[0] | proc_4_output_sync_blk[0]);
    assign proc_4_data_FIFO_blk[1] = 1'b0 | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_786_3_proc4_U0.out_mat_47_blk_n);
    assign proc_4_data_PIPO_blk[1] = 1'b0;
    assign proc_4_start_FIFO_blk[1] = 1'b0 | (~grp_proc_fu_347.start_for_proc_Loop_loop_write_proc8_U0_U.if_full_n & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.ap_start & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.real_start & (trans_in_cnt_5 == trans_out_cnt_5) & ~grp_proc_fu_347.start_for_proc_Loop_loop_write_proc8_U0_U.if_read);
    assign proc_4_TLF_FIFO_blk[1] = 1'b0;
    assign proc_4_input_sync_blk[1] = 1'b0;
    assign proc_4_output_sync_blk[1] = 1'b0;
    assign proc_dep_vld_vec_4[1] = dl_detect_out ? proc_dep_vld_vec_4_reg[1] : (proc_4_data_FIFO_blk[1] | proc_4_data_PIPO_blk[1] | proc_4_start_FIFO_blk[1] | proc_4_TLF_FIFO_blk[1] | proc_4_input_sync_blk[1] | proc_4_output_sync_blk[1]);
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            proc_dep_vld_vec_4_reg <= 'b0;
        end
        else begin
            proc_dep_vld_vec_4_reg <= proc_dep_vld_vec_4;
        end
    end
    assign in_chan_dep_vld_vec_4[0] = dep_chan_vld_3_4;
    assign in_chan_dep_data_vec_4[9 : 0] = dep_chan_data_3_4;
    assign token_in_vec_4[0] = token_3_4;
    assign in_chan_dep_vld_vec_4[1] = dep_chan_vld_9_4;
    assign in_chan_dep_data_vec_4[19 : 10] = dep_chan_data_9_4;
    assign token_in_vec_4[1] = token_9_4;
    assign dep_chan_vld_4_3 = out_chan_dep_vld_vec_4[0];
    assign dep_chan_data_4_3 = out_chan_dep_data_4;
    assign token_4_3 = token_out_vec_4[0];
    assign dep_chan_vld_4_9 = out_chan_dep_vld_vec_4[1];
    assign dep_chan_data_4_9 = out_chan_dep_data_4;
    assign token_4_9 = token_out_vec_4[1];

    // Process: grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_769_1_proc3_U0
    rt_imp_hls_deadlock_detect_unit #(10, 5, 2, 2) rt_imp_hls_deadlock_detect_unit_5 (
        .reset(dl_reset),
        .clock(dl_clock),
        .proc_dep_vld_vec(proc_dep_vld_vec_5),
        .in_chan_dep_vld_vec(in_chan_dep_vld_vec_5),
        .in_chan_dep_data_vec(in_chan_dep_data_vec_5),
        .token_in_vec(token_in_vec_5),
        .dl_detect_in(dl_detect_out),
        .origin(origin[5]),
        .token_clear(token_clear),
        .out_chan_dep_vld_vec(out_chan_dep_vld_vec_5),
        .out_chan_dep_data(out_chan_dep_data_5),
        .token_out_vec(token_out_vec_5),
        .dl_detect_out(dl_in_vec[5]));

    assign proc_5_data_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_769_1_proc3_U0.in_stream_blk_n);
    assign proc_5_data_PIPO_blk[0] = 1'b0;
    assign proc_5_start_FIFO_blk[0] = 1'b0;
    assign proc_5_TLF_FIFO_blk[0] = 1'b0;
    assign proc_5_input_sync_blk[0] = 1'b0;
    assign proc_5_output_sync_blk[0] = 1'b0;
    assign proc_dep_vld_vec_5[0] = dl_detect_out ? proc_dep_vld_vec_5_reg[0] : (proc_5_data_FIFO_blk[0] | proc_5_data_PIPO_blk[0] | proc_5_start_FIFO_blk[0] | proc_5_TLF_FIFO_blk[0] | proc_5_input_sync_blk[0] | proc_5_output_sync_blk[0]);
    assign proc_5_data_FIFO_blk[1] = 1'b0;
    assign proc_5_data_PIPO_blk[1] = 1'b0;
    assign proc_5_start_FIFO_blk[1] = 1'b0;
    assign proc_5_TLF_FIFO_blk[1] = 1'b0;
    assign proc_5_input_sync_blk[1] = 1'b0 | (grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.ap_sync_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_769_1_proc3_U0_ap_ready & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_769_1_proc3_U0.ap_idle & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.ap_sync_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_COPY_MAT1_proc_U0_ap_ready);
    assign proc_5_output_sync_blk[1] = 1'b0;
    assign proc_dep_vld_vec_5[1] = dl_detect_out ? proc_dep_vld_vec_5_reg[1] : (proc_5_data_FIFO_blk[1] | proc_5_data_PIPO_blk[1] | proc_5_start_FIFO_blk[1] | proc_5_TLF_FIFO_blk[1] | proc_5_input_sync_blk[1] | proc_5_output_sync_blk[1]);
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            proc_dep_vld_vec_5_reg <= 'b0;
        end
        else begin
            proc_dep_vld_vec_5_reg <= proc_dep_vld_vec_5;
        end
    end
    assign in_chan_dep_vld_vec_5[0] = dep_chan_vld_6_5;
    assign in_chan_dep_data_vec_5[9 : 0] = dep_chan_data_6_5;
    assign token_in_vec_5[0] = token_6_5;
    assign in_chan_dep_vld_vec_5[1] = dep_chan_vld_7_5;
    assign in_chan_dep_data_vec_5[19 : 10] = dep_chan_data_7_5;
    assign token_in_vec_5[1] = token_7_5;
    assign dep_chan_vld_5_7 = out_chan_dep_vld_vec_5[0];
    assign dep_chan_data_5_7 = out_chan_dep_data_5;
    assign token_5_7 = token_out_vec_5[0];
    assign dep_chan_vld_5_6 = out_chan_dep_vld_vec_5[1];
    assign dep_chan_data_5_6 = out_chan_dep_data_5;
    assign token_5_6 = token_out_vec_5[1];

    // Process: grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_COPY_MAT1_proc_U0
    rt_imp_hls_deadlock_detect_unit #(10, 6, 2, 1) rt_imp_hls_deadlock_detect_unit_6 (
        .reset(dl_reset),
        .clock(dl_clock),
        .proc_dep_vld_vec(proc_dep_vld_vec_6),
        .in_chan_dep_vld_vec(in_chan_dep_vld_vec_6),
        .in_chan_dep_data_vec(in_chan_dep_data_vec_6),
        .token_in_vec(token_in_vec_6),
        .dl_detect_in(dl_detect_out),
        .origin(origin[6]),
        .token_clear(token_clear),
        .out_chan_dep_vld_vec(out_chan_dep_vld_vec_6),
        .out_chan_dep_data(out_chan_dep_data_6),
        .token_out_vec(token_out_vec_6),
        .dl_detect_out(dl_in_vec[6]));

    assign proc_6_data_FIFO_blk[0] = 1'b0;
    assign proc_6_data_PIPO_blk[0] = 1'b0;
    assign proc_6_start_FIFO_blk[0] = 1'b0;
    assign proc_6_TLF_FIFO_blk[0] = 1'b0;
    assign proc_6_input_sync_blk[0] = 1'b0 | (grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.ap_sync_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_COPY_MAT1_proc_U0_ap_ready & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_COPY_MAT1_proc_U0.ap_idle & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.ap_sync_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_769_1_proc3_U0_ap_ready);
    assign proc_6_output_sync_blk[0] = 1'b0;
    assign proc_dep_vld_vec_6[0] = dl_detect_out ? proc_dep_vld_vec_6_reg[0] : (proc_6_data_FIFO_blk[0] | proc_6_data_PIPO_blk[0] | proc_6_start_FIFO_blk[0] | proc_6_TLF_FIFO_blk[0] | proc_6_input_sync_blk[0] | proc_6_output_sync_blk[0]);
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            proc_dep_vld_vec_6_reg <= 'b0;
        end
        else begin
            proc_dep_vld_vec_6_reg <= proc_dep_vld_vec_6;
        end
    end
    assign in_chan_dep_vld_vec_6[0] = dep_chan_vld_5_6;
    assign in_chan_dep_data_vec_6[9 : 0] = dep_chan_data_5_6;
    assign token_in_vec_6[0] = token_5_6;
    assign in_chan_dep_vld_vec_6[1] = dep_chan_vld_7_6;
    assign in_chan_dep_data_vec_6[19 : 10] = dep_chan_data_7_6;
    assign token_in_vec_6[1] = token_7_6;
    assign dep_chan_vld_6_5 = out_chan_dep_vld_vec_6[0];
    assign dep_chan_data_6_5 = out_chan_dep_data_6;
    assign token_6_5 = token_out_vec_6[0];

    // Process: grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0
    rt_imp_hls_deadlock_detect_unit #(10, 7, 2, 3) rt_imp_hls_deadlock_detect_unit_7 (
        .reset(dl_reset),
        .clock(dl_clock),
        .proc_dep_vld_vec(proc_dep_vld_vec_7),
        .in_chan_dep_vld_vec(in_chan_dep_vld_vec_7),
        .in_chan_dep_data_vec(in_chan_dep_data_vec_7),
        .token_in_vec(token_in_vec_7),
        .dl_detect_in(dl_detect_out),
        .origin(origin[7]),
        .token_clear(token_clear),
        .out_chan_dep_vld_vec(out_chan_dep_vld_vec_7),
        .out_chan_dep_data(out_chan_dep_data_7),
        .token_out_vec(token_out_vec_7),
        .dl_detect_out(dl_in_vec[7]));

    assign proc_7_data_FIFO_blk[0] = 1'b0;
    assign proc_7_data_PIPO_blk[0] = 1'b0;
    assign proc_7_start_FIFO_blk[0] = 1'b0;
    assign proc_7_TLF_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_3_loc_channel_U.if_empty_n & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.ap_idle & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_3_loc_channel_U.if_write) | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_5_loc_channel_U.if_empty_n & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.ap_idle & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_5_loc_channel_U.if_write) | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_loc_channel_U.if_empty_n & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.ap_idle & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_loc_channel_U.if_write) | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_2_loc_channel_U.if_empty_n & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.ap_idle & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_2_loc_channel_U.if_write) | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_4_loc_channel_U.if_empty_n & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.ap_idle & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_4_loc_channel_U.if_write) | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_1_loc_channel_U.if_empty_n & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.ap_idle & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.R_1_loc_channel_U.if_write);
    assign proc_7_input_sync_blk[0] = 1'b0;
    assign proc_7_output_sync_blk[0] = 1'b0;
    assign proc_dep_vld_vec_7[0] = dl_detect_out ? proc_dep_vld_vec_7_reg[0] : (proc_7_data_FIFO_blk[0] | proc_7_data_PIPO_blk[0] | proc_7_start_FIFO_blk[0] | proc_7_TLF_FIFO_blk[0] | proc_7_input_sync_blk[0] | proc_7_output_sync_blk[0]);
    assign proc_7_data_FIFO_blk[1] = 1'b0 | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_Pipeline_MAIN_COLS_fu_524.in_stream_blk_n);
    assign proc_7_data_PIPO_blk[1] = 1'b0;
    assign proc_7_start_FIFO_blk[1] = 1'b0;
    assign proc_7_TLF_FIFO_blk[1] = 1'b0;
    assign proc_7_input_sync_blk[1] = 1'b0;
    assign proc_7_output_sync_blk[1] = 1'b0;
    assign proc_dep_vld_vec_7[1] = dl_detect_out ? proc_dep_vld_vec_7_reg[1] : (proc_7_data_FIFO_blk[1] | proc_7_data_PIPO_blk[1] | proc_7_start_FIFO_blk[1] | proc_7_TLF_FIFO_blk[1] | proc_7_input_sync_blk[1] | proc_7_output_sync_blk[1]);
    assign proc_7_data_FIFO_blk[2] = 1'b0 | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_Pipeline_MAIN_COLS_fu_524.out_stream_blk_n);
    assign proc_7_data_PIPO_blk[2] = 1'b0;
    assign proc_7_start_FIFO_blk[2] = 1'b0 | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.start_for_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_bRq_U.if_full_n & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.ap_start & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_MAIN_ROWS_proc_U0.real_start & (trans_in_cnt_4 == trans_out_cnt_4) & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.start_for_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_bRq_U.if_read);
    assign proc_7_TLF_FIFO_blk[2] = 1'b0;
    assign proc_7_input_sync_blk[2] = 1'b0;
    assign proc_7_output_sync_blk[2] = 1'b0;
    assign proc_dep_vld_vec_7[2] = dl_detect_out ? proc_dep_vld_vec_7_reg[2] : (proc_7_data_FIFO_blk[2] | proc_7_data_PIPO_blk[2] | proc_7_start_FIFO_blk[2] | proc_7_TLF_FIFO_blk[2] | proc_7_input_sync_blk[2] | proc_7_output_sync_blk[2]);
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            proc_dep_vld_vec_7_reg <= 'b0;
        end
        else begin
            proc_dep_vld_vec_7_reg <= proc_dep_vld_vec_7;
        end
    end
    assign in_chan_dep_vld_vec_7[0] = dep_chan_vld_5_7;
    assign in_chan_dep_data_vec_7[9 : 0] = dep_chan_data_5_7;
    assign token_in_vec_7[0] = token_5_7;
    assign in_chan_dep_vld_vec_7[1] = dep_chan_vld_8_7;
    assign in_chan_dep_data_vec_7[19 : 10] = dep_chan_data_8_7;
    assign token_in_vec_7[1] = token_8_7;
    assign dep_chan_vld_7_6 = out_chan_dep_vld_vec_7[0];
    assign dep_chan_data_7_6 = out_chan_dep_data_7;
    assign token_7_6 = token_out_vec_7[0];
    assign dep_chan_vld_7_5 = out_chan_dep_vld_vec_7[1];
    assign dep_chan_data_7_5 = out_chan_dep_data_7;
    assign token_7_5 = token_out_vec_7[1];
    assign dep_chan_vld_7_8 = out_chan_dep_vld_vec_7[2];
    assign dep_chan_data_7_8 = out_chan_dep_data_7;
    assign token_7_8 = token_out_vec_7[2];

    // Process: grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_786_3_proc4_U0
    rt_imp_hls_deadlock_detect_unit #(10, 8, 1, 1) rt_imp_hls_deadlock_detect_unit_8 (
        .reset(dl_reset),
        .clock(dl_clock),
        .proc_dep_vld_vec(proc_dep_vld_vec_8),
        .in_chan_dep_vld_vec(in_chan_dep_vld_vec_8),
        .in_chan_dep_data_vec(in_chan_dep_data_vec_8),
        .token_in_vec(token_in_vec_8),
        .dl_detect_in(dl_detect_out),
        .origin(origin[8]),
        .token_clear(token_clear),
        .out_chan_dep_vld_vec(out_chan_dep_vld_vec_8),
        .out_chan_dep_data(out_chan_dep_data_8),
        .token_out_vec(token_out_vec_8),
        .dl_detect_out(dl_in_vec[8]));

    assign proc_8_data_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_786_3_proc4_U0.out_stream_blk_n);
    assign proc_8_data_PIPO_blk[0] = 1'b0;
    assign proc_8_start_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.start_for_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_bRq_U.if_empty_n & grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_786_3_proc4_U0.ap_idle & ~grp_proc_fu_347.warpTransform_100_50_0_true_7_600_1000_1_false_U0.grp_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_s_fu_22.start_for_warpTransformKrnl_100_50_0_true_7_600_1000_1_false_Loop_VITIS_LOOP_bRq_U.if_write);
    assign proc_8_TLF_FIFO_blk[0] = 1'b0;
    assign proc_8_input_sync_blk[0] = 1'b0;
    assign proc_8_output_sync_blk[0] = 1'b0;
    assign proc_dep_vld_vec_8[0] = dl_detect_out ? proc_dep_vld_vec_8_reg[0] : (proc_8_data_FIFO_blk[0] | proc_8_data_PIPO_blk[0] | proc_8_start_FIFO_blk[0] | proc_8_TLF_FIFO_blk[0] | proc_8_input_sync_blk[0] | proc_8_output_sync_blk[0]);
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            proc_dep_vld_vec_8_reg <= 'b0;
        end
        else begin
            proc_dep_vld_vec_8_reg <= proc_dep_vld_vec_8;
        end
    end
    assign in_chan_dep_vld_vec_8[0] = dep_chan_vld_7_8;
    assign in_chan_dep_data_vec_8[9 : 0] = dep_chan_data_7_8;
    assign token_in_vec_8[0] = token_7_8;
    assign dep_chan_vld_8_7 = out_chan_dep_vld_vec_8[0];
    assign dep_chan_data_8_7 = out_chan_dep_data_8;
    assign token_8_7 = token_out_vec_8[0];

    // Process: grp_proc_fu_347.proc_Loop_loop_write_proc8_U0
    rt_imp_hls_deadlock_detect_unit #(10, 9, 2, 2) rt_imp_hls_deadlock_detect_unit_9 (
        .reset(dl_reset),
        .clock(dl_clock),
        .proc_dep_vld_vec(proc_dep_vld_vec_9),
        .in_chan_dep_vld_vec(in_chan_dep_vld_vec_9),
        .in_chan_dep_data_vec(in_chan_dep_data_vec_9),
        .token_in_vec(token_in_vec_9),
        .dl_detect_in(dl_detect_out),
        .origin(origin[9]),
        .token_clear(token_clear),
        .out_chan_dep_vld_vec(out_chan_dep_vld_vec_9),
        .out_chan_dep_data(out_chan_dep_data_9),
        .token_out_vec(token_out_vec_9),
        .dl_detect_out(dl_in_vec[9]));

    assign proc_9_data_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.proc_Loop_loop_write_proc8_U0.out_mat_data_blk_n);
    assign proc_9_data_PIPO_blk[0] = 1'b0;
    assign proc_9_start_FIFO_blk[0] = 1'b0 | (~grp_proc_fu_347.start_for_proc_Loop_loop_write_proc8_U0_U.if_empty_n & grp_proc_fu_347.proc_Loop_loop_write_proc8_U0.ap_idle & ~grp_proc_fu_347.start_for_proc_Loop_loop_write_proc8_U0_U.if_write);
    assign proc_9_TLF_FIFO_blk[0] = 1'b0;
    assign proc_9_input_sync_blk[0] = 1'b0;
    assign proc_9_output_sync_blk[0] = 1'b0;
    assign proc_dep_vld_vec_9[0] = dl_detect_out ? proc_dep_vld_vec_9_reg[0] : (proc_9_data_FIFO_blk[0] | proc_9_data_PIPO_blk[0] | proc_9_start_FIFO_blk[0] | proc_9_TLF_FIFO_blk[0] | proc_9_input_sync_blk[0] | proc_9_output_sync_blk[0]);
    assign proc_9_data_FIFO_blk[1] = 1'b0;
    assign proc_9_data_PIPO_blk[1] = 1'b0;
    assign proc_9_start_FIFO_blk[1] = 1'b0;
    assign proc_9_TLF_FIFO_blk[1] = 1'b0;
    assign proc_9_input_sync_blk[1] = 1'b0;
    assign proc_9_output_sync_blk[1] = 1'b0 | (ap_done_reg_2 & grp_proc_fu_347.proc_Loop_loop_write_proc8_U0.ap_done & ~grp_proc_fu_347.proc_Loop_VITIS_LOOP_78_1_proc5_U0.ap_done);
    assign proc_dep_vld_vec_9[1] = dl_detect_out ? proc_dep_vld_vec_9_reg[1] : (proc_9_data_FIFO_blk[1] | proc_9_data_PIPO_blk[1] | proc_9_start_FIFO_blk[1] | proc_9_TLF_FIFO_blk[1] | proc_9_input_sync_blk[1] | proc_9_output_sync_blk[1]);
    always @ (negedge dl_reset or posedge dl_clock) begin
        if (~dl_reset) begin
            proc_dep_vld_vec_9_reg <= 'b0;
        end
        else begin
            proc_dep_vld_vec_9_reg <= proc_dep_vld_vec_9;
        end
    end
    assign in_chan_dep_vld_vec_9[0] = dep_chan_vld_0_9;
    assign in_chan_dep_data_vec_9[9 : 0] = dep_chan_data_0_9;
    assign token_in_vec_9[0] = token_0_9;
    assign in_chan_dep_vld_vec_9[1] = dep_chan_vld_4_9;
    assign in_chan_dep_data_vec_9[19 : 10] = dep_chan_data_4_9;
    assign token_in_vec_9[1] = token_4_9;
    assign dep_chan_vld_9_4 = out_chan_dep_vld_vec_9[0];
    assign dep_chan_data_9_4 = out_chan_dep_data_9;
    assign token_9_4 = token_out_vec_9[0];
    assign dep_chan_vld_9_0 = out_chan_dep_vld_vec_9[1];
    assign dep_chan_data_9_0 = out_chan_dep_data_9;
    assign token_9_0 = token_out_vec_9[1];


`include "rt_imp_hls_deadlock_report_unit.vh"
