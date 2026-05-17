`timescale 1ns / 1ps

module q3_am_measure_5k10k (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               enable_i,
    input  wire signed [15:0] sample_i,
    input  wire               sample_valid_i,

    output reg  [11:0]        ma_permille_o,
    output reg  [3:0]         fm_khz_o,
    output reg                am_valid_o,
    output reg  [11:0]        ma_metric_o
);

    localparam integer BOXCAR_N       = 500;
    localparam integer ENV_DECIM      = 1000;
    localparam integer ENV_WIN_POINTS = 1000;
    localparam integer DC_SHIFT       = 14;
    localparam integer ENV_DC_SHIFT   = 8;

    localparam integer DC_W       = 36;
    localparam integer ABS_W      = 17;
    localparam integer BOX_SUM_W  = 27;
    localparam integer BOX_PTR_W  = 9;
    localparam integer DECIM_W    = 10;
    localparam integer ENV_CNT_W  = 10;
    localparam integer TOP_SUM_W  = 30;
    localparam integer NUM_W      = 40;
    localparam integer DEN_W      = 31;

    localparam [BOX_SUM_W-1:0] TOP_INIT = {BOX_SUM_W{1'b0}};
    localparam [BOX_SUM_W-1:0] BOT_INIT = {BOX_SUM_W{1'b1}};

    reg signed [DC_W-1:0] dc_acc_r;

    wire signed [DC_W-1:0] sample_q_w =
        {{(DC_W-16){sample_i[15]}}, sample_i} <<< DC_SHIFT;

    wire signed [DC_W-1:0] dc_err_w  = sample_q_w - dc_acc_r;
    wire signed [DC_W-1:0] dc_step_w = dc_err_w >>> DC_SHIFT;
    wire signed [DC_W-1:0] ac_q_w    = sample_q_w - dc_acc_r;
    wire signed [ABS_W-1:0] ac_w     = (ac_q_w >>> DC_SHIFT);

    wire [ABS_W-1:0] abs_w =
        ac_w[ABS_W-1] ? (~ac_w + {{(ABS_W-1){1'b0}}, 1'b1}) : ac_w;

    reg              abs_pipe_valid_r;
    reg [ABS_W-1:0]  abs_pipe_r;

    reg [ABS_W-1:0] abs_mem [0:BOXCAR_N-1];

    reg [BOX_PTR_W-1:0] wr_ptr_r;
    reg [BOX_PTR_W-1:0] fill_cnt_r;
    reg                 box_full_r;
    reg [BOX_SUM_W-1:0] box_sum_r;

    wire [ABS_W-1:0] old_abs_async_w =
        box_full_r ? abs_mem[wr_ptr_r] : {ABS_W{1'b0}};

    reg                 box_s1_valid_r;
    reg [ABS_W-1:0]     box_new_abs_s1_r;
    reg [ABS_W-1:0]     box_old_abs_s1_r;
    reg [BOX_PTR_W-1:0] box_wr_ptr_s1_r;

    wire [BOX_SUM_W-1:0] box_new_ext_w =
        {{(BOX_SUM_W-ABS_W){1'b0}}, box_new_abs_s1_r};

    wire [BOX_SUM_W-1:0] box_old_ext_w =
        {{(BOX_SUM_W-ABS_W){1'b0}}, box_old_abs_s1_r};

    wire [BOX_SUM_W-1:0] box_sum_next_w =
        box_sum_r + box_new_ext_w - box_old_ext_w;

    reg [DECIM_W-1:0] decim_cnt_r;

    wire env_tick_w =
        abs_pipe_valid_r && box_full_r && (decim_cnt_r == ENV_DECIM - 1);

    reg [ENV_CNT_W-1:0] env_cnt_r;
    reg [BOX_SUM_W-1:0] top0_r, top1_r, top2_r, top3_r;
    reg [BOX_SUM_W-1:0] bot0_r, bot1_r, bot2_r, bot3_r;

    reg signed [39:0] env_dc_acc_r;
    reg               env_state_high_r;
    reg [7:0]         env_rise_count_r;
    reg [3:0]         fm_code_est_r;
    reg [9:0]         fm_count_sum_r;
    reg [1:0]         fm_avg_cnt_r;

    reg               env_s1_valid_r;
    reg signed [39:0] env_q_s1_r;
    reg [BOX_SUM_W-1:0] env_box_s1_r;

    reg               env_s2_valid_r;
    reg [BOX_SUM_W-1:0] env_box_s2_r;
    reg signed [39:0] env_step_s2_r;
    reg               env_high_s2_r;
    reg               env_low_s2_r;

    wire signed [39:0] env_err_s1_w  = env_q_s1_r - env_dc_acc_r;
    wire signed [39:0] env_step_s1_w = env_err_s1_w >>> ENV_DC_SHIFT;
    wire               env_high_s1_w = (env_err_s1_w >  40'sd1000);
    wire               env_low_s1_w  = (env_err_s1_w < -40'sd1000);

    wire env_rise_s2_w =
        env_s2_valid_r && (!env_state_high_r) && env_high_s2_r;

    wire [7:0] env_rise_count_next_w =
        env_rise_s2_w ? (env_rise_count_r + 8'd1) : env_rise_count_r;

    // 4-bit direct modulation-frequency code:
    //   4'd1~4'd10 -> 1~10 kHz
    //   4'd0       -> invalid / no confirmed modulation frequency
    // The AM path measures the number of envelope rising edges in a 10-ms window.
    // Clear valid frequencies therefore correspond approximately to counts:
    //   1 kHz -> 10, 2 kHz -> 20, ..., 10 kHz -> 100.
    function [3:0] encode_fm_from_count_10ms;
        input [7:0] edge_count;
        begin
            if      (edge_count < 8'd15) encode_fm_from_count_10ms = 4'd1;
            else if (edge_count < 8'd25) encode_fm_from_count_10ms = 4'd2;
            else if (edge_count < 8'd35) encode_fm_from_count_10ms = 4'd3;
            else if (edge_count < 8'd45) encode_fm_from_count_10ms = 4'd4;
            else if (edge_count < 8'd55) encode_fm_from_count_10ms = 4'd5;
            else if (edge_count < 8'd65) encode_fm_from_count_10ms = 4'd6;
            else if (edge_count < 8'd75) encode_fm_from_count_10ms = 4'd7;
            else if (edge_count < 8'd85) encode_fm_from_count_10ms = 4'd8;
            else if (edge_count < 8'd95) encode_fm_from_count_10ms = 4'd9;
            else                         encode_fm_from_count_10ms = 4'd10;
        end
    endfunction

    reg                 div_start_r;
    reg [NUM_W-1:0]     div_num_r;
    reg [DEN_W-1:0]     div_den_r;
    wire                div_busy_w;
    wire                div_done_w;
    wire [11:0]         ma_raw_w;

    unsigned_divider_saturating #(
        .NUM_W (NUM_W),
        .DEN_W (DEN_W),
        .Q_W   (12)
    ) u_ma_divider (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (div_start_r),
        .numer  (div_num_r),
        .denom  (div_den_r),
        .busy   (div_busy_w),
        .done   (div_done_w),
        .quot   (ma_raw_w)
    );

    function [11:0] quantize_ma_permille;
        input [11:0] ma_raw;
        begin
            if      (ma_raw < 12'd250) quantize_ma_permille = 12'd200;
            else if (ma_raw < 12'd350) quantize_ma_permille = 12'd300;
            else if (ma_raw < 12'd450) quantize_ma_permille = 12'd400;
            else if (ma_raw < 12'd550) quantize_ma_permille = 12'd500;
            else if (ma_raw < 12'd650) quantize_ma_permille = 12'd600;
            else if (ma_raw < 12'd750) quantize_ma_permille = 12'd700;
            else if (ma_raw < 12'd850) quantize_ma_permille = 12'd800;
            else if (ma_raw < 12'd950) quantize_ma_permille = 12'd900;
            else                       quantize_ma_permille = 12'd1000;
        end
    endfunction

    reg                 ma_s1_valid_r;
    reg [TOP_SUM_W-1:0] top01_s1_r;
    reg [TOP_SUM_W-1:0] top23_s1_r;
    reg [TOP_SUM_W-1:0] bot01_s1_r;
    reg [TOP_SUM_W-1:0] bot23_s1_r;
    reg [3:0]           ma_fm_s1_r;

    reg                 ma_s2_valid_r;
    reg [TOP_SUM_W-1:0] top_sum_s2_r;
    reg [TOP_SUM_W-1:0] bot_sum_s2_r;
    reg [3:0]           ma_fm_s2_r;

    reg                 ma_s3_valid_r;
    reg [TOP_SUM_W:0]   den_sum_s3_r;
    reg [TOP_SUM_W-1:0] diff_sum_s3_r;
    reg [3:0]           ma_fm_s3_r;

    reg                 ma_s4_valid_r;
    reg [NUM_W-1:0]     ma_num_s4_r;
    reg [DEN_W-1:0]     ma_den_s4_r;
    reg [3:0]           ma_fm_s4_r;

    reg [3:0]           ma_fm_pending_r;

    wire [9:0] fm_count_sum_next_w =
        fm_count_sum_r + {2'd0, env_rise_count_next_w};

    wire [7:0] fm_count_avg_next_w =
        (fm_count_sum_next_w + 10'd2) >> 2;   // average of 4 windows, rounded

    wire [3:0] fm_code_avg_next_w =
        encode_fm_from_count_10ms(fm_count_avg_next_w);

    wire fm_avg_ready_w = (fm_avg_cnt_r == 2'd3);

    wire [NUM_W-1:0] ma_num_calc_w =
        {diff_sum_s3_r, 10'd0}
        - {{(NUM_W-TOP_SUM_W-4){1'b0}}, diff_sum_s3_r, 4'd0}
        - {{(NUM_W-TOP_SUM_W-3){1'b0}}, diff_sum_s3_r, 3'd0};

    always @(posedge clk) begin
        if (!rst_n) begin
            dc_acc_r          <= {DC_W{1'b0}};

            abs_pipe_valid_r  <= 1'b0;
            abs_pipe_r        <= {ABS_W{1'b0}};

            box_s1_valid_r    <= 1'b0;
            box_new_abs_s1_r  <= {ABS_W{1'b0}};
            box_old_abs_s1_r  <= {ABS_W{1'b0}};
            box_wr_ptr_s1_r   <= {BOX_PTR_W{1'b0}};

            wr_ptr_r          <= {BOX_PTR_W{1'b0}};
            fill_cnt_r        <= {BOX_PTR_W{1'b0}};
            box_full_r        <= 1'b0;
            box_sum_r         <= {BOX_SUM_W{1'b0}};
            decim_cnt_r       <= {DECIM_W{1'b0}};

            env_cnt_r         <= {ENV_CNT_W{1'b0}};
            top0_r            <= TOP_INIT;
            top1_r            <= TOP_INIT;
            top2_r            <= TOP_INIT;
            top3_r            <= TOP_INIT;
            bot0_r            <= BOT_INIT;
            bot1_r            <= BOT_INIT;
            bot2_r            <= BOT_INIT;
            bot3_r            <= BOT_INIT;

            env_dc_acc_r      <= 40'sd0;
            env_state_high_r  <= 1'b0;
            env_rise_count_r  <= 8'd0;
            fm_code_est_r     <= 4'd0;
            fm_count_sum_r    <= 10'd0;
            fm_avg_cnt_r      <= 2'd0;

            env_s1_valid_r    <= 1'b0;
            env_q_s1_r        <= 40'sd0;
            env_box_s1_r      <= {BOX_SUM_W{1'b0}};
            env_s2_valid_r    <= 1'b0;
            env_box_s2_r      <= {BOX_SUM_W{1'b0}};
            env_step_s2_r     <= 40'sd0;
            env_high_s2_r     <= 1'b0;
            env_low_s2_r      <= 1'b0;

            ma_s1_valid_r     <= 1'b0;
            top01_s1_r        <= {TOP_SUM_W{1'b0}};
            top23_s1_r        <= {TOP_SUM_W{1'b0}};
            bot01_s1_r        <= {TOP_SUM_W{1'b0}};
            bot23_s1_r        <= {TOP_SUM_W{1'b0}};
            ma_fm_s1_r        <= 4'd0;

            ma_s2_valid_r     <= 1'b0;
            top_sum_s2_r      <= {TOP_SUM_W{1'b0}};
            bot_sum_s2_r      <= {TOP_SUM_W{1'b0}};
            ma_fm_s2_r        <= 4'd0;

            ma_s3_valid_r     <= 1'b0;
            den_sum_s3_r      <= {(TOP_SUM_W+1){1'b0}};
            diff_sum_s3_r     <= {TOP_SUM_W{1'b0}};
            ma_fm_s3_r        <= 4'd0;

            ma_s4_valid_r     <= 1'b0;
            ma_num_s4_r       <= {NUM_W{1'b0}};
            ma_den_s4_r       <= {DEN_W{1'b0}};
            ma_fm_s4_r        <= 4'd0;
            ma_fm_pending_r   <= 4'd0;

            div_start_r       <= 1'b0;
            div_num_r         <= {NUM_W{1'b0}};
            div_den_r         <= {DEN_W{1'b0}};

            ma_permille_o     <= 12'd0;
            fm_khz_o          <= 4'd0;
            am_valid_o        <= 1'b0;
            ma_metric_o       <= 12'd0;
        end else begin
            div_start_r       <= 1'b0;
            am_valid_o        <= 1'b0;

            abs_pipe_valid_r  <= 1'b0;
            box_s1_valid_r    <= 1'b0;
            env_s1_valid_r    <= 1'b0;
            env_s2_valid_r    <= 1'b0;
            ma_s1_valid_r     <= 1'b0;
            ma_s2_valid_r     <= 1'b0;
            ma_s3_valid_r     <= 1'b0;
            ma_s4_valid_r     <= 1'b0;

            if (!enable_i) begin
                abs_pipe_valid_r <= 1'b0;
                box_s1_valid_r   <= 1'b0;

                wr_ptr_r         <= {BOX_PTR_W{1'b0}};
                fill_cnt_r       <= {BOX_PTR_W{1'b0}};
                box_full_r       <= 1'b0;
                box_sum_r        <= {BOX_SUM_W{1'b0}};
                decim_cnt_r      <= {DECIM_W{1'b0}};

                env_cnt_r        <= {ENV_CNT_W{1'b0}};
                top0_r           <= TOP_INIT;
                top1_r           <= TOP_INIT;
                top2_r           <= TOP_INIT;
                top3_r           <= TOP_INIT;
                bot0_r           <= BOT_INIT;
                bot1_r           <= BOT_INIT;
                bot2_r           <= BOT_INIT;
                bot3_r           <= BOT_INIT;

                env_state_high_r <= 1'b0;
                env_rise_count_r <= 8'd0;
                fm_code_est_r    <= 4'd0;
                fm_count_sum_r   <= 10'd0;
                fm_avg_cnt_r     <= 2'd0;

                env_s1_valid_r   <= 1'b0;
                env_s2_valid_r   <= 1'b0;
                ma_s1_valid_r    <= 1'b0;
                ma_s2_valid_r    <= 1'b0;
                ma_s3_valid_r    <= 1'b0;
                ma_s4_valid_r    <= 1'b0;
                div_start_r      <= 1'b0;
            end else begin
                // Stage A: sample/DC/abs.
                if (sample_valid_i) begin
                    dc_acc_r         <= dc_acc_r + dc_step_w;
                    abs_pipe_r       <= abs_w;
                    abs_pipe_valid_r <= 1'b1;
                end

                // Stage B: request old sample from abs_mem and advance pointer.
                if (abs_pipe_valid_r) begin
                    box_s1_valid_r   <= 1'b1;
                    box_new_abs_s1_r <= abs_pipe_r;
                    box_old_abs_s1_r <= old_abs_async_w;
                    box_wr_ptr_s1_r  <= wr_ptr_r;

                    wr_ptr_r <= (wr_ptr_r == BOXCAR_N - 1) ?
                                {BOX_PTR_W{1'b0}} : (wr_ptr_r + 1'b1);

                    if (!box_full_r) begin
                        if (fill_cnt_r == BOXCAR_N - 1)
                            box_full_r <= 1'b1;
                        else
                            fill_cnt_r <= fill_cnt_r + 1'b1;
                    end

                    if (box_full_r) begin
                        if (decim_cnt_r == ENV_DECIM - 1)
                            decim_cnt_r <= {DECIM_W{1'b0}};
                        else
                            decim_cnt_r <= decim_cnt_r + 1'b1;
                    end
                end

                // Stage C: update RAM and box sum using registered old sample.
                if (box_s1_valid_r) begin
                    abs_mem[box_wr_ptr_s1_r] <= box_new_abs_s1_r;
                    box_sum_r                <= box_sum_next_w;
                end

                if (env_tick_w) begin
                    env_q_s1_r     <= $signed({13'd0, box_sum_r});
                    env_box_s1_r   <= box_sum_r;
                    env_s1_valid_r <= 1'b1;
                end

                if (env_s1_valid_r) begin
                    env_box_s2_r   <= env_box_s1_r;
                    env_step_s2_r  <= env_step_s1_w;
                    env_high_s2_r  <= env_high_s1_w;
                    env_low_s2_r   <= env_low_s1_w;
                    env_s2_valid_r <= 1'b1;
                end

                if (env_s2_valid_r) begin
                    env_dc_acc_r <= env_dc_acc_r + env_step_s2_r;

                    if (env_high_s2_r)
                        env_state_high_r <= 1'b1;
                    else if (env_low_s2_r)
                        env_state_high_r <= 1'b0;

                    if (env_box_s2_r > top0_r) begin
                        top3_r <= top2_r;
                        top2_r <= top1_r;
                        top1_r <= top0_r;
                        top0_r <= env_box_s2_r;
                    end else if (env_box_s2_r > top1_r) begin
                        top3_r <= top2_r;
                        top2_r <= top1_r;
                        top1_r <= env_box_s2_r;
                    end else if (env_box_s2_r > top2_r) begin
                        top3_r <= top2_r;
                        top2_r <= env_box_s2_r;
                    end else if (env_box_s2_r > top3_r) begin
                        top3_r <= env_box_s2_r;
                    end

                    if (env_box_s2_r < bot0_r) begin
                        bot3_r <= bot2_r;
                        bot2_r <= bot1_r;
                        bot1_r <= bot0_r;
                        bot0_r <= env_box_s2_r;
                    end else if (env_box_s2_r < bot1_r) begin
                        bot3_r <= bot2_r;
                        bot2_r <= bot1_r;
                        bot1_r <= env_box_s2_r;
                    end else if (env_box_s2_r < bot2_r) begin
                        bot3_r <= bot2_r;
                        bot2_r <= env_box_s2_r;
                    end else if (env_box_s2_r < bot3_r) begin
                        bot3_r <= env_box_s2_r;
                    end

                    if (env_cnt_r == ENV_WIN_POINTS - 1) begin
                        env_cnt_r        <= {ENV_CNT_W{1'b0}};
                        env_rise_count_r <= 8'd0;

                        // Average four 10-ms frequency measurements, then collapse
                        // the averaged count to the valid modulation-frequency code.
                        if (fm_avg_ready_w) begin
                            fm_code_est_r  <= fm_code_avg_next_w;
                            fm_count_sum_r <= 10'd0;
                            fm_avg_cnt_r   <= 2'd0;

                            ma_s1_valid_r <= 1'b1;
                            top01_s1_r    <= {3'd0, top0_r} + {3'd0, top1_r};
                            top23_s1_r    <= {3'd0, top2_r} + {3'd0, top3_r};
                            bot01_s1_r    <= {3'd0, bot0_r} + {3'd0, bot1_r};
                            bot23_s1_r    <= {3'd0, bot2_r} + {3'd0, bot3_r};
                            ma_fm_s1_r    <= fm_code_avg_next_w;
                        end else begin
                            fm_count_sum_r <= fm_count_sum_next_w;
                            fm_avg_cnt_r   <= fm_avg_cnt_r + 2'd1;
                        end

                        top0_r <= TOP_INIT;
                        top1_r <= TOP_INIT;
                        top2_r <= TOP_INIT;
                        top3_r <= TOP_INIT;
                        bot0_r <= BOT_INIT;
                        bot1_r <= BOT_INIT;
                        bot2_r <= BOT_INIT;
                        bot3_r <= BOT_INIT;
                    end else begin
                        env_cnt_r        <= env_cnt_r + 1'b1;
                        env_rise_count_r <= env_rise_count_next_w;
                    end
                end

                if (ma_s1_valid_r) begin
                    ma_s2_valid_r <= 1'b1;
                    top_sum_s2_r  <= top01_s1_r + top23_s1_r;
                    bot_sum_s2_r  <= bot01_s1_r + bot23_s1_r;
                    ma_fm_s2_r    <= ma_fm_s1_r;
                end

                if (ma_s2_valid_r) begin
                    ma_s3_valid_r <= 1'b1;
                    den_sum_s3_r  <= {1'b0, top_sum_s2_r} + {1'b0, bot_sum_s2_r};
                    diff_sum_s3_r <= (top_sum_s2_r > bot_sum_s2_r) ?
                                     (top_sum_s2_r - bot_sum_s2_r) :
                                     {TOP_SUM_W{1'b0}};
                    ma_fm_s3_r    <= ma_fm_s2_r;
                end

                if (ma_s3_valid_r) begin
                    ma_s4_valid_r <= 1'b1;
                    ma_num_s4_r   <= ma_num_calc_w;
                    ma_den_s4_r   <= den_sum_s3_r[DEN_W-1:0];
                    ma_fm_s4_r    <= ma_fm_s3_r;
                end

                if (ma_s4_valid_r) begin
                    if (!div_busy_w && ma_den_s4_r != {DEN_W{1'b0}}) begin
                        div_num_r       <= ma_num_s4_r;
                        div_den_r       <= ma_den_s4_r;
                        ma_fm_pending_r <= ma_fm_s4_r;
                        div_start_r     <= 1'b1;
                    end
                end
            end

            if (div_done_w) begin
                ma_metric_o   <= (ma_raw_w > 12'd1000) ? 12'd1000 : ma_raw_w;
                ma_permille_o <= quantize_ma_permille((ma_raw_w > 12'd1000) ? 12'd1000 : ma_raw_w);
                fm_khz_o      <= ma_fm_pending_r;
                am_valid_o    <= 1'b1;
            end
        end
    end

endmodule