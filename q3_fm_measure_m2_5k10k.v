`timescale 1ns / 1ps

module q3_fm_measure_m2_5k10k (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable_i,
    input  wire        cmp_i,
    input  wire        cmp_valid_i,

    output reg  [3:0]  fm_khz_o,
    output reg  [3:0]  mf_int_o,
    output reg  [6:0]  df_khz_o,
    output reg  [16:0] df_hz_o,
    output reg         fm_valid_o,
    output reg  [16:0] df_metric_hz_o
);

    localparam integer TS_W          = 32;
    localparam integer DELTA_W       = 16;
    localparam integer FI_W          = 24;
    localparam integer FD_W          = 25;
    localparam integer FAVG_SHIFT    = 10;
    // A larger statistics window reduces frame-to-frame df jitter near the 6/7 boundary.
    // At about 100 k frequency-estimate samples/s, 2048 samples are roughly 20 ms.
    localparam integer WIN_SAMPLES   = 2048;
    localparam integer WIN_CNT_W     = 11;
    localparam integer PERIOD_W      = 18;
    localparam integer PERIOD_SUM_W  = 20;

    // mf = 7 and fm = 10 kHz gives 200 kHz +/- 70 kHz, namely 130~270 kHz.
    // The old 120~280 kHz gate was too close to the edge after real front-end
    // zero-crossing jitter, so a few useful peak samples could be discarded.
    localparam [DELTA_W-1:0] DELTA_TICKS_MIN = 16'd1100;
    localparam [DELTA_W-1:0] DELTA_TICKS_MAX = 16'd5500;
    localparam [FI_W-1:0]    FI_MIN_HZ       = 24'd100000;
    localparam [FI_W-1:0]    FI_MAX_HZ       = 24'd300000;
    localparam [FI_W-1:0]    IF_INIT_HZ      = 24'd200000;
    localparam signed [FI_W+1:0] FBAR_MAX_HZ_S = 26'sd400000;

    localparam [FI_W-1:0] FI_TOP_INIT = {FI_W{1'b0}};
    localparam [FI_W-1:0] FI_BOT_INIT = {FI_W{1'b1}};

    reg [TS_W-1:0]     timestamp_r;
    reg                cmp_d_r;
    reg [1:0]          edge_count_r;
    reg [TS_W-1:0]     ts_z1_r;
    reg [TS_W-1:0]     ts_z2_r;
    reg [DELTA_W-1:0]  delta_ticks_r;
    reg                div_start_r;

    wire rising_edge_w = enable_i & cmp_valid_i & cmp_i & (~cmp_d_r);
    wire [TS_W-1:0] delta_ticks_full_w = timestamp_r - ts_z2_r;

    wire delta_range_ok_w =
        (delta_ticks_full_w[TS_W-1:DELTA_W] == {(TS_W-DELTA_W){1'b0}}) &&
        (delta_ticks_full_w[DELTA_W-1:0] >= DELTA_TICKS_MIN) &&
        (delta_ticks_full_w[DELTA_W-1:0] <= DELTA_TICKS_MAX);

    wire              div_busy_w;
    wire              div_done_w;
    wire [FI_W-1:0]   fi_hz_w;

    unsigned_divider_saturating #(
        .NUM_W (29),
        .DEN_W (DELTA_W),
        .Q_W   (FI_W)
    ) u_fi_divider (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (div_start_r),
        .numer  (29'd400000000),
        .denom  (delta_ticks_r),
        .busy   (div_busy_w),
        .done   (div_done_w),
        .quot   (fi_hz_w)
    );

    reg             fi_raw_valid_r;
    reg [FI_W-1:0] fi_raw_hz_r;
    reg             fi_proc_valid_r;
    reg [FI_W-1:0] fi_proc_hz_r;

    wire fi_raw_in_range_w =
        (fi_raw_hz_r >= FI_MIN_HZ) && (fi_raw_hz_r <= FI_MAX_HZ);

    reg [FI_W-1:0] fbar_hz_r;

    reg                   fbar_s1_valid_r;
    reg signed [FI_W:0]   fbar_err_s1_r;
    reg [FI_W-1:0]        fbar_base_s1_r;
    reg                   fbar_s2_valid_r;
    reg signed [FI_W:0]   fbar_step_s2_r;
    reg [FI_W-1:0]        fbar_base_s2_r;
    reg                   fbar_s3_valid_r;
    reg signed [FI_W+1:0] fbar_candidate_s3_r;
    reg                   fbar_s4_valid_r;
    reg [FI_W-1:0]        fbar_clamped_s4_r;

    wire signed [FI_W:0] fbar_err_now_w =
        $signed({1'b0, fi_proc_hz_r}) - $signed({1'b0, fbar_hz_r});

    wire signed [FI_W+1:0] fbar_candidate_w =
        $signed({2'b00, fbar_base_s2_r}) +
        $signed({fbar_step_s2_r[FI_W], fbar_step_s2_r});

    reg signed [FD_W-1:0] fd_sample_r;
    reg                   fd_valid_r;
    reg [TS_W-1:0]        fd_last_zc_ts_r;
    reg                   fd_state_high_r;
    reg [3:0]             fm_khz_est_r;
    reg [3:0]             fm_code_est_r;

    // Timing-friendly modulation-period averaging pipeline.
    // The valid modulation period is < 250000 ticks, so 18 low bits are enough
    // for a modulo timestamp subtraction at 200 MHz. This avoids a 32-bit
    // subtract + 34-bit add + large compare chain in one 200 MHz cycle.
    reg                         fd_period_s1_valid_r;
    reg [PERIOD_W-1:0]          fd_period_s1_ticks_r;
    reg [PERIOD_SUM_W-1:0]      fd_period_sum_r;
    reg [1:0]                   fd_period_avg_cnt_r;
    reg                         fd_period_avg_valid_r;
    reg [PERIOD_W-1:0]          fd_period_avg_ticks_r;

    // The minimum extended FM case is mf=1, fm=1 kHz, namely df=1 kHz.
    // Use a lower Schmitt threshold than the old 1-kHz boundary, otherwise the
    // fd zero-crossing period detector may miss this minimum-deviation case.
    wire fd_high_w = (fd_sample_r >  25'sd400);
    wire fd_low_w  = (fd_sample_r < -25'sd400);
    wire fd_rise_w = fd_valid_r && (!fd_state_high_r) && fd_high_w;
    wire [PERIOD_W-1:0] fd_period_ticks_w =
        timestamp_r[PERIOD_W-1:0] - fd_last_zc_ts_r[PERIOD_W-1:0];

    wire fd_period_range_ok_w =
        (fd_period_ticks_w > 18'd15000) &&
        (fd_period_ticks_w < 18'd250000);

    function [3:0] round_fm_khz_from_period;
        input [PERIOD_W-1:0] period_ticks;
        begin
            if      (period_ticks > 18'd133333) round_fm_khz_from_period = 4'd1;
            else if (period_ticks > 18'd80000)  round_fm_khz_from_period = 4'd2;
            else if (period_ticks > 18'd57143)  round_fm_khz_from_period = 4'd3;
            else if (period_ticks > 18'd44444)  round_fm_khz_from_period = 4'd4;
            else if (period_ticks > 18'd36364)  round_fm_khz_from_period = 4'd5;
            else if (period_ticks > 18'd30769)  round_fm_khz_from_period = 4'd6;
            else if (period_ticks > 18'd26667)  round_fm_khz_from_period = 4'd7;
            else if (period_ticks > 18'd23529)  round_fm_khz_from_period = 4'd8;
            else if (period_ticks > 18'd21053)  round_fm_khz_from_period = 4'd9;
            else                                round_fm_khz_from_period = 4'd10;
        end
    endfunction

    // 4-bit direct modulation-frequency code:
    //   4'd1~4'd10 -> 1~10 kHz
    //   4'd0       -> invalid / no confirmed modulation frequency
    function [3:0] encode_fm_from_period;
        input [PERIOD_W-1:0] period_ticks;
        begin
            encode_fm_from_period = round_fm_khz_from_period(period_ticks);
        end
    endfunction

    wire [PERIOD_SUM_W-1:0] fd_period_sum_next_w =
        fd_period_sum_r + {{(PERIOD_SUM_W-PERIOD_W){1'b0}}, fd_period_s1_ticks_r};

    wire [PERIOD_SUM_W-1:0] fd_period_sum_round_w =
        fd_period_sum_next_w + {{(PERIOD_SUM_W-3){1'b0}}, 3'd2};

    wire [PERIOD_W-1:0] fd_period_avg_next_w =
        fd_period_sum_round_w[PERIOD_W+1:2];

    wire fd_period_avg_ready_w = (fd_period_avg_cnt_r == 2'd3);

    reg [FI_W-1:0] top0_r, top1_r, top2_r, top3_r;
    reg [FI_W-1:0] bot0_r, bot1_r, bot2_r, bot3_r;
    reg [WIN_CNT_W-1:0] win_cnt_r;

    function [95:0] insert_top4;
        input [FI_W-1:0] v;
        input [FI_W-1:0] t0;
        input [FI_W-1:0] t1;
        input [FI_W-1:0] t2;
        input [FI_W-1:0] t3;
        begin
            if (v > t0)
                insert_top4 = {v, t0, t1, t2};
            else if (v > t1)
                insert_top4 = {t0, v, t1, t2};
            else if (v > t2)
                insert_top4 = {t0, t1, v, t2};
            else if (v > t3)
                insert_top4 = {t0, t1, t2, v};
            else
                insert_top4 = {t0, t1, t2, t3};
        end
    endfunction

    function [95:0] insert_bot4;
        input [FI_W-1:0] v;
        input [FI_W-1:0] b0;
        input [FI_W-1:0] b1;
        input [FI_W-1:0] b2;
        input [FI_W-1:0] b3;
        begin
            if (v < b0)
                insert_bot4 = {v, b0, b1, b2};
            else if (v < b1)
                insert_bot4 = {b0, v, b1, b2};
            else if (v < b2)
                insert_bot4 = {b0, b1, v, b2};
            else if (v < b3)
                insert_bot4 = {b0, b1, b2, v};
            else
                insert_bot4 = {b0, b1, b2, b3};
        end
    endfunction

    wire [95:0] top_next_pack_w = insert_top4(fi_proc_hz_r, top0_r, top1_r, top2_r, top3_r);
    wire [95:0] bot_next_pack_w = insert_bot4(fi_proc_hz_r, bot0_r, bot1_r, bot2_r, bot3_r);

    wire [FI_W-1:0] top0_next_w = top_next_pack_w[95:72];
    wire [FI_W-1:0] top1_next_w = top_next_pack_w[71:48];
    wire [FI_W-1:0] top2_next_w = top_next_pack_w[47:24];
    wire [FI_W-1:0] top3_next_w = top_next_pack_w[23:0];

    wire [FI_W-1:0] bot0_next_w = bot_next_pack_w[95:72];
    wire [FI_W-1:0] bot1_next_w = bot_next_pack_w[71:48];
    wire [FI_W-1:0] bot2_next_w = bot_next_pack_w[47:24];
    wire [FI_W-1:0] bot3_next_w = bot_next_pack_w[23:0];

    reg             stat_s1_valid_r;
    reg [FI_W-1:0] stat_top2_s1_r;
    reg [FI_W-1:0] stat_top3_s1_r;
    reg [FI_W-1:0] stat_bot2_s1_r;
    reg [FI_W-1:0] stat_bot3_s1_r;
    reg [3:0]      stat_fm_khz_s1_r;
    reg [3:0]      stat_fm_code_s1_r;

    reg             stat_s2_valid_r;
    reg [FI_W:0]   stat_top_sum_s2_r;
    reg [FI_W:0]   stat_bot_sum_s2_r;
    reg [3:0]      stat_fm_khz_s2_r;
    reg [3:0]      stat_fm_code_s2_r;

    reg             stat_s3_valid_r;
    reg [FI_W-1:0] stat_span_raw_s3_r;
    reg             stat_span_pos_s3_r;
    reg [3:0]      stat_fm_khz_s3_r;
    reg [3:0]      stat_fm_code_s3_r;

    reg             stat_s4_valid_r;
    reg [FI_W-1:0] stat_span_s4_r;
    reg [3:0]      stat_fm_khz_s4_r;
    reg [3:0]      stat_fm_code_s4_r;

    reg             stat_s5_valid_r;
    reg [16:0]     stat_df_s5_r;
    reg [3:0]      stat_fm_khz_s5_r;
    reg [3:0]      stat_fm_code_s5_r;

    wire [FI_W-1:0] stat_top_avg_s2_w = stat_top_sum_s2_r[FI_W:1];
    wire [FI_W-1:0] stat_bot_avg_s2_w = stat_bot_sum_s2_r[FI_W:1];
    wire [FI_W-1:0] stat_span_raw_w    = stat_top_avg_s2_w - stat_bot_avg_s2_w;
    wire            stat_span_pos_w    = stat_top_avg_s2_w > stat_bot_avg_s2_w;

    wire [FI_W-1:0] stat_span_clamped_w =
        stat_span_pos_s3_r ? stat_span_raw_s3_r : {FI_W{1'b0}};
    wire [FI_W-1:0] stat_df_half_w = {1'b0, stat_span_s4_r[FI_W-1:1]};
    wire [16:0] stat_df_sat_w =
        (stat_df_half_w > 24'd131071) ? 17'h1FFFF : stat_df_half_w[16:0];

    // Timing note:
    // The first 1~10 kHz extension used expressions such as
    //     th15 = fm_khz * 1500;
    //     df_khz = mf_int * fm_khz;
    // Vivado mapped these small products into long carry chains on the 200 MHz
    // result-output path.  The timing report showed the worst paths from
    // res_fm_khz_s1_r[0] to df_khz_o_reg.  Replace these variable products by
    // fixed lookup tables.  Functional output is unchanged because fm_khz is
    // only the integer set 1~10.
    function [3:0] round_mf_from_df;
        input [16:0] df_hz;
        input [3:0]  fm_khz;
        input [3:0]  last_mf;
        reg [20:0] th15;
        reg [20:0] th25;
        reg [20:0] th35;
        reg [20:0] th45;
        reg [20:0] th55;
        reg [20:0] th62;
        reg [20:0] th635;
        reg [20:0] df_ext;
        begin
            case (fm_khz)
                4'd1: begin th15=21'd1500;  th25=21'd2500;  th35=21'd3500;  th45=21'd4500;  th55=21'd5500;  th62=21'd6200;  th635=21'd6350;  end
                4'd2: begin th15=21'd3000;  th25=21'd5000;  th35=21'd7000;  th45=21'd9000;  th55=21'd11000; th62=21'd12400; th635=21'd12700; end
                4'd3: begin th15=21'd4500;  th25=21'd7500;  th35=21'd10500; th45=21'd13500; th55=21'd16500; th62=21'd18600; th635=21'd19050; end
                4'd4: begin th15=21'd6000;  th25=21'd10000; th35=21'd14000; th45=21'd18000; th55=21'd22000; th62=21'd24800; th635=21'd25400; end
                4'd5: begin th15=21'd7500;  th25=21'd12500; th35=21'd17500; th45=21'd22500; th55=21'd27500; th62=21'd31000; th635=21'd31750; end
                4'd6: begin th15=21'd9000;  th25=21'd15000; th35=21'd21000; th45=21'd27000; th55=21'd33000; th62=21'd37200; th635=21'd38100; end
                4'd7: begin th15=21'd10500; th25=21'd17500; th35=21'd24500; th45=21'd31500; th55=21'd38500; th62=21'd43400; th635=21'd44450; end
                4'd8: begin th15=21'd12000; th25=21'd20000; th35=21'd28000; th45=21'd36000; th55=21'd44000; th62=21'd49600; th635=21'd50800; end
                4'd9: begin th15=21'd13500; th25=21'd22500; th35=21'd31500; th45=21'd40500; th55=21'd49500; th62=21'd55800; th635=21'd57150; end
                default: begin th15=21'd15000; th25=21'd25000; th35=21'd35000; th45=21'd45000; th55=21'd55000; th62=21'd62000; th635=21'd63500; end
            endcase

            df_ext = {4'd0, df_hz};

            if      (df_ext < th15) round_mf_from_df = 4'd1;
            else if (df_ext < th25) round_mf_from_df = 4'd2;
            else if (df_ext < th35) round_mf_from_df = 4'd3;
            else if (df_ext < th45) round_mf_from_df = 4'd4;
            else if (df_ext < th55) round_mf_from_df = 4'd5;
            else if (last_mf == 4'd7) begin
                if (df_ext < th62) round_mf_from_df = 4'd6;
                else               round_mf_from_df = 4'd7;
            end else begin
                if (df_ext < th635) round_mf_from_df = 4'd6;
                else                round_mf_from_df = 4'd7;
            end
        end
    endfunction

    function [6:0] df_khz_from_mf_fm;
        input [3:0] mf_int;
        input [3:0] fm_khz;
        begin
            // Lookup table for df_kHz = mf * fm.  This keeps the result-output
            // path in LUT muxes instead of a synthesized variable multiplier.
            case (mf_int)
                4'd1: begin
                    case (fm_khz)
                        4'd1: df_khz_from_mf_fm = 7'd1;
                        4'd2: df_khz_from_mf_fm = 7'd2;
                        4'd3: df_khz_from_mf_fm = 7'd3;
                        4'd4: df_khz_from_mf_fm = 7'd4;
                        4'd5: df_khz_from_mf_fm = 7'd5;
                        4'd6: df_khz_from_mf_fm = 7'd6;
                        4'd7: df_khz_from_mf_fm = 7'd7;
                        4'd8: df_khz_from_mf_fm = 7'd8;
                        4'd9: df_khz_from_mf_fm = 7'd9;
                        default: df_khz_from_mf_fm = 7'd10;
                    endcase
                end
                4'd2: begin
                    case (fm_khz)
                        4'd1: df_khz_from_mf_fm = 7'd2;
                        4'd2: df_khz_from_mf_fm = 7'd4;
                        4'd3: df_khz_from_mf_fm = 7'd6;
                        4'd4: df_khz_from_mf_fm = 7'd8;
                        4'd5: df_khz_from_mf_fm = 7'd10;
                        4'd6: df_khz_from_mf_fm = 7'd12;
                        4'd7: df_khz_from_mf_fm = 7'd14;
                        4'd8: df_khz_from_mf_fm = 7'd16;
                        4'd9: df_khz_from_mf_fm = 7'd18;
                        default: df_khz_from_mf_fm = 7'd20;
                    endcase
                end
                4'd3: begin
                    case (fm_khz)
                        4'd1: df_khz_from_mf_fm = 7'd3;
                        4'd2: df_khz_from_mf_fm = 7'd6;
                        4'd3: df_khz_from_mf_fm = 7'd9;
                        4'd4: df_khz_from_mf_fm = 7'd12;
                        4'd5: df_khz_from_mf_fm = 7'd15;
                        4'd6: df_khz_from_mf_fm = 7'd18;
                        4'd7: df_khz_from_mf_fm = 7'd21;
                        4'd8: df_khz_from_mf_fm = 7'd24;
                        4'd9: df_khz_from_mf_fm = 7'd27;
                        default: df_khz_from_mf_fm = 7'd30;
                    endcase
                end
                4'd4: begin
                    case (fm_khz)
                        4'd1: df_khz_from_mf_fm = 7'd4;
                        4'd2: df_khz_from_mf_fm = 7'd8;
                        4'd3: df_khz_from_mf_fm = 7'd12;
                        4'd4: df_khz_from_mf_fm = 7'd16;
                        4'd5: df_khz_from_mf_fm = 7'd20;
                        4'd6: df_khz_from_mf_fm = 7'd24;
                        4'd7: df_khz_from_mf_fm = 7'd28;
                        4'd8: df_khz_from_mf_fm = 7'd32;
                        4'd9: df_khz_from_mf_fm = 7'd36;
                        default: df_khz_from_mf_fm = 7'd40;
                    endcase
                end
                4'd5: begin
                    case (fm_khz)
                        4'd1: df_khz_from_mf_fm = 7'd5;
                        4'd2: df_khz_from_mf_fm = 7'd10;
                        4'd3: df_khz_from_mf_fm = 7'd15;
                        4'd4: df_khz_from_mf_fm = 7'd20;
                        4'd5: df_khz_from_mf_fm = 7'd25;
                        4'd6: df_khz_from_mf_fm = 7'd30;
                        4'd7: df_khz_from_mf_fm = 7'd35;
                        4'd8: df_khz_from_mf_fm = 7'd40;
                        4'd9: df_khz_from_mf_fm = 7'd45;
                        default: df_khz_from_mf_fm = 7'd50;
                    endcase
                end
                4'd6: begin
                    case (fm_khz)
                        4'd1: df_khz_from_mf_fm = 7'd6;
                        4'd2: df_khz_from_mf_fm = 7'd12;
                        4'd3: df_khz_from_mf_fm = 7'd18;
                        4'd4: df_khz_from_mf_fm = 7'd24;
                        4'd5: df_khz_from_mf_fm = 7'd30;
                        4'd6: df_khz_from_mf_fm = 7'd36;
                        4'd7: df_khz_from_mf_fm = 7'd42;
                        4'd8: df_khz_from_mf_fm = 7'd48;
                        4'd9: df_khz_from_mf_fm = 7'd54;
                        default: df_khz_from_mf_fm = 7'd60;
                    endcase
                end
                default: begin
                    case (fm_khz)
                        4'd1: df_khz_from_mf_fm = 7'd7;
                        4'd2: df_khz_from_mf_fm = 7'd14;
                        4'd3: df_khz_from_mf_fm = 7'd21;
                        4'd4: df_khz_from_mf_fm = 7'd28;
                        4'd5: df_khz_from_mf_fm = 7'd35;
                        4'd6: df_khz_from_mf_fm = 7'd42;
                        4'd7: df_khz_from_mf_fm = 7'd49;
                        4'd8: df_khz_from_mf_fm = 7'd56;
                        4'd9: df_khz_from_mf_fm = 7'd63;
                        default: df_khz_from_mf_fm = 7'd70;
                    endcase
                end
            endcase
        end
    endfunction

    reg        res_s1_valid_r;
    reg [3:0]  res_fm_khz_s1_r;
    reg [3:0]  res_fm_code_s1_r;
    reg [16:0] res_df_s1_r;

    reg        res_s2_valid_r;
    reg [3:0]  res_fm_khz_s2_r;
    reg [3:0]  res_fm_code_s2_r;
    reg [16:0] res_df_s2_r;
    reg [3:0]  res_mf_s2_r;

    reg        res_s3_valid_r;
    reg [3:0]  res_fm_khz_s3_r;
    reg [3:0]  res_fm_code_s3_r;
    reg [16:0] res_df_s3_r;
    reg [3:0]  res_mf_s3_r;
    reg [6:0]  res_df_khz_s3_r;

    always @(posedge clk) begin
        if (!rst_n) begin
            timestamp_r          <= {TS_W{1'b0}};
            cmp_d_r              <= 1'b0;
            edge_count_r         <= 2'd0;
            ts_z1_r              <= {TS_W{1'b0}};
            ts_z2_r              <= {TS_W{1'b0}};
            delta_ticks_r        <= {DELTA_W{1'b0}};
            div_start_r          <= 1'b0;

            fi_raw_valid_r       <= 1'b0;
            fi_raw_hz_r          <= {FI_W{1'b0}};
            fi_proc_valid_r      <= 1'b0;
            fi_proc_hz_r         <= {FI_W{1'b0}};

            fbar_hz_r            <= IF_INIT_HZ;
            fbar_s1_valid_r      <= 1'b0;
            fbar_err_s1_r        <= {(FI_W+1){1'b0}};
            fbar_base_s1_r       <= IF_INIT_HZ;
            fbar_s2_valid_r      <= 1'b0;
            fbar_step_s2_r       <= {(FI_W+1){1'b0}};
            fbar_base_s2_r       <= IF_INIT_HZ;
            fbar_s3_valid_r      <= 1'b0;
            fbar_candidate_s3_r  <= {(FI_W+2){1'b0}};
            fbar_s4_valid_r      <= 1'b0;
            fbar_clamped_s4_r    <= IF_INIT_HZ;

            fd_sample_r          <= {FD_W{1'b0}};
            fd_valid_r           <= 1'b0;
            fd_state_high_r      <= 1'b0;
            fd_last_zc_ts_r      <= {TS_W{1'b0}};
            fm_khz_est_r         <= 4'd5;
            fm_code_est_r        <= 4'd0;
            fd_period_s1_valid_r <= 1'b0;
            fd_period_s1_ticks_r <= {PERIOD_W{1'b0}};
            fd_period_sum_r      <= {PERIOD_SUM_W{1'b0}};
            fd_period_avg_cnt_r  <= 2'd0;
            fd_period_avg_valid_r <= 1'b0;
            fd_period_avg_ticks_r <= {PERIOD_W{1'b0}};

            top0_r               <= FI_TOP_INIT;
            top1_r               <= FI_TOP_INIT;
            top2_r               <= FI_TOP_INIT;
            top3_r               <= FI_TOP_INIT;
            bot0_r               <= FI_BOT_INIT;
            bot1_r               <= FI_BOT_INIT;
            bot2_r               <= FI_BOT_INIT;
            bot3_r               <= FI_BOT_INIT;
            win_cnt_r            <= {WIN_CNT_W{1'b0}};

            stat_s1_valid_r      <= 1'b0;
            stat_top2_s1_r       <= {FI_W{1'b0}};
            stat_top3_s1_r       <= {FI_W{1'b0}};
            stat_bot2_s1_r       <= {FI_W{1'b0}};
            stat_bot3_s1_r       <= {FI_W{1'b0}};
            stat_fm_khz_s1_r     <= 4'd5;
            stat_fm_code_s1_r    <= 4'd0;
            stat_s2_valid_r      <= 1'b0;
            stat_top_sum_s2_r    <= {(FI_W+1){1'b0}};
            stat_bot_sum_s2_r    <= {(FI_W+1){1'b0}};
            stat_fm_khz_s2_r     <= 4'd5;
            stat_fm_code_s2_r    <= 4'd0;
            stat_s3_valid_r      <= 1'b0;
            stat_span_raw_s3_r   <= {FI_W{1'b0}};
            stat_span_pos_s3_r   <= 1'b0;
            stat_fm_khz_s3_r     <= 4'd5;
            stat_fm_code_s3_r    <= 4'd0;
            stat_s4_valid_r      <= 1'b0;
            stat_span_s4_r       <= {FI_W{1'b0}};
            stat_fm_khz_s4_r     <= 4'd5;
            stat_fm_code_s4_r    <= 4'd0;
            stat_s5_valid_r      <= 1'b0;
            stat_df_s5_r         <= 17'd0;
            stat_fm_khz_s5_r     <= 4'd5;
            stat_fm_code_s5_r    <= 4'd0;

            res_s1_valid_r       <= 1'b0;
            res_fm_khz_s1_r      <= 4'd5;
            res_fm_code_s1_r     <= 4'd0;
            res_df_s1_r          <= 17'd0;
            res_s2_valid_r       <= 1'b0;
            res_fm_khz_s2_r      <= 4'd5;
            res_fm_code_s2_r     <= 4'd0;
            res_df_s2_r          <= 17'd0;
            res_mf_s2_r          <= 4'd0;
            res_s3_valid_r       <= 1'b0;
            res_fm_khz_s3_r      <= 4'd5;
            res_fm_code_s3_r     <= 4'd0;
            res_df_s3_r          <= 17'd0;
            res_mf_s3_r          <= 4'd0;
            res_df_khz_s3_r      <= 7'd0;

            fm_khz_o             <= 4'd0;
            mf_int_o             <= 4'd0;
            df_khz_o             <= 7'd0;
            df_hz_o              <= 17'd0;
            fm_valid_o           <= 1'b0;
            df_metric_hz_o       <= 17'd0;
        end else begin
            timestamp_r     <= timestamp_r + 32'd1;
            div_start_r     <= 1'b0;
            fd_valid_r      <= 1'b0;
            fm_valid_o      <= 1'b0;
            fd_period_s1_valid_r  <= 1'b0;
            fd_period_avg_valid_r <= 1'b0;

            fi_raw_valid_r  <= 1'b0;
            fi_proc_valid_r <= 1'b0;
            fbar_s1_valid_r <= 1'b0;
            fbar_s2_valid_r <= 1'b0;
            fbar_s3_valid_r <= 1'b0;
            fbar_s4_valid_r <= 1'b0;
            stat_s1_valid_r <= 1'b0;
            stat_s2_valid_r <= 1'b0;
            stat_s3_valid_r <= 1'b0;
            stat_s4_valid_r <= 1'b0;
            stat_s5_valid_r <= 1'b0;
            res_s1_valid_r  <= 1'b0;
            res_s2_valid_r  <= 1'b0;
            res_s3_valid_r  <= 1'b0;

            if (cmp_valid_i)
                cmp_d_r <= cmp_i;

            if (!enable_i) begin
                edge_count_r         <= 2'd0;
                ts_z1_r              <= timestamp_r;
                ts_z2_r              <= timestamp_r;

                fi_raw_valid_r       <= 1'b0;
                fi_proc_valid_r      <= 1'b0;
                fbar_hz_r            <= IF_INIT_HZ;
                fbar_s1_valid_r      <= 1'b0;
                fbar_s2_valid_r      <= 1'b0;
                fbar_s3_valid_r      <= 1'b0;
                fbar_s4_valid_r      <= 1'b0;
                fbar_clamped_s4_r    <= IF_INIT_HZ;

                fd_state_high_r      <= 1'b0;
                fd_last_zc_ts_r      <= timestamp_r;
                fm_khz_est_r         <= 4'd5;
                fm_code_est_r        <= 4'd0;
                fd_period_s1_valid_r <= 1'b0;
                fd_period_s1_ticks_r <= {PERIOD_W{1'b0}};
                fd_period_sum_r      <= {PERIOD_SUM_W{1'b0}};
                fd_period_avg_cnt_r  <= 2'd0;
                fd_period_avg_valid_r <= 1'b0;
                fd_period_avg_ticks_r <= {PERIOD_W{1'b0}};

                top0_r               <= FI_TOP_INIT;
                top1_r               <= FI_TOP_INIT;
                top2_r               <= FI_TOP_INIT;
                top3_r               <= FI_TOP_INIT;
                bot0_r               <= FI_BOT_INIT;
                bot1_r               <= FI_BOT_INIT;
                bot2_r               <= FI_BOT_INIT;
                bot3_r               <= FI_BOT_INIT;
                win_cnt_r            <= {WIN_CNT_W{1'b0}};

                stat_s1_valid_r      <= 1'b0;
                stat_s2_valid_r      <= 1'b0;
                stat_s3_valid_r      <= 1'b0;
                stat_s4_valid_r      <= 1'b0;
                stat_s5_valid_r      <= 1'b0;
                res_s1_valid_r       <= 1'b0;
                res_s2_valid_r       <= 1'b0;
                res_s3_valid_r       <= 1'b0;
            end else begin
                if (rising_edge_w) begin
                    if (edge_count_r < 2'd2) begin
                        edge_count_r <= edge_count_r + 2'd1;
                    end else if (!div_busy_w && delta_range_ok_w) begin
                        delta_ticks_r <= delta_ticks_full_w[DELTA_W-1:0];
                        div_start_r   <= 1'b1;
                    end

                    ts_z2_r <= ts_z1_r;
                    ts_z1_r <= timestamp_r;
                end

                if (div_done_w) begin
                    fi_raw_hz_r    <= fi_hz_w;
                    fi_raw_valid_r <= 1'b1;
                end

                if (fi_raw_valid_r) begin
                    fi_proc_hz_r    <= fi_raw_hz_r;
                    fi_proc_valid_r <= fi_raw_in_range_w;
                end

                if (fi_proc_valid_r) begin
                    fbar_err_s1_r   <= fbar_err_now_w;
                    fbar_base_s1_r  <= fbar_hz_r;
                    fbar_s1_valid_r <= 1'b1;

                    if (win_cnt_r == WIN_SAMPLES - 1) begin
                        stat_s1_valid_r <= 1'b1;
                        // Use the middle high/low pair instead of the 3rd/4th extremes.
                        // The old top2/top3 and bot2/bot3 selection was conservative,
                        // but it under-estimated df when mf was extended to 7.
                        stat_top2_s1_r  <= top1_next_w;
                        stat_top3_s1_r  <= top2_next_w;
                        stat_bot2_s1_r  <= bot1_next_w;
                        stat_bot3_s1_r  <= bot2_next_w;
                        stat_fm_khz_s1_r  <= fm_khz_est_r;
                        stat_fm_code_s1_r <= fm_code_est_r;

                        win_cnt_r       <= {WIN_CNT_W{1'b0}};
                        top0_r          <= FI_TOP_INIT;
                        top1_r          <= FI_TOP_INIT;
                        top2_r          <= FI_TOP_INIT;
                        top3_r          <= FI_TOP_INIT;
                        bot0_r          <= FI_BOT_INIT;
                        bot1_r          <= FI_BOT_INIT;
                        bot2_r          <= FI_BOT_INIT;
                        bot3_r          <= FI_BOT_INIT;
                    end else begin
                        win_cnt_r <= win_cnt_r + 1'b1;
                        top0_r    <= top0_next_w;
                        top1_r    <= top1_next_w;
                        top2_r    <= top2_next_w;
                        top3_r    <= top3_next_w;
                        bot0_r    <= bot0_next_w;
                        bot1_r    <= bot1_next_w;
                        bot2_r    <= bot2_next_w;
                        bot3_r    <= bot3_next_w;
                    end
                end

                if (fbar_s1_valid_r) begin
                    fd_sample_r     <= fbar_err_s1_r[FD_W-1:0];
                    fd_valid_r      <= 1'b1;
                    fbar_step_s2_r  <= fbar_err_s1_r >>> FAVG_SHIFT;
                    fbar_base_s2_r  <= fbar_base_s1_r;
                    fbar_s2_valid_r <= 1'b1;
                end

                if (fbar_s2_valid_r) begin
                    fbar_candidate_s3_r <= fbar_candidate_w;
                    fbar_s3_valid_r     <= 1'b1;
                end

                if (fbar_s3_valid_r) begin
                    if ((fbar_candidate_s3_r < 0) || (fbar_candidate_s3_r > FBAR_MAX_HZ_S))
                        fbar_clamped_s4_r <= IF_INIT_HZ;
                    else
                        fbar_clamped_s4_r <= fbar_candidate_s3_r[FI_W-1:0];
                    fbar_s4_valid_r <= 1'b1;
                end

                if (fbar_s4_valid_r) begin
                    fbar_hz_r <= fbar_clamped_s4_r;
                end

                if (stat_s1_valid_r) begin
                    stat_top_sum_s2_r <= {1'b0, stat_top2_s1_r} + {1'b0, stat_top3_s1_r};
                    stat_bot_sum_s2_r <= {1'b0, stat_bot2_s1_r} + {1'b0, stat_bot3_s1_r};
                    stat_fm_khz_s2_r  <= stat_fm_khz_s1_r;
                    stat_fm_code_s2_r <= stat_fm_code_s1_r;
                    stat_s2_valid_r   <= 1'b1;
                end

                if (stat_s2_valid_r) begin
                    stat_span_raw_s3_r <= stat_span_raw_w;
                    stat_span_pos_s3_r <= stat_span_pos_w;
                    stat_fm_khz_s3_r  <= stat_fm_khz_s2_r;
                    stat_fm_code_s3_r <= stat_fm_code_s2_r;
                    stat_s3_valid_r    <= 1'b1;
                end

                if (stat_s3_valid_r) begin
                    stat_span_s4_r  <= stat_span_clamped_w;
                    stat_fm_khz_s4_r  <= stat_fm_khz_s3_r;
                    stat_fm_code_s4_r <= stat_fm_code_s3_r;
                    stat_s4_valid_r <= 1'b1;
                end

                if (stat_s4_valid_r) begin
                    stat_df_s5_r    <= stat_df_sat_w;
                    stat_fm_khz_s5_r  <= stat_fm_khz_s4_r;
                    stat_fm_code_s5_r <= stat_fm_code_s4_r;
                    stat_s5_valid_r <= 1'b1;
                end

                if (stat_s5_valid_r) begin
                    res_fm_khz_s1_r  <= stat_fm_khz_s5_r;
                    res_fm_code_s1_r <= stat_fm_code_s5_r;
                    res_df_s1_r      <= stat_df_s5_r;
                    res_s1_valid_r   <= 1'b1;
                end

                if (res_s1_valid_r) begin
                    res_fm_khz_s2_r  <= res_fm_khz_s1_r;
                    res_fm_code_s2_r <= res_fm_code_s1_r;
                    res_df_s2_r      <= res_df_s1_r;
                    res_mf_s2_r      <= round_mf_from_df(res_df_s1_r, res_fm_khz_s1_r, mf_int_o);
                    res_s2_valid_r   <= 1'b1;
                end

                if (res_s2_valid_r) begin
                    res_fm_khz_s3_r  <= res_fm_khz_s2_r;
                    res_fm_code_s3_r <= res_fm_code_s2_r;
                    res_df_s3_r      <= res_df_s2_r;
                    res_mf_s3_r      <= res_mf_s2_r;
                    res_df_khz_s3_r  <= df_khz_from_mf_fm(res_mf_s2_r, res_fm_khz_s2_r);
                    res_s3_valid_r   <= 1'b1;
                end

                if (res_s3_valid_r) begin
                    fm_khz_o       <= res_fm_code_s3_r;
                    mf_int_o       <= res_mf_s3_r;
                    df_khz_o       <= res_df_khz_s3_r;
                    df_hz_o        <= res_df_s3_r;
                    df_metric_hz_o <= res_df_s3_r;
                    fm_valid_o     <= 1'b1;
                end

                // Stage P0: detect fd zero-crossing rise and register only the period.
                // This removes the timestamp subtraction from the averaging / quantization path.
                if (fd_valid_r) begin
                    if (fd_high_w)
                        fd_state_high_r <= 1'b1;
                    else if (fd_low_w)
                        fd_state_high_r <= 1'b0;

                    if (fd_rise_w) begin
                        if (fd_period_range_ok_w) begin
                            fd_period_s1_ticks_r <= fd_period_ticks_w;
                            fd_period_s1_valid_r <= 1'b1;
                        end
                        fd_last_zc_ts_r <= timestamp_r;
                    end
                end

                // Stage P1: accumulate four registered period measurements.
                if (fd_period_s1_valid_r) begin
                    if (fd_period_avg_ready_w) begin
                        fd_period_avg_ticks_r <= fd_period_avg_next_w;
                        fd_period_avg_valid_r <= 1'b1;
                        fd_period_sum_r       <= {PERIOD_SUM_W{1'b0}};
                        fd_period_avg_cnt_r   <= 2'd0;
                    end else begin
                        fd_period_sum_r       <= fd_period_sum_next_w;
                        fd_period_avg_cnt_r   <= fd_period_avg_cnt_r + 2'd1;
                    end
                end

                // Stage P2: collapse averaged period to the valid frequency code.
                if (fd_period_avg_valid_r) begin
                    fm_khz_est_r  <= round_fm_khz_from_period(fd_period_avg_ticks_r);
                    fm_code_est_r <= encode_fm_from_period(fd_period_avg_ticks_r);
                end
            end
        end
    end

endmodule