`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// AM / FM / CW mode classifier.
//
// Modified points:
//   1. The old classifier waited until BOTH AM and FM measurement paths had fresh
//      results.  For high-depth AM, the carrier nearly disappears at the envelope
//      valley, so the FM zero-crossing path can temporarily fail to produce a
//      valid window.  The result was MODE_SEARCH: all three LEDs off and display 0.
//   2. Strong AM evidence is now allowed to confirm AM without waiting for a fresh
//      FM metric.  Low-depth AM still waits for both paths, so FM with small false
//      envelope ripple is not stolen by AM.
//   3. FM threshold and confirmation are made slightly more conservative to reduce
//      occasional CW -> FM flicker caused by isolated zero-crossing outliers.
// -----------------------------------------------------------------------------

module q3_mode_classifier #(
    parameter [11:0] AM_TH_PERMILLE          = 12'd120,
    parameter [11:0] AM_STRONG_TH_PERMILLE   = 12'd280,
    parameter [11:0] AM_FORCE_TH_PERMILLE    = 12'd650,
    parameter [16:0] FM_TH_HZ                = 17'd700,
    parameter [16:0] FM_STRONG_TH_HZ         = 17'd3000,
    parameter [1:0]  CONFIRM_N               = 2'd3
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable_i,
    input  wire        lock_i,

    input  wire [11:0] am_ma_permille_i,
    input  wire [11:0] am_metric_i,
    input  wire [3:0]  am_fm_khz_i,
    input  wire        am_valid_i,

    input  wire [3:0]  fm_fm_khz_i,
    input  wire [3:0]  fm_mf_int_i,
    input  wire [6:0]  fm_df_khz_i,
    input  wire [16:0] fm_df_hz_i,
    input  wire [16:0] fm_metric_hz_i,
    input  wire        fm_valid_i,

    output reg  [1:0]  mode_o,          // 0:CW, 1:AM, 2:FM, 3:search/invalid
    output reg  [11:0] ma_permille_o,
    output reg  [3:0]  fm_khz_o,
    output reg  [3:0]  mf_int_o,
    output reg  [6:0]  df_khz_o,
    output reg         result_valid_o
);

    localparam [1:0] MODE_CW     = 2'd0;
    localparam [1:0] MODE_AM     = 2'd1;
    localparam [1:0] MODE_FM     = 2'd2;
    localparam [1:0] MODE_SEARCH = 2'd3;

    reg [11:0] am_ma_r;
    reg [11:0] am_metric_r;
    reg [3:0]  am_fm_r;
    reg        am_fresh_r;

    reg [3:0]  fm_fm_r;
    reg [3:0]  fm_mf_r;
    reg [6:0]  fm_df_khz_r;
    reg [16:0] fm_df_hz_r;
    reg [16:0] fm_metric_r;
    reg        fm_fresh_r;

    reg [1:0] last_cand_r;
    reg [1:0] confirm_cnt_r;

    wire [11:0] am_ma_next_w     = am_valid_i ? am_ma_permille_i : am_ma_r;
    wire [11:0] am_metric_next_w = am_valid_i ? am_metric_i      : am_metric_r;
    wire [3:0]  am_fm_next_w     = am_valid_i ? am_fm_khz_i      : am_fm_r;

    wire [3:0]  fm_fm_next_w     = fm_valid_i ? fm_fm_khz_i      : fm_fm_r;
    wire [3:0]  fm_mf_next_w     = fm_valid_i ? fm_mf_int_i      : fm_mf_r;
    wire [6:0]  fm_df_khz_next_w = fm_valid_i ? fm_df_khz_i      : fm_df_khz_r;
    wire [16:0] fm_df_hz_next_w  = fm_valid_i ? fm_df_hz_i       : fm_df_hz_r;
    wire [16:0] fm_metric_next_w = fm_valid_i ? fm_metric_hz_i   : fm_metric_r;

    wire am_fresh_next_w = am_fresh_r | am_valid_i;
    wire fm_fresh_next_w = fm_fresh_r | fm_valid_i;
    wire metric_event_w  = am_valid_i | fm_valid_i;

    wire force_am_w =
        am_fresh_next_w && (am_metric_next_w >= AM_FORCE_TH_PERMILLE);

    wire pair_ready_w =
        am_fresh_next_w && fm_fresh_next_w;

    wire decision_ready_w =
        metric_event_w && (force_am_w || pair_ready_w);

    function [1:0] choose_mode;
        input [11:0] am_metric;
        input [16:0] fm_metric;
        begin
            if (am_metric >= AM_FORCE_TH_PERMILLE) begin
                choose_mode = MODE_AM;
            end else if ((fm_metric >= FM_STRONG_TH_HZ) &&
                         (am_metric < AM_STRONG_TH_PERMILLE)) begin
                choose_mode = MODE_FM;
            end else if (am_metric >= AM_TH_PERMILLE) begin
                choose_mode = MODE_AM;
            end else if (fm_metric >= FM_TH_HZ) begin
                choose_mode = MODE_FM;
            end else begin
                choose_mode = MODE_CW;
            end
        end
    endfunction

    wire [1:0] cand_mode_w = choose_mode(am_metric_next_w, fm_metric_next_w);

    always @(posedge clk) begin
        if (!rst_n) begin
            am_ma_r        <= 12'd0;
            am_metric_r    <= 12'd0;
            am_fm_r        <= 4'd0;
            am_fresh_r     <= 1'b0;

            fm_fm_r        <= 4'd0;
            fm_mf_r        <= 4'd0;
            fm_df_khz_r    <= 7'd0;
            fm_df_hz_r     <= 17'd0;
            fm_metric_r    <= 17'd0;
            fm_fresh_r     <= 1'b0;

            last_cand_r    <= MODE_SEARCH;
            confirm_cnt_r  <= 2'd0;

            mode_o         <= MODE_SEARCH;
            ma_permille_o  <= 12'd0;
            fm_khz_o       <= 4'd0;
            mf_int_o       <= 4'd0;
            df_khz_o       <= 7'd0;
            result_valid_o <= 1'b0;
        end else begin
            result_valid_o <= 1'b0;

            if (!enable_i || !lock_i) begin
                am_ma_r        <= 12'd0;
                am_metric_r    <= 12'd0;
                am_fm_r        <= 4'd0;
                am_fresh_r     <= 1'b0;

                fm_fm_r        <= 4'd0;
                fm_mf_r        <= 4'd0;
                fm_df_khz_r    <= 7'd0;
                fm_df_hz_r     <= 17'd0;
                fm_metric_r    <= 17'd0;
                fm_fresh_r     <= 1'b0;

                last_cand_r    <= MODE_SEARCH;
                confirm_cnt_r  <= 2'd0;

                mode_o         <= MODE_SEARCH;
                ma_permille_o  <= 12'd0;
                fm_khz_o       <= 4'd0;
                mf_int_o       <= 4'd0;
                df_khz_o       <= 7'd0;
            end else begin
                if (am_valid_i) begin
                    am_ma_r     <= am_ma_permille_i;
                    am_metric_r <= am_metric_i;
                    am_fm_r     <= am_fm_khz_i;
                    am_fresh_r  <= 1'b1;
                end

                if (fm_valid_i) begin
                    fm_fm_r     <= fm_fm_khz_i;
                    fm_mf_r     <= fm_mf_int_i;
                    fm_df_khz_r <= fm_df_khz_i;
                    fm_df_hz_r  <= fm_df_hz_i;
                    fm_metric_r <= fm_metric_hz_i;
                    fm_fresh_r  <= 1'b1;
                end

                if (decision_ready_w) begin
                    if (cand_mode_w == last_cand_r) begin
                        if (confirm_cnt_r >= CONFIRM_N - 1) begin
                            mode_o <= cand_mode_w;

                            case (cand_mode_w)
                                MODE_AM: begin
                                    ma_permille_o <= am_ma_next_w;
                                    fm_khz_o      <= am_fm_next_w;
                                    mf_int_o      <= 4'd0;
                                    df_khz_o      <= 7'd0;
                                end

                                MODE_FM: begin
                                    ma_permille_o <= 12'd0;
                                    fm_khz_o      <= fm_fm_next_w;
                                    mf_int_o      <= fm_mf_next_w;
                                    df_khz_o      <= fm_df_khz_next_w;
                                end

                                default: begin
                                    ma_permille_o <= 12'd0;
                                    fm_khz_o      <= 4'd0;
                                    mf_int_o      <= 4'd0;
                                    df_khz_o      <= 7'd0;
                                end
                            endcase

                            result_valid_o <= 1'b1;
                        end else begin
                            confirm_cnt_r <= confirm_cnt_r + 2'd1;
                        end
                    end else begin
                        last_cand_r   <= cand_mode_w;
                        confirm_cnt_r <= 2'd1;
                    end
                end
            end
        end
    end

endmodule