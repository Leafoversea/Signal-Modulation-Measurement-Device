`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// 200-kHz IF lock detector for question 3.
//
// Fixes for high-depth AM:
//   1. The old detector used the sign bit of ac_w directly as the zero-crossing
//      comparator.  When AM depth is high, the carrier envelope becomes very
//      small near the modulation valley and the raw comparator chatters.
//   2. This version uses a Schmitt comparator and an edge debounce counter.
//      The Schmitt comparator removes small zero-level noise, while the debounce
//      counter prevents one real zero crossing from being counted many times.
//   3. Interface is compatible with the original top module.
// -----------------------------------------------------------------------------

module q3_if_lock_detector #(
    parameter integer WINDOW_SAMPLES        = 100000,
    parameter [31:0]  AMP_SUM_MIN           = 32'd2000000,
    parameter [10:0]  EDGE_MIN              = 11'd120,
    parameter [10:0]  EDGE_MAX              = 11'd280,
    parameter [16:0]  CMP_HYST              = 17'd128,
    parameter integer EDGE_DEBOUNCE_SAMPLES = 64
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start_i,
    input  wire signed [15:0] sample_i,
    input  wire               sample_valid_i,

    output reg                busy_o,
    output reg                done_o,
    output reg                lock_ok_o,
    output reg  [23:0]        center_hz_o,
    output reg  [10:0]        edge_count_o,
    output reg  [31:0]        amp_sum_o
);

    localparam integer CNT_W      = 17;
    localparam integer DC_SHIFT   = 14;
    localparam integer DC_W       = 34;
    localparam integer EDGE_GAP_W = 8;

    // Must be 32-bit. 24'd100000000 will overflow/truncate.
    // With WINDOW_SAMPLES=50000, CENTER_MUL=2000.
    localparam [31:0] CENTER_MUL = 32'd100000000 / WINDOW_SAMPLES;

    localparam [EDGE_GAP_W-1:0] EDGE_DEBOUNCE_VALUE = EDGE_DEBOUNCE_SAMPLES;

    reg [CNT_W-1:0] sample_cnt_r;
    reg [10:0]      edge_cnt_r;
    reg [31:0]      amp_sum_r;

    reg signed [DC_W-1:0] dc_acc_r;

    wire signed [DC_W-1:0] sample_q_w =
        {{(DC_W-16){sample_i[15]}}, sample_i} <<< DC_SHIFT;

    wire signed [DC_W-1:0] dc_err_w  = sample_q_w - dc_acc_r;
    wire signed [DC_W-1:0] dc_step_w = dc_err_w >>> DC_SHIFT;
    wire signed [DC_W-1:0] ac_q_w    = sample_q_w - dc_acc_r;
    wire signed [16:0]     ac_w      = ac_q_w >>> DC_SHIFT;

    wire [16:0] abs_w = ac_w[16] ? (~ac_w + 17'd1) : ac_w;

    wire signed [16:0] cmp_pos_th_w = $signed(CMP_HYST);
    wire signed [16:0] cmp_neg_th_w = -$signed(CMP_HYST);

    reg        cmp_schmitt_r;
    reg        abs_pipe_valid_r;
    reg [16:0] abs_pipe_r;
    reg        cmp_pipe_r;
    reg        cmp_pipe_d_r;
    reg [EDGE_GAP_W-1:0] edge_gap_cnt_r;

    wire raw_rising_pipe_w = abs_pipe_valid_r && cmp_pipe_r && (~cmp_pipe_d_r);
    wire edge_gap_clear_w  = (edge_gap_cnt_r == {EDGE_GAP_W{1'b0}});
    wire rising_pipe_w     = raw_rising_pipe_w && edge_gap_clear_w;

    // edge_cnt_r * CENTER_MUL gives measured center frequency.
    wire [31:0] center_hz_w32 = {21'd0, edge_cnt_r} * CENTER_MUL;
    wire [23:0] center_hz_w   = center_hz_w32[23:0];

    wire window_last_w = (sample_cnt_r == WINDOW_SAMPLES - 1);
    wire amp_ok_w      = (amp_sum_r >= AMP_SUM_MIN);
    wire edge_ok_w     = (edge_cnt_r >= EDGE_MIN) && (edge_cnt_r <= EDGE_MAX);

    // Question (3): target IF is 200 kHz.
    // FM max deviation may make instantaneous IF about 140~260 kHz, but the
    // average/center frequency should still be close to 200 kHz.  Keep this
    // range narrow enough to reject false IF/subharmonic edge counts.
    wire center_ok_w = (center_hz_w32 >= 32'd100000) &&
                       (center_hz_w32 <= 32'd400000);

    always @(posedge clk) begin
        if (!rst_n) begin
            busy_o           <= 1'b0;
            done_o           <= 1'b0;
            lock_ok_o        <= 1'b0;
            center_hz_o      <= 24'd0;
            edge_count_o     <= 11'd0;
            amp_sum_o        <= 32'd0;

            sample_cnt_r     <= {CNT_W{1'b0}};
            edge_cnt_r       <= 11'd0;
            amp_sum_r        <= 32'd0;
            dc_acc_r         <= {DC_W{1'b0}};

            cmp_schmitt_r    <= 1'b0;
            abs_pipe_valid_r <= 1'b0;
            abs_pipe_r       <= 17'd0;
            cmp_pipe_r       <= 1'b0;
            cmp_pipe_d_r     <= 1'b0;
            edge_gap_cnt_r   <= {EDGE_GAP_W{1'b0}};
        end else begin
            done_o <= 1'b0;

            if (sample_valid_i) begin
                dc_acc_r <= dc_acc_r + dc_step_w;

                if (ac_w > cmp_pos_th_w)
                    cmp_schmitt_r <= 1'b1;
                else if (ac_w < cmp_neg_th_w)
                    cmp_schmitt_r <= 1'b0;

                abs_pipe_r       <= abs_w;
                cmp_pipe_r       <= cmp_schmitt_r;
                abs_pipe_valid_r <= 1'b1;
            end else begin
                abs_pipe_valid_r <= 1'b0;
            end

            if (abs_pipe_valid_r) begin
                cmp_pipe_d_r <= cmp_pipe_r;

                if (edge_gap_cnt_r != {EDGE_GAP_W{1'b0}})
                    edge_gap_cnt_r <= edge_gap_cnt_r - {{(EDGE_GAP_W-1){1'b0}}, 1'b1};
            end

            if (start_i) begin
                busy_o         <= 1'b1;
                lock_ok_o      <= 1'b0;
                sample_cnt_r   <= {CNT_W{1'b0}};
                edge_cnt_r     <= 11'd0;
                amp_sum_r      <= 32'd0;
                amp_sum_o      <= 32'd0;
                edge_count_o   <= 11'd0;
                center_hz_o    <= 24'd0;
                edge_gap_cnt_r <= {EDGE_GAP_W{1'b0}};
            end else if (busy_o && abs_pipe_valid_r) begin
                amp_sum_r <= amp_sum_r + {15'd0, abs_pipe_r};

                if (rising_pipe_w) begin
                    if (edge_cnt_r != 11'h7FF)
                        edge_cnt_r <= edge_cnt_r + 11'd1;
                    edge_gap_cnt_r <= EDGE_DEBOUNCE_VALUE;
                end

                if (window_last_w) begin
                    busy_o       <= 1'b0;
                    done_o       <= 1'b1;
                    lock_ok_o    <= amp_ok_w && edge_ok_w && center_ok_w;
                    center_hz_o  <= center_hz_w;
                    edge_count_o <= edge_cnt_r;
                    amp_sum_o    <= amp_sum_r;
                    sample_cnt_r <= {CNT_W{1'b0}};
                end else begin
                    sample_cnt_r <= sample_cnt_r + {{(CNT_W-1){1'b0}}, 1'b1};
                end
            end
        end
    end

endmodule