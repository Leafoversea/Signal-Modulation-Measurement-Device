`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// Question-3 top for an EXTERNAL DDS scan controller.
//
// The FPGA does NOT generate RF-DDS FTW/scan words.
// The external DDS control board steps the LO frequency by itself.
// The FPGA only observes the mixed IF signal. When the IF is judged to be
// 200 kHz, dds_stop_o is asserted high so that the external DDS control board
// can stop frequency stepping.
//
// This simplified version exports one 4-bit direct modulation-frequency code:
//
//   demod_freq_code_o = 4'd0  : invalid / CW / no-lock
//   demod_freq_code_o = 4'd1  : 1 kHz
//   demod_freq_code_o = 4'd2  : 2 kHz
//   ...
//   demod_freq_code_o = 4'd10 : 10 kHz
//
// demod_enable_o and demod_update_o are intentionally not exported in this
// simplified version. The 4-bit code is held by a register and can be directly
// used for binary display or continuously sampled by an external controller.
// -----------------------------------------------------------------------------

module q3_signal_mod_measure_ext_dds_top (
    input  wire        clk,
    input  wire        rst_n,

    // Optional one-key restart, active low.
    // If you need it later, restore this port and the synchronizer assignment.
    // input  wire        start_key_n,

    // ADC interface.
    output wire        adc_clk,
    input  wire        adc_dco,
    input  wire [15:0] adc_data_i,
    output wire        adc_pwdn_o,
    input  wire        OF,

    // External DDS scan-control interface.
    output wire        dds_stop_o,
    output wire        stop_led,
    // Four-bit direct modulation-frequency code output: 1~10 means 1~10 kHz.
    output reg  [3:0]  demod_freq_code_o,

    // 74HC595 display interface.
    output wire        sh_cp,
    output wire        st_cp,
    output wire        ds,

    // Mode LEDs, active high.
    output wire        led_cw,
    output wire        led_am,
    output wire        led_fm,
    output wire        led_of
);

    assign adc_pwdn_o = 1'b0;
    assign led_of     = OF;

    // ------------------------------------------------------------------
    // Clock generation and ODDR clock output.
    // ------------------------------------------------------------------
    wire clk_adc_100m;
    wire clk_calc_200m;
    wire pll_locked;
    wire sys_rst_n = rst_n & pll_locked;

    clk_wiz_0 u_clk_wiz (
        .clk_100 (clk_adc_100m),
        .clk_200 (clk_calc_200m),
        .locked  (pll_locked),
        .clk_in1 (clk)
    );

    ODDR #(
        .DDR_CLK_EDGE("SAME_EDGE")
    ) u_adc_clk_oddr (
        .C  (clk_adc_100m),
        .CE (1'b1),
        .D1 (1'b1),
        .D2 (1'b0),
        .Q  (adc_clk),
        .R  (1'b0),
        .S  (1'b0)
    );

    wire rst_calc_n;

    reset_sync_low u_rst_calc (
        .clk       (clk_calc_200m),
        .rst_n_in  (sys_rst_n),
        .rst_n_out (rst_calc_n)
    );

    // ------------------------------------------------------------------
    // ADC-DCO negedge-domain reset.
    // ------------------------------------------------------------------
    reg [2:0] rst_adc_neg_sr;

    always @(negedge adc_dco or negedge sys_rst_n) begin
        if (!sys_rst_n)
            rst_adc_neg_sr <= 3'b000;
        else
            rst_adc_neg_sr <= {rst_adc_neg_sr[1:0], 1'b1};
    end

    wire rst_adc_neg_n = rst_adc_neg_sr[2];

    // ------------------------------------------------------------------
    // Start-key synchronizer.
    // Current simplified version ties start_key_n to 1'b1 internally.
    // If you restore start_key_n as a real port, replace 1'b1 below with
    // start_key_n.
    // ------------------------------------------------------------------
    reg [2:0] start_sync_r;

    always @(posedge clk_calc_200m) begin
        if (!rst_calc_n)
            start_sync_r <= 3'b111;
        else
            start_sync_r <= {start_sync_r[1:0], 1'b1};
            // start_sync_r <= {start_sync_r[1:0], start_key_n};
    end

    wire start_pulse_w = start_sync_r[2] & (~start_sync_r[1]);

    // ------------------------------------------------------------------
    // ADC sample CDC: adc_dco negedge domain -> 200-MHz processing domain.
    // ------------------------------------------------------------------
    wire signed [15:0] adc_sample_200m;
    wire               adc_sample_valid_200m;
    wire               adc_fifo_full_dbg;

    am_adc16_sample_async_fifo u_adc_cdc_fifo (
        .src_clk       (adc_dco),
        .src_rst_n     (rst_adc_neg_n),
        .src_data_i    (adc_data_i),
        .src_full_o    (adc_fifo_full_dbg),
        .dst_clk       (clk_calc_200m),
        .dst_rst_n     (rst_calc_n),
        .dst_data_o    (adc_sample_200m),
        .dst_valid_o   (adc_sample_valid_200m)
    );

    // ------------------------------------------------------------------
    // Continuous 200-kHz IF lock detector.
    // ------------------------------------------------------------------
    localparam [2:0] LOCK_CONFIRM_N   = 3'd4;
    localparam [3:0] UNLOCK_CONFIRM_N = 4'd8;

    reg        lock_start_r;
    reg        if_locked_r;
    reg [1:0]  lock_good_cnt_r;
    reg [3:0]  lock_bad_cnt_r;
    reg        lock_window_ok_r;
    reg        clear_result_r;
    reg        final_stop_r;

    wire        lock_busy_w;
    wire        lock_done_w;
    wire        lock_ok_w;
    wire [23:0] lock_center_hz_w;
    wire [10:0] lock_edge_count_w;
    wire [31:0] lock_amp_sum_w;

    q3_if_lock_detector #(
        .WINDOW_SAMPLES        (50000),
        .AMP_SUM_MIN           (32'd1000000),
        .EDGE_MIN              (11'd50),
        .EDGE_MAX              (11'd200),
        .CMP_HYST              (17'd1024),
        .EDGE_DEBOUNCE_SAMPLES (64)
    ) u_if_lock_detector (
        .clk             (clk_calc_200m),
        .rst_n           (rst_calc_n),
        .start_i         (lock_start_r),
        .sample_i        (adc_sample_200m),
        .sample_valid_i  (adc_sample_valid_200m),
        .busy_o          (lock_busy_w),
        .done_o          (lock_done_w),
        .lock_ok_o       (lock_ok_w),
        .center_hz_o     (lock_center_hz_w),
        .edge_count_o    (lock_edge_count_w),
        .amp_sum_o       (lock_amp_sum_w)
    );

    // dds_stop_o/stop_led are no longer driven by raw IF lock here.
    // They are driven by final_stop_r after AM/FM/CW mode confirmation.
    always @(posedge clk_calc_200m) begin
        if (!rst_calc_n) begin
            lock_start_r     <= 1'b0;
            if_locked_r      <= 1'b0;
            lock_good_cnt_r  <= 2'd0;
            lock_bad_cnt_r   <= 4'd0;
            lock_window_ok_r <= 1'b0;
            clear_result_r   <= 1'b1;
        end else begin
            lock_start_r   <= 1'b0;
            clear_result_r <= 1'b0;

            if (start_pulse_w) begin
                if_locked_r      <= 1'b0;
                lock_good_cnt_r  <= 2'd0;
                lock_bad_cnt_r   <= 4'd0;
                lock_window_ok_r <= 1'b0;
                clear_result_r   <= 1'b1;
            end else begin
                // Start a new IF-check window whenever the detector is idle.
                if (!lock_busy_w && !lock_done_w)
                    lock_start_r <= 1'b1;

                if (lock_done_w) begin
                    lock_window_ok_r <= lock_ok_w;

                    if (lock_ok_w) begin
                        lock_bad_cnt_r <= 4'd0;

                        if (lock_good_cnt_r >= LOCK_CONFIRM_N - 1) begin
                            if_locked_r <= 1'b1;
                        end else begin
                            lock_good_cnt_r <= lock_good_cnt_r + 2'd1;
                        end
                    end else begin
                        lock_good_cnt_r <= 2'd0;

                        // Dynamic unlock:
                        // Do not drop lock on one bad window. Release only
                        // after several consecutive bad windows.
                        if (lock_bad_cnt_r >= UNLOCK_CONFIRM_N - 1) begin
                            lock_bad_cnt_r <= 4'd0;
                            if_locked_r    <= 1'b0;
                            clear_result_r <= 1'b1;
                        end else begin
                            lock_bad_cnt_r <= lock_bad_cnt_r + 4'd1;
                        end
                    end
                end
            end
        end
    end

    wire measure_enable_w = if_locked_r;

    // ------------------------------------------------------------------
    // AM measurement path.
    // ------------------------------------------------------------------
    wire [11:0] am_ma_permille_w;
    wire [3:0]  am_fm_khz_w;
    wire        am_valid_w;
    wire [11:0] am_metric_w;

    q3_am_measure_5k10k u_am_measure (
        .clk             (clk_calc_200m),
        .rst_n           (rst_calc_n),
        .enable_i        (measure_enable_w),
        .sample_i        (adc_sample_200m),
        .sample_valid_i  (adc_sample_valid_200m),
        .ma_permille_o   (am_ma_permille_w),
        .fm_khz_o        (am_fm_khz_w),
        .am_valid_o      (am_valid_w),
        .ma_metric_o     (am_metric_w)
    );

    // ------------------------------------------------------------------
    // FM measurement path.
    // ------------------------------------------------------------------
    wire cmp_level_200m_w = ~adc_sample_200m[15];

    wire [3:0]  fm_fm_khz_w;
    wire [3:0]  fm_mf_int_w;
    wire [6:0]  fm_df_khz_w;
    wire [16:0] fm_df_hz_w;
    wire        fm_valid_w;
    wire [16:0] fm_metric_hz_w;

    q3_fm_measure_m2_5k10k u_fm_measure (
        .clk             (clk_calc_200m),
        .rst_n           (rst_calc_n),
        .enable_i        (measure_enable_w),
        .cmp_i           (cmp_level_200m_w),
        .cmp_valid_i     (adc_sample_valid_200m),
        .fm_khz_o        (fm_fm_khz_w),
        .mf_int_o        (fm_mf_int_w),
        .df_khz_o        (fm_df_khz_w),
        .df_hz_o         (fm_df_hz_w),
        .fm_valid_o      (fm_valid_w),
        .df_metric_hz_o  (fm_metric_hz_w)
    );

    // ------------------------------------------------------------------
    // Automatic AM/FM/CW mode classification.
    // ------------------------------------------------------------------
    wire [1:0]  mode_w;
    wire [11:0] mode_ma_permille_w;
    wire [3:0]  mode_fm_khz_w;
    wire [3:0]  mode_mf_int_w;
    wire [6:0]  mode_df_khz_w;
    wire        mode_result_valid_w;

    q3_mode_classifier u_mode_classifier (
        .clk              (clk_calc_200m),
        .rst_n            (rst_calc_n),
        .enable_i         (measure_enable_w),
        .lock_i           (if_locked_r),
        .am_ma_permille_i (am_ma_permille_w),
        .am_metric_i      (am_metric_w),
        .am_fm_khz_i      (am_fm_khz_w),
        .am_valid_i       (am_valid_w),
        .fm_fm_khz_i      (fm_fm_khz_w),
        .fm_mf_int_i      (fm_mf_int_w),
        .fm_df_khz_i      (fm_df_khz_w),
        .fm_df_hz_i       (fm_df_hz_w),
        .fm_metric_hz_i   (fm_metric_hz_w),
        .fm_valid_i       (fm_valid_w),
        .mode_o           (mode_w),
        .ma_permille_o    (mode_ma_permille_w),
        .fm_khz_o         (mode_fm_khz_w),
        .mf_int_o         (mode_mf_int_w),
        .df_khz_o         (mode_df_khz_w),
        .result_valid_o   (mode_result_valid_w)
    );

    // ------------------------------------------------------------------
    // Final stop/lock qualification.
    //
    // if_locked_r is only a raw IF candidate lock.  It enables AM/FM/CW
    // measurement internally.  The external stop signal and stop LED are
    // asserted only after the mode classifier confirms CW/AM/FM.  This avoids
    // stopping the external DDS at a false IF point where all three mode LEDs
    // would otherwise remain off.
    // ------------------------------------------------------------------
    always @(posedge clk_calc_200m) begin
        if (!rst_calc_n) begin
            final_stop_r <= 1'b0;
        end else begin
            if (start_pulse_w || (!if_locked_r)) begin
                final_stop_r <= 1'b0;
            end else if (mode_result_valid_w && (mode_w != 2'd3)) begin
                final_stop_r <= 1'b1;
            end
        end
    end

    assign dds_stop_o = final_stop_r;
    assign stop_led   = final_stop_r;

    assign led_cw = final_stop_r && (mode_w == 2'd0);
    assign led_am = final_stop_r && (mode_w == 2'd1);
    assign led_fm = final_stop_r && (mode_w == 2'd2);

    // ------------------------------------------------------------------
    // Simplified four-bit direct frequency-code output logic.
    //
    // This is the "internal update logic" mentioned earlier. It belongs to
    // this top module, not to a separate module.
    //
    // 4'd0 : invalid / CW / no-lock
    // 4'd1 : 1 kHz
    // 4'd2 : 2 kHz
    // ...
    // 4'd10: 10 kHz
    // ------------------------------------------------------------------
    always @(posedge clk_calc_200m) begin
        if (!rst_calc_n) begin
            demod_freq_code_o <= 4'd0;
        end else begin
            if (clear_result_r) begin
                demod_freq_code_o <= 4'd0;
            end else if (mode_result_valid_w) begin
                if ((mode_w == 2'd1) || (mode_w == 2'd2))
                    demod_freq_code_o <= mode_fm_khz_w;
                else
                    demod_freq_code_o <= 4'd0;
            end
        end
    end

    // ------------------------------------------------------------------
    // Four-digit HC595 parameter display.
    // CW/search/no-lock clears to 0000.
    // ------------------------------------------------------------------
    wire        display_update_w = mode_result_valid_w | clear_result_r;
    wire [1:0]  display_mode_w   = clear_result_r ? 2'd0  : mode_w;
    wire [11:0] display_ma_w     = clear_result_r ? 12'd0 : mode_ma_permille_w;
    wire [3:0]  display_mf_w     = clear_result_r ? 4'd0  : mode_mf_int_w;
    wire [6:0]  display_df_w     = clear_result_r ? 7'd0  : mode_df_khz_w;

    q3_param_hc595_display u_param_display (
        .clk            (clk_calc_200m),
        .rst_n          (rst_calc_n),
        .mode_i         (display_mode_w),
        .ma_permille_i  (display_ma_w),
        .mf_int_i       (display_mf_w),
        .df_khz_i       (display_df_w),
        .update_i       (display_update_w),
        .sh_cp          (sh_cp),
        .st_cp          (st_cp),
        .ds             (ds)
    );

endmodule