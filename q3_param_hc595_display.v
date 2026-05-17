`timescale 1ns / 1ps

// -----------------------------------------------------------------------------
// Four-digit 74HC595 parameter display for question 3.
//
// mode_i = 0/CW : display 0000
// mode_i = 1/AM : display ma as 0.200..1.000 using four digits and a decimal
//                 point after the leftmost digit.
// mode_i = 2/FM : display [mf] [blank] [df tens] [df ones], e.g. "1 05",
//                 "7 70".  This extends the original FM display to 70 kHz.
// -----------------------------------------------------------------------------

module q3_param_hc595_display (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [1:0]  mode_i,
    input  wire [11:0] ma_permille_i,
    input  wire [3:0]  mf_int_i,
    input  wire [6:0]  df_khz_i,
    input  wire        update_i,

    output reg         sh_cp,
    output reg         st_cp,
    output reg         ds
);

    localparam integer SCAN_DIV      = 200000;
    localparam integer HC595_CLK_DIV = 10;

    localparam [1:0] MODE_CW = 2'd0;
    localparam [1:0] MODE_AM = 2'd1;
    localparam [1:0] MODE_FM = 2'd2;

    function [6:0] seg7_active_low;
        input [3:0] data;
        begin
            case (data)
                4'd0:    seg7_active_low = 7'b1000000;
                4'd1:    seg7_active_low = 7'b1111001;
                4'd2:    seg7_active_low = 7'b0100100;
                4'd3:    seg7_active_low = 7'b0110000;
                4'd4:    seg7_active_low = 7'b0011001;
                4'd5:    seg7_active_low = 7'b0010010;
                4'd6:    seg7_active_low = 7'b0000010;
                4'd7:    seg7_active_low = 7'b1111000;
                4'd8:    seg7_active_low = 7'b0000000;
                4'd9:    seg7_active_low = 7'b0010000;
                default: seg7_active_low = 7'b1111111;
            endcase
        end
    endfunction

    function [15:0] ma_to_bcd4;
        input [11:0] ma;
        begin
            case (ma)
                12'd200:  ma_to_bcd4 = {4'd0, 4'd2, 4'd0, 4'd0};
                12'd300:  ma_to_bcd4 = {4'd0, 4'd3, 4'd0, 4'd0};
                12'd400:  ma_to_bcd4 = {4'd0, 4'd4, 4'd0, 4'd0};
                12'd500:  ma_to_bcd4 = {4'd0, 4'd5, 4'd0, 4'd0};
                12'd600:  ma_to_bcd4 = {4'd0, 4'd6, 4'd0, 4'd0};
                12'd700:  ma_to_bcd4 = {4'd0, 4'd7, 4'd0, 4'd0};
                12'd800:  ma_to_bcd4 = {4'd0, 4'd8, 4'd0, 4'd0};
                12'd900:  ma_to_bcd4 = {4'd0, 4'd9, 4'd0, 4'd0};
                12'd1000: ma_to_bcd4 = {4'd1, 4'd0, 4'd0, 4'd0};
                default:  ma_to_bcd4 = {4'd0, 4'd0, 4'd0, 4'd0};
            endcase
        end
    endfunction

    function [7:0] khz_to_bcd2;
        input [6:0] khz;
        begin
            case (khz)
                7'd0:    khz_to_bcd2 = {4'd0, 4'd0};
                7'd5:    khz_to_bcd2 = {4'd0, 4'd5};
                7'd6:    khz_to_bcd2 = {4'd0, 4'd6};
                7'd7:    khz_to_bcd2 = {4'd0, 4'd7};
                7'd8:    khz_to_bcd2 = {4'd0, 4'd8};
                7'd9:    khz_to_bcd2 = {4'd0, 4'd9};
                7'd10:   khz_to_bcd2 = {4'd1, 4'd0};
                7'd12:   khz_to_bcd2 = {4'd1, 4'd2};
                7'd14:   khz_to_bcd2 = {4'd1, 4'd4};
                7'd15:   khz_to_bcd2 = {4'd1, 4'd5};
                7'd16:   khz_to_bcd2 = {4'd1, 4'd6};
                7'd18:   khz_to_bcd2 = {4'd1, 4'd8};
                7'd20:   khz_to_bcd2 = {4'd2, 4'd0};
                7'd21:   khz_to_bcd2 = {4'd2, 4'd1};
                7'd24:   khz_to_bcd2 = {4'd2, 4'd4};
                7'd25:   khz_to_bcd2 = {4'd2, 4'd5};
                7'd27:   khz_to_bcd2 = {4'd2, 4'd7};
                7'd28:   khz_to_bcd2 = {4'd2, 4'd8};
                7'd30:   khz_to_bcd2 = {4'd3, 4'd0};
                7'd32:   khz_to_bcd2 = {4'd3, 4'd2};
                7'd35:   khz_to_bcd2 = {4'd3, 4'd5};
                7'd36:   khz_to_bcd2 = {4'd3, 4'd6};
                7'd40:   khz_to_bcd2 = {4'd4, 4'd0};
                7'd42:   khz_to_bcd2 = {4'd4, 4'd2};
                7'd45:   khz_to_bcd2 = {4'd4, 4'd5};
                7'd48:   khz_to_bcd2 = {4'd4, 4'd8};
                7'd49:   khz_to_bcd2 = {4'd4, 4'd9};
                7'd50:   khz_to_bcd2 = {4'd5, 4'd0};
                7'd54:   khz_to_bcd2 = {4'd5, 4'd4};
                7'd56:   khz_to_bcd2 = {4'd5, 4'd6};
                7'd60:   khz_to_bcd2 = {4'd6, 4'd0};
                7'd63:   khz_to_bcd2 = {4'd6, 4'd3};
                7'd70:   khz_to_bcd2 = {4'd7, 4'd0};
                default: khz_to_bcd2 = {4'hF, 4'hF};
            endcase
        end
    endfunction

    reg [15:0] display_bcd4; // leftmost nibble is [15:12], rightmost is [3:0]
    reg        display_dp_left_r;

    wire [7:0] df_bcd_w = khz_to_bcd2(df_khz_i);

    always @(posedge clk) begin
        if (!rst_n) begin
            display_bcd4     <= {4'd0, 4'd0, 4'd0, 4'd0};
            display_dp_left_r <= 1'b0;
        end else if (update_i) begin
            case (mode_i)
                MODE_AM: begin
                    display_bcd4     <= ma_to_bcd4(ma_permille_i);
                    display_dp_left_r <= 1'b1;
                end
                MODE_FM: begin
                    display_bcd4     <= {mf_int_i, 4'hF, df_bcd_w[7:4], df_bcd_w[3:0]};
                    display_dp_left_r <= 1'b0;
                end
                default: begin
                    display_bcd4     <= {4'd0, 4'd0, 4'd0, 4'd0};
                    display_dp_left_r <= 1'b0;
                end
            endcase
        end
    end

    reg [17:0] scan_cnt;
    reg [1:0]  scan_idx;

    wire scan_tick_w = (scan_cnt == SCAN_DIV - 1);
    wire [1:0] scan_idx_next_w = scan_idx + 2'd1;

    always @(posedge clk) begin
        if (!rst_n) begin
            scan_cnt <= 18'd0;
            scan_idx <= 2'd3;
        end else begin
            if (scan_tick_w) begin
                scan_cnt <= 18'd0;
                scan_idx <= scan_idx_next_w;
            end else begin
                scan_cnt <= scan_cnt + 18'd1;
            end
        end
    end

    reg [3:0] scan_bcd;
    always @(*) begin
        case (scan_idx_next_w)
            2'd0: scan_bcd = display_bcd4[ 3: 0];
            2'd1: scan_bcd = display_bcd4[ 7: 4];
            2'd2: scan_bcd = display_bcd4[11: 8];
            2'd3: scan_bcd = display_bcd4[15:12];
            default: scan_bcd = 4'hF;
        endcase
    end

    wire [6:0] scan_seg_w = seg7_active_low(scan_bcd);
    wire [7:0] scan_sel_w = (8'b0000_0001 << scan_idx_next_w);
    wire dp_on_w = display_dp_left_r && (scan_idx_next_w == 2'd3);
    wire [15:0] scan_word_w = {~dp_on_w, scan_seg_w, scan_sel_w};

    localparam [1:0] ST_IDLE  = 2'd0;
    localparam [1:0] ST_LOW   = 2'd1;
    localparam [1:0] ST_HIGH  = 2'd2;
    localparam [1:0] ST_LATCH = 2'd3;

    reg [1:0]  shift_state;
    reg [15:0] shift_word;
    reg [3:0]  shift_bit;
    reg [7:0]  hc595_cnt;

    always @(posedge clk) begin
        if (!rst_n) begin
            shift_state <= ST_IDLE;
            shift_word  <= 16'd0;
            shift_bit   <= 4'd15;
            hc595_cnt   <= 8'd0;
            sh_cp       <= 1'b0;
            st_cp       <= 1'b0;
            ds          <= 1'b0;
        end else begin
            case (shift_state)
                ST_IDLE: begin
                    sh_cp     <= 1'b0;
                    st_cp     <= 1'b0;
                    hc595_cnt <= 8'd0;
                    if (scan_tick_w) begin
                        shift_word  <= scan_word_w;
                        shift_bit   <= 4'd15;
                        ds          <= scan_word_w[15];
                        shift_state <= ST_LOW;
                    end
                end
                ST_LOW: begin
                    sh_cp <= 1'b0;
                    st_cp <= 1'b0;
                    if (hc595_cnt == HC595_CLK_DIV - 1) begin
                        hc595_cnt   <= 8'd0;
                        sh_cp       <= 1'b1;
                        shift_state <= ST_HIGH;
                    end else begin
                        hc595_cnt <= hc595_cnt + 8'd1;
                    end
                end
                ST_HIGH: begin
                    sh_cp <= 1'b1;
                    st_cp <= 1'b0;
                    if (hc595_cnt == HC595_CLK_DIV - 1) begin
                        hc595_cnt <= 8'd0;
                        sh_cp     <= 1'b0;
                        if (shift_bit == 4'd0) begin
                            shift_state <= ST_LATCH;
                        end else begin
                            shift_bit   <= shift_bit - 4'd1;
                            ds          <= shift_word[shift_bit - 4'd1];
                            shift_state <= ST_LOW;
                        end
                    end else begin
                        hc595_cnt <= hc595_cnt + 8'd1;
                    end
                end
                ST_LATCH: begin
                    sh_cp       <= 1'b0;
                    st_cp       <= 1'b1;
                    shift_state <= ST_IDLE;
                end
                default: begin
                    shift_state <= ST_IDLE;
                    sh_cp       <= 1'b0;
                    st_cp       <= 1'b0;
                    ds          <= 1'b0;
                end
            endcase
        end
    end

endmodule
