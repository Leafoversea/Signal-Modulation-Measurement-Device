`timescale 1ns / 1ps

// Unsigned restoring divider with timing-friendly two-cycle-per-bit datapath.
//
// Original one-cycle-per-bit version did this in a single 200 MHz cycle:
//     remainder shift -> compare -> subtract -> remainder register
// That created a long carry-chain path. This version splits each quotient-bit
// iteration into two registered phases:
//     ST_CMP    : remainder shift + compare, register compare result
//     ST_UPDATE : optional subtract + quotient/remainder update
//
// Interface is unchanged. Latency is about 2*NUM_W cycles instead of NUM_W cycles.
// If denom is zero, quot saturates to all ones and done is asserted for one cycle.
// If the real quotient is wider than Q_W, quot saturates to all ones.

module unsigned_divider_saturating #(
    parameter integer NUM_W = 56,
    parameter integer DEN_W = 48,
    parameter integer Q_W   = 23
)(
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 start,
    input  wire [NUM_W-1:0]     numer,
    input  wire [DEN_W-1:0]     denom,
    output reg                  busy,
    output reg                  done,
    output reg  [Q_W-1:0]       quot
);
    function integer clog2;
        input integer value;
        integer i;
        begin
            value = value - 1;
            for (i = 0; value > 0; i = i + 1)
                value = value >> 1;
            clog2 = i;
        end
    endfunction

    localparam integer COUNT_W = clog2(NUM_W + 1);

    localparam [1:0] ST_IDLE   = 2'd0;
    localparam [1:0] ST_CMP    = 2'd1;
    localparam [1:0] ST_UPDATE = 2'd2;

    reg [1:0] state_r;

    reg [NUM_W-1:0] dividend_r;
    reg [NUM_W-1:0] quotient_r;
    reg [DEN_W:0]   remainder_r;
    reg [DEN_W:0]   denom_r;
    reg [COUNT_W-1:0] count_r;

    // 比较阶段和减法更新阶段之间的流水寄存器
    reg [DEN_W:0] rem_shift_cmp_r;
    reg           ge_cmp_r;

    wire [DEN_W:0] rem_shift_w;
    wire           ge_w;
    wire [DEN_W:0] sub_result_w;
    wire [DEN_W:0] remainder_update_w;
    wire [NUM_W-1:0] quotient_update_w;

    assign rem_shift_w        = {remainder_r[DEN_W-1:0], dividend_r[NUM_W-1]};
    assign ge_w               = (rem_shift_w >= denom_r);

    assign sub_result_w       = rem_shift_cmp_r - denom_r;
    assign remainder_update_w = ge_cmp_r ? sub_result_w : rem_shift_cmp_r;
    assign quotient_update_w  = {quotient_r[NUM_W-2:0], ge_cmp_r};

    always @(posedge clk) begin
        if (!rst_n) begin
            state_r         <= ST_IDLE;
            dividend_r      <= {NUM_W{1'b0}};
            quotient_r      <= {NUM_W{1'b0}};
            remainder_r     <= {(DEN_W+1){1'b0}};
            denom_r         <= {(DEN_W+1){1'b0}};
            count_r         <= {COUNT_W{1'b0}};
            rem_shift_cmp_r <= {(DEN_W+1){1'b0}};
            ge_cmp_r        <= 1'b0;
            busy            <= 1'b0;
            done            <= 1'b0;
            quot            <= {Q_W{1'b0}};
        end else begin
            done <= 1'b0;

            case (state_r)
                ST_IDLE: begin
                    busy <= 1'b0;

                    if (start) begin
                        if (denom == {DEN_W{1'b0}}) begin
                            dividend_r      <= {NUM_W{1'b0}};
                            quotient_r      <= {NUM_W{1'b0}};
                            remainder_r     <= {(DEN_W+1){1'b0}};
                            denom_r         <= {(DEN_W+1){1'b0}};
                            count_r         <= {COUNT_W{1'b0}};
                            rem_shift_cmp_r <= {(DEN_W+1){1'b0}};
                            ge_cmp_r        <= 1'b0;

                            quot <= {Q_W{1'b1}};
                            done <= 1'b1;
                        end else begin
                            dividend_r      <= numer;
                            quotient_r      <= {NUM_W{1'b0}};
                            remainder_r     <= {(DEN_W+1){1'b0}};
                            denom_r         <= {1'b0, denom};
                            count_r         <= NUM_W;
                            rem_shift_cmp_r <= {(DEN_W+1){1'b0}};
                            ge_cmp_r        <= 1'b0;

                            busy    <= 1'b1;
                            state_r <= ST_CMP;
                        end
                    end
                end

                ST_CMP: begin
                    // 第一拍：左移余数并比较
                    // 原关键路径中的“比较”被单独寄存
                    busy            <= 1'b1;
                    rem_shift_cmp_r <= rem_shift_w;
                    ge_cmp_r        <= ge_w;
                    state_r         <= ST_UPDATE;
                end

                ST_UPDATE: begin
                    // 第二拍：根据比较结果更新余数和商
                    // 原关键路径中的“减法”被单独放到这一拍
                    busy        <= 1'b1;
                    dividend_r  <= {dividend_r[NUM_W-2:0], 1'b0};
                    quotient_r  <= quotient_update_w;
                    remainder_r <= remainder_update_w;

                    if (count_r == {{(COUNT_W-1){1'b0}}, 1'b1}) begin
                        state_r <= ST_IDLE;
                        busy    <= 1'b0;
                        done    <= 1'b1;
                        count_r <= {COUNT_W{1'b0}};

                        if (|quotient_update_w[NUM_W-1:Q_W])
                            quot <= {Q_W{1'b1}};
                        else
                            quot <= quotient_update_w[Q_W-1:0];
                    end else begin
                        count_r <= count_r - {{(COUNT_W-1){1'b0}}, 1'b1};
                        state_r <= ST_CMP;
                    end
                end

                default: begin
                    state_r <= ST_IDLE;
                    busy    <= 1'b0;
                    done    <= 1'b0;
                end
            endcase
        end
    end

endmodule