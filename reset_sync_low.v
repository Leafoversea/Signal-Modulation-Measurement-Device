module reset_sync_low (
    input  wire clk,
    input  wire rst_n_in,
    output wire rst_n_out
);
    reg [2:0] sr;

    always @(posedge clk or negedge rst_n_in) begin
        if (!rst_n_in)
            sr <= 3'b000;
        else
            sr <= {sr[1:0], 1'b1};
    end

    assign rst_n_out = sr[2];

endmodule