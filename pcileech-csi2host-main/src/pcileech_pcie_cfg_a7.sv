// ------------------------------------------------------------------
// PCILeech FPGA - PCIe Configuration Module - Multi-Function Support
// Artix-7 FPGA Example - Child Device Handling
// ------------------------------------------------------------------

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_pcie_cfg_a7(
    input                   rst,
    input                   clk_sys,
    input                   clk_pcie,
    IfPCIeFifoCfg.mp_pcie   dfifo,
    IfPCIeSignals.mpm       ctx,
    IfAXIS128.source        tlps_static,
    input                   int_enable,
    output [15:0]           pcie_id,
    output wire [31:0]      base_address_register
    );

    // ---- Multi-Function Support ----
    localparam integer NUM_FUNCTIONS = 2; // Change as needed
    localparam [7:0] HEADER_TYPE_MULTIFUNC = 8'h80; // Bit 7 enabled for multi-function

    // Per-function config and registers
    reg      [703:0]  rw      [NUM_FUNCTIONS-1:0];
    wire     [383:0]  ro      [NUM_FUNCTIONS-1:0];
    reg      [31:0]   bar_reg [NUM_FUNCTIONS-1:0];

    reg      [9:0]    cfgrd_addr    [NUM_FUNCTIONS-1:0];
    reg      [3:0]    cfgrd_byte_en [NUM_FUNCTIONS-1:0];
    reg      [31:0]   cfgrd_data    [NUM_FUNCTIONS-1:0];
    reg               cfg_rd_en     [NUM_FUNCTIONS-1:0];
    reg               cfg_wr_en     [NUM_FUNCTIONS-1:0];
    reg               cfgrd_valid   [NUM_FUNCTIONS-1:0];

    // Function selector parsing from PCIe request (adjust as needed for your stack)
    wire [2:0] function_num = dfifo.tx_data[10:8];

    // ---- Initialization for all functions ----
    task automatic init_function_cfg;
        integer f;
        begin
            for (f = 0; f < NUM_FUNCTIONS; f = f + 1) begin
                rw[f][15:0]    <= 16'h6745 + f;          // Unique MAGIC per function
                rw[f][115:108] <= HEADER_TYPE_MULTIFUNC; // Header type: multi-function
                bar_reg[f]     <= 32'h00000000;
                cfg_rd_en[f]   <= 1'b0;
                cfg_wr_en[f]   <= 1'b0;
                cfgrd_valid[f] <= 1'b0;
                cfgrd_addr[f]  <= 10'd0;
                cfgrd_byte_en[f]<=4'd0;
                cfgrd_data[f]  <= 32'd0;
            end
        end
    endtask

    // ----- PCIe ID and BAR output (per function) -----
    assign pcie_id = ro[function_num][79:64];
    assign base_address_register = bar_reg[function_num];

    // ----- Data access helpers -----
    wire [15:0] in_cmd_data_in = (dfifo.tx_data[11]) ? // Read/write flag, adjust as needed
        rw[function_num][dfifo.tx_data[31:16] +: 16] : ro[function_num][dfifo.tx_data[31:16] +: 16];

    // ---- Main State Machine ----
    initial begin
        init_function_cfg();
    end

    always @(posedge clk_pcie) begin
        if (rst)
            init_function_cfg();
        else begin
            integer i_write;
            // READ config request
            if (dfifo.tx_valid) begin
                dfifo.rx_data[31:16] <= dfifo.tx_data[31:16];
                dfifo.rx_data[15:0]  <= {in_cmd_data_in[7:0], in_cmd_data_in[15:8]};
            end

            // WRITE config request
            if (dfifo.tx_valid) begin
                for (i_write = 0; i_write < 16; i_write = i_write + 1) begin
                    if (dfifo.tx_data[32+:8][i_write])
                        rw[function_num][dfifo.tx_data[31:16]+i_write] <= dfifo.tx_data[48+:8][i_write];
                end
            end

            // Example: Save BAR value on config completion (adjust to your logic)
            if (cfg_rd_en[function_num]) begin
                bar_reg[function_num] <= rw[function_num][BAR_OFFSET]; // Use proper BAR_OFFSET
                cfg_rd_en[function_num] <= 1'b0;
            end

            // Status register logic, interrupt logic and static TLP TX can be ported from your original design
            // Use function_num selector in all per-function operations.

            // You may expand here to add MSIx, DMA, internal function-specific handlers, etc.
        end
    end

    // ---- Interrupt and advanced features logic per function (expand as needed) ----

    // Example stub for interrupt (implement per your design)
    wire o_int;
    reg  cfg_int_valid [NUM_FUNCTIONS-1:0];
    reg  cfg_msg_num   [NUM_FUNCTIONS-1:0];
    reg  cfg_int_assert[NUM_FUNCTIONS-1:0];
    reg  cfg_int_di    [NUM_FUNCTIONS-1:0];
    reg  cfg_int_stat  [NUM_FUNCTIONS-1:0];

    always @(posedge clk_pcie) begin
        integer f;
        if (rst) begin
            for (f = 0; f < NUM_FUNCTIONS; f = f + 1) begin
                cfg_int_valid[f]   <= 1'b0;
                cfg_msg_num[f]     <= 5'b0;
                cfg_int_assert[f]  <= 1'b0;
                cfg_int_di[f]      <= 8'b0;
                cfg_int_stat[f]    <= 1'b0;
            end
        end
        else if (int_enable) begin
            for (f = 0; f < NUM_FUNCTIONS; f = f + 1)
                if (o_int)
                    cfg_int_valid[f] <= 1'b1;
        end
    end

endmodule
