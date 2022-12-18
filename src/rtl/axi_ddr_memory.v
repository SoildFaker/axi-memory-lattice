/*
* AXI-4 memory model for Lattice DDR3 Controller
* Author: Enbin Li
* E-mail: enbinli@outlook.com
*
* Description:
*    A Simple AXI4 <-> DDR MEM module with many limitations
*
* Limitation:
*  - no 4K boundary check
*  - only aligned transfer was supported
*  - only INCR type burst transfer was supported
*  - narrow transfer was not supported
*  - burst length width was limited to 4 bit for hardware copatibility
*  - lock, cache, prot bus functions was not implemented
*
* Reversion:
*  - 0.1: Sun Dec 18 12:18:18 PM CST 2022
*     - file created
*/

`define DATA_WIDTH 16
`define CS_WIDTH 2
`define ROW_WIDTH 14
`define COL_WIDTH 10
`define BANK_WIDTH 3

module axi_ddr_memory #(
    parameter AXI_ADDR_WIDTH    = 32,
    parameter AXI_DATA_WIDTH    = `DATA_WIDTH * 4,
    parameter AXI_ID_WIDTH      = 4,
    parameter AXI_STRB_WIDTH    = AXI_DATA_WIDTH >> 3
) (
    input                               ddr_clk,
    input                               rstn,
    input                               aclk,
    input                               aresetn,
    input   [AXI_ADDR_WIDTH - 1 : 0]    awaddr,
    input   [AXI_ID_WIDTH - 1 : 0]      awid,
    input   [3 : 0]                     awlen,
    /* not used
    input   [1 : 0]                     awburst,
    input   [2 : 0]                     awsize,
    input   [1 : 0]                     awlock,
    input   [3 : 0]                     awcache,
    input   [2 : 0]                     awport,
    */
    input                               awvalid,
    output                              awready,
    /* not used
    input   [AXI_STRB_WIDTH - 1 : 0]    wstrb,
    */
    input   [AXI_DATA_WIDTH - 1 : 0]    wdata,
    input                               wlast,
    input                               wvalid,
    output                              wready,
    output  [AXI_ID_WIDTH - 1 : 0]      bid,
    output  [1 : 0]                     bresp,
    output                              bvalid,
    input                               bready,
    input   [AXI_ID_WIDTH - 1 : 0]      arid,
    input   [AXI_ADDR_WIDTH - 1 : 0]    araddr,
    input   [3 : 0]                     arlen,
    /* not used
    input   [1 : 0]                     arburst,
    input   [2 : 0]                     arsize,
    input   [1 : 0]                     arlock,
    input   [3 : 0]                     arcache,
    input   [2 : 0]                     arport,
    */
    input                               arvalid,
    output                              arready,
    output  [AXI_ID_WIDTH - 1 : 0]      rid,
    output  [AXI_DATA_WIDTH - 1 : 0]    rdata,
    output  [1 : 0]                     rresp,
    output                              rvalid,
    input                               rready,
    output                              rlast,
    // inputs & outputs for memory interface
    inout  [`DATA_WIDTH - 1 : 0]        em_ddr_data,
    inout  [`DATA_WIDTH / 8 - 1 : 0]    em_ddr_dqs,
    output [`DATA_WIDTH / 8 - 1 : 0]    em_ddr_dm,
    output [`CS_WIDTH - 1 : 0]          em_ddr_clk,
    output [`CS_WIDTH - 1 : 0]          em_ddr_cke,
    output                              em_ddr_ras_n,
    output                              em_ddr_cas_n,
    output                              em_ddr_we_n,
    output  [`CS_WIDTH - 1 : 0]         em_ddr_cs_n,
    output  [`CS_WIDTH - 1 : 0]         em_ddr_odt,
    output  [`ROW_WIDTH - 1 : 0]        em_ddr_addr,
    output  [`BANK_WIDTH - 1 : 0]       em_ddr_ba,
    output                              em_ddr_reset_n,

    output  [4 : 0]                     ddr_status
);

wire ddr_clk;
wire rstn;
wire aclk;
wire aresetn;
wire sclk;

/*
* Lattice defined DDR command
* Using READ and WRITE only.
*/
localparam DDR_CMD_READ     = 4'b0001;
localparam DDR_CMD_WRITE    = 4'b0010;

/*
* AXI defined response type
*/
localparam AXI_RESP_OKAY    = 2'b00;
localparam AXI_RESP_EXOKAY  = 2'b01;
localparam AXI_RESP_SLVERR  = 2'b10;
localparam AXI_RESP_DECERR  = 2'b11;

/*
* AXI Read/Write QoS and Arbitration
* Using simple R/W siwtch arbiter since we use different bank for read and write. Row miss is out of consideration.
* The command valid signal is required to be asserted before ready signal and deasserted after ready signal.
*/

localparam DDR_CMD_WIDTH    = 4;
localparam DDR_ADDR_WIDTH   = `BANK_WIDTH + `ROW_WIDTH + `COL_WIDTH;
localparam DDR_BURST_WIDTH  = 5;
localparam CMD_FIFO_WIDTH   = AXI_ADDR_WIDTH + DDR_CMD_WIDTH + DDR_BURST_WIDTH + AXI_ID_WIDTH;

wire cmd_fifo_push, cmd_fifo_pop, cmd_fifo_empty, cmd_fifo_full;
wire [CMD_FIFO_WIDTH - 1 : 0] cmd_fifo_odata, cmd_fifo_idata;

wire ddr_cmd_ready;
wire ddr_cmd_ok;

wire [DDR_CMD_WIDTH - 1 : 0] ddr_cmd;
wire [DDR_ADDR_WIDTH - 1 : 0] ddr_cmd_addr;
wire [AXI_ADDR_WIDTH - 1 : 0] ddr_cmd_axi_addr;
wire [AXI_ID_WIDTH - 1 : 0] ddr_cmd_axi_id;
wire [4 : 0] ddr_cmd_burst_len;
wire [4 : 0] ddr_cmd_axi_burst_len;
reg ddr_cmd_valid;

assign cmd_fifo_pop = ~ddr_cmd_valid & ~cmd_fifo_empty;
assign ddr_cmd_ok = ddr_cmd_ready & ddr_cmd_valid;
assign {ddr_cmd_axi_burst_len, ddr_cmd_axi_id, ddr_cmd_axi_addr, ddr_cmd} = cmd_fifo_odata;
assign ddr_cmd_burst_len = ddr_cmd_axi_burst_len + 1'b1;
assign ddr_cmd_addr = {ddr_cmd_axi_addr[`COL_WIDTH + `ROW_WIDTH - 1 -: `ROW_WIDTH], ddr_cmd_axi_addr[AXI_ADDR_WIDTH - 1 -: `CS_WIDTH + `BANK_WIDTH], ddr_cmd_axi_addr[`COL_WIDTH - 1 : 0]};
/*
* Expected Command Pop and DDR Response Waveform
*                   ____      ____ ____ ____           ____
* cmd_fifo_empty        \____/              \____ ____/
*                        ____                ____
* cmd_fifo_pop      ____/    \____ ____ ____/    \____ ____
*                             ____ ____           ____
* ddr_cmd_valid     ____ ____/         \____ ____/    \____
*                                  ____           ____
* ddr_cmd_ready     ____ ____ ____/    \____ ____/    \____
*
* ddr_cmd           XXXX-XXXX- D0 - D0 - D0 - D0 - D1 - D1
*/

always @(posedge sclk) begin
    if (~rstn) begin
        ddr_cmd_valid <= 1'b0;
    end
    else begin
        if (ddr_cmd_ok) begin
            ddr_cmd_valid <= 1'b0;
        end
        else begin
            ddr_cmd_valid <= ~cmd_fifo_empty;
        end
    end
end

/*
* Simple Arbiter
*/
wire arbiter_en;
wire awvalid, arvalid;
wire [AXI_ADDR_WIDTH - 1 : 0] awaddr, araddr;
wire [AXI_ID_WIDTH - 1 : 0] awid, arid;
wire [3 : 0] awlen, arlen;

reg arbiter_write;
reg r_awready, r_arready;

assign arbiter_en = (awvalid & arvalid);
assign cmd_fifo_push = ((awvalid & awready) | (arvalid & arvalid));
assign cmd_fifo_idata = arbiter_write ? {1'b0, awlen, awid, awaddr, DDR_CMD_WRITE} : {1'b0, arlen, arid, araddr, DDR_CMD_READ};

assign awready = r_awready;
assign arready = r_arready;

always @(posedge aclk) begin
    if (~aresetn) begin
        arbiter_write <= 1'b0;
    end
    else begin
        arbiter_write <= arbiter_en ? ~arbiter_write : 1'b0;
    end
end

always @(posedge aclk) begin
    if (~aresetn) begin
        r_awready <= 1'b0;
        r_arready <= 1'b0;
    end
    else begin
        if (arbiter_en) begin
            r_awready <= arbiter_write ? ~cmd_fifo_full : 1'b0;
            r_arready <= arbiter_write ? 1'b0 : ~cmd_fifo_full;
        end
        else begin
            r_awready <= ~cmd_fifo_full;
            r_arready <= ~cmd_fifo_full;
        end
    end
end

/*
* AXI Write Data Channel
* Based on Lattice DDR Controller IP limitation, we highly recommend you to write data before address
*/
localparam DATA_FIFO_WIDTH = AXI_DATA_WIDTH;

wire data_ififo_push, data_ififo_pop;
wire data_ififo_empty, cmd_fifo_full;
wire [DATA_FIFO_WIDTH - 1 : 0] data_ififo_idata, data_ififo_odata;

wire wready, wvalid;
wire wlast;
wire [AXI_DATA_WIDTH - 1 : 0] wdata;

assign data_ififo_push = wvalid;
assign data_ififo_idata = wdata;
assign wready = ~data_ififo_full;

/*
* AXI Write Data Response Channel
*/
wire [AXI_ID_WIDTH - 1 : 0] bid;
wire [1 : 0] bresp;

reg r_bvalid;

assign bid = ddr_cmd_axi_id;
assign bresp = AXI_RESP_OKAY;
assign bvalid = r_bvalid;

always @(posedge aclk) begin
    if (~aresetn) begin
        r_bvalid <= 1'b0;
    end
    else begin
        if (bvalid & bready) begin
            r_bvalid <= 1'b0;
        end
        else begin
            if (wlast) begin
                r_bvalid <= 1'b1;
            end
        end
    end
end

/*
* AXI Read Response Channel
*/

wire data_ofifo_empty, data_ofifo_full;
wire data_ofifo_pop, data_ofifo_push;
wire [DATA_FIFO_WIDTH - 1 : 0] data_ofifo_idata, data_ofifo_odata;

wire [AXI_ID_WIDTH - 1 : 0] rid;
wire [AXI_DATA_WIDTH - 1 : 0] rdata;
wire [1 : 0] rresp;
wire rready;

reg [4 : 0] burst_read_counter;
reg r_rvalid;
reg r_rlast;

assign rid = ddr_cmd_axi_id;
assign rdata = data_ofifo_odata;
assign rresp = AXI_RESP_OKAY;
assign rvalid = r_valid;
assign rlast = r_rlast;

assign data_ofifo_pop = (~data_ofifo_empty & rready);

always @(posedge aclk)) begin
    if (~aresetn) begin
        r_rvalid <= 1'b0
    end
    else begin
        r_rvalid <= ~data_ofifo_empty;
    end
end

always @(posedge aclk) begin
    if (~aresetn) begin
        burst_read_counter <= 'b0;
        r_rlast <= 1'b0;
    end
    else begin
        if (data_ofifo_pop) begin
            if (burst_read_counter == ddr_cmd_axi_burst_len) begin
                burst_read_counter <= 'b0;
                r_rlast <= 1'b1;
            end
            else begin
                burst_read_counter <= burst_read_counter + 1'b1;
            end
        end
        else begin
            r_rlast <= 1'b0;
        end
    end
end

/*
* DDR Write Channel
* No need to check fifo empty flag, if DDR write ready and fifo empty was asserted at same time, it was considered as an error.
*/
localparam DDR_DATA_WIDTH = AXI_DATA_WIDTH;

wire ddr_datain_ready;
wire ddr_write_error_flag;
wire [DDR_DATA_WIDTH - 1 : 0] ddr_write_data;

assign data_ififo_pop = ddr_datain_ready;
assign ddr_write_data = data_ififo_odata;
assign ddr_write_error_flag = (ddr_datain_ready & data_ififo_empty);

/*
* DDR Read Channel
* No need to check fifo full flag, if DDR read data valid and fifo full was asserted at same time, it was considered as an error.
*/

wire ddr_dataout_valid;
wire ddr_read_error_flag;
wire [DDR_DATA_WIDTH - 1 : 0] ddr_read_data;

assign data_ofifo_push = ddr_dataout_valid;
assign data_ofifo_idata = ddr_read_data;
assign ddr_read_error_flag = (ddr_dataout_valid & data_ofifo_full);

async_fifo_wrapper #(
    .DEPTH                  (16                 ),
    .DATA_WIDTH             (CMD_FIFO_WIDTH     )
) u_ddr_cmd_fifo (
    .iclk                   (aclk               ),
    .ipush                  (cmd_fifo_push      ),
    .idata                  (cmd_fifo_idata     ),
    .oclk                   (sclk               ),
    .opop                   (cmd_fifo_pop       ),
    .odata                  (cmd_fifo_odata     ),
    .oempty                 (cmd_fifo_empty     ),
    .ofull                  (cmd_fifo_full      )
);

async_fifo_wrapper #(
    .DEPTH                  (16                 ),
    .DATA_WIDTH             (DATA_IFIFO_WIDTH   )
) u_ddr_data_ififo (
    .iclk                   (aclk               ),
    .ipush                  (data_ififo_push    ),
    .idata                  (data_ififo_idata   ),
    .oclk                   (sclk               ),
    .opop                   (data_ififo_pop     ),
    .odata                  (data_ififo_odata   ),
    .oempty                 (data_ififo_empty   ),
    .ofull                  (data_ififo_full    )
);

async_fifo_wrapper #(
    .DEPTH                  (16                 ),
    .DATA_WIDTH             (DATA_OFIFO_WIDTH   )
) u_ddr_data_ofifo (
    .iclk                   (sclk               ),
    .ipush                  (data_ofifo_push    ),
    .idata                  (data_ofifo_idata   ),
    .oclk                   (aclk               ),
    .opop                   (data_ofifo_pop     ),
    .odata                  (data_ofifo_odata   ),
    .oempty                 (data_ofifo_empty   ),
    .ofull                  (data_ofifo_full    )
);

/*
* DDR Reset and Initilization
* Lattice require a 200us reset duration at least before initial sequence start.
*/
localparam DDR_RESET_DURATION_CNT = 'd40000; // if freq. = 100MHz, then period = 10ns, 10ns * 40000 = 400us

wire init_done;
wire mem_rst_n;

reg [31 : 0] r_ddr_init_counter;
reg ddr_init_start;
reg ddr_init_done;
reg mem_rst_n;

assign mem_rst_n = r_ddr_reset_counter == DDR_RESET_DURATION_CNT;

always @(posedge sclk) begin
    if (~rstn) begin
        r_ddr_reset_counter <= 'b0;
    end
    else begin
        if (r_ddr_reset_counter < DDR_RESET_DURATION_CNT) begin
            r_ddr_reset_counter <= r_ddr_reset_counter + 1'b1;
        end
    end
end

always @(posedge sclk) begin
    if (~rstn) begin
        ddr_init_start <= 1'b0;
        ddr_init_done <= 1'b0;
    end
    else begin
        if (r_ddr_reset_counter == (DDR_RESET_DURATION_CNT - 1'b1)) begin
            ddr_init_start <= 1'b1;
        end
        else begin
            if (init_done) begin
                ddr_init_start <= 1'b0;
                ddr_init_done <= 1'b1;
            end
        end
    end
end

/*
* DDR3 IP core instantiation
*/
localparam DDR_DQS_WIDTH    = 2;

// READ_PULSE_TAP setting for PCB routing compensation
assign rpt  = {1'b0, 1'b1, 1'b0};   // use the default = 2
assign ddr_read_pulse_tap = {DDR_DQS_WIDTH{rpt}};
assign ddr_status = {ddr_wl_err, ddr_read_error_flag, ddr_write_error_flag, ddr_clocking_good, ddr_init_done};

ddr3_sdram_mem_top u_ddr3_sdram_mem_top (
    .clk_in                 (ddr_clk            ),
    .rst_n                  (rstn               ),
    .mem_rst_n              (mem_rst_n          ),
    .cmd                    (ddr_cmd            ),
    .addr                   (ddr_cmd_addr       ),
    .cmd_valid              (ddr_cmd_valid      ),
    .ofly_burst_len         (1'b1               ), // fixed 8 burst length
    .cmd_burst_cnt          (ddr_cmd_burst_len  ),
    .init_start             (ddr_init_start     ),
    .write_data             (ddr_write_data     ),
    .data_mask              (2'b11              ), // all byte enabled
    .cmd_rdy                (ddr_cmd_ready      ),
    .init_done              (init_done          ),
    .datain_rdy             (ddr_datain_ready   ),
    .wl_err                 (ddr_wl_err         ),
    .clocking_good          (ddr_clocking_good  ),
  `ifdef EXT_AUTO_REF
    .ext_auto_ref           (1'b0               ),
    .ext_auto_ref_ack       (                   ),
  `endif
    .read_data              (ddr_read_data      ),
    .read_data_valid        (ddr_dataout_valid  ),
    .read_pulse_tap         (ddr_read_pulse_tap ),
    .em_ddr_data            (em_ddr_data        ),
    .em_ddr_dqs             (em_ddr_dqs         ),
    .em_ddr_clk             (em_ddr_clk         ),
    .em_ddr_reset_n         (em_ddr_reset_n     ),
    .em_ddr_cke             (em_ddr_cke         ),
    .em_ddr_ras_n           (em_ddr_ras_n       ),
    .em_ddr_cas_n           (em_ddr_cas_n       ),
    .em_ddr_we_n            (em_ddr_we_n        ),
    .em_ddr_cs_n            (em_ddr_cs_n        ),
    .em_ddr_odt             (em_ddr_odt         ),
    .em_ddr_dm              (em_ddr_dm          ),
    .em_ddr_ba              (em_ddr_ba          ),
    .em_ddr_addr            (em_ddr_addr        ),
    .sclk_out               (sclk               )
);

endmodule