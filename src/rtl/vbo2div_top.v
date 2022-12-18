/*
* 4K VBO video Convert to 4 * 1080p DVI video signal
* Author: Enbin Li
* E-mail: enbinli@outlook.com
*
* Description:
*
* Reversion:
*  - 0.1: Sun Dec 18 10:25:04 AM CST 2022
*     - file created
*/

`define DATA_WIDTH 16
`define CS_WIDTH 2
`define ROW_WIDTH 14
`define COL_WIDTH 10
`define BANK_WIDTH 3

module vbo2div_top (
    input                               clk,
    input                               rstn,
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
    output  [1:0]                       leds
);

wire ddr_clk;
wire clk, rstn;
wire aclk, aresetn;
wire [4 : 0] ddr_status;

rgb_transform (
    .clk                    (clk                    ),
    .rstn                   (rstn | ddr_status[0]   ),

    // vbo rgb input
    .ipclk,
    .ihs,
    .ivs,
    .ide,
    .rgb_bus_in,

    // dvi rgb output
    .opclk,
    .ohs,
    .ovs,
    .ode,
    .rgb_bus_out,

    // AXI Bus
    .aclk                   (aclk                   ),
    .aresetn                (aresetn                ),
    .awaddr                 (awaddr                 ),
    .awid                   (awid                   ),
    .awlen                  (awlen                  ),
    .awvalid                (awvalid                ),
    .awready                (awready                ),
    .wdata                  (wdata                  ),
    .wlast                  (wlast                  ),
    .wvalid                 (wvalid                 ),
    .wready                 (wready                 ),
    .bid                    (bid                    ),
    .bresp                  (bresp                  ),
    .bvalid                 (bvalid                 ),
    .bready                 (bready                 ),
    .arid                   (arid                   ),
    .araddr                 (araddr                 ),
    .arlen                  (arlen                  ),
    .arvalid                (arvalid                ),
    .arready                (arready                ),
    .rid                    (rid                    ),
    .rdata                  (rdata                  ),
    .rresp                  (rresp                  ),
    .rvalid                 (rvalid                 ),
    .rready                 (rready                 ),
    .rlast                  (rlast                  )
);

axi_ddr_memory u_axi_ddr (
    .ddr_clk                (ddr_clk                ),
    .rstn                   (rstn                   ),
    .aclk                   (aclk                   ),
    .aresetn                (aresetn                ),
    .awaddr                 (awaddr                 ),
    .awid                   (awid                   ),
    .awlen                  (awlen                  ),
    .awvalid                (awvalid                ),
    .awready                (awready                ),
    .wdata                  (wdata                  ),
    .wlast                  (wlast                  ),
    .wvalid                 (wvalid                 ),
    .wready                 (wready                 ),
    .bid                    (bid                    ),
    .bresp                  (bresp                  ),
    .bvalid                 (bvalid                 ),
    .bready                 (bready                 ),
    .arid                   (arid                   ),
    .araddr                 (araddr                 ),
    .arlen                  (arlen                  ),
    .arvalid                (arvalid                ),
    .arready                (arready                ),
    .rid                    (rid                    ),
    .rdata                  (rdata                  ),
    .rresp                  (rresp                  ),
    .rvalid                 (rvalid                 ),
    .rready                 (rready                 ),
    .rlast                  (rlast                  ),
    .em_ddr_data            (em_ddr_data            ),
    .em_ddr_dqs             (em_ddr_dqs             ),
    .em_ddr_dm              (em_ddr_dm              ),
    .em_ddr_clk             (em_ddr_clk             ),
    .em_ddr_cke             (em_ddr_cke             ),
    .em_ddr_ras_n           (em_ddr_ras_n           ),
    .em_ddr_cas_n           (em_ddr_cas_n           ),
    .em_ddr_we_n            (em_ddr_we_n            ),
    .em_ddr_cs_n            (em_ddr_cs_n            ),
    .em_ddr_odt             (em_ddr_odt             ),
    .em_ddr_addr            (em_ddr_addr            ),
    .em_ddr_ba              (em_ddr_ba              ),
    .em_ddr_reset_n         (em_ddr_reset_n         ),
    .ddr_status             (ddr_status             )
);

endmodule