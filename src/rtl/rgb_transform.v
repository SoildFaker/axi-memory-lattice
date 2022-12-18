/*
* Main timing transform module
* Author: Enbin Li
* E-mail: enbinli@outlook.com
*
* Description:
*   Using DDR3 to buffer 3 frames of inoput video
*
* Reversion:
*  - 0.1: Sun Dec 18 10:30:45 AM CST 2022
*     - file created
*/

module rgb_transform # (
    parameter AXI_ADDR_WIDTH    = 32,
    parameter AXI_DATA_WIDTH    = 64,
    parameter AXI_ID_WIDTH      = 4,
    parameter RGB_BUS_IN_WIDTH  = 4 * 24,
    parameter RGB_BUS_OUT_WIDTH = 4 * 24
) (
    input                                   clk,
    input                                   rstn,
    input                                   ipclk,
    input                                   ihs, ivs, ide,
    input   [RGB_BUS_IN_WIDTH - 1 : 0]      rgb_bus_in,
    output                                  opclk,
    output                                  ohs, ovs, ode,
    output  [RGB_BUS_OUT_WIDTH - 1 : 0]     rgb_bus_out,

    output                                  aclk,
    output                                  aresetn,
    output  [AXI_ADDR_WIDTH - 1 : 0]        awaddr,
    output  [AXI_ID_WIDTH - 1 : 0]          awid,
    output  [3 : 0]                         awlen,
    output                                  awvalid,
    input                                   awready,
    output  [AXI_DATA_WIDTH - 1 : 0]        wdata,
    output                                  wlast,
    output                                  wvalid,
    input                                   wready,
    input   [AXI_ID_WIDTH - 1 : 0]          bid,
    input   [1 : 0]                         bresp,
    input                                   bvalid,
    output                                  bready,
    output  [AXI_ID_WIDTH - 1 : 0]          arid,
    output  [AXI_ADDR_WIDTH - 1 : 0]        araddr,
    output  [3 : 0]                         arlen,
    output                                  arvalid,
    input                                   arready,
    input   [AXI_ID_WIDTH - 1 : 0]          rid,
    input   [AXI_DATA_WIDTH - 1 : 0]        rdata,
    input   [1 : 0]                         rresp,
    input                                   rvalid,
    output                                  rready,
    input                                   rlast
);

wire aclk;
wire aresetn;

assign aresetn = rstn;

/*
* Prior parameter of RGB INPUT BUS
*/
localparam RGB_RESX      = 1920;
localparam RGB_RESY      = 1080;

/*
* AXI defined response type
*/
localparam AXI_RESP_OKAY    = 2'b00;
localparam AXI_RESP_EXOKAY  = 2'b01;
localparam AXI_RESP_SLVERR  = 2'b10;
localparam AXI_RESP_DECERR  = 2'b11;

/*
* RGB BUS Input -> FIFO
*/
localparam RGB_IFIFO_WIDTH  = RGB_BUS_IN_WIDTH + 1;

wire frame_start_rgb_in;
reg frame_start_rgb_in_en;

reg ide_d, ivs_d;

wire rgb_ififo_full;
wire rgb_ififo_push;
wire [RGB_IFIFO_WIDTH - 1 : 0] rgb_ififo_idata;

assign frame_start_rgb_in = (~ide_d & ide) & frame_start_rgb_in_en;

assign rgb_ififo_push = ide;
assign rgb_ififo_idata = {rgb_bus_in, frame_start_rgb_in};

always @(posedge ipclk) begin
    ide_d <= ide;
    ivs_d <= ivs;
    if (~ivs_d & ivs) begin
        frame_start_rgb_in_en <= 1'b1;
    end
    if (frame_start_rgb_in) begin
        frame_start_rgb_in_en <= 1'b0;
    end
end

/*
* FIFO -> AXI Write Channel
*/
localparam AXI_BURST_LEN                = 3'b111;

localparam DDR_CS_BANK0                 = 5'b00_000;
localparam DDR_CS_BANK1                 = 5'b00_001;
localparam DDR_CS_BANK2                 = 5'b00_010;

localparam AXI_INPUT_FRAME_LENGTH       = RGB_RESX * RGB_RESY * 2;
localparam AXI_INPUT_FRAME_LOWER_ADDR0  = {DDR_CS_BANK0, 27'b0};
localparam AXI_INPUT_FRAME_UPPER_ADDR0  = AXI_INPUT_FRAME_LOWER_ADDR0 + AXI_INPUT_FRAME_LENGTH;
localparam AXI_INPUT_FRAME_LOWER_ADDR1  = {DDR_CS_BANK1, 27'b0};
localparam AXI_INPUT_FRAME_UPPER_ADDR1  = AXI_INPUT_FRAME_LOWER_ADDR1 + AXI_INPUT_FRAME_LENGTH;
localparam AXI_INPUT_FRAME_LOWER_ADDR2  = {DDR_CS_BANK2, 27'b0};
localparam AXI_INPUT_FRAME_UPPER_ADDR2  = AXI_INPUT_FRAME_LOWER_ADDR2 + AXI_INPUT_FRAME_LENGTH;

// Almost empty flag was asserted when the FIFO data count was less than 4 (4 * 2 = 8, which is the burst length)
wire rgb_ififo_aempty;
wire rgb_ififo_empty;
wire rgb_ififo_pop;
wire [RGB_IFIFO_WIDTH - 1 : 0] rgb_ififo_odata;

reg [2 : 0] write_frame_buffer_pos;
reg [2 : 0] read_frame_buffer_pos;
reg [AXI_ADDR_WIDTH - 1 : 0] axi_write_addr;
reg [AXI_ADDR_WIDTH - 1 : 0] axi_read_addr;

wire no_write_collision;
wire no_read_collision;

assign no_write_collision = |{{read_frame_buffer_pos[1 : 0], read_frame_buffer_pos[2]} & write_frame_buffer_pos};
assign no_read_collision = |{{write_frame_buffer_pos[1 : 0], write_frame_buffer_pos[2]} & read_frame_buffer_pos};

// write address
always @(posedge aclk) begin
    if (~rstn) begin
        axi_write_addr <= AXI_INPUT_FRAME_LOWER_ADDR0;
        write_frame_buffer_pos <= 3'b001;
    end
    else begin
        // frame start
        if (rgb_ififo_pop & rgb_ififo_odata[0]) begin
            if (no_write_collision) begin
                axi_write_addr <= write_frame_buffer_pos[2] ? AXI_INPUT_FRAME_LOWER_ADDR0 : (write_frame_buffer_pos[0] ? AXI_INPUT_FRAME_LOWER_ADDR1 : AXI_INPUT_FRAME_LOWER_ADDR2);
                write_frame_buffer_pos <= {write_frame_buffer_pos[1 : 0], write_frame_buffer_pos[2]};
            end
            else begin
                axi_write_addr <= write_frame_buffer_pos[0] ? AXI_INPUT_FRAME_LOWER_ADDR0 : (write_frame_buffer_pos[1] ? AXI_INPUT_FRAME_LOWER_ADDR1 : AXI_INPUT_FRAME_LOWER_ADDR2);
            end
        end
        else begin
            if (awready & awvalid) begin
                axi_write_addr <= axi_write_addr + AXI_BURST_LEN + 1'b1;
            end
        end
    end
end

// read address
always @(posedge aclk) begin
    if (~rstn) begin
        axi_read_addr <= AXI_INPUT_FRAME_LOWER_ADDR2;
        read_frame_buffer_pos <= 3'b100;
    end
    else begin
        if ((read_frame_buffer_pos[0] & (axi_read_addr >= AXI_INPUT_FRAME_UPPER_ADDR0)) |
            (read_frame_buffer_pos[1] & (axi_read_addr >= AXI_INPUT_FRAME_UPPER_ADDR1)) |
            (read_frame_buffer_pos[2] & (axi_read_addr >= AXI_INPUT_FRAME_UPPER_ADDR2))) begin
            if (no_read_collision) begin
                axi_read_addr <= read_frame_buffer_pos[2] ? AXI_INPUT_FRAME_LOWER_ADDR0 : (read_frame_buffer_pos[0] ? AXI_INPUT_FRAME_LOWER_ADDR1 : AXI_INPUT_FRAME_LOWER_ADDR2);
                read_frame_buffer_pos <= {read_frame_buffer_pos[1 : 0], read_frame_buffer_pos[2]};
            end
            else begin
                axi_read_addr <= read_frame_buffer_pos[0] ? AXI_INPUT_FRAME_LOWER_ADDR0 : (read_frame_buffer_pos[1] ? AXI_INPUT_FRAME_LOWER_ADDR1 : AXI_INPUT_FRAME_LOWER_ADDR2);
            end
        end
        else begin
            if (arready & arvalid) begin
                axi_read_addr <= axi_read_addr + AXI_BURST_LEN + 1'b1;
            end
        end
    end
end

/*
* AXI Write Command Generate
*/

localparam S_AXI_WRITE_IDLE     = 4'b0001;
localparam S_AXI_WRITE_DATA     = 4'b0010;
localparam S_AXI_WRITE_ADDR     = 4'b0100;
localparam S_AXI_WRITE_RESP     = 4'b1000;

reg [3 : 0] axi_write_state;

wire [AXI_ADDR_WIDTH - 1 : 0] awaddr;
wire [AXI_ID_WIDTH - 1 : 0] awid;
wire [3 : 0] awlen;
wire awready;
wire awvalid;

wire bvalid;
wire bready;
wire [AXI_ID_WIDTH - 1 : 0] bid;
wire [1 : 0] bresp;

assign awaddr = axi_write_addr;
assign awlen = AXI_BURST_LEN; // fixed 8 burst length
assign awid = {AXI_ID_WIDTH{1'b0}}; // donot care
assign awvalid = (axi_write_state == S_AXI_WRITE_ADDR);

assign bready = (axi_write_state == S_AXI_WRITE_RESP);

// AXI write state machine
always @(posedge aclk) begin
    if (~rstn) begin
        axi_write_state <= S_AXI_WRITE_IDLE;
    end
    else begin
        case (axi_write_state)
            S_AXI_WRITE_IDLE: begin
                // fifo gets more than burst length data
                if (~rgb_ififo_aempty) begin
                    axi_write_state <= S_AXI_WRITE_DATA;
                end
            end
            S_AXI_WRITE_DATA: begin
                // last data was wrote
                if (wlast & wvalid & wready) begin
                    axi_write_state <= S_AXI_WRITE_ADDR;
                end
            end
            S_AXI_WRITE_ADDR: begin
                if (awready & awvalid) begin
                    axi_write_state <= S_AXI_WRITE_RESP;
                end
            end
            S_AXI_WRITE_RESP: begin
                if (bvalid & bready) begin
                    // TODO: check bid == awid?
                    if (bresp == AXI_RESP_OKAY) begin
                        axi_write_state >= S_AXI_WRITE_IDLE;
                    end
                    else begin
                        // should not be here
                        $display("[%t] axi write response error %x", $realtime, bresp);
                    end
                end
            end
            default: begin
                axi_write_state <= S_AXI_WRITE_IDLE
            end
        endcase
    end
end

/*
* AXI Write Data Generate
*/

wire wready;
wire wlast;
reg [AXI_DATA_WIDTH - 1 : 0] r_wdata;
reg r_wvalid;

reg [4 : 0] axi_write_counter;
reg [1 : 0] axi_write_phase;

assign rgb_ififo_pop = (axi_write_state == S_AXI_WRITE_DATA) & (axi_write_phase == 2'b00) & ~rgb_ififo_empty;
assign wvalid = r_wvalid;
assign wlast = (axi_write_counter == AXI_BURST_LEN);
assign wdata = r_wdata;

always @(posedge aclk) begin
    if (~rstn) begin
        r_wvalid <= 1'b0;
        r_wdata <= 'b0;
    end
    else begin
        if (axi_write_state == S_AXI_WRITE_DATA) begin
            case (axi_write_phase)
                2'b00: begin
                    if (rgb_ififo_pop) begin
                        r_wvalid <= 1'b1;
                        r_wdata <= {15'b0, rgb_ififo_odata[0], rgb_ififo_odata[48 : 1]};
                    end
                    else begin
                        r_wvalid <= 1'b0;
                    end
                end
                2'b01: begin
                    if (wready & wvalid) begin
                        r_wdata <= {16'b0, rgb_ififo_odata[96 : 49]};
                    end
                end
                2'b10: begin
                    if (wready & wvalid) begin
                        r_wvalid <= 1'b0;
                    end
                end
                default: begin
                    r_wvalid <= 1'b0;
                    r_wdata <= 'b0;
                end
            endcase
        end
        else begin
            r_wvalid <= 1'b0;
        end
    end
end

always @(posedge aclk) begin
    if (~rstn) begin
        axi_write_counter <= 'b0;
    end
    else begin
        if (axi_write_state == S_AXI_WRITE_DATA) begin
            if (wready & wvalid) begin
                axi_write_counter <= (axi_write_counter == AXI_BURST_LEN) ? 'b0 : axi_write_counter + 1'b1;
            end
        end
        else begin
            axi_write_counter <= 'b0;
        end
    end
end

always @(posedge aclk) begin
    if (~rstn) begin
        axi_write_phase <= 2'b00;
    end
    else begin
        if (axi_write_state == S_AXI_WRITE_DATA) begin
            case (axi_write_phase)
                2'b00: begin
                    if (rgb_ififo_pop) begin
                        axi_write_phase <= 2'b01;
                    end
                end
                2'b01: begin
                    if (wready & wvalid) begin
                        axi_write_phase <= 2'b10;
                    end
                end
                2'b10: begin
                    if (wready & wvalid) begin
                        axi_write_phase <= 2'b00;
                    end
                end
                default: begin
                    axi_write_phase <= 2'b00;
                end
            endcase
        end
        else begin
            axi_write_phase <= 2'b00;
        end
    end
end

/*
* AXI Read -> FIFO
*/
localparam RGB_OFIFO_WIDTH  = RGB_BUS_IN_WIDTH + 1;

wire rgb_ofifo_afull;
wire arready;
wire arvalid;
wire [AXI_ADDR_WIDTH - 1 : 0] araddr;
wire [AXI_ID_WIDTH - 1 : 0] arid;
wire [3 : 0] arlen;

wire rlast;
wire rvalid;
wire rready;
wire [AXI_DATA_WIDTH - 1 : 0] rdata;
wire [1 : 0] rresp;

/*
* AXI Read Command Generate
*/

localparam S_AXI_READ_IDLE  = 3'b0001;
localparam S_AXI_READ_ADDR  = 3'b0010;
localparam S_AXI_READ_DATA  = 3'b0100;

reg [2 : 0] axi_read_state;

assign araddr = axi_read_addr;
assign arlen = AXI_BURST_LEN; // fixed 8 burst length
assign arid = {AXI_ID_WIDTH{1'b0}}; // donot care
assign arvalid = (axi_read_state == S_AXI_READ_ADDR);

assign rready = (axi_read_state == S_AXI_READ_DATA);

always @(posedge aclk) begin
    if (~rstn) begin
        axi_read_state <= S_AXI_READ_IDLE;
    end
    else begin
        case (axi_read_state)
            S_AXI_READ_IDLE: begin
                // rgb output fifo has more than 8 room
                if (~rgb_ofifo_afull) begin
                    axi_read_state <= S_AXI_READ_ADDR;
                end
            end
            S_AXI_READ_ADDR: begin
                if (arready & arvalid) begin
                    axi_read_state <= S_AXI_READ_DATA;
                end
            end
            S_AXI_READ_DATA: begin
                if (rlast & rvalid & rready) begin
                    if (rresp == AXI_RESP_OKAY) begin
                        axi_read_state <= S_AXI_READ_IDLE;
                    end
                    else begin
                        // should not be here
                        $display("[%t] axi read response error %x", $realtime, rresp);
                    end
                end
            end
            default: begin
                axi_read_state <= S_AXI_READ_IDLE;
            end
        endcase
    end
end

/*
* AXI Read Data -> FIFO
*/

wire rgb_ofifo_push;
wire rgb_ofifo_full;
wire [RGB_OFIFO_WIDTH - 1 : 0] rgb_ofifo_idata;

reg [1 : 0] axi_read_phase;
reg [AXI_DATA_WIDTH - 1 : 0] axi_read_data_p0, axi_read_data_p1;

assign rgb_ofifo_push = (axi_read_state == S_AXI_READ_DATA || axi_read_state == S_AXI_READ_IDLE ) & (axi_read_phase == 2'b10) & ~rgb_ofifo_full;
assign rgb_ofifo_idata = (axi_read_data_p1[47 : 0], axi_read_data_p0[47 : 0], axi_read_data_p0[48]);

always @(posedge aclk) begin
    if (~rstn) begin
        axi_read_phase <= 2'b00;
    end
    else begin
        if (axi_read_state == S_AXI_READ_ADDR) begin
            axi_read_phase <= 2'b00;
        end
        else begin
            case (axi_read_phase)
                2'b00: begin
                    if (rready & rvalid) begin
                        axi_read_phase <= 2'b01;
                        axi_read_data_p0 <= rdata;
                    end
                end
                2'b01: begin
                    if (rready & rvalid) begin
                        axi_read_phase <= 2'b10;
                        axi_read_data_p1 <= rdata;
                    end
                end
                2'b10: begin
                    if (rgb_ofifo_push) begin
                        axi_read_phase <= 2'b00;
                    end
                end
                default: begin
                    axi_read_phase <= 2'b00;
                end
            endcase
        end
    end
end

/*
* FIFO -> RGB BUS output
*/

/*
* Prior parameter of RGB OUTPUT BUS
*/
localparam RGB_OUT_HS   = 44;
localparam RGB_OUT_HFP  = 88;
localparam RGB_OUT_HBP  = 148;
localparam RGB_OUT_HACT = 1920;
localparam RGB_OUT_VS   = 5;
localparam RGB_OUT_VFP  = 4;
localparam RGB_OUT_VBP  = 36;
localparam RGB_OUT_VACT = 1080;
localparam RGB_OUT_HLEN = RGB_OUT_HS + RGB_OUT_HBP + RGB_OUT_HFP + RGB_OUT_HACT;
localparam RGB_OUT_VLEN = RGB_OUT_VS + RGB_OUT_VBP + RGB_OUT_VFP + RGB_OUT_VACT;

localparam S_RGBOUT_WAIT_FIFO   = 3'b001;
localparam S_RGBOUT_WAIT_FRAME  = 3'b010;
localparam S_RGBOUT_DISPLAY     = 3'b100;

wire [RGB_BUS_OUT_WIDTH - 1 : 0] rgb_bus_out;
wire [RGB_OFIFO_WIDTH - 1 : 0] rgb_ofifo_odata;
wire rgb_ofifo_pop;
wire rgb_out_en;

reg [2 : 0] rgb_out_state;
reg rgb_out_init_pop;

reg [15 : 0] counter_x, counter_y;
reg ode, ohs, ovs;

assign rgb_bus_out = {rgb_ofifo_odata[48 : 1]};
assign rgb_ofifo_pop = ode | rgb_out_init_pop;

assign rgb_out_en = (rgb_out_state == S_RGBOUT_DISPLAY);

always @(posedge opclk) begin
    if (~rstn) begin
        rgb_out_state <= S_RGBOUT_WAIT_FIFO;
        rgb_out_init_pop <= 1'b0;
    end
    else begin
        case (rgb_out_state)
            S_RGBOUT_WAIT_FIFO: begin
                if (~rgb_ofifo_empty) begin
                    rgb_out_state <= S_RGBOUT_WAIT_FRAME;
                    rgb_out_init_pop <= 1'b1;
                end
            end
            S_RGBOUT_WAIT_FRAME: begin
                rgb_out_init_pop <= 1'b0;
                if (rgb_ififo_odata[0]) begin
                    rgb_out_state <= S_RGBOUT_DISPLAY;
                end
                else begin
                    rgb_out_state <= S_RGBOUT_WAIT_FIFO;
                end
            end
            S_RGBOUT_DISPLAY: begin
                if (rgb_ofifo_empty) begin
                    // should not be here
                    rgb_out_state <= S_RGBOUT_WAIT_FIFO;
                end
            end
            default: begin
                rgb_out_state <= S_RGBOUT_WAIT_FIFO;
            end
        endcase
    end
end

always @(posedge opclk) begin
    if (~rstn | ~rgb_out_en) begin
        counter_x <= 16'h0000;
    end
    else begin
        counter_x <= (counter_x == (RGB_OUT_HLEN - 1)) ? 0 : counter_x + 1;
    end
end

always @(posedge opclk) begin
    if (~rstn | ~rgb_out_en) begin
        counter_y <= 16'h0000;
    end
    else begin
        if (counter_x == (RGB_OUT_HLEN - 1)) begin
            counter_y <= (counter_y == (RGB_OUT_VLEN - 1)) ? 0 : counter_y + 1;
        end
    end
end

always @(posedge opclk) begin
    if (~rstn | ~rgb_out_en) begin
        ode <= 1'b0;
    end
    else begin
        ode <= (counter_x < RGB_OUT_HACT) && (counter_y < RGB_OUT_VACT);
    end
end

always @(posedge opclk) begin
    if (~rstn | ~rgb_out_en) begin
        ohs <= 1'b1;
    end
    else begin
        ohs <= (counter_x < RGB_OUT_HACT + RGB_OUT_HFP) || (counter_x >= RGB_OUT_HACT + RGB_OUT_HFP + RGB_OUT_HS);
    end
end

always @(posedge opclk) begin
    if (~rstn | ~rgb_out_en) begin
        ovs <= 1'b1;
    end
    else begin
        ovs <= (counter_y < RGB_OUT_VACT + RGB_OUT_VFP) || (counter_y >= RGB_OUT_VACT + RGB_OUT_VFP + RGB_OUT_VS);
    end
end

async_fifo_wrapper #(
    .DEPTH                  (16                 ),
    .DATA_WIDTH             (RGB_IFIFO_WIDTH    )
) u_rgb_ififo (
    .iclk                   (ipclk              ),
    .ipush                  (rgb_ififo_push     ),
    .idata                  (rgb_ififo_idata    ),
    .oclk                   (aclk               ),
    .opop                   (rgb_ififo_pop      ),
    .odata                  (rgb_ififo_odata    ),
    .oempty                 (rgb_ififo_empty    ),
    .oaempty                (rgb_ififo_aempty   ),
    .ofull                  (rgb_ififo_full     )
);

async_fifo_wrapper #(
    .DEPTH                  (32                 ),
    .DATA_WIDTH             (RGB_OFIFO_WIDTH    )
) u_rgb_ofifo (
    .iclk                   (aclk               ),
    .ipush                  (rgb_ofifo_push     ),
    .idata                  (rgb_ofifo_idata    ),
    .oclk                   (opclk              ),
    .opop                   (rgb_ofifo_pop      ),
    .odata                  (rgb_ofifo_odata    ),
    .oempty                 (rgb_ofifo_empty    ),
    .ofull                  (rgb_ofifo_full     ),
    .oafull                 (rgb_ofifo_afull    )
);


endmodule