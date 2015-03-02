//////////////////////////////////////////////////////////////////
//                                                              //
//  Top-level module instantiating the entire Amber 2 system.   //
//                                                              //
//  This file is part of the Amber project                      //
//  http://www.opencores.org/project,amber                      //
//                                                              //
//  Description                                                 //
//  This is the highest level synthesizable module in the       //
//  project. The ports in this module represent pins on the     //
//  FPGA.                                                       //
//                                                              //
//  Author(s):                                                  //
//      - Conor Santifort, csantifort.amber@gmail.com           //
//                                                              //
//////////////////////////////////////////////////////////////////
//                                                              //
// Copyright (C) 2010 Authors and OPENCORES.ORG                 //
//                                                              //
// This source file may be used and distributed without         //
// restriction provided that this copyright statement is not    //
// removed from the file and that any derivative work contains  //
// the original copyright notice and the associated disclaimer. //
//                                                              //
// This source file is free software; you can redistribute it   //
// and/or modify it under the terms of the GNU Lesser General   //
// Public License as published by the Free Software Foundation; //
// either version 2.1 of the License, or (at your option) any   //
// later version.                                               //
//                                                              //
// This source is distributed in the hope that it will be       //
// useful, but WITHOUT ANY WARRANTY; without even the implied   //
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU Lesser General Public License for more //
// details.                                                     //
//                                                              //
// You should have received a copy of the GNU Lesser General    //
// Public License along with this source; if not, download it   //
// from http://www.opencores.org/lgpl.shtml                     //
//                                                              //
//////////////////////////////////////////////////////////////////


module system
(
input                       brd_rst,
input                       brd_clk,


// UART 0 Interface
input                       i_uart0_rts,
output                      o_uart0_rx,
output                      o_uart0_cts,
input                       i_uart0_tx,

output  [7:0]               LED_BAR_O,
output  [2:0]			    GP_OUTPUT_O,

// SDRAM pins
output [12:0]     		    SDRAM_ADDR,
output [1:0]                SDRAM_BA,
output			            SDRAM_CAS,
output					    SDRAM_CKE,
output				      	SDRAM_CLK,
output						SDRAM_CS,
inout [15:0]				SDRAM_DATA,
output [1:0]				SDRAM_DQM,
output		  				SDRAM_RAS,
output		  				SDRAM_WE
);

wire                       brd_clk_n;
wire                       brd_clk_p;

wire            sys_clk;    // System clock
wire            sys_rst;    // Active high reset, synchronous to sys_clk
wire            clk_200;    // 200MHz from board



// ======================================
// Wishbone Buses
// ======================================

localparam WB_MASTER  = 1;
localparam WB_SLAVES  = 9;

`ifdef AMBER_A25_CORE
localparam WB_DWIDTH  = 128;
localparam WB_SWIDTH  = 16;
`else
localparam WB_DWIDTH  = 32;
localparam WB_SWIDTH  = 4;
`endif


// Wishbone Master Buses
wire      [31:0]           m_wb_adr       ;
wire      [WB_SWIDTH-1:0]  m_wb_sel       ;
wire      [WB_MASTER-1:0]  m_wb_we        ;
wire      [WB_DWIDTH-1:0]  m_wb_dat_w     ;
wire      [WB_DWIDTH-1:0]  m_wb_dat_r     ;
wire      [WB_MASTER-1:0]  m_wb_cyc       ;
wire      [WB_MASTER-1:0]  m_wb_stb       ;
wire      [WB_MASTER-1:0]  m_wb_ack       ;
wire      [WB_MASTER-1:0]  m_wb_err       ;


// Wishbone Slave Buses
wire      [31:0]            s_wb_adr      [WB_SLAVES-1:0];
wire      [WB_SWIDTH-1:0]   s_wb_sel      [WB_SLAVES-1:0];
wire      [WB_SLAVES-1:0]   s_wb_we                      ;
wire      [WB_DWIDTH-1:0]   s_wb_dat_w    [WB_SLAVES-1:0];
wire      [WB_DWIDTH-1:0]   s_wb_dat_r    [WB_SLAVES-1:0];
wire      [WB_SLAVES-1:0]   s_wb_cyc                     ;
wire      [WB_SLAVES-1:0]   s_wb_stb                     ;
wire      [WB_SLAVES-1:0]   s_wb_ack                     ;
wire      [WB_SLAVES-1:0]   s_wb_err                     ;

wire      [31:0]            emm_wb_adr;
wire      [3:0]             emm_wb_sel;
wire                        emm_wb_we;
wire      [31:0]            emm_wb_rdat;
wire      [31:0]            emm_wb_wdat;
wire                        emm_wb_cyc;
wire                        emm_wb_stb;
wire                        emm_wb_ack;
wire                        emm_wb_err;

wire      [31:0]            ems_wb_adr;
wire      [3:0]             ems_wb_sel;
wire                        ems_wb_we;
wire      [31:0]            ems_wb_rdat;
wire      [31:0]            ems_wb_wdat;
wire                        ems_wb_cyc;
wire                        ems_wb_stb;
wire                        ems_wb_ack;
wire                        ems_wb_err;


// ======================================
// Interrupts
// ======================================
wire                        amber_irq;
wire                        amber_firq;
wire                        ethmac_int;
wire                        test_reg_irq;
wire                        test_reg_firq;
wire                        uart0_int;
wire                        uart1_int;
wire      [2:0]             timer_int;


// ======================================
// Clocks and Resets Module
// ======================================
//clocks_resets u_clocks_resets (
//    .i_brd_rst          ( brd_rst           ),
//    .i_brd_clk_n        ( brd_clk_n         ),
//    .i_brd_clk_p        ( brd_clk_p         ),
//    .i_ddr_calib_done   ( phy_init_done     ),
//    .o_sys_rst          ( sys_rst           ),
//    .o_sys_clk          ( sys_clk           ),
//    .o_clk_200          ( clk_200           )
//);
//assign brd_clk_n = brd_clk;
//assign brd_clk_p = ~brd_clk;  

reg [23:0] clk_divider = 0;
always @(posedge brd_clk)
	clk_divider <= clk_divider + 1;


reg reset_reg=1;  // initial value is '1'
reg [7:0] reset_dly_cnt=0;
always @(posedge brd_clk)
  begin
    if (~brd_rst) 
                begin
                        reset_reg <= 1; 
                        reset_dly_cnt <= 0;
                end
    if (reset_dly_cnt < 8'hff) reset_dly_cnt <= reset_dly_cnt + 1;  // count to 255d, then stop
    else reset_reg <= 0;  // deassert reset at terminal count
  end
 
assign sys_rst = reset_reg; //~brd_rst;
assign sys_clk = brd_clk;
//assign sys_clk = clk_divider[8];

//assign system_rdy = phy_init_done && !sys_rst;
assign system_rdy = !sys_rst;

/*
assign LED_BAR_O[0] = m_wb_dat_r[24];
assign LED_BAR_O[1] = m_wb_dat_r[25];
assign LED_BAR_O[2] = m_wb_dat_r[26];
assign LED_BAR_O[3] = m_wb_dat_r[27];
assign LED_BAR_O[4] = m_wb_dat_r[28];
assign LED_BAR_O[5] = m_wb_dat_r[29];
assign LED_BAR_O[6] = m_wb_dat_r[30];
assign LED_BAR_O[7] = m_wb_dat_r[31];
*/
/*
assign LED_BAR_O[0] = m_wb_dat_r[16];
assign LED_BAR_O[1] = m_wb_dat_r[17];
assign LED_BAR_O[2] = m_wb_dat_r[18];
assign LED_BAR_O[3] = m_wb_dat_r[19];
assign LED_BAR_O[4] = m_wb_dat_r[20];
assign LED_BAR_O[5] = m_wb_dat_r[21];
assign LED_BAR_O[6] = m_wb_dat_r[22];
assign LED_BAR_O[7] = m_wb_dat_r[23];
*/
/*
assign LED_BAR_O[0] = m_wb_adr[8];
assign LED_BAR_O[1] = m_wb_adr[9];
assign LED_BAR_O[2] = m_wb_adr[10];
assign LED_BAR_O[3] = m_wb_adr[11];
assign LED_BAR_O[4] = m_wb_adr[12];
assign LED_BAR_O[5] = m_wb_adr[13];
assign LED_BAR_O[6] = m_wb_adr[14];
assign LED_BAR_O[7] = m_wb_adr[15];
*/

assign LED_BAR_O[0] = m_wb_adr[0];
assign LED_BAR_O[1] = m_wb_adr[1];
assign LED_BAR_O[2] = m_wb_adr[2];
assign LED_BAR_O[3] = m_wb_adr[3];
assign LED_BAR_O[4] = m_wb_adr[4];
assign LED_BAR_O[5] = m_wb_adr[5];
assign LED_BAR_O[6] = m_wb_adr[6];
assign LED_BAR_O[7] = m_wb_adr[7];


assign GP_OUTPUT_O[2] = ~o_uart0_rx;
//assign GP_OUTPUT_O[0] = in_uart0 ( m_wb_adr ) ? 1'd0  : 1'd1 ;
assign GP_OUTPUT_O[0] = m_wb_adr[31:16] == 16'h1600 ? 1'd1 : 1'd0 ;
assign GP_OUTPUT_O[1] = m_wb_adr[15:0] == 16'h001C ? 1'd1 : 1'd0;


// -------------------------------------------------------------
// Instantiate Amber Processor Core
// -------------------------------------------------------------
`ifdef AMBER_A25_CORE
a25_core u_amber (
`else
a23_core u_amber (
`endif
    .i_clk          ( sys_clk         ),

    .i_irq          ( amber_irq       ),
    .i_firq         ( amber_firq      ),

    .i_system_rdy   ( system_rdy      ),

    .o_wb_adr       ( m_wb_adr     ),
    .o_wb_sel       ( m_wb_sel     ),
    .o_wb_we        ( m_wb_we      ),
    .i_wb_dat       ( m_wb_dat_r   ),
    .o_wb_dat       ( m_wb_dat_w   ),
    .o_wb_cyc       ( m_wb_cyc     ),
    .o_wb_stb       ( m_wb_stb     ),
    .i_wb_ack       ( m_wb_ack     ),
    .i_wb_err       ( m_wb_err     )
);


// -------------------------------------------------------------
// Instantiate Boot Memory - 8KBytes of Embedded SRAM
// -------------------------------------------------------------

generate
if (WB_DWIDTH == 32) begin : boot_mem32
    boot_mem32 u_boot_mem (
        .i_wb_clk               ( sys_clk         ),
        .i_wb_adr               ( s_wb_adr  [1]   ),
        .i_wb_sel               ( s_wb_sel  [1]   ),
        .i_wb_we                ( s_wb_we   [1]   ),
        .o_wb_dat               ( s_wb_dat_r[1]   ),
        .i_wb_dat               ( s_wb_dat_w[1]   ),
        .i_wb_cyc               ( s_wb_cyc  [1]   ),
        .i_wb_stb               ( s_wb_stb  [1]   ),
        .o_wb_ack               ( s_wb_ack  [1]   ),
        .o_wb_err               ( s_wb_err  [1]   )
    );
end
else begin : boot_mem128
    boot_mem128 u_boot_mem (
        .i_wb_clk               ( sys_clk         ),
        .i_wb_adr               ( s_wb_adr  [1]   ),
        .i_wb_sel               ( s_wb_sel  [1]   ),
        .i_wb_we                ( s_wb_we   [1]   ),
        .o_wb_dat               ( s_wb_dat_r[1]   ),
        .i_wb_dat               ( s_wb_dat_w[1]   ),
        .i_wb_cyc               ( s_wb_cyc  [1]   ),
        .i_wb_stb               ( s_wb_stb  [1]   ),
        .o_wb_ack               ( s_wb_ack  [1]   ),
        .o_wb_err               ( s_wb_err  [1]   )
    );
end
endgenerate

// -------------------------------------------------------------
// Instantiate SDRAM Memory 
// -------------------------------------------------------------
SDRAM_WB_CTRL sdram0 (
        .WB_CLK_I               ( sys_clk         ),
        .WB_RST_I               ( sys_rst         ),
        .WB_ADR_I               ( s_wb_adr  [2]   ),
        .WB_SEL_I               ( s_wb_sel  [2]   ),
        .WB_WE_I                ( s_wb_we   [2]   ),
        .WB_DATA_O              ( s_wb_dat_r[2]   ),
        .WB_DATA_I              ( s_wb_dat_w[2]   ),
        .WB_CYC_I               ( s_wb_cyc  [2]   ),
        .WB_STB_I               ( s_wb_stb  [2]   ),
        .WB_ACK_O               ( s_wb_ack  [2]   ),
		  // The SDRAM pins
		  .SDRAM_ADDR             ( SDRAM_ADDR  ),
		  .SDRAM_BA               ( SDRAM_BA    ),
		  .SDRAM_CAS              ( SDRAM_CAS   ),
		  .SDRAM_CKE              ( SDRAM_CKE   ),
		  .SDRAM_CLK              ( SDRAM_CLK   ),
		  .SDRAM_CS               ( SDRAM_CS    ),
		  .SDRAM_DATA             ( SDRAM_DATA  ),
		  .SDRAM_DQM              ( SDRAM_DQM   ),
		  .SDRAM_RAS              ( SDRAM_RAS   ),
		  .SDRAM_WE               ( SDRAM_WE    )

    );
	 
assign s_wb_err[2] = 1'd0;

// -------------------------------------------------------------
// Instantiate UART0
// -------------------------------------------------------------
uart  #(
    .WB_DWIDTH              ( WB_DWIDTH       ),
    .WB_SWIDTH              ( WB_SWIDTH       )
    )
u_uart0 (
    .i_clk                  ( sys_clk        ),

    .o_uart_int             ( uart0_int      ),

//    .i_uart_cts_n           ( i_uart0_rts    ),
    .i_uart_cts_n           ( 1'd0    ),
    .o_uart_txd             ( o_uart0_rx     ),
    .o_uart_rts_n           ( o_uart0_cts    ),
    .i_uart_rxd             ( i_uart0_tx     ),

    .i_wb_adr               ( s_wb_adr  [3]  ),
    .i_wb_sel               ( s_wb_sel  [3]  ),
    .i_wb_we                ( s_wb_we   [3]  ),
    .o_wb_dat               ( s_wb_dat_r[3]  ),
    .i_wb_dat               ( s_wb_dat_w[3]  ),
    .i_wb_cyc               ( s_wb_cyc  [3]  ),
    .i_wb_stb               ( s_wb_stb  [3]  ),
    .o_wb_ack               ( s_wb_ack  [3]  ),
    .o_wb_err               ( s_wb_err  [3]  )
	 
);


// -------------------------------------------------------------
// Instantiate UART1
// -------------------------------------------------------------
/*
uart  #(
    .WB_DWIDTH              ( WB_DWIDTH       ),
    .WB_SWIDTH              ( WB_SWIDTH       )
    )
u_uart1 (
    .i_clk                  ( sys_clk        ),

    .o_uart_int             ( uart1_int      ),

    // These are not connected. Only pins for 1 UART
    // on my development board
    .i_uart_cts_n           ( 1'd1           ),
    .o_uart_txd             (                ),
    .o_uart_rts_n           (                ),
    .i_uart_rxd             ( 1'd1           ),

    .i_wb_adr               ( s_wb_adr  [4]  ),
    .i_wb_sel               ( s_wb_sel  [4]  ),
    .i_wb_we                ( s_wb_we   [4]  ),
    .o_wb_dat               ( s_wb_dat_r[4]  ),
    .i_wb_dat               ( s_wb_dat_w[4]  ),
    .i_wb_cyc               ( s_wb_cyc  [4]  ),
    .i_wb_stb               ( s_wb_stb  [4]  ),
    .o_wb_ack               ( s_wb_ack  [4]  ),
    .o_wb_err               ( s_wb_err  [4]  )
);
*/

// -------------------------------------------------------------
// Instantiate Timer Module
// -------------------------------------------------------------

timer_module  #(
    .WB_DWIDTH              ( WB_DWIDTH      ),
    .WB_SWIDTH              ( WB_SWIDTH      )
    )
u_timer_module (
    .i_clk                  ( sys_clk        ),

    // Interrupt outputs
    .o_timer_int            ( timer_int      ),

    // Wishbone interface
    .i_wb_adr               ( s_wb_adr  [6]  ),
    .i_wb_sel               ( s_wb_sel  [6]  ),
    .i_wb_we                ( s_wb_we   [6]  ),
    .o_wb_dat               ( s_wb_dat_r[6]  ),
    .i_wb_dat               ( s_wb_dat_w[6]  ),
    .i_wb_cyc               ( s_wb_cyc  [6]  ),
    .i_wb_stb               ( s_wb_stb  [6]  ),
    .o_wb_ack               ( s_wb_ack  [6]  ),
    .o_wb_err               ( s_wb_err  [6]  )
);


// -------------------------------------------------------------
// Instantiate Interrupt Controller Module
// -------------------------------------------------------------
interrupt_controller  #(
    .WB_DWIDTH              ( WB_DWIDTH      ),
    .WB_SWIDTH              ( WB_SWIDTH      )
    )
u_interrupt_controller (
    .i_clk                  ( sys_clk        ),

    // Interrupt outputs
    .o_irq                  ( amber_irq      ),
    .o_firq                 ( amber_firq     ),

    // Interrupt inputs
    .i_uart0_int            ( uart0_int      ),
    .i_uart1_int            ( uart1_int      ),
    .i_ethmac_int           ( ethmac_int     ),
    .i_test_reg_irq         ( test_reg_irq   ),
    .i_test_reg_firq        ( test_reg_firq  ),
    .i_tm_timer_int         ( timer_int      ),

    // Wishbone interface
    .i_wb_adr               ( s_wb_adr  [7]  ),
    .i_wb_sel               ( s_wb_sel  [7]  ),
    .i_wb_we                ( s_wb_we   [7]  ),
    .o_wb_dat               ( s_wb_dat_r[7]  ),
    .i_wb_dat               ( s_wb_dat_w[7]  ),
    .i_wb_cyc               ( s_wb_cyc  [7]  ),
    .i_wb_stb               ( s_wb_stb  [7]  ),
    .o_wb_ack               ( s_wb_ack  [7]  ),
    .o_wb_err               ( s_wb_err  [7]  )
);



/*
`ifndef XILINX_FPGA
    // ======================================
    // Instantiate non-synthesizable main memory model
    // ======================================

    assign phy_init_done = 1'd1;

    main_mem #(
                .WB_DWIDTH             ( WB_DWIDTH             ),
                .WB_SWIDTH             ( WB_SWIDTH             )
                )
    u_main_mem (
               .i_clk                  ( sys_clk               ),
               .i_mem_ctrl             ( test_mem_ctrl         ),
               .i_wb_adr               ( s_wb_adr  [2]         ),
               .i_wb_sel               ( s_wb_sel  [2]         ),
               .i_wb_we                ( s_wb_we   [2]         ),
               .o_wb_dat               ( s_wb_dat_r[2]         ),
               .i_wb_dat               ( s_wb_dat_w[2]         ),
               .i_wb_cyc               ( s_wb_cyc  [2]         ),
               .i_wb_stb               ( s_wb_stb  [2]         ),
               .o_wb_ack               ( s_wb_ack  [2]         ),
               .o_wb_err               ( s_wb_err  [2]         )
            );

`endif
*/

// -------------------------------------------------------------
// Instantiate Wishbone Arbiter
// -------------------------------------------------------------
wishbone_arbiter #(
    .WB_DWIDTH              ( WB_DWIDTH         ),
    .WB_SWIDTH              ( WB_SWIDTH         )
    )
u_wishbone_arbiter (
    .i_wb_clk               ( sys_clk           ),

    // WISHBONE master 0 - Amber Process or
    .i_m0_wb_adr            ( m_wb_adr       ),
    .i_m0_wb_sel            ( m_wb_sel       ),
    .i_m0_wb_we             ( m_wb_we        ),
    .o_m0_wb_dat            ( m_wb_dat_r     ),
    .i_m0_wb_dat            ( m_wb_dat_w     ),
    .i_m0_wb_cyc            ( m_wb_cyc       ),
    .i_m0_wb_stb            ( m_wb_stb       ),
    .o_m0_wb_ack            ( m_wb_ack       ),
    .o_m0_wb_err            ( m_wb_err       ),

    // WISHBONE slave 1 - Boot Memory
    .o_s1_wb_adr            ( s_wb_adr   [1]    ),
    .o_s1_wb_sel            ( s_wb_sel   [1]    ),
    .o_s1_wb_we             ( s_wb_we    [1]    ),
    .i_s1_wb_dat            ( s_wb_dat_r [1]    ),
    .o_s1_wb_dat            ( s_wb_dat_w [1]    ),
    .o_s1_wb_cyc            ( s_wb_cyc   [1]    ),
    .o_s1_wb_stb            ( s_wb_stb   [1]    ),
    .i_s1_wb_ack            ( s_wb_ack   [1]    ),
    .i_s1_wb_err            ( s_wb_err   [1]    ),

    // WISHBONE slave 2 - Main Memory
    .o_s2_wb_adr            ( s_wb_adr   [2]    ),
    .o_s2_wb_sel            ( s_wb_sel   [2]    ),
    .o_s2_wb_we             ( s_wb_we    [2]    ),
    .i_s2_wb_dat            ( s_wb_dat_r [2]    ),
    .o_s2_wb_dat            ( s_wb_dat_w [2]    ),
    .o_s2_wb_cyc            ( s_wb_cyc   [2]    ),
    .o_s2_wb_stb            ( s_wb_stb   [2]    ),
    .i_s2_wb_ack            ( s_wb_ack   [2]    ),
    .i_s2_wb_err            ( s_wb_err   [2]    ),

    // WISHBONE slave 3 - UART 0
    .o_s3_wb_adr            ( s_wb_adr   [3]    ),
    .o_s3_wb_sel            ( s_wb_sel   [3]    ),
    .o_s3_wb_we             ( s_wb_we    [3]    ),
    .i_s3_wb_dat            ( s_wb_dat_r [3]    ),
    .o_s3_wb_dat            ( s_wb_dat_w [3]    ),
    .o_s3_wb_cyc            ( s_wb_cyc   [3]    ),
    .o_s3_wb_stb            ( s_wb_stb   [3]    ),
    .i_s3_wb_ack            ( s_wb_ack   [3]    ),
    .i_s3_wb_err            ( s_wb_err   [3]    ),

    // WISHBONE slave 4 - UART 1
    .o_s4_wb_adr            ( s_wb_adr   [4]    ),
    .o_s4_wb_sel            ( s_wb_sel   [4]    ),
    .o_s4_wb_we             ( s_wb_we    [4]    ),
    .i_s4_wb_dat            ( s_wb_dat_r [4]    ),
    .o_s4_wb_dat            ( s_wb_dat_w [4]    ),
    .o_s4_wb_cyc            ( s_wb_cyc   [4]    ),
    .o_s4_wb_stb            ( s_wb_stb   [4]    ),
    .i_s4_wb_ack            ( s_wb_ack   [4]    ),
    .i_s4_wb_err            ( s_wb_err   [4]    ),

    // WISHBONE slave 6 - Timer Module
    .o_s6_wb_adr            ( s_wb_adr   [6]    ),
    .o_s6_wb_sel            ( s_wb_sel   [6]    ),
    .o_s6_wb_we             ( s_wb_we    [6]    ),
    .i_s6_wb_dat            ( s_wb_dat_r [6]    ),
    .o_s6_wb_dat            ( s_wb_dat_w [6]    ),
    .o_s6_wb_cyc            ( s_wb_cyc   [6]    ),
    .o_s6_wb_stb            ( s_wb_stb   [6]    ),
    .i_s6_wb_ack            ( s_wb_ack   [6]    ),
    .i_s6_wb_err            ( s_wb_err   [6]    ),

    // WISHBONE slave 7 - Interrupt Controller
    .o_s7_wb_adr            ( s_wb_adr   [7]    ),
    .o_s7_wb_sel            ( s_wb_sel   [7]    ),
    .o_s7_wb_we             ( s_wb_we    [7]    ),
    .i_s7_wb_dat            ( s_wb_dat_r [7]    ),
    .o_s7_wb_dat            ( s_wb_dat_w [7]    ),
    .o_s7_wb_cyc            ( s_wb_cyc   [7]    ),
    .o_s7_wb_stb            ( s_wb_stb   [7]    ),
    .i_s7_wb_ack            ( s_wb_ack   [7]    ),
    .i_s7_wb_err            ( s_wb_err   [7]    )
    );

endmodule


