`default_nettype none
module top (
	    input  clk48,
	    output reg rgb_led0_r,
	    output reg rgb_led0_g,
	    output reg rgb_led0_b,
	    output reg gpio_0,
	    input  gpio_a0,
	    output gpio_9,
	    input gpio_6,
	    inout  scl,
	    inout  sda,
	    input  usr_btn,
	    output rst_n
	    );

   // Reset when btn0 is pressed for easy access to DFU mode
   assign rst_n = usr_btn;   
 
   // Counter for timed events
   reg [31:0] 	   counter = 0;
   
   // I2C bus speed
   parameter signed [31:0] INPUT_CLK_RATE = 48000000;
   parameter signed [31:0] TARGET_SCL_RATE = 100000;
   // I2C device address (in 7-bit format)
   parameter [6:0] ADDRESS = 7'h24;
   // I2C interface signals
   reg [6:0] 	   i2c_addr;   
   reg 		   i2c_command_en = 0;
   reg 		   i2c_rw;
   reg [7:0] 	   i2c_wdata;
   wire [7:0] 	   i2c_rdata;
   wire		   i2c_error;
   wire		   i2c_busy;
   reg 		   i2c_busy_last = 0;
   reg 		   i2c_reset_n = 0;

   reg [1:0] 	   i2c_busy_prev_state = 0;
   // Remember which register pair we are writing to
   reg [1:0] 	   reg_pair = 2'b10; 	      

   reg [7:0] 	   uart_xilinx0_txdata;
   reg 		   uart_xilinx0_txtrigger;
   reg 		   uart_xilinx0_dispatched;
   wire		   uart_xilinx0_txready;
   reg [7:0] 	   uart_xilinx0_txstate; 		   
   
   // UART TX to Xilinx FPGA
   uart_tx xilinx_uart0_tx (
	    .CLK(clk48),
	    .BIT_TMR_MAX(24'd23), // 48MHz/2Mbps = 24. 24 - 1 = 23
	    .DATA(uart_xilinx0_txdata),
	    .SEND(uart_xilinx0_txtrigger),
	    .READY(uart_xilinx0_txready),
	    // XXX for now feed to ESP32 board UART pins during bring-up testing
	    .UART_TX(gpio_9)	    
	    );
   

   // I2C bus   
   i2c_master #(
		.input_clk(INPUT_CLK_RATE),
		.bus_clk(TARGET_SCL_RATE)
		) 
   i2c_master(
	      .clk(clk48),
	      .reset_n(i2c_reset_n),
	      .ena(i2c_command_en),
	      .addr(i2c_addr),
	      .rw(i2c_rw),
	      .data_wr(i2c_wdata),
	      .busy(i2c_busy),
	      .data_rd(i2c_rdata),
	      .ack_error(i2c_error),
	      .sda(sda),
	      .scl(scl),
	      .swap(1'd0),
	      .debug_scl(1'd0),
	      .debug_sda(1'd0)
	      );


   // State machine for performing I2C operations
   reg [7:0] 	   busy_count;
   
   
   //
   // Debug output on RGB LED
   //    
//   assign rgb_led0_r = i2c_busy;   
//   assign rgb_led0_g = busy_count[0];   
//   assign rgb_led0_b = counter[24];   

   initial begin
      rgb_led0_r <= ~0;
      rgb_led0_g <= ~0;
      rgb_led0_b <= ~0;
      busy_count <= 99;
      // Start by setting up inversion and DDR bits for ports
      reg_pair <= 2'b10;
      
      // Put TX UARTs idle on reset
      uart_xilinx0_txtrigger <= 1'b0;      
      uart_xilinx0_dispatched <= 1'b0;
      uart_xilinx0_txstate <= 8'd0;
      
   end
   
   always @(posedge clk48) begin

      $display("txready=",uart_xilinx0_txready,", dispatched=", uart_xilinx0_dispatched);

      rgb_led0_r = ~uart_xilinx0_txready;      
      rgb_led0_g = ~uart_xilinx0_dispatched;      
      rgb_led0_b = ~1'b0;      
      
      if ( uart_xilinx0_txready == 1'b1 && uart_xilinx0_dispatched == 1'b0 ) begin
	 // Send next char via UART
	 uart_xilinx0_dispatched <= 1'b1;
	 uart_xilinx0_txtrigger <= 1'b1;

	 
	 if (uart_xilinx0_txstate < 255) uart_xilinx0_txstate <= uart_xilinx0_txstate + 1;

	 $display("Sending char ",uart_xilinx0_txstate);
	 
	 case (uart_xilinx0_txstate)
	   // Display ready message
	   // MEGAphone CTL0<CRLF>
	   8'd0: uart_xilinx0_txdata <= 8'h4D;
	   8'd1: uart_xilinx0_txdata <= 8'h45;
	   8'd2: uart_xilinx0_txdata <= 8'h47;
	   8'd3: uart_xilinx0_txdata <= 8'h41;
	   8'd4: uart_xilinx0_txdata <= 8'h70;
	   8'd5: uart_xilinx0_txdata <= 8'h68;
	   8'd6: uart_xilinx0_txdata <= 8'h6f;
	   8'd7: uart_xilinx0_txdata <= 8'h6e;
	   8'd8: uart_xilinx0_txdata <= 8'h65;
	   8'd9: uart_xilinx0_txdata <= 8'h20;
	   8'd10: uart_xilinx0_txdata <= 8'h43;
	   8'd11: uart_xilinx0_txdata <= 8'h54;
	   8'd12: uart_xilinx0_txdata <= 8'h4c;
	   8'd13: uart_xilinx0_txdata <= 8'h30;
	   8'd14: uart_xilinx0_txdata <= 8'h0d;
	   8'd15: begin
	      uart_xilinx0_txdata <= 8'h0a;
	      // End of message
	   end
	   default: begin
	      // Do nothing while idle
	      uart_xilinx0_txtrigger <= 1'b0;	   
	      uart_xilinx0_dispatched <= 1'b0;
	   end
	 endcase
	   
      end
      if ( uart_xilinx0_txready == 1'b0 ) begin
	 uart_xilinx0_dispatched <= 1'b0;	 
	 uart_xilinx0_txtrigger <= 1'b0;	 
      end
      
      // Detect rising edge of busy signals
      i2c_busy_last <= i2c_busy;

//      rgb_led0_g = ~counter[24];      
      
      // Clear initial reset of I2C master
      i2c_reset_n <= 1;
      
      // Retrigger I2C every ~0.125 sec
      counter <= counter + 1;
      if (counter[21:0] == 22'd0) begin
	 busy_count <= 99;
	 // XXX DEBUG and re-display UART message
	 uart_xilinx0_txstate <= 8'd0;
      end
      
      // Now each time i2c_busy goes high we schedule
      // the next read or write action to the I2C state machine
      if ( { i2c_busy,i2c_busy_last } != i2c_busy_prev_state) begin
	 $display($time,": i2c_busy=",i2c_busy,", i2c_busy_last=",i2c_busy_last);
	 i2c_busy_prev_state <= { i2c_busy,i2c_busy_last};	 
      end
      
      if ( ((i2c_busy == 1) && (i2c_busy_last == 0))
	   || ( busy_count == 99 ) )
	  begin

	 // Advance to next state in the FSM
	 if ( busy_count != 8'd20 ) begin
	    busy_count <= busy_count + 1;
	 end
	 	 
	 // Schedule I2C byte transfers
	 if ( busy_count != 99 ) begin
	    $display("dispatching I2C command, busy_count=", busy_count);
	 end
	 else
	   $display("Kick-starting I2C communications");	    
	 
	 case (busy_count)
	   8'd99: begin
	      // Send address and start read transaction by first writing the selected register number 
	      i2c_command_en <= 1;
	      i2c_rw <= 0;
	      i2c_addr <= ADDRESS;
	      i2c_wdata <= 8'd0; // Port 0 read address
	      busy_count <= 0;	      
	   end
	   8'd0: begin
	      // Read first byte (register 6)
	      i2c_command_en <= 1;	      
	      i2c_rw <= 1;
	     end	   

	   8'd1: begin
	      // Send address and start write transaction to select register 2
	      // except during start-up, when we initialise the other registers
	      i2c_command_en <= 1;
	      i2c_rw <= 0;
	      i2c_addr <= ADDRESS;
	      // Select the register pair to write to	      
	      i2c_wdata[7:3] <= 5'd0;     
	      i2c_wdata[2:1] <= reg_pair;
	      i2c_wdata[0] <= 1'b0;     
	     end
 	   8'd2: begin
	      // Write to register 2 : Power rail enables
	      // (or reg 4 (inversions of port 0) or reg 6 (ddr of port 0)
	      i2c_command_en <= 1;	      
	      i2c_rw <= 0;
	      case (reg_pair)
		2'b01: begin	       
		   i2c_wdata[7:6] <= 2'b00;	      
//		   i2c_wdata[5:0] <= counter[29:24];
		   case (counter[26:24])
		     3'b000: begin
			i2c_wdata[5:0] <= 6'b000000;
			gpio_0 <= 1'b0;
		     end
		     3'b001: i2c_wdata[5:0] <= 6'b000000;
		     3'b010: i2c_wdata[5:0] <= 6'b000000;
		     3'b011: i2c_wdata[5:0] <= 6'b000000;
		     3'b100: i2c_wdata[5:0] <= 6'b000000;
		     3'b101: i2c_wdata[5:0] <= 6'b000000;
		     3'b110: i2c_wdata[5:0] <= 6'b000000;
		     3'b111: begin 
			i2c_wdata[5:0] <= 6'b000000;
			gpio_0 <= 1'b1;
		     end
		   endcase
	        end
	        2'b10: i2c_wdata <= 8'h00; // port 0 inversions
		2'b11: i2c_wdata <= 8'b11000000; // port 0 DDR
	      endcase
	     end	   
 	   8'd3: begin
	      // Write to register 3 : Power rail enable in bit 5 for headphones amplifier
	      i2c_command_en <= 1;	      
	      i2c_rw <= 0;
	      case (reg_pair)
		2'b01: begin
		   i2c_wdata[7:6] <= 2'b00;	      
		   i2c_wdata[4:0] <= 5'b00000;
		   case (counter[26:24])
		     3'b110: i2c_wdata[5] <= 1'b0;
		     default: i2c_wdata[5] <= 1'b0;
		   endcase
		end
		2'b10: i2c_wdata <= 8'h00; // port 1 inversions
		2'b11: i2c_wdata <= 8'b11011111; // port 1 DDR
	      endcase
	     end	   
	   8'd4: begin
	      // Complete write transaction
	      i2c_command_en <= 0;
	      // Select the next register pair to write
	      case (reg_pair)
		2'b10: reg_pair <= 2'b11; // setup DDRs for ports after clearing inversions
		2'b11: reg_pair <= 2'b01; // then write port 0/1 outputs forever after
	      endcase // case reg_pair
	      
	   end
	 endcase // case (sensor_state)

      end // if i2c_busy
   end // always @ (posedge clk48)
   
   
endmodule
