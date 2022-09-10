// File uart_rx.vhdl translated with vhd2vl v3.0 VHDL to Verilog RTL translator
// vhd2vl settings:
//  * Verilog Module Declaration Style: 2001

// vhd2vl is Free (libre) Software:
//   Copyright (C) 2001 Vincenzo Liguori - Ocean Logic Pty Ltd
//     http://www.ocean-logic.com
//   Modifications Copyright (C) 2006 Mark Gonzales - PMC Sierra Inc
//   Modifications (C) 2010 Shankar Giri
//   Modifications Copyright (C) 2002-2017 Larry Doolittle
//     http://doolittle.icarus.com/~larry/vhd2vl/
//   Modifications (C) 2017 Rodrigo A. Melo
//
//   vhd2vl comes with ABSOLUTELY NO WARRANTY.  Always check the resulting
//   Verilog for correctness, ideally with a formal verification tool.
//
//   You are welcome to redistribute vhd2vl under certain conditions.
//   See the license (GPLv2) file included with the source for details.

// The result of translation follows.  Its copyright status should be
// considered unchanged from the original VHDL.

// no timescale needed

module uart_rx(
input wire clk,
input wire [23:0] bit_rate_divisor,
input wire UART_RX,
output reg [7:0] data,
output reg data_ready,
input wire data_acknowledge
);


// Timer for the above
reg [23:0] bit_timer = 1'b0;
reg [31:0] bit_position = 0;
reg [9:0] rx_data;
parameter [1:0]
  Idle = 0,
  WaitingForMidBit = 1,
  WaitingForNextBit = 2,
  WaitForRise = 3;

reg [1:0] rx_state = Idle;
reg [3:0] uart_rx_debounced = 1'b1;
reg uart_rx_bit = 1'b1;
reg data_ready_internal = 1'b0;
reg rx_gone_high = 1'b0;


  // behavioural
  always @(posedge clk) begin : P1
  // purpose: based on last 8 samples of uart_rx, decide if the average signal is a 1 or a 0

    uart_rx_debounced <= {uart_rx_debounced[2:0],UART_RX};
    if(uart_rx_debounced == 4'h0 && uart_rx_bit == 1'b1) begin
      uart_rx_bit <= 1'b0;
    end
    if(uart_rx_debounced == 4'hF && uart_rx_bit == 1'b0) begin
      uart_rx_bit <= 1'b1;
    end
    if(uart_rx_debounced == 4'hF) begin
      rx_gone_high <= 1'b1;
    end
    // Update bit clock
    if((bit_timer) < (bit_rate_divisor)) begin
      bit_timer <= bit_timer + 1;
    end
    else begin
      bit_timer <= {24{1'b0}};
    end
    // Look for start of first bit
    // XXX Should debounce this!
    if(rx_state == Idle && uart_rx_bit == 1'b0 && rx_gone_high == 1'b1) begin
      // Start receiving next byte
      //        report "UART"&name&": zeroing bit_timer";
      bit_timer <= {24{1'b0}};
      bit_position <= 0;
      rx_state <= WaitingForMidBit;
    end
    // Check for data_acknowledge before potentially reasserting data_ready
    // so that we can't miss characters
    if(data_acknowledge == 1'b1 && data_ready_internal == 1'b1) begin
      data_ready <= 1'b0;
      data_ready_internal <= 1'b0;
    end
    // Sample bit in the middle of the frame
    if(rx_state == WaitingForMidBit && bit_timer == {1'b0,bit_rate_divisor[23:1]}) begin
      //        report "UART"&name&": reached mid bit point, bit = " & integer'image(bit_position) severity note;
      // Reached mid bit
      rx_data[bit_position] <= uart_rx_bit;
      if(bit_position < 9) begin
        // More bits to get
        bit_position <= bit_position + 1;
        rx_state <= WaitingForNextBit;
      end
      else begin
        // This was the last bit
        data <= rx_data[8:1];
        data_ready <= 1'b1;
        data_ready_internal <= 1'b1;
        bit_timer <= 1;
        rx_state <= WaitForRise;
      end
    end
    if(bit_timer == 0 && rx_state == WaitingForNextBit) begin
      rx_state <= WaitingForMidBit;
    end
    // Wait for most of a bit after receiving a byte before going back
    // to idle state
    if((bit_timer == 0 || uart_rx_bit == 1'b1) && rx_state == WaitForRise) begin
      //        report "UART"&name&": Cancelling reception in WaitForRise";
      rx_state <= Idle;
      // Don't keep receiving $00 if UART_RX stays low.
      rx_gone_high <= 1'b0;
    end
  end


endmodule
