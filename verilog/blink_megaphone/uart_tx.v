// File UART_TX_CTRL.vhdl translated with vhd2vl v3.0 VHDL to Verilog RTL translator
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

//--------------------------------------------------------------------------
//	UART_TX_CTRL.vhd -- UART Data Transfer Component
//--------------------------------------------------------------------------
// Author:  Sam Bobrowicz
//          Copyright 2011 Digilent, Inc.
//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------
//	This component may be used to transfer data over a UART device. It will
// serialize a byte of data and transmit it over a TXD line. The serialized
// data has the following characteristics:
//         *9600 Baud Rate
//         *8 data bits, LSB first
//         *1 stop bit
//         *no parity
//         				
// Port Descriptions:
//
//    SEND - Used to trigger a send operation. The upper layer logic should 
//           set this signal high for a single clock cycle to trigger a 
//           send. When this signal is set high DATA must be valid . Should 
//           not be asserted unless READY is high.
//    DATA - The parallel data to be sent. Must be valid the clock cycle
//           that SEND has gone high.
//    CLK  - A 100 MHz clock is expected
//   READY - This signal goes low once a send operation has begun and
//           remains low until it has completed and the module is ready to
//           send another byte.
// UART_TX - This signal should be routed to the appropriate TX pin of the 
//           external UART device.
//   
//--------------------------------------------------------------------------
//
//--------------------------------------------------------------------------
// Revision History:
//  08/08/2011(SamB): Created using Xilinx Tools 13.2
//--------------------------------------------------------------------------
// no timescale needed

module uart_tx(
input wire SEND,
input wire [23:0] BIT_TMR_MAX,
input wire [7:0] DATA,
input wire CLK,
output wire READY,
output wire UART_TX
);




parameter [1:0]
  RDY = 0,
  LOAD_BIT = 1,
  SEND_BIT = 2;

parameter BIT_INDEX_MAX = 10;  //Counter that keeps track of the number of clock cycles the current bit has been held stable over the
//UART TX line. It is used to signal when the ne
reg [23:0] bitTmr = 1'b0;  //combinatorial logic that goes high when bitTmr has counted to the proper value to ensure
//a 9600 baud rate
wire bitDone;  //Contains the index of the next bit in txData that needs to be transferred 
reg [31:0] bitIndex;  //a register that holds the current data being sent over the UART TX line
reg txBit = 1'b1;  //A register that contains the whole data packet to be sent, including start and stop bits. 
reg [9:0] txData;
reg [1:0] txState = RDY;

  //Next state logic
  always @(posedge CLK) begin
    case(txState)
    RDY : begin
      if((SEND == 1'b1)) begin
        txState <= LOAD_BIT;
      end
    end
    LOAD_BIT : begin
      txState <= SEND_BIT;
    end
    SEND_BIT : begin
      if((bitDone == 1'b1)) begin
        if((bitIndex == BIT_INDEX_MAX)) begin
          txState <= RDY;
        end
        else begin
          txState <= LOAD_BIT;
        end
      end
    end
    default : begin
      //should never be reached
      txState <= RDY;
    end
    endcase
  end

  always @(posedge CLK) begin
    if((txState == RDY)) begin
      bitTmr <= {24{1'b0}};
    end
    else begin
      if((bitDone == 1'b1)) begin
        bitTmr <= {24{1'b0}};
      end
      else begin
        bitTmr <= bitTmr + 1;
      end
    end
  end

  assign bitDone = (bitTmr == BIT_TMR_MAX) ? 1'b1 : 1'b0;
  always @(posedge CLK) begin
    if((txState == RDY)) begin
      bitIndex <= 0;
    end
    else if((txState == LOAD_BIT)) begin
      bitIndex <= bitIndex + 1;
    end
  end

  always @(posedge CLK) begin
    if((txState == RDY)) begin
      if((SEND == 1'b1)) begin
        txData <= {1'b1,DATA,1'b0};
        // report "UART_TX: Sending byte $" & to_hstring(DATA);
      end
    end
    else if((txState == LOAD_BIT)) begin
      txBit <= txData[0];
      txData <= {1'b1,txData[9:1]};
    end
  end

  assign UART_TX = txBit;
  assign READY = (txState == RDY) ? 1'b1 : 1'b0;

endmodule
