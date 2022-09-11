// File ../../../mega65-core/src/vhdl/i2c_master.vhdl translated with vhd2vl v3.0 VHDL to Verilog RTL translator
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

//------------------------------------------------------------------------------
//
//   FileName:         i2c_master.vhd
//   Dependencies:     none
//   Design Software:  Quartus II 64-bit Version 13.1 Build 162 SJ Full Version
//
//   HDL CODE IS PROVIDED "AS IS."  DIGI-KEY EXPRESSLY DISCLAIMS ANY
//   WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//   PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL DIGI-KEY
//   BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
//   DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
//   PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
//   BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
//   ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
//
//   Version History
//   Version 1.0 11/01/2012 Scott Larson
//     Initial Public Release
//   Version 2.0 06/20/2014 Scott Larson
//     Added ability to interface with different slaves in the same transaction
//     Corrected ack_error bug where ack_error went 'Z' instead of '1' on error
//     Corrected timing of when ack_error signal clears
//   Version 2.1 10/21/2014 Scott Larson
//     Replaced gated clock with clock enable
//     Adjusted timing of SCL during start and stop conditions
//   Version 2.2 02/05/2015 Scott Larson
//     Corrected small SDA glitch introduced in version 2.1
// 
//------------------------------------------------------------------------------
// no timescale needed

module i2c_master(
input wire clk,
input wire reset_n,
input wire ena,
input wire [6:0] addr,
input wire rw,
input wire [7:0] data_wr,
output reg busy,
output reg [7:0] data_rd,
output reg ack_error,
inout wire sda,
inout wire scl,
input wire swap,
input wire debug_scl,
input wire debug_sda
);

parameter [31:0] input_clk=40500000;
parameter [31:0] bus_clk=400000;
//speed the i2c bus (scl) will run at in Hz
//system clock
//active low reset
//latch in command
//address of target slave
//'0' is write, '1' is read
//data to write to slave
//indicates transaction in progress
//data read from slave
//flag if improper acknowledge from slave
//serial data output of i2c bus
//serial clock output of i2c bus
// Debug inputs that allow us to pull the lines low to test



parameter divider = (input_clk / bus_clk) / 4;  //number of clocks in 1/4 cycle of scl
parameter divider_minus_1 = divider - 1;
parameter divider2_minus_1 = divider * 2 - 1;
parameter divider2 = divider * 2;
parameter divider3_minus_1 = divider * 3 - 1;
parameter divider3 = divider * 3;
parameter [3:0]
  ready = 0,
  start = 1,
  command = 2,
  slv_ack1 = 3,
  wr = 4,
  rd = 5,
  slv_ack2 = 6,
  mstr_ack = 7,
  stop = 8;
  //needed states
reg [3:0] state = ready;  //state machine
reg data_clk;  //data clock for sda
reg data_clk_prev;  //data clock during previous system clock
reg scl_clk;  //constantly running internal scl
reg scl_ena = 1'b0;  //enables internal scl to output
reg sda_int = 1'b1;  //internal sda
reg sda_ena_n;  //enables internal sda to output
reg [7:0] addr_rw;  //latched in address and read/write
reg [7:0] data_tx;  //latched in data to write to slave
reg [7:0] data_rx;  //data received from slave
reg [31:0] bit_cnt = 7;  //tracks bit number in transaction
reg stretch = 1'b0;  //identifies if slave is stretching scl
reg ack_error_int = 1'b0;  //local copy of ack_error for checking

initial begin
      busy <= 1'b0;
end
   
   
  //generate the timing for the bus clock (scl_clk) and the data clock (data_clk)
  always @(posedge clk) begin : P1
    reg [31:0] count;
  //timing for clock generation

    if((reset_n == 1'b0)) begin
      //reset asserted
      stretch <= 1'b0;
      count = 0;
    end else begin
      data_clk_prev <= data_clk;
      //store previous value of data clock
      if((count == (divider * 4 - 1))) begin
        //end of timing cycle
        count = 0;
        //reset timer
      end
      else if((stretch == 1'b0)) begin
        //clock stretching from slave not detected
        count = count + 1;
        //continue clock generation timing
      end
      if(count >= 0 && count <= divider_minus_1) begin
        //first 1/4 cycle of clocking
        scl_clk <= 1'b0;
        data_clk <= 1'b0;
      end
      else if(count >= divider && count <= divider2_minus_1) begin
        //second 1/4 cycle of clocking
        scl_clk <= 1'b0;
        data_clk <= 1'b1;
      end
      else if(count >= divider2 && count <= divider3_minus_1) begin
        //third 1/4 cycle of clocking
        scl_clk <= 1'b1;
        //release scl
        if((scl == 1'b0)) begin
          //detect if slave is stretching clock
          stretch <= 1'b1;
        end
        else begin
          stretch <= 1'b0;
        end
        data_clk <= 1'b1;
      end
      else begin
        //last 1/4 cycle of clocking
        scl_clk <= 1'b1;
        data_clk <= 1'b0;
      end
    end
  end

  //state machine and writing to sda during scl low (data_clk rising edge)
  always @(posedge clk) begin
     
    if((reset_n == 1'b0)) begin
       $display("i2c_master RESET");
       
      //reset asserted
      state <= ready;
      //return to initial state
      busy <= 1'b1;
      //indicate not available
      scl_ena <= 1'b0;
      //sets scl high impedance
      sda_int <= 1'b1;
      //sets sda high impedance
      ack_error <= 1'b0;
      //clear acknowledge error flag
      ack_error_int <= 1'b0;
      //clear acknowledge error flag
      bit_cnt <= 7;
      //restarts data bit counter
      data_rd <= 8'b00000000;
      //clear data read port
    end else begin
      if((data_clk == 1'b1 && data_clk_prev == 1'b0)) begin
        //data clock rising edge
        case(state)
        ready : begin
          //idle state
          if((ena == 1'b1)) begin
            //transaction requested
             $display($time,": Accepting job: addr=$%x",addr,", rw= ",rw);	     
             busy <= 1'b1;
            //flag busy
            addr_rw <= {addr,rw};
            //collect requested slave address and command
            data_tx <= data_wr;
            //collect requested data to write
            state <= start;
            //go to start bit
          end
          else begin
            //remain idle
            busy <= 1'b0;
            //unflag busy
            state <= ready;
            //remain idle
          end
        end
        start : begin
          //start bit of transaction
          //            report "sending start for transaction";
          busy <= 1'b1;
          //resume busy if continuous mode
          //            report "sending command bit " & integer'image(bit_cnt) & " = " & std_logic'image(addr_rw(bit_cnt));
          sda_int <= addr_rw[bit_cnt];
          //set first address bit to bus
          state <= command;
          //go to command
        end
        command : begin
          //address and command byte of transaction
          if((bit_cnt == 0)) begin
            //command transmit finished
            sda_int <= 1'b1;
            //release sda for slave acknowledge
            bit_cnt <= 7;
            //reset bit counter for "byte" states
            state <= slv_ack1;
            //go to slave acknowledge (command)
          end
          else begin
            //next clock cycle of command state
            //              report "sending command bit " & integer'image(bit_cnt-1) & " = " & std_logic'image(addr_rw(bit_cnt-1));
            bit_cnt <= bit_cnt - 1;
            //keep track of transaction bits
            sda_int <= addr_rw[bit_cnt - 1];
            //write address/command bit to bus
            state <= command;
            //continue with command
          end
        end
        slv_ack1 : begin
          //slave acknowledge bit (command)
          if((addr_rw[0] == 1'b0)) begin
            //write command
            sda_int <= data_tx[bit_cnt];
            //write first bit of data
             $display("switching to wr following command. Writing $%02x",data_tx);
            state <= wr;
            //go to write byte
          end
          else begin
            //read command
            //              report "switching to rd following command $" & to_hstring(addr_rw);
            sda_int <= 1'b1;
            //release sda from incoming data
            state <= rd;
            //go to read byte
          end
        end
        wr : begin
          //write byte of transaction
           //            report "writing data bit " & integer'image(bit_cnt) & " of $" & to_hstring(data_tx) & " as " & std_logic'image(sda_int);
          busy <= 1'b1;
          //resume busy if continuous mode
          if((bit_cnt == 0)) begin
             $display("Finished writing byte $%02x",data_tx);
	     
            //write byte transmit finished
            sda_int <= 1'b1;
            //release sda for slave acknowledge
            bit_cnt <= 7;
            //reset bit counter for "byte" states
            state <= slv_ack2;
            //go to slave acknowledge (write)
          end
          else begin
            //next clock cycle of write state
            bit_cnt <= bit_cnt - 1;
            //keep track of transaction bits
            sda_int <= data_tx[bit_cnt - 1];
            //write next bit to bus
            state <= wr;
            //continue writing
          end
        end
        rd : begin
          //read byte of transaction
          //            report "reading data bit " & integer'image(bit_cnt);
          busy <= 1'b1;
          //resume busy if continuous mode
          if((bit_cnt == 0)) begin
            //read byte receive finished
            if((ena == 1'b1 && addr_rw == {addr,rw})) begin
              //continuing with another read at same address
              sda_int <= 1'b0;
              //acknowledge the byte has been received
            end
            else begin
              //stopping or continuing with a write
              sda_int <= 1'b1;
              //send a no-acknowledge (before stop or repeated start)
            end
            bit_cnt <= 7;
            //reset bit counter for "byte" states
            data_rd <= data_rx;
            //output received data
            //              report "Read byte $" & to_hstring(data_rx);
            state <= mstr_ack;
            //go to master acknowledge
          end
          else begin
            //next clock cycle of read state
            bit_cnt <= bit_cnt - 1;
            //keep track of transaction bits
            state <= rd;
            //continue reading
          end
        end
        slv_ack2 : begin
          //slave acknowledge bit (write)
          if((ena == 1'b1)) begin
            //continue transaction
            busy <= 1'b0;
            //continue is accepted
            addr_rw <= {addr,rw};
            //collect requested slave address and command
            data_tx <= data_wr;
            //collect requested data to write
            //              report "Writing byte $" & to_hstring(data_wr) & " via slave_ack2";
            if(addr_rw == {addr,rw}) begin
              //continue transaction with
              //another write
              sda_int <= data_wr[bit_cnt];
              //write first bit of data
              $display("re-trigging byte write of $%02x, because ena is still high, and addr is unchanged at $%02x",
		       data_wr,
		       addr_rw);
              state <= wr;
              //go to write byte
            end
            else begin
              //continue transaction with a read or new slave
              //                report "repeating start, because target differs";
              state <= start;
              //go to repeated start
            end
          end
          else begin
            //complete transaction
            //              report "stopping";
            state <= stop;
            //go to stop bit
          end
        end
        mstr_ack : begin
          //master acknowledge bit after a read
          if((ena == 1'b1)) begin
            //continue transaction
            busy <= 1'b0;
            //continue is accepted and data received is available on bus
            addr_rw <= {addr,rw};
            //collect requested slave address and command
            data_tx <= data_wr;
            //collect requested data to write
            if(addr_rw == {addr,rw}) begin
              //continue transaction with another
              //read (or write PGS)
              sda_int <= 1'b1;
              //release sda from incoming data
              state <= rd;
              //go to read byte
            end
            else begin
              //continue transaction with a write or new slave
              state <= start;
              //repeated start
              //                report "Repeating start";
            end
          end
          else begin
            //complete transaction
            state <= stop;
            //go to stop bit
            //              report "Stopping";
          end
        end
        stop : begin
          //stop bit of transaction
          busy <= 1'b0;
          //unflag busy
          state <= ready;
          //go to idle state
        end
        endcase
      end
      else if((data_clk == 1'b0 && data_clk_prev == 1'b1)) begin
        //data clock falling edge
        case(state)
        start : begin
          if((scl_ena == 1'b0)) begin
            //starting new transaction
            scl_ena <= 1'b1;
            //enable scl output
            ack_error <= 1'b0;
            //reset acknowledge error output
            ack_error_int <= 1'b0;
            //reset acknowledge error output
          end
        end
        slv_ack1 : begin
          //receiving slave acknowledge (command)
          if((sda != 1'b0 || ack_error_int == 1'b1)) begin
            //no-acknowledge or previous no-acknowledge
            ack_error <= 1'b1;
            //set error output if no-acknowledge
            ack_error_int <= 1'b1;
            //set error output if no-acknowledge
          end
        end
        rd : begin
          //receiving slave data
          data_rx[bit_cnt] <= sda;
          //receive current slave data bit
        end
        slv_ack2 : begin
          //receiving slave acknowledge (write)
          if((sda != 1'b0 || ack_error_int == 1'b1)) begin
            //no-acknowledge or previous no-acknowledge
            ack_error <= 1'b1;
            //set error output if no-acknowledge
            ack_error_int <= 1'b1;
            //set error output if no-acknowledge
          end
        end
        stop : begin
          scl_ena <= 1'b0;
          //disable scl
        end
        default : begin
        end
        endcase
      end
    end
  end

  //set sda output
  always @(*) begin
    case(state)
      start : sda_ena_n <= data_clk_prev;
  //generate start condition
      stop : sda_ena_n <=  ~data_clk_prev;
  //generate stop condition
      default : begin
	 sda_ena_n <= sda_int;	 
      end
    endcase // case (state)

  end

     //Propagate internal sda/scl signals to scl and sda outputs
     assign sda = (((swap == 1'b1) && (scl_ena == 1'b1 && scl_clk == 1'b0)) || ((swap == 1'b0) && (sda_ena_n == 1'b0)) || (debug_sda == 1'b1)) ? 1'b0 : 1'bZ;
     assign scl = (((swap == 1'b0) && (scl_ena == 1'b1 && scl_clk == 1'b0)) || ((swap == 1'b1) && (sda_ena_n == 1'b0)) || (debug_scl == 1'b1)) ? 1'b0 : 1'bZ;
   
endmodule
