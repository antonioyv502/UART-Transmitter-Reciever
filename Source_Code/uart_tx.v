
//=============================================================================
// uart_tx.v - UART Transmitter (Parallel-to-Serial)
//
// Target: Altera MAX-10 (10M50DAF484C7G), DE10-Lite
//
// Frame Format:
//   [IDLE(1] [START(0)] [D0..D7] [PARITY(opt)] [STOP x1 or x2 (1)] [DONE]
//
// The module counts 16 x16-baud ticks per bit period, matching the
// oversampling ratio produced by baud_gen.v.
//
// Parity Modes:
//   EVEN parity = total number of 1s (data + parity) is even
//   ODD parity  = total number of 1s (data + parity) is odd
//
// FSM States:
//   S_IDLE   - Line high, waiting for tx_start_i
//   S_START  - Transmit start bit (logic 0) for 1 baud period 
//   S_DATA   - Transmit 8 data bits (LSB first)
//   S_PARITY - Transmit parity bit (when parity_en_i = 1)
//   S_STOP1  - Transmit first stop bit (logic 1)
//   S_STOP2  - Transmit second stop bit (when two_stop_i = 1)
//   S_DONE   - Transmission complete, return to IDLE
//=============================================================================

module uart_tx (
    clk_i,
    rst_ni,
    tick_x16_i,
    tx_start_i,
    tx_data_i,
    two_stop_i,
    parity_en_i,
    parity_type_i,
    tx_o,
    tx_busy_o
);

    //-------------------------------------------------------------------------
    // Input and Output Declarations
    //-------------------------------------------------------------------------

    input           clk_i;
    input           rst_ni;         // Active-low reset
    input           tick_x16_i;     // 16x baud tick from baud_gen
    input           tx_start_i;     // One-cycle pulse to begin transmission
    input  [7:0]    tx_data_i;      // 8-bit parallel data
    input           two_stop_i;     // 1 = 2 stop bits, 0 = 1 stop bit
    input           parity_en_i;    // 1 = Enabled, 0 = Disabled
    input           parity_type_i;  // 0 = EVEN, 1 = ODD

    output          tx_o;           // Serial UART output (idle = 1)
    output          tx_busy_o;      // 1 while transmitting

    reg             tx_o;
    reg             tx_busy_o;

    //-------------------------------------------------------------------------
    // FSM State Encoding
    //-------------------------------------------------------------------------

    localparam [2:0]
        S_IDLE   = 3'd0,
        S_START  = 3'd1,
        S_DATA   = 3'd2,
        S_PARITY = 3'd3,
        S_STOP1  = 3'd4,
        S_STOP2  = 3'd5,
        S_DONE   = 3'd6;

    localparam EVEN_PARITY_MODE = 1'b0; // the total # of 1's in the data are even
    localparam ODD_PARITY_MODE  = 1'b1; // the total # of 1's in the data are odd 

  
	reg [2:0] state;         // Current FSM state(8 possible states)
    reg [7:0] data_r;        // Shift register (LSB transmitted first)
	reg [3:0] tick_cnt_r;    // Counts 16 baud ticks (0 to 15 = 1 baud period)
	reg [2:0] bit_cnt_r;     // Counts 8 data bits (0 to 7), keeps track of the 8 data bits being transmitted 
    reg       parity_r;      // Computed parity bit
    reg       pre_tx_start;  // Previous tx_start_i value

    //-------------------------------------------------------------------------
    // UART Transmitter FSM
    //-------------------------------------------------------------------------

    always @(posedge clk_i or negedge rst_ni) begin
		if (!rst_ni) begin // asynchronous active-low reset 
            state        <= S_IDLE;
            tx_o         <= 1'b1;
            tx_busy_o    <= 1'b0;
            data_r       <= 8'd0;
            bit_cnt_r    <= 3'd0;
            tick_cnt_r   <= 4'd0;
            parity_r     <= 1'b0;
            pre_tx_start <= 1'b0; 
        end else begin
            // Store previous tx_start_i for rising-edge detection
            pre_tx_start <= tx_start_i;
			
            case (state)

                //-------------------------------------------------------------
                // IDLE: Wait for a transmission request
                //-------------------------------------------------------------
                S_IDLE: begin
                    tx_o      <= 1'b1; // Mark idle (line high)
                    tx_busy_o <= 1'b0; // 
                    // Detect rising edge of tx_start_i
					if (!pre_tx_start && tx_start_i) begin 
                        data_r      <= tx_data_i;
						parity_r    <= (^tx_data_i) ^ parity_type_i; // reduction XOR to produce a single bit 
                        bit_cnt_r   <= 3'd0;
                        tick_cnt_r  <= 4'd0;
                        tx_busy_o   <= 1'b1;  

                        state <= S_START; // next state is S_START
                    end
                end

                //-------------------------------------------------------------
                // START: Transmit start bit (logic 0)
                //-------------------------------------------------------------
                S_START: begin

                    tx_o <= 1'b0; // starts bit always equal to zero 

                    if (tick_x16_i == 1'b1) begin
                        if (tick_cnt_r == 4'd15) begin
                            tick_cnt_r <= 4'd0;
                            state      <= S_DATA; // next state is S_DATA
                        end else begin
                            tick_cnt_r <= tick_cnt_r + 4'd1;
                        end
                    end
                end

                //-------------------------------------------------------------
                // DATA: Transmit 8 data bits (LSB first)
                //-------------------------------------------------------------
                S_DATA: begin
                    // Transmit the current LSB
                    tx_o <= data_r[0];

                    if (tick_x16_i == 1'b1) begin
                        if (tick_cnt_r == 4'd15) begin
                            tick_cnt_r <= 4'd0;
                            // Shift right by 1 to make next data bit the new LSB
                            data_r <= data_r >> 1;
                            // Check if all 8 data bits have been transmitted
                            if (bit_cnt_r == 3'd7) begin
                                bit_cnt_r <= 3'd0;

                                // Determine whether parity is enabled
                                if (parity_en_i == 1'b1) begin
                                    state <= S_PARITY; // next state is S_PARRITY if parity_en_i is enabled 
                                end else begin
                                    state <= S_STOP1; // next state is S_STOP1 if parity_en_i is not enabled
                                end
                            end else begin
                                bit_cnt_r <= bit_cnt_r + 3'd1;
                            end
                        end else begin
                            tick_cnt_r <= tick_cnt_r + 4'd1;
                        end
                    end
                end

                //-------------------------------------------------------------
                // PARITY: Transmit optional parity bit
                //-------------------------------------------------------------
                S_PARITY: begin
                    tx_o <= parity_r;
                    if (tick_x16_i == 1'b1) begin
                        if (tick_cnt_r == 4'd15) begin
                            tick_cnt_r <= 4'd0;
                            state      <= S_STOP1; // next state is S_STOP1
                        end else begin
                            tick_cnt_r <= tick_cnt_r + 4'd1;
                        end
                    end
                end

                //-------------------------------------------------------------
                // STOP1: Transmit first stop bit (logic 1)
                //-------------------------------------------------------------
                S_STOP1: begin
                    tx_o <= 1'b1; // stop bit needs to be pulled high 
                    if (tick_x16_i == 1'b1) begin
                        if (tick_cnt_r == 4'd15) begin
                            tick_cnt_r <= 4'd0;
                            // Select one or two stop bits
							state <= (two_stop_i == 1'b1) ? S_STOP2 : S_DONE; // // if two_stop_i is high(1) next state is S_STOP2, otherwise next state is S_DONE
                        end else begin
                            tick_cnt_r <= tick_cnt_r + 4'd1;
                        end
                    end
                end

                //-------------------------------------------------------------
                // STOP2: Transmit second stop bit (logic 1)
                //-------------------------------------------------------------
                S_STOP2: begin
                    tx_o <= 1'b1; // stop bit needs to be pulled high
                    if (tick_x16_i == 1'b1) begin
                        if (tick_cnt_r == 4'd15) begin
                            tick_cnt_r <= 4'd0;
                            state      <= S_DONE; // next state is S_DONE
                        end else begin
                            tick_cnt_r <= tick_cnt_r + 4'd1;
                        end
                    end
                end

                //-------------------------------------------------------------
                // DONE: Transmission complete
                //-------------------------------------------------------------
                S_DONE: begin
                    tx_o      <= 1'b1; // 
                    tx_busy_o <= 1'b0; // 

                    state <= S_IDLE; // next state is S_IDLE
                end
				
                //-------------------------------------------------------------
                // Default State
                //-------------------------------------------------------------
                default: begin
                    state <= S_IDLE;
                end
            endcase
        end
    end
endmodule
