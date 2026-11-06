//=============================================================================
// uart_tx.v  -  UART Transmitter (Parallel-to-Serial)
//
// Target : Altera MAX-10 (10M50DAF484C7G), DE10-Lite
//
// Frame format:
//   [START(0)] [D0..D7] [PARITY(opt)] [STOP x1 or x2 (1)]
//
// The module counts 16 x16-baud ticks per bit period, matching the
// oversampling ratio produced by baud_gen.v.
//
// ODD parity bit = ~(^data[7:0])
//   -> total 1s in {data, parity} is always odd.
//
// State encoding:
//   S_IDLE   - line high, waiting for tx_start_i
//   S_START  - transmit start bit (logic 0) for 1 baud period
//   S_DATA   - transmit 8 data bits (LSB first)
//   S_PARITY - transmit odd parity bit (when parity_en_i = 1)
//   S_STOP1  - transmit first stop bit (logic 1)
//   S_STOP2  - transmit second stop bit (when two_stop_i = 1)
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

    input           clk_i;           
    input           rst_ni;         // active-low reset 
    input           tick_x16_i;     // 16x baud tick from baud_gen
    input           tx_start_i;     // One-cycle high pulse: latch data & begin TX
    input [7:0]     tx_data_i;      // 8-bit parallel data
    input           two_stop_i;     // 1 = 2 stop bits
    input           parity_en_i;    // 1 = Parity Enabled
    input           parity_type_i;  // 0 = EVEN, 1 = ODD
    
	  output          tx_o;           // Serial UART output (idle = 1)
    output          tx_busy_o;      // 1 while frame is being transmitted

    reg tx_o;
    reg tx_busy_o;

    //---------------------------------------------------------
    // State encoding (one-hot friendly, 3-bit binary used here)
    //---------------------------------------------------------
    localparam [2:0]
                S_IDLE    = 3'd0,
                S_START   = 3'd1,
                S_DATA    = 3'd2,
                S_PARITY  = 3'd3,
                S_STOP1   = 3'd4,
                S_STOP2   = 3'd5,
                S_DONE    = 3'd6;

    localparam  EVEN_PARITY_MODE = 1'b0,
                ODD_PARITY_MODE  = 1'b1;

    reg [2:0]   state;        // state register - 8 possible states
    reg [7:0]   data_r;       // Shift register (LSB transmitted first)
    reg [3:0]   tick_cnt_r;   // Sub-baud tick counter (0..15 = 1 baud period)
    reg [2:0]   bit_cnt_r;    // Data bit counter (0..7)
    reg         parity_r;     // Computed odd parity bit    

    reg         pre_tx_start;
    
    //---------------------------------------------------------
    // Transmitter state machine
    //---------------------------------------------------------
    always @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            state        <= S_IDLE;
            tx_o         <= 1'b1;
            tx_busy_o    <= 1'b0;
            data_r       <= 8'd0;
            bit_cnt_r    <= 3'd0;
            tick_cnt_r   <= 4'd0;
            parity_r     <= 1'b0;
            pre_tx_start <= 1'b0;  
        end else begin
            pre_tx_start <= tx_start_i;  // store previous state
            

            case (state)

                //-----------------------------------------------------
                S_IDLE: begin
                    tx_o      <= 1'b1;      // Mark idle (line high)
                    tx_busy_o <= 1'b0;
                    if (!pre_tx_start && tx_start_i) begin             //detect rising-edge on tx_start_i signal
                        data_r   <= tx_data_i;
                        parity_r <= (parity_type_i == ODD_PARITY_MODE) ? 1'b1: 1'b0;
                                                                         
                        bit_cnt_r  <= 3'd0;
                        tx_busy_o  <= 1'b1;
                        tick_cnt_r <= 4'd0;
                        
                        state <= S_START;
                    end 
                end

                //-----------------------------------------------------
                S_START: begin
                    tx_o <= 1'b0; //start bit always equal to 0
                    if (tick_x16_i == 1'b1) begin 
                        if(tick_cnt_r == 4'd15) begin 
                            //reset tick_cnt_r to zero
                            //jump to next state
                            tick_cnt_r <= 4'd0;
                            state <= S_DATA;
                        end else begin 
                            //increase tick_cnt by 1
                            tick_cnt_r <= tick_cnt_r + 4'd1;
                        end 
                    end 
                end

                //-----------------------------------------------------
                S_DATA: begin
                    //set tx_o to d[0]
                    tx_o <= data_r[0];
                    if (tick_x16_i == 1'b1) begin 
                        if (tick_cnt_r == 4'd15) begin 
									          tick_cnt_r <= 4'd0;
                            //shift data to right by 1 bit
									          data_r <= (data_r >> 1);
                            if (bit_cnt_r == 3'd7) begin 
                                //clear bit_cnt to zero
                                //jump to S_PARITY if parity_en_i enabled
                                //otherwise, jump to S_STOP1
										            bit_cnt_r <= 3'd0;
										            if(parity_en_i) begin 
												           state <= S_PARITY;
										            end else begin 
												           state <= S_STOP1;
										            end 
                            end else begin 
                                bit_cnt_r <= bit_cnt_r + 3'd1;
                            end
                        end else begin 
                            //increase tick_cnt
									          tick_cnt_r <= tick_cnt_r + 4'd1;
                        end
                    end
                end

                //-----------------------------------------------------
                S_PARITY: begin
                    //send parity to tx
                    //if tick_cnt_r == 15, jump to S_STOP1
						  
						        tx_o <= parity_r;
						  
						        if (tick_x16_i) begin
                        if (tick_cnt_r == 4'd15) begin
                            tick_cnt_r <= 4'd0;
                            state <= S_STOP1;
                        end else begin
                            tick_cnt_r <= tick_cnt_r + 4'd1;
                        end
                    end
                end

                //-----------------------------------------------------
                S_STOP1: begin
                    //sent 1 to tx
                    //if tick_cnt_r == 15
                    //  --> check stop_bits, if stop_bit == 0 -> jump to S_DONE
                    //                       otherwise, jump to S_STOP2
					 	        tx_o <= 1'b1;
                
                    if (tick_x16_i) begin
                        if (tick_cnt_r == 4'd15) begin
                            tick_cnt_r <= 4'd0;
                            state <= two_stop_i ? S_STOP2 : S_DONE;
                        end else begin
                            tick_cnt_r <= tick_cnt_r + 4'd1;
                        end
                    end
                end

                //-----------------------------------------------------
                S_STOP2: begin
                    //sent 1 to tx
                    //if tick_cnt_r == 15, jump to S_DONE
						        tx_o <= 1'b1;
						  
						        if (tick_x16_i) begin
                        if (tick_cnt_r == 4'd15) begin
                            tick_cnt_r <= 4'd0;
                            state <= S_DONE;
                        end else begin
                            tick_cnt_r <= tick_cnt_r + 4'd1;
                        end
                    end
                end

                S_DONE: begin
                    // set tx to 1
                    //reset tx busy to 0
                    //jump to S_IDLE
						        tx_o      <= 1'b1;
						        tx_busy_o <= 1'b0;
						  
						       state <= S_IDLE;
                end
                //-----------------------------------------------------
                default: begin
						      state <= S_IDLE;
					      end
            endcase
        end
    end
endmodule
