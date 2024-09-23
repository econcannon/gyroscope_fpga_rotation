`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/12/2024 11:53:10 AM
// Design Name: 
// Module Name: uart_tx
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module uart_tx #(
    parameter CLK_FREQ = 100_000_000,  // Default clock frequency (100 MHz)
    parameter BAUD_RATE = 115200       // Default baud rate
)(
    input logic clk,
    input logic rst,
    input logic [7:0] data_in,         // Data to be transmitted
    input logic send,                  // Trigger to send data
    output logic tx,                   // UART transmission line
    output logic ready                 // Ready signal
);

    // Calculate bit period based on parameters
    localparam integer BIT_PERIOD = CLK_FREQ / BAUD_RATE;

    logic [15:0] bit_counter;
    logic [9:0] shift_reg;             // Holds start bit, 8 data bits, stop bit
    logic [3:0] bit_index;
    logic sending;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            bit_counter <= 0;
            shift_reg <= 0;
            bit_index <= 0;
            sending <= 0;
            tx <= 1;                   // UART idle state is high
            ready <= 1;                // Ready to send new data
        end else if (send && ready) begin
            shift_reg <= {1'b1, data_in, 1'b0};  // Add start and stop bits
            bit_index <= 0;
            sending <= 1;
            ready <= 0;
        end else if (sending) begin
            if (bit_counter == BIT_PERIOD - 1) begin
                bit_counter <= 0;
                tx <= shift_reg[bit_index];      // Transmit bits
                bit_index <= bit_index + 1;
                if (bit_index == 9) begin        // Finish sending frame
                    sending <= 0;
                    ready <= 1;                  // Ready for new data
                    tx <= 1;                     // Idle state
                end
            end else begin
                bit_counter <= bit_counter + 1;
            end
        end
    end
endmodule
