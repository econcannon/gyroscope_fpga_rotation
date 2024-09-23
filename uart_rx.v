`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/12/2024 11:42:16 AM
// Design Name: 
// Module Name: uart_rx
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

module uart_rx #(
    parameter CLK_FREQ = 100_000_000,  // Default clock frequency (100 MHz)
    parameter BAUD_RATE = 115200       // Default baud rate
)(
    input logic clk,
    input logic rst,
    input logic rx,                    // UART receive line
    output logic [7:0] data_out,        // Received data
    output logic received               // Indicates if data has been received
);

    // Calculate bit period based on parameters
    localparam integer BIT_PERIOD = CLK_FREQ / BAUD_RATE;

    logic [15:0] bit_counter;
    logic [9:0] shift_reg;
    logic [3:0] bit_index;
    logic receiving;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            bit_counter <= 0;
            shift_reg <= 0;
            bit_index <= 0;
            receiving <= 0;
            received <= 0;
        end else if (!rx && !receiving) begin  // Detect start bit (rx goes low)
            receiving <= 1;
            bit_counter <= 0;
            bit_index <= 0;
        end else if (receiving) begin
            if (bit_counter == BIT_PERIOD - 1) begin
                bit_counter <= 0;
                shift_reg[bit_index] <= rx;      // Capture bits from RX line
                bit_index <= bit_index + 1;
                if (bit_index == 8) begin        // Byte received
                    received <= 1;
                    receiving <= 0;
                    data_out <= shift_reg[7:0];  // Output data byte
                end
            end else begin
                bit_counter <= bit_counter + 1;
            end
        end else begin
            received <= 0;
        end
    end
endmodule


