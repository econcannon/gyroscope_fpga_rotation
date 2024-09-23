`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/12/2024 12:26:39 PM
// Design Name: 
// Module Name: I2C_master
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

module i2c_master #(
    parameter CLK_FREQ = 100_000_000,  // System clock frequency (100 MHz)
    parameter I2C_FREQ = 100_000       // I2C clock frequency (100 kHz)
)(
    input logic clk,                  // System clock
    input logic rst,                  // Reset signal
    input logic start,                // Start transaction
    input logic [6:0] device_addr,    // I2C device address
    input logic [7:0] write_data,     // Data to be written to the slave (register address)
    input logic read_req,             // Read request signal
    output logic scl,                 // I2C clock
    inout logic sda,                  // I2C data line
    output logic done,                // Signals that transaction is complete
    output logic [7:0] read_data,     // Data read from the slave
    output logic ack                  // Acknowledge signal from slave
);

    // Local parameters for I2C bit timing
    localparam integer HALF_PERIOD = CLK_FREQ / (2 * I2C_FREQ);

    // State machine states
    typedef enum logic [3:0] {
        IDLE, START, ADDR, WRITE, READ, STOP, WAIT_ACK, DONE
    } state_t;

    state_t current_state, next_state;

    logic [15:0] clk_count;           // Counter for generating I2C clock
    logic scl_en;                     // Enable clock for SCL toggling
    logic sda_out, sda_en;            // SDA output and enable control
    logic [7:0] shift_reg;            // Shift register for data transmission
    logic [3:0] bit_counter;          // Counts the bits for each byte transfer

    assign scl = scl_en ? clk_count[15] : 1'b1;  // SCL toggles based on counter
    assign sda = sda_en ? sda_out : 1'bz;        // SDA is tri-stated when not transmitting

    // SCL clock generation
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            clk_count <= 0;
        end else if (scl_en) begin
            if (clk_count == HALF_PERIOD - 1) begin
                clk_count <= 0;        // Reset counter after half period for clock
            end else begin
                clk_count <= clk_count + 1;
            end
        end
    end

    // State machine
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    always_comb begin
        next_state = current_state;  // Default state transition
        sda_en = 1'b0;
        scl_en = 1'b0;
        case (current_state)
            IDLE: begin
                if (start) begin
                    next_state = START;
                end
            end
            START: begin
                sda_en = 1'b1;      // SDA goes low for start condition
                scl_en = 1'b1;      // Enable SCL clock
                done = 1'b0;
                next_state = ADDR;
            end
            ADDR: begin
                if (bit_counter < 7) begin
                    sda_en = 1'b1;    // Send address bits
                    sda_out = device_addr[6 - bit_counter];  // Transmit address MSB first
                    scl_en = 1'b1;
                    if (scl == 1'b1) begin  // On SCL high, shift to next bit
                        bit_counter = bit_counter + 1;
                    end
                end else begin
                    next_state = WAIT_ACK;  // Move to ACK wait after address
                end
            end
            WAIT_ACK: begin
                sda_en = 1'b0;       // Release SDA for ACK
                scl_en = 1'b1;
                if (scl == 1'b1) begin  // Check ACK bit when SCL is high
                    ack = ~sda;      // ACK is low from the slave (active low)
                    next_state = (read_req) ? READ : WRITE;
                end
            end
            WRITE: begin
                if (bit_counter < 8) begin
                    sda_en = 1'b1;
                    sda_out = write_data[7 - bit_counter];  // Send data byte (register address)
                    scl_en = 1'b1;
                    if (scl == 1'b1) begin  // On SCL high, shift next bit
                        bit_counter = bit_counter + 1;
                    end
                end else begin
                    next_state = STOP;  // Transition to STOP condition after writing register address
                end
            end
            READ: begin
                if (bit_counter < 8) begin
                    sda_en = 1'b0;       // Slave drives the line
                    scl_en = 1'b1;
                    if (scl == 1'b1) begin  // On SCL high, read data bit
                        read_data[7 - bit_counter] = sda;
                        bit_counter = bit_counter + 1;
                    end
                end else begin
                    next_state = STOP;  // End with STOP condition
                end
            end
            STOP: begin
                sda_en = 1'b1;
                sda_out = 1'b0;         // Set SDA low
                scl_en = 1'b1;
                if (scl == 1'b1) begin  // Wait for SCL high
                    sda_out = 1'b1;    // Stop condition (SDA goes high while SCL is high)
                    next_state = DONE;
                end
            end
            DONE: begin
                done = 1'b1;  // Transaction complete
                next_state = IDLE;
            end
        endcase
    end
endmodule

