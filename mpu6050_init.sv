`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/12/2024 12:43:39 PM
// Design Name: 
// Module Name: mpu6050_init
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

module mpu6050_init(
    input logic clk,
    input logic rst,
    input logic start,
    output logic scl,
    inout logic sda,
    output logic done
);

    logic [7:0] mpu6050_address = 7'h68;  // MPU6050 I2C address (0x68)
    logic [7:0] pwr_mgmt_1 = 8'h6B;       // Power Management 1 register
    logic [7:0] pwr_mgmt_1_value = 8'h00; // Value to wake up MPU6050

    logic [7:0] write_data;                // Data to be written
    logic [7:0] device_addr;
    logic start_i2c;
    logic ack;
    logic done_i2c;

    i2c_master_1 i2c_master_inst (
        .clk(clk),
        .rst(rst),
        .start(start_i2c),
        .device_addr(device_addr),
        .write_data(write_data),
        .scl(scl),
        .sda(sda),
        .done(done_i2c),
        .ack(ack)
    );

    // State machine to initialize MPU6050
    typedef enum logic [1:0] {
        IDLE,
        WRITE_PWR_MGMT_1,
        DONE
    } state_t;

    state_t current_state, next_state;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    always_comb begin
        case (current_state)
            IDLE: begin
                if (start) begin
                    device_addr = {mpu6050_address, 1'b0};  // Write operation
                    write_data = pwr_mgmt_1_value;
                    start_i2c = 1;
                    next_state = WRITE_PWR_MGMT_1;
                end else begin
                    next_state = IDLE;
                end
            end

            WRITE_PWR_MGMT_1: begin
                if (done_i2c) begin
                    next_state = DONE;
                end else begin
                    next_state = WRITE_PWR_MGMT_1;
                end
            end

            DONE: begin
                done = 1;
                next_state = DONE;
            end

            default: next_state = IDLE;
        endcase
    end
endmodule
