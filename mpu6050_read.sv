`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/12/2024 12:54:59 PM
// Design Name: 
// Module Name: mpu6050_read
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


module mpu6050_read(
    input logic clk,
    input logic rst,
    input logic start,
    output logic scl,
    inout logic sda,
    output logic done,
    output logic [15:0] gyro_x,  // Gyroscope X-axis data
    output logic [15:0] gyro_y,  // Gyroscope Y-axis data
    output logic [15:0] gyro_z   // Gyroscope Z-axis data
);

    logic [6:0] mpu6050_address = 7'h68;  // MPU6050 I2C address
    logic [7:0] read_data;
    logic [7:0] device_addr;
    logic start_i2c, read_req;
    logic ack, done_i2c;
    logic [7:0] reg_addr;

    // I2C master instantiation
    i2c_master_0 i2c_master_inst (
        .clk(clk),
        .rst(rst),
        .start(start_i2c),
        .device_addr(mpu6050_address),
        .write_data(reg_addr),  // Register address
        .read_req(read_req),
        .scl(scl),
        .sda(sda),
        .done(done_i2c),
        .read_data(read_data),
        .ack(ack)
    );

    typedef enum logic [3:0] {
        IDLE,
        WRITE_GYRO_XH_ADDR,
        READ_GYRO_XH,
        WRITE_GYRO_XL_ADDR,
        READ_GYRO_XL,
        WRITE_GYRO_YH_ADDR,
        READ_GYRO_YH,
        WRITE_GYRO_YL_ADDR,
        READ_GYRO_YL,
        WRITE_GYRO_ZH_ADDR,
        READ_GYRO_ZH,
        WRITE_GYRO_ZL_ADDR,
        READ_GYRO_ZL,
        DONE
    } state_t;

    state_t current_state, next_state;
    logic [7:0] gyro_x_l, gyro_x_h, gyro_y_h, gyro_y_l, gyro_z_h, gyro_z_l;

    // State machine for reading gyroscope data
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

    always_comb begin
        // Default signals
        next_state = current_state;
        start_i2c = 0;
        read_req = 0;
        reg_addr = 8'h00;

        case (current_state)
            IDLE: begin
                if (start) begin
                    reg_addr = 8'h43;  // Address of Gyro X high byte
                    start_i2c = 1;  // Start I2C write operation
                    next_state = WRITE_GYRO_XH_ADDR;
                end
            end

            // Write Gyro X high byte address
            WRITE_GYRO_XH_ADDR: begin
                if (done_i2c) begin
                    start_i2c = 1;  // Start I2C read operation
                    read_req = 1;   // Read the data
                    next_state = READ_GYRO_XH;
                end
            end

            // Read Gyro X high byte data
            READ_GYRO_XH: begin
                if (done_i2c) begin
                    gyro_x[15:8] = read_data;  // Store high byte
                    reg_addr = 8'h44;  // Address of Gyro X low byte
                    start_i2c = 1;  // Start I2C write operation
                    next_state = WRITE_GYRO_XL_ADDR;
                end
            end

            // Write Gyro X low byte address
            WRITE_GYRO_XL_ADDR: begin
                if (done_i2c) begin
                    start_i2c = 1;  // Start I2C read operation
                    read_req = 1;
                    next_state = READ_GYRO_XL;
                end
            end

            // Read Gyro X low byte data
            READ_GYRO_XL: begin
                if (done_i2c) begin
                    gyro_x[7:0] = read_data;  // Store low byte
                    reg_addr = 8'h45;  // Address of Gyro Y high byte
                    start_i2c = 1;  // Start I2C write operation
                    next_state = WRITE_GYRO_YH_ADDR;
                end
            end

            // Continue similar for Y and Z axes...

            DONE: begin
                done = 1;  // Indicate that all reads are done
                next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end
endmodule


