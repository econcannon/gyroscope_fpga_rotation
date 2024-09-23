`timescale 1ns / 1ps

module top_mpu6050_rotation_with_uart
    #(parameter MAX_VERTICES = 100)
    (
    input logic clk,
    input logic rst,
    input logic rx,               // UART RX pin
    output logic tx,              // UART TX pin for responses
    output logic scl,             // I2C clock for MPU6050
    inout logic sda,              // I2C data for MPU6050
    output logic done             // System completion signal
);

    // Internal signals for MPU6050
    logic start_init, start_read;
    logic init_done, read_done;
    logic [15:0] gyro_x, gyro_y, gyro_z;  // Gyroscope data from MPU6050

    // Internal signals for UART
    logic [7:0] uart_data;
    logic uart_received;
    logic uart_send;
    logic [7:0] uart_data_to_send;
    logic uart_ready_to_send;

    // Internal storage for vertices
    logic signed [31:0] vertices[MAX_VERTICES-1:0][2];  // Storage for vertices (x, y, z)
    logic [7:0] vertex_count;
    logic [7:0] vertex_index;
    logic [1:0] coord_index;
    logic [7:0] expected_vertices;
    logic [7:0] send_index;
    logic [1:0] byte_count;

    // Instantiate MPU6050 Initialization Module
    mpu6050_init_0 mpu6050_init_inst (
        .clk(clk),
        .rst(rst),
        .start(start_init),
        .scl(scl),
        .sda(sda),
        .done(init_done)
    );

    // Instantiate MPU6050 Read Module
    mpu6050_read_0 mpu6050_read_inst (
        .clk(clk),
        .rst(rst),
        .start(start_read),
        .scl(scl),
        .sda(sda),
        .gyro_x(gyro_x),
        .gyro_y(gyro_y),
        .gyro_z(gyro_z),
        .done(read_done)
    );

    // Instantiate UART Receiver
    uart_rx_0 uart_rx_inst (
        .clk(clk),
        .rst(rst),
        .rx(rx),
        .data_out(uart_data),
        .received(uart_received)
    );

    // Instantiate UART Transmitter
    uart_tx_0 uart_tx_inst (
        .clk(clk),
        .rst(rst),
        .data_in(uart_data_to_send),
        .send(uart_send),
        .tx(tx),
        .ready(uart_ready_to_send)
    );

    // UART State Machine
    typedef enum logic [2:0] {
        WAIT_HEADER,
        RECEIVE_VERTEX_COUNT,
        RECEIVE_VERTICES,
        WAIT_FOR_REQUEST,
        SEND_GYRO_DATA,
        SEND_VERTICES
    } uart_state_t;

    uart_state_t uart_state;

    // State machine logic
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            uart_state <= WAIT_HEADER;
            vertex_count <= 0;
            vertex_index <= 0;
            coord_index <= 0;
            byte_count <= 0;
            uart_send <= 0;
            send_index <= 0;
        end else if (uart_received) begin
            case (uart_state)
                WAIT_HEADER: begin
                    if (uart_data == 8'hAA) begin  // Detect header byte (0xAA)
                        uart_state <= RECEIVE_VERTEX_COUNT;
                    end
                end

                RECEIVE_VERTEX_COUNT: begin
                    expected_vertices <= uart_data;
                    vertex_count <= 0;
                    uart_state <= RECEIVE_VERTICES;
                end

                RECEIVE_VERTICES: begin
                    if (vertex_count < expected_vertices) begin
                        vertices[vertex_count][coord_index][8 * byte_count +: 8] <= uart_data;
                        byte_count <= byte_count + 1;
                        if (byte_count == 3) begin
                            byte_count <= 0;
                            coord_index <= coord_index + 1;
                            if (coord_index == 2) begin
                                coord_index <= 0;
                                vertex_count <= vertex_count + 1;
                            end
                        end
                    end else begin
                        uart_state <= WAIT_FOR_REQUEST;
                    end
                end

                WAIT_FOR_REQUEST: begin
                    if (uart_data == 8'hBB) begin  // Request to send data (0xBB)
                        send_index <= 0;
                        uart_state <= SEND_GYRO_DATA;
                    end
                end

                SEND_GYRO_DATA: begin
                    // Send Gyroscope data (X, Y, Z axis)
                    if (send_index < 6) begin
                        case (send_index)
                            0: uart_data_to_send <= gyro_x[15:8];  // Gyro X high byte
                            1: uart_data_to_send <= gyro_x[7:0];   // Gyro X low byte
                            2: uart_data_to_send <= gyro_y[15:8];  // Gyro Y high byte
                            3: uart_data_to_send <= gyro_y[7:0];   // Gyro Y low byte
                            4: uart_data_to_send <= gyro_z[15:8];  // Gyro Z high byte
                            5: uart_data_to_send <= gyro_z[7:0];   // Gyro Z low byte
                        endcase
                        uart_send <= 1;
                        if (uart_ready_to_send) begin
                            send_index <= send_index + 1;
                        end
                    end else begin
                        send_index <= 0;
                        uart_state <= SEND_VERTICES;  // After gyro data, send vertices
                    end
                end

                SEND_VERTICES: begin
                    if (send_index < expected_vertices * 3 * 4) begin
                        uart_data_to_send <= vertices[send_index / 12][(send_index / 4) % 3][8 * (send_index % 4) +: 8];
                        uart_send <= 1;
                        if (uart_ready_to_send) begin
                            send_index <= send_index + 1;
                        end
                    end else begin
                        uart_state <= WAIT_HEADER;  // Return to waiting for header
                    end
                end
            endcase
        end
    end
endmodule
