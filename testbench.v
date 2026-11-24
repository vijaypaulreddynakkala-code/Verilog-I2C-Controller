`timescale 1ns / 1ps

module i2c_master_tb;

    reg        clk;
    reg        rst_n;
    reg        start;
    reg  [6:0] slave_addr;
    reg  [7:0] data_in;
    wire       busy;
    wire       ack_error;
    wire       scl;
    wire       sda;

    reg        slave_ack_drive;

    // Pull-up resistor behavior + simulated slave ACK
    pullup(sda);
    assign sda = (slave_ack_drive) ? 1'b0 : 1'bz;

    i2c_master #(
        .CLK_DIV(4)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .start(start),
        .slave_addr(slave_addr),
        .data_in(data_in),
        .busy(busy),
        .ack_error(ack_error),
        .scl(scl),
        .sda(sda)
    );

    // 50 MHz Clock
    initial clk = 0;
    always #10 clk = ~clk;

    initial begin
        $dumpfile("i2c_wave.vcd");
        $dumpvars(0, i2c_master_tb);

        rst_n           = 0;
        start           = 0;
        slave_addr      = 7'h44; // SHT31 / Sensor address
        data_in         = 8'hA5;
        slave_ack_drive = 0;

        #50 rst_n = 1;
        #40 start = 1;
        #20 start = 0;

        // Simulate Slave ACK during ADDR_ACK and DATA_ACK states
        wait (dut.state == 3'd3); // ADDR_ACK
        slave_ack_drive = 1;
        wait (dut.state == 3'd4); // SEND_DATA
        slave_ack_drive = 0;

        wait (dut.state == 3'd5); // DATA_ACK
        slave_ack_drive = 1;
        wait (dut.state == 3'd6); // STOP_COND
        slave_ack_drive = 0;

        wait (!busy);
        #100;

        if (!ack_error)
            $display("PASS: I2C Write Transaction Completed with Valid ACKs!");
        else
            $display("FAIL: ACK Error Detected!");

        $finish;
    end
endmodule
