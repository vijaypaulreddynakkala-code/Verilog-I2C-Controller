# Verilog-I2C-Controller
Synthesizable Verilog RTL implementation and self-checking testbench for an Inter-Integrated Circuit (I2C) Master Controller supporting standard 7-bit addressing and single-byte write transactions.
# Verilog I2C Master Controller

A synthesizable **Verilog HDL** RTL design and verification testbench for an **I2C (Inter-Integrated Circuit) Master Controller**. Implemented using a deterministic Finite State Machine (FSM) with open-drain `SDA`/`SCL` bus control, 7-bit slave addressing, ACK/NACK detection, and `START`/`STOP` condition generation.

## FSM Architecture
[ IDLE ] ---> [ START ] ---> [ SLAVE_ADDR (7b+W) ] ---> [ ADDR_ACK ]
|
[ IDLE ] <--- [ STOP  ] <--- [ DATA_ACK          ] <--- [ WRITE_DATA (8b) ]
## Module Interface (`i2c_master.v`)
| Port | Direction | Width | Description |
| :--- | :--- | :--- | :--- |
| `clk` | Input | 1 | System clock |
| `rst_n` | Input | 1 | Active-low synchronous reset |
| `start` | Input | 1 | Trigger pulse to initiate I2C write transfer |
| `slave_addr` | Input | 7 | 7-bit target I2C slave address |
| `data_in` | Input | 8 | Data byte to transmit to slave |
| `busy` | Output | 1 | High while transaction is in progress |
| `ack_error` | Output | 1 | High if slave fails to assert ACK (`SDA=0`) |
| `scl` | Output | 1 | Serial Clock line |
| `sda` | Inout | 1 | Open-drain bidirectional Serial Data line |

## Simulation Instructions
Using **Icarus Verilog (`iverilog`)** and **GTKWave**:
```bash
### **RTL Code (`rtl/i2c_master.v`)**
```verilog
`timescale 1ns / 1ps

module i2c_master #(
    parameter CLK_DIV = 4  // Quarter-bit clock divider
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       start,
    input  wire [6:0] slave_addr,
    input  wire [7:0] data_in,
    output reg        busy,
    output reg        ack_error,
    output reg        scl,
    inout  wire       sda
);

    // FSM States
    localparam [2:0] IDLE       = 3'd0,
                     START_COND = 3'd1,
                     SEND_ADDR  = 3'd2,
                     ADDR_ACK   = 3'd3,
                     SEND_DATA  = 3'd4,
                     DATA_ACK   = 3'd5,
                     STOP_COND  = 3'd6;

    reg [2:0] state;
    reg [7:0] shift_reg;
    reg [2:0] bit_cnt;
    reg [1:0] phase;
    reg [$clog2(CLK_DIV)-1:0] clk_cnt;
    reg       sda_out;
    reg       sda_oe; // 1 = drive low, 0 = release (high-Z)

    // Open-drain SDA buffer
    assign sda = (sda_oe && !sda_out) ? 1'b0 : 1'bz;

    wire tick = (clk_cnt == CLK_DIV - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            clk_cnt <= 0;
        else if (state == IDLE)
            clk_cnt <= 0;
        else if (tick)
            clk_cnt <= 0;
        else
            clk_cnt <= clk_cnt + 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            busy      <= 1'b0;
            ack_error <= 1'b0;
            scl       <= 1'b1;
            sda_out   <= 1'b1;
            sda_oe    <= 1'b0;
            phase     <= 2'd0;
            bit_cnt   <= 3'd7;
            shift_reg <= 8'd0;
        end else begin
            case (state)
                IDLE: begin
                    scl       <= 1'b1;
                    sda_out   <= 1'b1;
                    sda_oe    <= 1'b0;
                    busy      <= 1'b0;
                    phase     <= 2'd0;
                    if (start) begin
                        busy      <= 1'b1;
                        ack_error <= 1'b0;
                        shift_reg <= {slave_addr, 1'b0}; // 7-bit addr + Write(0)
                        state     <= START_COND;
                    end
                end

                START_COND: if (tick) begin
                    case (phase)
                        2'd0: begin sda_oe <= 1'b1; sda_out <= 1'b0; scl <= 1'b1; phase <= 2'd1; end
                        2'd1: begin scl <= 1'b0; bit_cnt <= 3'd7; phase <= 2'd0; state <= SEND_ADDR; end
                    endcase
                end

                SEND_ADDR: if (tick) begin
                    case (phase)
                        2'd0: begin scl <= 1'b0; sda_oe <= 1'b1; sda_out <= shift_reg[bit_cnt]; phase <= 2'd1; end
                        2'd1: begin scl <= 1'b1; phase <= 2'd2; end
                        2'd2: begin scl <= 1'b1; phase <= 2'd3; end
                        2'd3: begin
                            scl   <= 1'b0;
                            phase <= 2'd0;
                            if (bit_cnt == 0)
                                state <= ADDR_ACK;
                            else
                                bit_cnt <= bit_cnt - 1'b1;
                        end
                    endcase
                end

                ADDR_ACK: if (tick) begin
                    case (phase)
                        2'd0: begin scl <= 1'b0; sda_oe <= 1'b0; phase <= 2'd1; end // Release SDA for slave ACK
                        2'd1: begin scl <= 1'b1; phase <= 2'd2; end
                        2'd2: begin
                            if (sda !== 1'b0) ack_error <= 1'b1; // Sample ACK
                            phase <= 2'd3;
                        end
                        2'd3: begin
                            scl       <= 1'b0;
                            phase     <= 2'd0;
                            shift_reg <= data_in;
                            bit_cnt   <= 3'd7;
                            state     <= SEND_DATA;
                        end
                    endcase
                end

                SEND_DATA: if (tick) begin
                    case (phase)
                        2'd0: begin scl <= 1'b0; sda_oe <= 1'b1; sda_out <= shift_reg[bit_cnt]; phase <= 2'd1; end
                        2'd1: begin scl <= 1'b1; phase <= 2'd2; end
                        2'd2: begin scl <= 1'b1; phase <= 2'd3; end
                        2'd3: begin
                            scl   <= 1'b0;
                            phase <= 2'd0;
                            if (bit_cnt == 0)
                                state <= DATA_ACK;
                            else
                                bit_cnt <= bit_cnt - 1'b1;
                        end
                    endcase
                end

                DATA_ACK: if (tick) begin
                    case (phase)
                        2'd0: begin scl <= 1'b0; sda_oe <= 1'b0; phase <= 2'd1; end
                        2'd1: begin scl <= 1'b1; phase <= 2'd2; end
                        2'd2: begin
                            if (sda !== 1'b0) ack_error <= 1'b1;
                            phase <= 2'd3;
                        end
                        2'd3: begin scl <= 1'b0; phase <= 2'd0; state <= STOP_COND; end
                    endcase
                end

                STOP_COND: if (tick) begin
                    case (phase)
                        2'd0: begin scl <= 1'b0; sda_oe <= 1'b1; sda_out <= 1'b0; phase <= 2'd1; end
                        2'd1: begin scl <= 1'b1; phase <= 2'd2; end
                        2'd2: begin sda_oe <= 1'b0; sda_out <= 1'b1; phase <= 2'd0; state <= IDLE; end
                    endcase
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule
iverilog -o i2c_sim rtl/i2c_master.v tb/i2c_master_tb.v
vvp i2c_sim
gtkwave i2c_wave.vcd
