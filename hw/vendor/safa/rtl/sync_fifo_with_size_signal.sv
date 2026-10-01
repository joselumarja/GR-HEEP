// Copyright 2026 Universidad de Castilla-La Mancha.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 2.0 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-2.0. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// File: sync_fifo_with_size_signal.sv
// Author(s):
//   Jose Luis Mira Serrano
// Date: 01/10/2026
// Description: Parameterized fifo with size counter signal

module sync_fifo_with_size_signal #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 16
)(
    input  wire                 clk,
    input  wire                 rst,       // Synchronous reset
    input  wire                 wr_en,     // Write enable
    input  wire                 rd_en,     // Read enable
    input  wire [DATA_WIDTH-1:0] din,      // Data in
    output wire  [DATA_WIDTH-1:0] dout,     // Data out
    output wire  [ADDR_WIDTH:0]  size,      // Number elements in fifo
    output wire                 full,
    output wire                 empty
);

    localparam int ADDR_WIDTH = $clog2(DEPTH);
    localparam logic [ADDR_WIDTH-1:0] LAST_PTR = ADDR_WIDTH'(DEPTH - 1);

    // Memory to store FIFO data
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // Write and read pointers
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;

    reg [ADDR_WIDTH:0] fifo_count;
    
    // Signal logic
    assign full  = (fifo_count == DEPTH);
    assign empty = (fifo_count == 0);
    assign size = fifo_count;
    
    //read logic
    assign dout = mem[rd_ptr];

    // Write logic
    always_ff @(posedge clk) begin
        if (!rst && wr_en && !full) begin
            mem[wr_ptr] <= din;
        end
    end
    
    // Pointer logic
    always_ff @(posedge clk) begin
        if (rst) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            
        end else begin
        
            if(wr_en && !full) begin
                if(wr_ptr == LAST_PTR)
                    wr_ptr <= 0;
                else
                    wr_ptr <= wr_ptr + 1;
            end
            
            if(rd_en && !empty) begin
                if(rd_ptr == LAST_PTR)
                    rd_ptr <= 0;
                else
                    rd_ptr <= rd_ptr + 1;
            end
        end
    end
    
    
    // Counter logic
    always_ff @(posedge clk) begin
        if (rst) begin
            fifo_count <= 0;
        end else begin
            case ({wr_en && !full, rd_en && !empty})
                2'b10: begin
                    fifo_count <= fifo_count + 1; // Solo escritura
                end
                2'b01: begin 
                    fifo_count <= fifo_count - 1; // Solo lectura
                end
                default: begin 
                    fifo_count <= fifo_count;   // Sin operaciones o Lectura y escritura simultanea
                end
            endcase
        end
    end

endmodule
