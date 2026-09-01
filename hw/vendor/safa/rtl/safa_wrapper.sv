module safa_wrapper #(
    parameter int unsigned FIFO_DEPTH = @ACCELERATOR_FIFO_DEPTH@,
    parameter int unsigned FIFO_ALMOST_FULL_MARGIN = @ACCELERATOR_FIFO_ALMOST_FULL_MARGIN@
) (
    input  logic clk_i,
    input  logic rst_ni,
    input  reg_pkg::reg_req_t reg_req_i,
    output reg_pkg::reg_rsp_t reg_rsp_o,
    input  fifo_pkg::fifo_req_t  hw_fifo_req_i,
    output fifo_pkg::fifo_resp_t hw_fifo_rsp_o,
    output logic                 hw_fifo_done_o,
    output logic                 safa_interrupt_o
);

    localparam int unsigned DATA_WIDTH = 32;
    localparam int unsigned FIFO_COUNT_WIDTH =
        (FIFO_DEPTH < 2) ? 1 : $clog2(FIFO_DEPTH + 1);
    localparam int unsigned ALMOST_FULL_THRESHOLD =
        FIFO_DEPTH - FIFO_ALMOST_FULL_MARGIN;

    localparam logic [11:0] REG_CONTROL          = 12'h000;
    localparam logic [11:0] REG_CONFIG           = 12'h004;
    localparam logic [11:0] REG_STATUS           = 12'h008;
    localparam logic [11:0] REG_INPUT_WORDS      = 12'h00C;
    localparam logic [11:0] REG_OUTPUT_WORDS     = 12'h010;
    localparam logic [11:0] REG_IN_FIFO_LEVEL    = 12'h014;
    localparam logic [11:0] REG_OUT_FIFO_LEVEL   = 12'h018;
    localparam logic [11:0] REG_IRQ_ENABLE       = 12'h01C;
    localparam logic [11:0] REG_IRQ_STATUS       = 12'h020;
    localparam logic [11:0] REG_ERROR_STATUS     = 12'h024;
    localparam logic [11:0] REG_INPUT_ACCEPTED   = 12'h028;
    localparam logic [11:0] REG_INPUT_CONSUMED   = 12'h02C;
    localparam logic [11:0] REG_OUTPUT_GENERATED = 12'h030;
    localparam logic [11:0] REG_OUTPUT_POPPED    = 12'h034;
    localparam logic [11:0] REG_VERSION          = 12'h038;
    localparam logic [11:0] REG_ACTIVE_CYCLES    = 12'h03C;
    localparam logic [11:0] REG_INPUT_STALLS     = 12'h040;
    localparam logic [11:0] REG_OUTPUT_STALLS    = 12'h044;
    localparam logic [11:0] REG_DMA_PUSH_STALLS  = 12'h048;
    localparam logic [11:0] REG_DMA_POP_STALLS   = 12'h04C;

    localparam logic [31:0] SAFA_VERSION = 32'h0002_0000;

    function automatic logic [31:0] merge_wstrb(
        input logic [31:0] old_value,
        input logic [31:0] new_value,
        input logic [3:0] wstrb
    );
        logic [31:0] value;
        int unsigned index;
        begin
            value = old_value;
            for (index = 0; index < 4; index++) begin
                if (wstrb[index]) value[index*8 +: 8] = new_value[index*8 +: 8];
            end
            merge_wstrb = value;
        end
    endfunction

    logic reg_write;
    logic start_cmd, stop_cmd, soft_reset_cmd;
    logic clear_done_cmd, clear_error_cmd, clear_aborted_cmd;
    logic irq_status_write, error_status_write;
    assign reg_write = reg_req_i.valid && reg_req_i.write;
    assign start_cmd = reg_write && reg_req_i.addr[11:0] == REG_CONTROL &&
                       reg_req_i.wstrb[0] && reg_req_i.wdata[0];
    assign stop_cmd = reg_write && reg_req_i.addr[11:0] == REG_CONTROL &&
                      reg_req_i.wstrb[0] && reg_req_i.wdata[1];
    assign soft_reset_cmd = reg_write && reg_req_i.addr[11:0] == REG_CONTROL &&
                            reg_req_i.wstrb[0] && reg_req_i.wdata[2];
    assign clear_done_cmd = reg_write && reg_req_i.addr[11:0] == REG_CONTROL &&
                            reg_req_i.wstrb[0] && reg_req_i.wdata[3];
    assign clear_error_cmd = reg_write && reg_req_i.addr[11:0] == REG_CONTROL &&
                             reg_req_i.wstrb[0] && reg_req_i.wdata[4];
    assign clear_aborted_cmd = reg_write && reg_req_i.addr[11:0] == REG_CONTROL &&
                               reg_req_i.wstrb[0] && reg_req_i.wdata[5];
    assign irq_status_write = reg_write && reg_req_i.addr[11:0] == REG_IRQ_STATUS;
    assign error_status_write = reg_write && reg_req_i.addr[11:0] == REG_ERROR_STATUS;

    logic auto_start_q;
    logic [31:0] input_words_cfg_q, output_words_cfg_q, irq_enable_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            auto_start_q <= 1'b0;
            input_words_cfg_q <= '0;
            output_words_cfg_q <= '0;
            irq_enable_q <= '0;
        end else begin
            if (reg_write && reg_req_i.addr[11:0] == REG_CONFIG && reg_req_i.wstrb[0])
                auto_start_q <= reg_req_i.wdata[0];
            if (reg_write && reg_req_i.addr[11:0] == REG_INPUT_WORDS)
                input_words_cfg_q <= merge_wstrb(input_words_cfg_q, reg_req_i.wdata, reg_req_i.wstrb);
            if (reg_write && reg_req_i.addr[11:0] == REG_OUTPUT_WORDS)
                output_words_cfg_q <= merge_wstrb(output_words_cfg_q, reg_req_i.wdata, reg_req_i.wstrb);
            if (reg_write && reg_req_i.addr[11:0] == REG_IRQ_ENABLE)
                irq_enable_q <= merge_wstrb(irq_enable_q, reg_req_i.wdata, reg_req_i.wstrb);
        end
    end

    logic [DATA_WIDTH-1:0] BUS_IN_dout, BUS_OUT_din;
    logic BUS_IN_empty_n, BUS_IN_read, BUS_OUT_full_n, BUS_OUT_write;
    logic ap_rst, ap_start, ap_done, ap_ready, ap_idle;

    typedef enum logic [2:0] {
        SAFA_IDLE, SAFA_START_WAIT, SAFA_RUNNING,
        SAFA_DRAINING, SAFA_DONE, SAFA_ABORT_RESET
    } safa_state_e;
    safa_state_e state_q, state_d;
    logic state_is_busy, state_can_start, transaction_active;
    logic auto_start_event, start_request, start_accepted;
    logic abort_request, abort_event, complete_event;
    logic out_fifo_empty, input_push_accepted;

    assign state_is_busy = state_q == SAFA_START_WAIT || state_q == SAFA_RUNNING ||
                           state_q == SAFA_DRAINING;
    assign transaction_active = state_is_busy;
    assign state_can_start = state_q == SAFA_IDLE || state_q == SAFA_DONE;
    assign abort_request = stop_cmd || soft_reset_cmd || hw_fifo_req_i.flush;
    assign start_request = start_cmd || auto_start_event;
    assign start_accepted = start_request && state_can_start && !abort_request;
    assign abort_event = (stop_cmd || hw_fifo_req_i.flush) && state_is_busy;
    assign complete_event = state_q == SAFA_DRAINING && out_fifo_empty &&
                            !BUS_OUT_write && !abort_request;

    always_comb begin
        state_d = state_q;
        unique case (state_q)
            SAFA_IDLE: if (abort_request) state_d = SAFA_ABORT_RESET;
                       else if (start_accepted) state_d = SAFA_START_WAIT;
            SAFA_START_WAIT: if (abort_request) state_d = SAFA_ABORT_RESET;
                             else if (ap_done) state_d = SAFA_DRAINING;
                             else if (ap_start && ap_ready) state_d = SAFA_RUNNING;
            SAFA_RUNNING: if (abort_request) state_d = SAFA_ABORT_RESET;
                          else if (ap_done) state_d = SAFA_DRAINING;
            SAFA_DRAINING: if (abort_request) state_d = SAFA_ABORT_RESET;
                           else if (complete_event) state_d = SAFA_DONE;
            SAFA_DONE: if (abort_request) state_d = SAFA_ABORT_RESET;
                       else if (start_accepted) state_d = SAFA_START_WAIT;
                       else if (clear_done_cmd) state_d = SAFA_IDLE;
            SAFA_ABORT_RESET: if (!abort_request) state_d = SAFA_IDLE;
            default: state_d = SAFA_ABORT_RESET;
        endcase
    end
    always_ff @(posedge clk_i or negedge rst_ni)
        if (!rst_ni) state_q <= SAFA_IDLE; else state_q <= state_d;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) ap_start <= 1'b0;
        else if (abort_request || state_q == SAFA_ABORT_RESET) ap_start <= 1'b0;
        else if (start_accepted) ap_start <= 1'b1;
        else if (ap_start && ap_ready) ap_start <= 1'b0;
    end

    logic hls_done_seen_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) hls_done_seen_q <= 1'b0;
        else if (abort_request || state_q == SAFA_ABORT_RESET || start_accepted)
            hls_done_seen_q <= 1'b0;
        else if (ap_done) hls_done_seen_q <= 1'b1;
    end

    logic fifo_reset;
    assign fifo_reset = !rst_ni || state_q == SAFA_ABORT_RESET ||
                        (state_q == SAFA_DONE && (start_accepted || clear_done_cmd));
    assign ap_rst = !rst_ni || state_q == SAFA_ABORT_RESET;

    logic [DATA_WIDTH-1:0] in_fifo_dout, out_fifo_dout;
    logic [FIFO_COUNT_WIDTH-1:0] in_fifo_size, out_fifo_size;
    logic in_fifo_full, in_fifo_empty, in_fifo_almost_full;
    logic in_fifo_wr_en, in_fifo_rd_en, input_blocked, input_read_accepted;
    logic out_fifo_full, out_fifo_wr_en, out_fifo_rd_en;
    logic output_write_accepted, output_pop_accepted;

    assign input_blocked = in_fifo_full || ap_done || state_q == SAFA_DRAINING ||
                           state_q == SAFA_DONE || state_q == SAFA_ABORT_RESET;
    assign in_fifo_wr_en = hw_fifo_req_i.push && !input_blocked;
    assign in_fifo_rd_en = BUS_IN_read && !in_fifo_empty && !abort_request &&
                           state_q != SAFA_ABORT_RESET;
    assign input_push_accepted = in_fifo_wr_en;
    assign input_read_accepted = in_fifo_rd_en;
    assign auto_start_event = auto_start_q && input_push_accepted && state_q == SAFA_IDLE;
    assign in_fifo_almost_full = in_fifo_size >= ALMOST_FULL_THRESHOLD;
    assign BUS_IN_dout = in_fifo_dout;
    assign BUS_IN_empty_n = !in_fifo_empty && !abort_request && state_q != SAFA_ABORT_RESET;

    sync_fifo_with_size_signal #(.DATA_WIDTH(DATA_WIDTH), .DEPTH(FIFO_DEPTH)) i_input_fifo (
        .clk(clk_i), .rst(fifo_reset), .wr_en(in_fifo_wr_en), .rd_en(in_fifo_rd_en),
        .din(hw_fifo_req_i.data), .dout(in_fifo_dout), .size(in_fifo_size),
        .full(in_fifo_full), .empty(in_fifo_empty)
    );

    assign out_fifo_wr_en = BUS_OUT_write && !out_fifo_full && !abort_request &&
                            state_q != SAFA_ABORT_RESET;
    assign out_fifo_rd_en = hw_fifo_req_i.pop && !out_fifo_empty && !abort_request &&
                            state_q != SAFA_ABORT_RESET;
    assign output_write_accepted = out_fifo_wr_en;
    assign output_pop_accepted = out_fifo_rd_en;
    assign BUS_OUT_full_n = !out_fifo_full && !abort_request && state_q != SAFA_ABORT_RESET;

    sync_fifo_with_size_signal #(.DATA_WIDTH(DATA_WIDTH), .DEPTH(FIFO_DEPTH)) i_output_fifo (
        .clk(clk_i), .rst(fifo_reset), .wr_en(out_fifo_wr_en), .rd_en(out_fifo_rd_en),
        .din(BUS_OUT_din), .dout(out_fifo_dout), .size(out_fifo_size),
        .full(out_fifo_full), .empty(out_fifo_empty)
    );

    always_comb begin
        hw_fifo_rsp_o = '0;
        hw_fifo_rsp_o.data = out_fifo_dout;
        hw_fifo_rsp_o.full = input_blocked;
        hw_fifo_rsp_o.alm_full = input_blocked || in_fifo_almost_full;
        hw_fifo_rsp_o.empty = out_fifo_empty || state_q == SAFA_ABORT_RESET;
    end

    logic [31:0] input_words_accepted_q, input_words_consumed_q;
    logic [31:0] output_words_generated_q, output_words_popped_q;
    logic [31:0] active_cycles_q, input_stall_cycles_q, output_stall_cycles_q;
    logic [31:0] dma_push_stall_cycles_q, dma_pop_stall_cycles_q;
    logic [31:0] input_consumed_effective, output_generated_effective;
    assign input_consumed_effective = input_words_consumed_q + input_read_accepted;
    assign output_generated_effective = output_words_generated_q + output_write_accepted;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni || state_q == SAFA_ABORT_RESET) begin
            input_words_accepted_q <= '0; input_words_consumed_q <= '0;
            output_words_generated_q <= '0; output_words_popped_q <= '0;
            active_cycles_q <= '0; input_stall_cycles_q <= '0;
            output_stall_cycles_q <= '0; dma_push_stall_cycles_q <= '0;
            dma_pop_stall_cycles_q <= '0;
        end else if (start_accepted) begin
            input_words_accepted_q <= state_q == SAFA_IDLE ?
                in_fifo_size + input_push_accepted : 32'd0;
            input_words_consumed_q <= '0; output_words_generated_q <= '0;
            output_words_popped_q <= '0; active_cycles_q <= '0;
            input_stall_cycles_q <= '0; output_stall_cycles_q <= '0;
            dma_push_stall_cycles_q <= '0; dma_pop_stall_cycles_q <= '0;
        end else if (transaction_active) begin
            if (input_push_accepted) input_words_accepted_q <= input_words_accepted_q + 1;
            if (input_read_accepted) input_words_consumed_q <= input_words_consumed_q + 1;
            if (output_write_accepted) output_words_generated_q <= output_words_generated_q + 1;
            if (output_pop_accepted) output_words_popped_q <= output_words_popped_q + 1;
            active_cycles_q <= active_cycles_q + 1;
            if (BUS_IN_read && in_fifo_empty) input_stall_cycles_q <= input_stall_cycles_q + 1;
            if (BUS_OUT_write && out_fifo_full) output_stall_cycles_q <= output_stall_cycles_q + 1;
            if (hw_fifo_req_i.push && input_blocked)
                dma_push_stall_cycles_q <= dma_push_stall_cycles_q + 1;
            if (hw_fifo_req_i.pop && out_fifo_empty)
                dma_pop_stall_cycles_q <= dma_pop_stall_cycles_q + 1;
        end
    end

    logic [31:0] new_error_events, error_status_q;
    always_comb begin
        new_error_events = '0;
        if (start_cmd && !state_can_start) new_error_events[0] = 1'b1;
        if (hw_fifo_req_i.push && input_blocked) new_error_events[1] = 1'b1;
        if (hw_fifo_req_i.pop && out_fifo_empty) new_error_events[2] = 1'b1;
        if (BUS_IN_read && in_fifo_empty) new_error_events[3] = 1'b1;
        if (BUS_OUT_write && out_fifo_full) new_error_events[4] = 1'b1;
        if (ap_done && state_q != SAFA_START_WAIT && state_q != SAFA_RUNNING &&
            state_q != SAFA_DRAINING) new_error_events[5] = 1'b1;
        if (complete_event && input_words_cfg_q != 0 &&
            input_consumed_effective != input_words_cfg_q) new_error_events[6] = 1'b1;
        if (complete_event && output_words_cfg_q != 0 &&
            output_generated_effective != output_words_cfg_q) new_error_events[7] = 1'b1;
        if (complete_event && !in_fifo_empty) new_error_events[8] = 1'b1;
    end
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) error_status_q <= '0;
        else if (soft_reset_cmd || clear_error_cmd) error_status_q <= new_error_events;
        else if (error_status_write)
            error_status_q <= (error_status_q &
                ~merge_wstrb('0, reg_req_i.wdata, reg_req_i.wstrb)) | new_error_events;
        else error_status_q <= error_status_q | new_error_events;
    end

    logic done_sticky_q, aborted_sticky_q;
    logic [31:0] irq_status_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni || soft_reset_cmd) begin
            done_sticky_q <= 1'b0; aborted_sticky_q <= 1'b0; irq_status_q <= '0;
        end else begin
            if (start_accepted || clear_done_cmd || abort_request) begin
                done_sticky_q <= 1'b0; irq_status_q[0] <= 1'b0;
            end
            if (start_accepted || clear_aborted_cmd) begin
                aborted_sticky_q <= 1'b0; irq_status_q[1] <= 1'b0;
            end
            if (clear_error_cmd) irq_status_q[2] <= 1'b0;
            if (irq_status_write) irq_status_q <= irq_status_q &
                ~merge_wstrb('0, reg_req_i.wdata, reg_req_i.wstrb);
            if (complete_event) begin done_sticky_q <= 1'b1; irq_status_q[0] <= 1'b1; end
            if (abort_event) begin aborted_sticky_q <= 1'b1; irq_status_q[1] <= 1'b1; end
            if (|new_error_events) irq_status_q[2] <= 1'b1;
        end
    end
    assign hw_fifo_done_o = done_sticky_q;
    assign safa_interrupt_o = |(irq_status_q[2:0] & irq_enable_q[2:0]);

    logic [31:0] status_value;
    always_comb begin
        status_value = '0;
        status_value[0] = state_q == SAFA_IDLE; status_value[1] = state_is_busy;
        status_value[2] = done_sticky_q; status_value[3] = aborted_sticky_q;
        status_value[4] = |error_status_q; status_value[5] = state_q == SAFA_START_WAIT;
        status_value[6] = state_q == SAFA_RUNNING; status_value[7] = state_q == SAFA_DRAINING;
        status_value[8] = ap_idle; status_value[9] = ap_ready; status_value[10] = ap_start;
        status_value[11] = ap_done; status_value[12] = in_fifo_empty;
        status_value[13] = in_fifo_full; status_value[14] = out_fifo_empty;
        status_value[15] = out_fifo_full; status_value[16] = auto_start_q;
        status_value[17] = hls_done_seen_q; status_value[26:24] = state_q;
    end

    always_comb begin
        reg_rsp_o = '0; reg_rsp_o.ready = 1'b1;
        if (reg_req_i.valid) begin
            unique case (reg_req_i.addr[11:0])
                REG_CONTROL: reg_rsp_o.rdata = '0;
                REG_CONFIG: reg_rsp_o.rdata = {31'd0, auto_start_q};
                REG_STATUS: reg_rsp_o.rdata = status_value;
                REG_INPUT_WORDS: reg_rsp_o.rdata = input_words_cfg_q;
                REG_OUTPUT_WORDS: reg_rsp_o.rdata = output_words_cfg_q;
                REG_IN_FIFO_LEVEL: reg_rsp_o.rdata = {{(32-FIFO_COUNT_WIDTH){1'b0}}, in_fifo_size};
                REG_OUT_FIFO_LEVEL: reg_rsp_o.rdata = {{(32-FIFO_COUNT_WIDTH){1'b0}}, out_fifo_size};
                REG_IRQ_ENABLE: reg_rsp_o.rdata = irq_enable_q;
                REG_IRQ_STATUS: reg_rsp_o.rdata = irq_status_q;
                REG_ERROR_STATUS: reg_rsp_o.rdata = error_status_q;
                REG_INPUT_ACCEPTED: reg_rsp_o.rdata = input_words_accepted_q;
                REG_INPUT_CONSUMED: reg_rsp_o.rdata = input_words_consumed_q;
                REG_OUTPUT_GENERATED: reg_rsp_o.rdata = output_words_generated_q;
                REG_OUTPUT_POPPED: reg_rsp_o.rdata = output_words_popped_q;
                REG_VERSION: reg_rsp_o.rdata = SAFA_VERSION;
                REG_ACTIVE_CYCLES: reg_rsp_o.rdata = active_cycles_q;
                REG_INPUT_STALLS: reg_rsp_o.rdata = input_stall_cycles_q;
                REG_OUTPUT_STALLS: reg_rsp_o.rdata = output_stall_cycles_q;
                REG_DMA_PUSH_STALLS: reg_rsp_o.rdata = dma_push_stall_cycles_q;
                REG_DMA_POP_STALLS: reg_rsp_o.rdata = dma_pop_stall_cycles_q;
                default: begin reg_rsp_o.rdata = '0; reg_rsp_o.error = 1'b1; end
            endcase
        end
    end

    @HLS_TOP_MODULE@ i_hls_top (
        .BUS_IN_dout(BUS_IN_dout), 
        .BUS_IN_empty_n(BUS_IN_empty_n),
        .BUS_IN_read(BUS_IN_read), 
        .BUS_OUT_din(BUS_OUT_din),
        .BUS_OUT_full_n(BUS_OUT_full_n), 
        .BUS_OUT_write(BUS_OUT_write),
        .ap_clk(clk_i), .ap_rst(ap_rst), 
        .ap_start(ap_start),
        .ap_done(ap_done), 
        .ap_ready(ap_ready), 
        .ap_idle(ap_idle)
    );

endmodule
