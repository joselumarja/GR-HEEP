module traffic_generator (
    input  logic               clk_i,
    input  logic               rst_ni,
    input  reg_pkg::reg_req_t  reg_req_i,
    output reg_pkg::reg_rsp_t  reg_rsp_o,
    output obi_pkg::obi_req_t  obi_req_o,
    input  obi_pkg::obi_resp_t obi_resp_i,
    output logic               interrupt_o
);

  localparam logic [11:0] RegControl             = 12'h000;
  localparam logic [11:0] RegConfig              = 12'h004;
  localparam logic [11:0] RegStatus              = 12'h008;
  localparam logic [11:0] RegBaseAddr            = 12'h00c;
  localparam logic [11:0] RegAddressMask         = 12'h010;
  localparam logic [11:0] RegStride              = 12'h014;
  localparam logic [11:0] RegSeed                = 12'h018;
  localparam logic [11:0] RegWriteData           = 12'h01c;
  localparam logic [11:0] RegByteEnable          = 12'h020;
  localparam logic [11:0] RegInjectionRate       = 12'h024;
  localparam logic [11:0] RegPeriod              = 12'h028;
  localparam logic [11:0] RegBurstLength         = 12'h02c;
  localparam logic [11:0] RegIdleLength          = 12'h030;
  localparam logic [11:0] RegDurationLimit       = 12'h034;
  localparam logic [11:0] RegIrqEnable           = 12'h038;
  localparam logic [11:0] RegIrqStatus           = 12'h03c;
  localparam logic [11:0] RegElapsedCycles       = 12'h040;
  localparam logic [11:0] RegRequests            = 12'h044;
  localparam logic [11:0] RegGrants              = 12'h048;
  localparam logic [11:0] RegCompleted           = 12'h04c;
  localparam logic [11:0] RegReads               = 12'h050;
  localparam logic [11:0] RegWrites              = 12'h054;
  localparam logic [11:0] RegWaitGrantCycles     = 12'h058;
  localparam logic [11:0] RegWaitResponseCycles  = 12'h05c;
  localparam logic [11:0] RegLatencySum          = 12'h060;
  localparam logic [11:0] RegLatencyMax          = 12'h064;
  localparam logic [11:0] RegMissedOpportunities = 12'h068;
  localparam logic [11:0] RegLastAddress         = 12'h06c;
  localparam logic [11:0] RegPrngState           = 12'h070;
  localparam logic [11:0] RegErrorStatus         = 12'h074;
  localparam logic [11:0] RegVersion             = 12'h078;
  localparam logic [11:0] RegLastReadData        = 12'h07c;
  localparam logic [11:0] RegTemporalPrngState   = 12'h080;

  localparam logic [31:0] Version = 32'h0001_0000;

  localparam logic [2:0] AddrFixed      = 3'd0;
  localparam logic [2:0] AddrSequential = 3'd1;
  localparam logic [2:0] AddrStride     = 3'd2;
  localparam logic [2:0] AddrUniform    = 3'd3;
  localparam logic [2:0] AddrGaussian   = 3'd4;

  localparam logic [2:0] TemporalSaturated = 3'd0;
  localparam logic [2:0] TemporalPeriodic  = 3'd1;
  localparam logic [2:0] TemporalBernoulli = 3'd2;
  localparam logic [2:0] TemporalBurst     = 3'd3;

  localparam logic [1:0] DurationCycles       = 2'd0;
  localparam logic [1:0] DurationTransactions = 2'd1;
  localparam logic [1:0] DurationReserved     = 2'd2;
  localparam logic [1:0] DurationInfinite     = 2'd3;

  localparam logic [1:0] RwRead      = 2'd0;
  localparam logic [1:0] RwWrite     = 2'd1;
  localparam logic [1:0] RwAlternate = 2'd2;
  localparam logic [1:0] RwRandom    = 2'd3;

  localparam logic [1:0] WdataFixed     = 2'd0;
  localparam logic [1:0] WdataAddress   = 2'd1;
  localparam logic [1:0] WdataRandom    = 2'd2;
  localparam logic [1:0] WdataIncrement = 2'd3;

  typedef enum logic [1:0] {
    StateIdle,
    StateRunning,
    StateDraining,
    StateDone
  } state_e;

  function automatic logic [31:0] merge_wstrb(
      input logic [31:0] old_value,
      input logic [31:0] new_value,
      input logic [3:0]  wstrb
  );
    logic [31:0] value;
    value = old_value;
    for (int unsigned i = 0; i < 4; i++) begin
      if (wstrb[i]) value[i*8+:8] = new_value[i*8+:8];
    end
    return value;
  endfunction

  function automatic logic [31:0] lfsr_next(input logic [31:0] value);
    logic feedback;
    feedback = value[31] ^ value[21] ^ value[1] ^ value[0];
    return {value[30:0], feedback};
  endfunction

  function automatic logic [31:0] sat_inc(input logic [31:0] value);
    return (&value) ? value : value + 32'd1;
  endfunction

  function automatic logic [31:0] sat_add(
      input logic [31:0] value,
      input logic [31:0] increment
  );
    logic [32:0] sum;
    sum = {1'b0, value} + {1'b0, increment};
    return sum[32] ? 32'hffff_ffff : sum[31:0];
  endfunction

  // Four uniform byte samples form an Irwin-Hall approximation centred in the region.
  function automatic logic [31:0] gaussian_offset(
      input logic [31:0] random_value,
      input logic [31:0] mask
  );
    logic [9:0]  sample_sum;
    /* verilator lint_off UNUSEDSIGNAL */
    logic [41:0] scaled;
    /* verilator lint_on UNUSEDSIGNAL */
    sample_sum = {2'b0, random_value[7:0]} + {2'b0, random_value[15:8]} +
                 {2'b0, random_value[23:16]} + {2'b0, random_value[31:24]};
    scaled = sample_sum * mask;
    return scaled[41:10] & mask & 32'hffff_fffc;
  endfunction

  logic reg_write;
  logic start_cmd, stop_cmd, soft_reset_request, soft_reset_accepted;
  logic clear_done_cmd, clear_error_cmd;
  logic irq_status_write, error_status_write;

  assign reg_write = reg_req_i.valid && reg_req_i.write;
  assign start_cmd = reg_write && reg_req_i.addr[11:0] == RegControl &&
                     reg_req_i.wstrb[0] && reg_req_i.wdata[0];
  assign stop_cmd = reg_write && reg_req_i.addr[11:0] == RegControl &&
                    reg_req_i.wstrb[0] && reg_req_i.wdata[1];
  assign soft_reset_request = reg_write && reg_req_i.addr[11:0] == RegControl &&
                              reg_req_i.wstrb[0] && reg_req_i.wdata[2];
  assign soft_reset_accepted = soft_reset_request &&
                               (state_q == StateIdle || state_q == StateDone);
  assign clear_done_cmd = reg_write && reg_req_i.addr[11:0] == RegControl &&
                          reg_req_i.wstrb[0] && reg_req_i.wdata[3];
  assign clear_error_cmd = reg_write && reg_req_i.addr[11:0] == RegControl &&
                           reg_req_i.wstrb[0] && reg_req_i.wdata[4];
  assign irq_status_write = reg_write && reg_req_i.addr[11:0] == RegIrqStatus;
  assign error_status_write = reg_write && reg_req_i.addr[11:0] == RegErrorStatus;

  state_e state_q;
  logic [31:0] config_q;
  logic [31:0] base_addr_q, address_mask_q, stride_q, seed_q, write_data_q;
  logic [31:0] byte_enable_q, injection_rate_q, period_q;
  logic [31:0] burst_length_q, idle_length_q, duration_limit_q, irq_enable_q;

  logic configuration_write;
  always_comb begin
    configuration_write = 1'b0;
    unique case (reg_req_i.addr[11:0])
      RegConfig, RegBaseAddr, RegAddressMask, RegStride, RegSeed,
      RegWriteData, RegByteEnable, RegInjectionRate, RegPeriod,
      RegBurstLength, RegIdleLength, RegDurationLimit: configuration_write = reg_write;
      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni || soft_reset_accepted) begin
      config_q         <= '0;
      base_addr_q      <= '0;
      address_mask_q   <= '0;
      stride_q         <= 32'd4;
      seed_q           <= 32'd1;
      write_data_q     <= '0;
      byte_enable_q    <= 32'h0000_000f;
      injection_rate_q <= 32'hffff_ffff;
      period_q         <= 32'd1;
      burst_length_q   <= 32'd1;
      idle_length_q    <= '0;
      duration_limit_q <= '0;
      irq_enable_q     <= '0;
    end else begin
      if (state_q == StateIdle || state_q == StateDone) begin
        unique case (reg_req_i.addr[11:0])
          RegConfig: if (reg_write)
            config_q <= merge_wstrb(config_q, reg_req_i.wdata, reg_req_i.wstrb);
          RegBaseAddr: if (reg_write)
            base_addr_q <= merge_wstrb(base_addr_q, reg_req_i.wdata, reg_req_i.wstrb);
          RegAddressMask: if (reg_write)
            address_mask_q <= merge_wstrb(address_mask_q, reg_req_i.wdata, reg_req_i.wstrb);
          RegStride: if (reg_write)
            stride_q <= merge_wstrb(stride_q, reg_req_i.wdata, reg_req_i.wstrb);
          RegSeed: if (reg_write)
            seed_q <= merge_wstrb(seed_q, reg_req_i.wdata, reg_req_i.wstrb);
          RegWriteData: if (reg_write)
            write_data_q <= merge_wstrb(write_data_q, reg_req_i.wdata, reg_req_i.wstrb);
          RegByteEnable: if (reg_write)
            byte_enable_q <= merge_wstrb(byte_enable_q, reg_req_i.wdata, reg_req_i.wstrb);
          RegInjectionRate: if (reg_write)
            injection_rate_q <= merge_wstrb(injection_rate_q, reg_req_i.wdata, reg_req_i.wstrb);
          RegPeriod: if (reg_write)
            period_q <= merge_wstrb(period_q, reg_req_i.wdata, reg_req_i.wstrb);
          RegBurstLength: if (reg_write)
            burst_length_q <= merge_wstrb(burst_length_q, reg_req_i.wdata, reg_req_i.wstrb);
          RegIdleLength: if (reg_write)
            idle_length_q <= merge_wstrb(idle_length_q, reg_req_i.wdata, reg_req_i.wstrb);
          RegDurationLimit: if (reg_write)
            duration_limit_q <= merge_wstrb(duration_limit_q, reg_req_i.wdata, reg_req_i.wstrb);
          default: ;
        endcase
      end
      if (reg_write && reg_req_i.addr[11:0] == RegIrqEnable)
        irq_enable_q <= merge_wstrb(irq_enable_q, reg_req_i.wdata, reg_req_i.wstrb);
    end
  end

  logic [2:0] address_mode, temporal_mode;
  logic [1:0] duration_mode, rw_mode, write_data_mode;
  assign address_mode    = config_q[2:0];
  assign temporal_mode   = config_q[6:4];
  assign duration_mode   = config_q[9:8];
  assign rw_mode         = config_q[13:12];
  assign write_data_mode = config_q[17:16];

  logic config_valid;
  always_comb begin
    config_valid = address_mode <= AddrGaussian && temporal_mode <= TemporalBurst &&
                   duration_mode != DurationReserved &&
                   base_addr_q[1:0] == 2'b00 &&
                   (base_addr_q & address_mask_q) == 0 &&
                   address_mask_q[1:0] == 2'b00 && byte_enable_q[3:0] != 4'b0000;
    if (temporal_mode == TemporalPeriodic && period_q == 0) config_valid = 1'b0;
    if (temporal_mode == TemporalBurst && burst_length_q == 0) config_valid = 1'b0;
    if (address_mode == AddrStride && stride_q[1:0] != 2'b00) config_valid = 1'b0;
  end

  logic [31:0] address_prng_q, temporal_prng_q;
  logic [31:0] current_offset_q, incrementing_data_q;
  logic [31:0] period_counter_q, burst_remaining_q, idle_counter_q;
  logic [31:0] elapsed_cycles_q, requests_q, grants_q, completed_q;
  logic [31:0] reads_q, writes_q, wait_grant_cycles_q, wait_response_cycles_q;
  logic [31:0] latency_q, latency_sum_q, latency_max_q;
  logic [31:0] missed_opportunities_q, last_address_q, last_read_data_q;
  logic [31:0] error_status_q, irq_status_q;
  logic request_pending_q, response_pending_q;
  logic request_we_q, response_we_q;
  logic [3:0] request_be_q;
  logic [31:0] request_addr_q, request_wdata_q;

  logic [31:0] candidate_offset, candidate_addr, candidate_wdata;
  logic candidate_we;
  always_comb begin
    candidate_offset = current_offset_q;
    unique case (address_mode)
      AddrFixed, AddrSequential, AddrStride: candidate_offset = current_offset_q;
      AddrUniform:  candidate_offset = address_prng_q & address_mask_q;
      AddrGaussian: candidate_offset = gaussian_offset(address_prng_q, address_mask_q);
      default: ;
    endcase
    candidate_addr = (base_addr_q & ~address_mask_q) |
                     (candidate_offset & address_mask_q);
    candidate_addr[1:0] = 2'b00;

    unique case (rw_mode)
      RwRead:      candidate_we = 1'b0;
      RwWrite:     candidate_we = 1'b1;
      RwAlternate: candidate_we = grants_q[0];
      RwRandom:    candidate_we = address_prng_q[0];
      default:     candidate_we = 1'b0;
    endcase

    unique case (write_data_mode)
      WdataFixed:     candidate_wdata = write_data_q;
      WdataAddress:   candidate_wdata = candidate_addr;
      WdataRandom:    candidate_wdata = address_prng_q;
      WdataIncrement: candidate_wdata = incrementing_data_q;
      default:        candidate_wdata = write_data_q;
    endcase
  end

  logic temporal_allow, transaction_budget;
  always_comb begin
    unique case (temporal_mode)
      TemporalSaturated: temporal_allow = 1'b1;
      TemporalPeriodic: temporal_allow = period_counter_q == 0;
      TemporalBernoulli: temporal_allow = injection_rate_q == 32'hffff_ffff ||
                                                    temporal_prng_q < injection_rate_q;
      TemporalBurst: temporal_allow = idle_counter_q == 0 && burst_remaining_q != 0;
      default: temporal_allow = 1'b1;
    endcase
    transaction_budget = duration_mode != DurationTransactions || duration_limit_q == 0 ||
                         requests_q < duration_limit_q;
  end

  logic issue_event, request_accepted, response_received, stop_condition;
  logic request_pending_next, response_pending_next;
  assign issue_event = state_q == StateRunning && !request_pending_q &&
                       (!response_pending_q || response_received) && temporal_allow &&
                       transaction_budget && !stop_condition;
  assign request_accepted = request_pending_q && obi_resp_i.gnt;
  assign response_received = response_pending_q && obi_resp_i.rvalid;
  assign request_pending_next = (request_pending_q || issue_event) && !request_accepted;
  assign response_pending_next = (response_pending_q && !response_received) || request_accepted;

  always_comb begin
    stop_condition = stop_cmd;
    unique case (duration_mode)
      DurationCycles: if (duration_limit_q != 0 &&
                          elapsed_cycles_q >= duration_limit_q) stop_condition = 1'b1;
      DurationTransactions: if (duration_limit_q != 0 && response_received &&
                                completed_q >= duration_limit_q - 1) stop_condition = 1'b1;
      DurationReserved: ;
      DurationInfinite: ;
      default: ;
    endcase
  end

  assign obi_req_o = '{
    req: request_pending_q,
    we: request_we_q,
    be: request_be_q,
    addr: request_addr_q,
    wdata: request_wdata_q
  };

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni || soft_reset_accepted) begin
      state_q                 <= StateIdle;
      address_prng_q          <= 32'd1;
      temporal_prng_q         <= 32'h1a2b_3c4d;
      current_offset_q        <= '0;
      incrementing_data_q     <= '0;
      period_counter_q        <= '0;
      burst_remaining_q       <= '0;
      idle_counter_q          <= '0;
      elapsed_cycles_q        <= '0;
      requests_q              <= '0;
      grants_q                <= '0;
      completed_q             <= '0;
      reads_q                 <= '0;
      writes_q                <= '0;
      wait_grant_cycles_q     <= '0;
      wait_response_cycles_q  <= '0;
      latency_q               <= '0;
      latency_sum_q           <= '0;
      latency_max_q           <= '0;
      missed_opportunities_q  <= '0;
      last_address_q          <= '0;
      last_read_data_q        <= '0;
      error_status_q          <= '0;
      irq_status_q            <= '0;
      request_pending_q       <= 1'b0;
      response_pending_q      <= 1'b0;
      request_we_q            <= 1'b0;
      response_we_q           <= 1'b0;
      request_be_q            <= '0;
      request_addr_q          <= '0;
      request_wdata_q         <= '0;
    end else begin
      if (irq_status_write)
        irq_status_q <= irq_status_q & ~merge_wstrb('0, reg_req_i.wdata, reg_req_i.wstrb);
      if (error_status_write)
        error_status_q <= error_status_q & ~merge_wstrb('0, reg_req_i.wdata, reg_req_i.wstrb);
      if (clear_error_cmd) begin
        error_status_q <= '0;
        irq_status_q[1] <= 1'b0;
      end
      if (configuration_write && state_q != StateIdle && state_q != StateDone) begin
        error_status_q[1] <= 1'b1;
        irq_status_q[1] <= 1'b1;
      end
      if (start_cmd && state_q != StateIdle && state_q != StateDone) begin
        error_status_q[0] <= 1'b1;
        irq_status_q[1] <= 1'b1;
      end
      if (soft_reset_request && !soft_reset_accepted) begin
        error_status_q[4] <= 1'b1;
        irq_status_q[1] <= 1'b1;
      end
      if (obi_resp_i.rvalid && !response_pending_q) begin
        error_status_q[3] <= 1'b1;
        irq_status_q[1] <= 1'b1;
      end

      if (start_cmd && (state_q == StateIdle || state_q == StateDone)) begin
        if (!config_valid) begin
          error_status_q[2] <= 1'b1;
          irq_status_q[1] <= 1'b1;
        end else begin
          state_q                <= StateRunning;
          address_prng_q         <= seed_q == 0 ? 32'd1 : seed_q;
          temporal_prng_q        <= (seed_q ^ 32'ha5a5_5a5a) == 0 ?
                                    32'h1a2b_3c4d : seed_q ^ 32'ha5a5_5a5a;
          current_offset_q       <= '0;
          incrementing_data_q    <= write_data_q;
          period_counter_q       <= '0;
          burst_remaining_q      <= burst_length_q;
          idle_counter_q         <= '0;
          elapsed_cycles_q       <= '0;
          requests_q             <= '0;
          grants_q               <= '0;
          completed_q            <= '0;
          reads_q                <= '0;
          writes_q               <= '0;
          wait_grant_cycles_q    <= '0;
          wait_response_cycles_q <= '0;
          latency_q              <= '0;
          latency_sum_q          <= '0;
          latency_max_q          <= '0;
          missed_opportunities_q <= '0;
          last_address_q         <= '0;
          last_read_data_q       <= '0;
          request_pending_q      <= 1'b0;
          response_pending_q     <= 1'b0;
          irq_status_q[0]        <= 1'b0;
        end
      end else begin
        if (clear_done_cmd && state_q == StateDone) begin
          state_q <= StateIdle;
          irq_status_q[0] <= 1'b0;
        end

        if (state_q == StateRunning || state_q == StateDraining) begin
          request_pending_q <= request_pending_next;
          response_pending_q <= response_pending_next;

          if (request_pending_q && !obi_resp_i.gnt)
            wait_grant_cycles_q <= sat_inc(wait_grant_cycles_q);
          if (response_pending_q && !obi_resp_i.rvalid)
            wait_response_cycles_q <= sat_inc(wait_response_cycles_q);

          if (response_pending_q && !response_received)
            latency_q <= sat_inc(latency_q);
          if (request_accepted && (!response_pending_q || response_received))
            latency_q <= '0;
          else if (response_received)
            latency_q <= '0;

          if (response_received) begin
            logic [31:0] observed_latency;
            observed_latency = sat_inc(latency_q);
            completed_q <= sat_inc(completed_q);
            latency_sum_q <= sat_add(latency_sum_q, observed_latency);
            if (observed_latency > latency_max_q) latency_max_q <= observed_latency;
            if (!response_we_q) last_read_data_q <= obi_resp_i.rdata;
          end

          if (request_accepted) begin
            grants_q <= sat_inc(grants_q);
            last_address_q <= request_addr_q;
            response_we_q <= request_we_q;
            if (request_we_q) writes_q <= sat_inc(writes_q);
            else reads_q <= sat_inc(reads_q);

            unique case (address_mode)
              AddrSequential: current_offset_q <=
                  (current_offset_q + 32'd4) & address_mask_q & 32'hffff_fffc;
              AddrStride: current_offset_q <=
                  (current_offset_q + stride_q) & address_mask_q & 32'hffff_fffc;
              default: ;
            endcase
            if (write_data_mode == WdataIncrement)
              incrementing_data_q <= incrementing_data_q + 32'd1;
          end
        end

        if (state_q == StateRunning) begin
          if (!(duration_mode == DurationCycles && duration_limit_q != 0 &&
                elapsed_cycles_q >= duration_limit_q))
            elapsed_cycles_q <= sat_inc(elapsed_cycles_q);
          temporal_prng_q <= lfsr_next(temporal_prng_q);

          if (temporal_mode == TemporalPeriodic) begin
            if (issue_event) period_counter_q <= period_q - 1;
            else if (period_counter_q != 0) period_counter_q <= period_counter_q - 1;
          end
          if (temporal_mode == TemporalBurst) begin
            if (idle_counter_q != 0) begin
              idle_counter_q <= idle_counter_q - 1;
            end else if (issue_event) begin
              if (burst_remaining_q <= 1) begin
                burst_remaining_q <= burst_length_q;
                idle_counter_q <= idle_length_q;
              end else begin
                burst_remaining_q <= burst_remaining_q - 1;
              end
            end
          end

          if (temporal_allow && transaction_budget && request_pending_q)
            missed_opportunities_q <= sat_inc(missed_opportunities_q);

          if (issue_event) begin
            requests_q <= sat_inc(requests_q);
            address_prng_q <= lfsr_next(address_prng_q);
            request_we_q <= candidate_we;
            request_be_q <= byte_enable_q[3:0];
            request_addr_q <= candidate_addr;
            request_wdata_q <= candidate_wdata;
          end

          if (stop_condition) begin
            if (request_pending_next || response_pending_next) begin
              state_q <= StateDraining;
            end else begin
              state_q <= StateDone;
              irq_status_q[0] <= 1'b1;
            end
          end
        end else if (state_q == StateDraining &&
                     !request_pending_next && !response_pending_next) begin
          state_q <= StateDone;
          irq_status_q[0] <= 1'b1;
        end
      end
    end
  end

  logic [31:0] status_value;
  always_comb begin
    status_value = '0;
    status_value[0] = state_q == StateIdle;
    status_value[1] = state_q == StateRunning;
    status_value[2] = state_q == StateDraining;
    status_value[3] = state_q == StateDone;
    status_value[4] = |error_status_q;
    status_value[5] = request_pending_q;
    status_value[6] = response_pending_q;
    status_value[25:24] = state_q;
  end

  always_comb begin
    reg_rsp_o = '0;
    reg_rsp_o.ready = 1'b1;
    if (reg_req_i.valid) begin
      unique case (reg_req_i.addr[11:0])
        RegControl:             reg_rsp_o.rdata = '0;
        RegConfig:              reg_rsp_o.rdata = config_q;
        RegStatus:              reg_rsp_o.rdata = status_value;
        RegBaseAddr:            reg_rsp_o.rdata = base_addr_q;
        RegAddressMask:         reg_rsp_o.rdata = address_mask_q;
        RegStride:              reg_rsp_o.rdata = stride_q;
        RegSeed:                reg_rsp_o.rdata = seed_q;
        RegWriteData:           reg_rsp_o.rdata = write_data_q;
        RegByteEnable:          reg_rsp_o.rdata = byte_enable_q;
        RegInjectionRate:       reg_rsp_o.rdata = injection_rate_q;
        RegPeriod:              reg_rsp_o.rdata = period_q;
        RegBurstLength:         reg_rsp_o.rdata = burst_length_q;
        RegIdleLength:          reg_rsp_o.rdata = idle_length_q;
        RegDurationLimit:       reg_rsp_o.rdata = duration_limit_q;
        RegIrqEnable:           reg_rsp_o.rdata = irq_enable_q;
        RegIrqStatus:           reg_rsp_o.rdata = irq_status_q;
        RegElapsedCycles:       reg_rsp_o.rdata = elapsed_cycles_q;
        RegRequests:            reg_rsp_o.rdata = requests_q;
        RegGrants:              reg_rsp_o.rdata = grants_q;
        RegCompleted:           reg_rsp_o.rdata = completed_q;
        RegReads:               reg_rsp_o.rdata = reads_q;
        RegWrites:              reg_rsp_o.rdata = writes_q;
        RegWaitGrantCycles:     reg_rsp_o.rdata = wait_grant_cycles_q;
        RegWaitResponseCycles:  reg_rsp_o.rdata = wait_response_cycles_q;
        RegLatencySum:          reg_rsp_o.rdata = latency_sum_q;
        RegLatencyMax:          reg_rsp_o.rdata = latency_max_q;
        RegMissedOpportunities: reg_rsp_o.rdata = missed_opportunities_q;
        RegLastAddress:         reg_rsp_o.rdata = last_address_q;
        RegPrngState:           reg_rsp_o.rdata = address_prng_q;
        RegErrorStatus:         reg_rsp_o.rdata = error_status_q;
        RegVersion:             reg_rsp_o.rdata = Version;
        RegLastReadData:        reg_rsp_o.rdata = last_read_data_q;
        RegTemporalPrngState:   reg_rsp_o.rdata = temporal_prng_q;
        default: begin
          reg_rsp_o.rdata = '0;
          reg_rsp_o.error = 1'b1;
        end
      endcase
    end
  end

  assign interrupt_o = |(irq_status_q[1:0] & irq_enable_q[1:0]);

endmodule
