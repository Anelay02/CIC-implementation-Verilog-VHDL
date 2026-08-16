//============================================================================//
// File:    tb_CIC.v
// Author:	Ayoub el idrissi Achraf
// Description: Testbench for CIC.v
// (Optionally logs input/outputs to a CSV).
//============================================================================//

// Defining timescale
`timescale 1ns/1ps

  ////////////////////////
 // MODULE DECLARATION //
////////////////////////

module tb_CIC;

	//======================================================================
	// CSV control
	//======================================================================
	parameter        CSV_ENABLE     = 1;              	// 1 = write tb_CIC.csv
	parameter        CSV_FILE       = "tb_CIC.csv";		// Name of the CSV

	//======================================================================
	// CIC parameters
	//======================================================================
	parameter        N              = 3;				// Order of the filters
	parameter        OSR_DEC        = 64;              	// decimator OSR    (must be a power of 2 !)
	parameter        OSR_INT        = 64;              	// interpolator OSR (must be a power of 2 !)
	parameter        INPUT_SIZE     = 8;
	parameter        OUTPUT_SIZE    = 16;
	parameter real   CLK_PERIOD_NS  = 10.0;             // 100 MHz clock

	//======================================================================
	// Input sine parameters
	//======================================================================
	parameter real   PI             = 3.14159265358979;
	parameter real   F_SIGNAL_HZ    = 50_000.0;         						// must be << Fclk/(2*OSR)
	parameter real   AMPLITUDE      = (2.0**(INPUT_SIZE-1)) - 1.0; 				// Max amp
	parameter real   OFFSET         = 2.0**(INPUT_SIZE-1);
	parameter integer NB_PERIODS    = 4;                						// Sine periods to run (at interpolator's low rate)
	parameter integer OSR_MAX       = (OSR_DEC > OSR_INT) ? OSR_DEC : OSR_INT;
	parameter integer DRAIN_CYCLES  = 4*OSR_MAX;        						// Extra cycles to flush both pipelines

	//======================================================================
	// DUT signals
	//======================================================================
	reg                      clk;
	reg                      rst_n;

	reg  [INPUT_SIZE-1:0]    dec_i_data;   // decimator
	reg                      dec_i_valid;
	reg  [INPUT_SIZE-1:0]    int_i_data;   // interpolator
	reg                      int_i_valid;

	wire [OUTPUT_SIZE-1:0]   dec_o_data,    interp_o_data;
	wire                     dec_o_valid,   interp_o_valid;
	wire                     dec_o_ready,   interp_o_ready;

	//======================================================================
	// Bookkeeping
	//======================================================================
	real     t_full;                 // full-rate sample period, seconds
	real     t_int_low;              // interpolator's own low-rate sample period, seconds
	real     sine_real_dec, sine_real_int;
	integer  sine_int_dec, sine_int_int;
	integer  csv_file;
	integer  dec_cyc_count;          // free-running counter for the decimator's fast stream
	integer  int_cyc_count;          // free-running counter for the interpolator's OSR_INT gating
	integer  int_low_sample_count;   // interpolator low-rate sample index
	integer  samples_per_period;     // at the interpolator's low rate
	integer  run_cycles;
	integer  max_input_code;

	//======================================================================
	// Main code
	//======================================================================

	  ////////////////////
	 // DUT instances  //
	////////////////////
	
	CIC #( // Decimator
		.N(N),
		.OSR(OSR_DEC),
		.INPUT_SIZE(INPUT_SIZE),
		.OUTPUT_SIZE(OUTPUT_SIZE),
		.CONFIG("decimator")
	)dec_inst(
		.i_clk(clk),
		.i_rst_n(rst_n),
		.i_data(dec_i_data),
		.i_valid(dec_i_valid),
		.o_data(dec_o_data),
		.o_valid(dec_o_valid),
		.o_ready(dec_o_ready)
	);

	CIC #( // Interpolator
		.N(N),
		.OSR(OSR_INT),
		.INPUT_SIZE(INPUT_SIZE),
		.OUTPUT_SIZE(OUTPUT_SIZE),
		.CONFIG("interpolator")
	) interp_inst (
		.i_clk(clk),
		.i_rst_n(rst_n),
		.i_data(int_i_data),
		.i_valid(int_i_valid),
		.o_data(interp_o_data),
		.o_valid(interp_o_valid),
		.o_ready(interp_o_ready)
	);

	  ////////////////////////
	 // Clock & reset      //
	////////////////////////
	
	initial clk = 1'b0;
	always #(CLK_PERIOD_NS/2.0) clk = ~clk;

	initial begin
		rst_n       = 1'b0;
		dec_i_valid = 1'b0;
		int_i_valid = 1'b0;
		dec_i_data  = {INPUT_SIZE{1'b0}};
		int_i_data  = {INPUT_SIZE{1'b0}};
		repeat (4) @(posedge clk);
		@(negedge clk);
		rst_n = 1'b1;
	end

	// Pre-computation
	initial begin
		t_full             = CLK_PERIOD_NS * 1.0e-9;
		t_int_low          = OSR_INT * CLK_PERIOD_NS * 1.0e-9;
		samples_per_period = $rtoi(1.0 / (F_SIGNAL_HZ * t_int_low) + 0.5);
		if (samples_per_period < 2)
			samples_per_period = 2; // avoid degenerate/aliased low-rate stimulus
		run_cycles         = NB_PERIODS * samples_per_period * OSR_INT + DRAIN_CYCLES;
		max_input_code     = (2**INPUT_SIZE) - 1;
	end

	  ////////////////////////////////////////
	 // Input drivers
	////////////////////////////////////////

	// Decimator 
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			dec_cyc_count <= 0;
			dec_i_valid   <= 1'b0;
			dec_i_data    <= $rtoi(OFFSET);
		end else begin
			dec_i_valid <= 1'b1;
			if (dec_i_valid) begin
				sine_real_dec = OFFSET + AMPLITUDE * $sin(2.0 * PI * F_SIGNAL_HZ * (dec_cyc_count * t_full));
				sine_int_dec  = $rtoi(sine_real_dec + 0.5);
				if (sine_int_dec < 0)              sine_int_dec = 0;
				if (sine_int_dec > max_input_code) sine_int_dec = max_input_code;
				dec_i_data    <= sine_int_dec[INPUT_SIZE-1:0];
				dec_cyc_count <= dec_cyc_count + 1;
			end
		end
	end

	// Interpolator
	always @(posedge clk or negedge rst_n) begin
		if (~rst_n) begin
			int_cyc_count         <= 0;
			int_low_sample_count  <= 0;
			int_i_valid           <= 1'b0;
			int_i_data            <= $rtoi(OFFSET);
		end else begin
			int_i_valid <= 1'b1;
			if (int_i_valid) begin
				if ((int_cyc_count % OSR_INT) == 0) begin
					sine_real_int = OFFSET + AMPLITUDE * $sin(2.0 * PI * F_SIGNAL_HZ * (int_low_sample_count * t_int_low));
					sine_int_int  = $rtoi(sine_real_int + 0.5);
					if (sine_int_int < 0)              sine_int_int = 0;
					if (sine_int_int > max_input_code) sine_int_int = max_input_code;
					int_i_data            <= sine_int_int[INPUT_SIZE-1:0];
					int_low_sample_count  <= int_low_sample_count + 1;
				end
				int_cyc_count <= int_cyc_count + 1;
			end
		end
	end


	  ////////////////////
	 // CSV logging    //
	////////////////////

	generate
		if (CSV_ENABLE) begin : gen_csv
			initial begin
				csv_file = $fopen(CSV_FILE, "w");
				$fdisplay(csv_file,
					"time_ns,dec_i_data,dec_o_valid,dec_o_data,int_i_data,interp_o_valid,interp_o_data");
			end

			always @(posedge clk) begin
				if (rst_n) begin
					$fdisplay(csv_file, "%0.1f,%0d,%0b,%0d,%0d,%0b,%0d",
						$realtime,
						dec_i_data, dec_o_valid,    dec_o_data,
						int_i_data, interp_o_valid, interp_o_data);
				end
			end
		end
	endgenerate

	initial begin
		$dumpfile("tb_CIC.vcd");
		$dumpvars(0, tb_CIC);
	end

	//======================================================================
	// Run sequence
	//======================================================================
	
	initial begin
		@(posedge rst_n);
		repeat (run_cycles) @(posedge clk);

		if (CSV_ENABLE) $fclose(csv_file);
		$display("[tb_CIC] simulation done.");
		$finish;
	end


endmodule
