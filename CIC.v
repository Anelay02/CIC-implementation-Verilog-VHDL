//============================================================================//
// File:	CIC.v
// Author:	Ayoub el idrissi Achraf
// Description:	Implements an N-stage CIC filter, configurable as either a
//              decimator or an interpolator via the CONFIG parameter.
//===========================================================================//

// Defining timescale
`timescale 1ns/1ps

  ////////////////////////
 // MODULE DECLARATION //
////////////////////////

module CIC #(
	parameter N           = 3,			// Order of the filter
	parameter OSR         = 512,       	// Oversampling ratio, only powers of 2
	parameter INPUT_SIZE  = 1,  		// Input size
	parameter OUTPUT_SIZE = 16, 		// Output size
	parameter CONFIG      = "decimator"	// "decimator" or "interpolator", default is "decimator"
)(
	input                                 i_clk,       	// Input clock   
	input                                 i_rst_n,     	// Asynchronous active low reset    
	input      [INPUT_SIZE-1:0]           i_data, 		// Unsigned input data
	input     					          i_valid, 		// Valid flag for input data	
	output reg [OUTPUT_SIZE-1:0]          o_data,    	// Unsigned output data     
	output reg                            o_valid, 		// Valid flag for output data
	output                                o_ready  		// Ready flag
);


  //////////////////
 // LOCALPARAMS  //
//////////////////

genvar i;
localparam LOG_OSR  = $clog2(OSR);
localparam REG_SIZE = (CONFIG == "interpolator") ? (N-1)*LOG_OSR + INPUT_SIZE : N*LOG_OSR + INPUT_SIZE;
localparam LSB_OUT  = (REG_SIZE > OUTPUT_SIZE ? REG_SIZE-OUTPUT_SIZE : 0);


  ////////////////////
 // WIRES AND REGS //
////////////////////

reg  [LOG_OSR-1:0] samples_counter;	// Counter, OSR periodic
wire [N*REG_SIZE-1:0] final_stage;	// Output
wire condition_valid;
wire accept_sample = (&samples_counter) && i_valid;

reg  [N*REG_SIZE-1:0] Integ_stor;	// Integrator(s)
wire [N*REG_SIZE-1:0] Integrator;
reg  [N*REG_SIZE-1:0] Comb_stor;	// Comb(s)
wire [N*REG_SIZE-1:0] Comb;


//===========================================================================//
// Code
//===========================================================================//

	assign o_ready = i_rst_n && ((CONFIG == "interpolator") ? accept_sample : 1'b1);

	always @(posedge i_clk or negedge i_rst_n) begin
		if (~i_rst_n)
			samples_counter <= {LOG_OSR{1'b0}};
		else if (i_valid)
			samples_counter <= samples_counter + 1'b1;
	end


	generate
		if (CONFIG == "interpolator") begin : gen_interpolator

			  //////////////////
			 // Combs Stages //
			//////////////////

			assign Comb[REG_SIZE-1:0] = i_data - Comb_stor[REG_SIZE-1:0];

			always @(posedge i_clk or negedge i_rst_n) begin
				if (~i_rst_n)
					Comb_stor[REG_SIZE-1:0] <= {REG_SIZE{1'b0}};
				else if (accept_sample)
					Comb_stor[REG_SIZE-1:0] <= i_data;
			end

			for (i = 1; i < N; i = i + 1) begin : gen_combs
				assign Comb[(i+1)*REG_SIZE-1:i*REG_SIZE] = Comb[i*REG_SIZE-1:(i-1)*REG_SIZE] - Comb_stor[(i+1)*REG_SIZE-1:i*REG_SIZE];

				always @(posedge i_clk or negedge i_rst_n) begin
					if (~i_rst_n)
						Comb_stor[(i+1)*REG_SIZE-1:i*REG_SIZE] <= {REG_SIZE{1'b0}};
					else if (accept_sample)
						Comb_stor[(i+1)*REG_SIZE-1:i*REG_SIZE] <= Comb[i*REG_SIZE-1:(i-1)*REG_SIZE];
				end
			end


			  /////////////////////////
			 // Integrators Stages  //
			/////////////////////////

			wire [REG_SIZE-1:0] stuffed_sample = accept_sample ? Comb[N*REG_SIZE-1:(N-1)*REG_SIZE] : {REG_SIZE{1'b0}};

			assign Integrator[REG_SIZE-1:0] = stuffed_sample + Integ_stor[REG_SIZE-1:0];

			always @(posedge i_clk or negedge i_rst_n) begin
				if (~i_rst_n)
					Integ_stor[REG_SIZE-1:0] <= {REG_SIZE{1'b0}};
				else if (i_valid)
					Integ_stor[REG_SIZE-1:0] <= Integrator[REG_SIZE-1:0];
			end

			for (i = 1; i < N; i = i + 1) begin : gen_integrators
				assign Integrator[(i+1)*REG_SIZE-1:i*REG_SIZE] =
					Integrator[i*REG_SIZE-1:(i-1)*REG_SIZE] + Integ_stor[(i+1)*REG_SIZE-1:i*REG_SIZE];

				always @(posedge i_clk or negedge i_rst_n) begin
					if (~i_rst_n)
						Integ_stor[(i+1)*REG_SIZE-1:i*REG_SIZE] <= {REG_SIZE{1'b0}};
					else if (i_valid)
						Integ_stor[(i+1)*REG_SIZE-1:i*REG_SIZE] <= Integrator[(i+1)*REG_SIZE-1:i*REG_SIZE];
				end
			end

		end : gen_interpolator


		else begin : gen_decimator

			  /////////////////////////
			 // Integrators Stages  //
			/////////////////////////

			assign Integrator[REG_SIZE-1:0] = i_data + Integ_stor[REG_SIZE-1:0];

			always @(posedge i_clk or negedge i_rst_n) begin
				if (~i_rst_n)
					Integ_stor[REG_SIZE-1:0] <= {REG_SIZE{1'b0}};
				else if (i_valid)
					Integ_stor[REG_SIZE-1:0] <= Integrator[REG_SIZE-1:0];
			end

			for (i = 1; i < N; i = i + 1) begin : gen_integrators
				assign Integrator[(i+1)*REG_SIZE-1:i*REG_SIZE] =
					Integrator[i*REG_SIZE-1:(i-1)*REG_SIZE] + Integ_stor[(i+1)*REG_SIZE-1:i*REG_SIZE];

				always @(posedge i_clk or negedge i_rst_n) begin
					if (~i_rst_n)
						Integ_stor[(i+1)*REG_SIZE-1:i*REG_SIZE] <= {REG_SIZE{1'b0}};
					else if (i_valid)
						Integ_stor[(i+1)*REG_SIZE-1:i*REG_SIZE] <= Integrator[(i+1)*REG_SIZE-1:i*REG_SIZE];
				end
			end


			  //////////////////
			 // Combs Stages //
			//////////////////

			assign Comb[REG_SIZE-1:0] = Integrator[N*REG_SIZE-1:(N-1)*REG_SIZE] - Comb_stor[REG_SIZE-1:0];

			always @(posedge i_clk or negedge i_rst_n) begin
				if (~i_rst_n)
					Comb_stor[REG_SIZE-1:0] <= {REG_SIZE{1'b0}};
				else if (accept_sample)
					Comb_stor[REG_SIZE-1:0] <= Integrator[N*REG_SIZE-1:(N-1)*REG_SIZE];
			end

			for (i = 1; i < N; i = i + 1) begin : gen_combs
				assign Comb[(i+1)*REG_SIZE-1:i*REG_SIZE] =
					Comb[i*REG_SIZE-1:(i-1)*REG_SIZE] - Comb_stor[(i+1)*REG_SIZE-1:i*REG_SIZE];

				always @(posedge i_clk or negedge i_rst_n) begin
					if (~i_rst_n)
						Comb_stor[(i+1)*REG_SIZE-1:i*REG_SIZE] <= {REG_SIZE{1'b0}};
					else if (accept_sample)
						Comb_stor[(i+1)*REG_SIZE-1:i*REG_SIZE] <= Comb[i*REG_SIZE-1:(i-1)*REG_SIZE];
				end
			end

		end : gen_decimator

	endgenerate


	  /////////////
	 // Outputs //
	/////////////

	assign final_stage     = (CONFIG == "interpolator") ? Integrator : Comb;
	assign condition_valid = (CONFIG == "interpolator") ? i_valid    : accept_sample;

	always @(posedge i_clk or negedge i_rst_n) begin
		if (~i_rst_n)
			o_data <= {OUTPUT_SIZE{1'b0}};
		else if (condition_valid)
			o_data <= final_stage[N*REG_SIZE-1:LSB_OUT+(N-1)*REG_SIZE];
	end

	always @(posedge i_clk or negedge i_rst_n) begin
		if (~i_rst_n)
			o_valid <= 1'b0;
		else
			o_valid <= condition_valid;
	end


endmodule
