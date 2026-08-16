--============================================================================--
-- File:        tb_CIC.vhd
-- Author:      Ayoub el idrissi Achraf
-- Description: Testbench for CIC.vhd
-- (Optionally logs input/outputs to a CSV).
--============================================================================--

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;
use std.textio.all;

  ------------------------
 -- ENTITY DECLARATION --
------------------------

entity tb_CIC is
--  Port ( );
end entity tb_CIC;

  --------------------------
 -- ARCHITECTURE (test) --
--------------------------

architecture tb of tb_CIC is

	component CIC
		generic (
			N           : integer := 3;              -- Order of the filter
			OSR         : integer := 512;             -- Oversampling ratio, only powers of 2
			INPUT_SIZE  : integer := 1;               -- Input size
			OUTPUT_SIZE : integer := 16;              -- Output size
			CONFIG      : string  := "decimator"      -- "decimator" or "interpolator", default is "decimator"
		);
		port (
			i_clk   : in  std_logic;                                  -- Input clock
			i_rst_n : in  std_logic;                                  -- Asynchronous active low reset
			i_data  : in  std_logic_vector(INPUT_SIZE-1 downto 0);    -- Unsigned input data
			i_valid : in  std_logic;                                  -- Valid flag for input data
			o_data  : out std_logic_vector(OUTPUT_SIZE-1 downto 0);   -- Unsigned output data
			o_valid : out std_logic;                                  -- Valid flag for output data
			o_ready : out std_logic                                   -- Ready flag
		);
	end component;


    function imax(a, b : integer) return integer is
    begin
        if a > b then
            return a;
        else
            return b;
        end if;
    end function imax;

    function iclamp_min2(a : integer) return integer is
    begin
        if a < 2 then
            return 2;
        else
            return a;
        end if;
    end function iclamp_min2;
	
	
	--======================================================================
	-- CSV control
	--======================================================================
	constant CSV_ENABLE     : boolean := true;              -- true = write tb_CIC.csv
	constant CSV_FILE       : string  := "tb_CIC.csv";       -- Name of the CSV

	--======================================================================
	-- CIC parameters
	--======================================================================
	constant N              : integer := 3;                 -- Order of the filters
	constant OSR_DEC        : integer := 64;                 -- decimator OSR    (must be a power of 2 !)
	constant OSR_INT        : integer := 64;                 -- interpolator OSR (must be a power of 2 !)
	constant INPUT_SIZE     : integer := 8;
	constant OUTPUT_SIZE    : integer := 16;
	constant CLK_PERIOD_NS  : real    := 10.0;               -- 100 MHz clock

	--======================================================================
	-- Input sine parameters
	--======================================================================
	constant PI             : real    := 3.14159265358979;
	constant F_SIGNAL_HZ    : real    := 50_000.0;           -- must be << Fclk/(2*OSR)
	constant NB_PERIODS     : integer := 4;                  -- Sine periods to run (at interpolator's low rate)	

    --======================================================================
    -- Derived parameters (localparam-equivalent)
    --======================================================================
    constant AMPLITUDE     : real    := (2.0**(INPUT_SIZE-1)) - 1.0;     -- Max amp
    constant OFFSET        : real    := 2.0**(INPUT_SIZE-1);
    constant OSR_MAX       : integer := imax(OSR_DEC, OSR_INT);
    constant DRAIN_CYCLES  : integer := 4*OSR_MAX;                       -- Extra cycles to flush both pipelines
    constant CLK_PERIOD    : time    := CLK_PERIOD_NS * 1 ns;

    --======================================================================
    -- DUT signals
    --======================================================================
    signal clk   : std_logic;
    signal rst_n : std_logic;

    signal dec_i_data  : std_logic_vector(INPUT_SIZE-1 downto 0);   -- decimator
    signal dec_i_valid : std_logic;
    signal int_i_data  : std_logic_vector(INPUT_SIZE-1 downto 0);   -- interpolator
    signal int_i_valid : std_logic;

    signal dec_o_data,  interp_o_data  : std_logic_vector(OUTPUT_SIZE-1 downto 0);
    signal dec_o_valid, interp_o_valid : std_logic;
    signal dec_o_ready, interp_o_ready : std_logic;

    --======================================================================
    -- Bookkeeping
    --======================================================================
    constant t_full    : real := CLK_PERIOD_NS * 1.0e-9;               -- full-rate sample period, seconds
    constant t_int_low : real := real(OSR_INT) * CLK_PERIOD_NS * 1.0e-9; -- interpolator's own low-rate sample period, seconds

    constant samples_per_period_raw : integer := integer(1.0 / (F_SIGNAL_HZ * t_int_low)  );
    constant samples_per_period     : integer := iclamp_min2(samples_per_period_raw);
                                                                         -- avoid degenerate/aliased low-rate stimulus
    constant run_cycles      : integer := NB_PERIODS * samples_per_period * OSR_INT + DRAIN_CYCLES;
    constant max_input_code  : integer := (2**INPUT_SIZE) - 1;

    signal dec_cyc_count         : integer;   -- free-running counter for the decimator's fast stream
    signal int_cyc_count         : integer;   -- free-running counter for the interpolator's OSR_INT gating
    signal int_low_sample_count  : integer;   -- interpolator low-rate sample index

    -- File handle for CSV logging (opened/closed conditionally on CSV_ENABLE)
    file csv_fh : text;

begin

    --======================================================================
    -- Main code
    --======================================================================

      ------------------
     -- DUT instances --
    ------------------

    dec_inst : CIC   -- Decimator
        generic map (
            N           => N,
            OSR         => OSR_DEC,
            INPUT_SIZE  => INPUT_SIZE,
            OUTPUT_SIZE => OUTPUT_SIZE,
            CONFIG      => "decimator"
        )
        port map (
            i_clk   => clk,
            i_rst_n => rst_n,
            i_data  => dec_i_data,
            i_valid => dec_i_valid,
            o_data  => dec_o_data,
            o_valid => dec_o_valid,
            o_ready => dec_o_ready
        );

    interp_inst : CIC   -- Interpolator
        generic map (
            N           => N,
            OSR         => OSR_INT,
            INPUT_SIZE  => INPUT_SIZE,
            OUTPUT_SIZE => OUTPUT_SIZE,
            CONFIG      => "interpolator"
        )
        port map (
            i_clk   => clk,
            i_rst_n => rst_n,
            i_data  => int_i_data,
            i_valid => int_i_valid,
            o_data  => interp_o_data,
            o_valid => interp_o_valid,
            o_ready => interp_o_ready
        );

      --------------------
     -- Clock & reset  --
    --------------------

    clk_gen : process
    begin
        clk <= '0';
        wait for CLK_PERIOD/2.0;
        clk <= '1';
        wait for CLK_PERIOD/2.0;
    end process clk_gen;

    reset_gen : process
    begin
        rst_n <= '0';
        for k in 1 to 4 loop
            wait until rising_edge(clk);
        end loop;
        wait until falling_edge(clk);
        rst_n <= '1';
        wait;   -- run once, like Verilog's initial block
    end process reset_gen;

      ----------------------
     -- Input drivers    --
    ----------------------

    -- Decimator
    dec_driver : process(clk, rst_n)
        variable sine_real_dec : real;
        variable sine_int_dec  : integer;
    begin
        if rst_n = '0' then
            dec_cyc_count <= 0;
            dec_i_valid   <= '0';
            dec_i_data    <= std_logic_vector(to_unsigned(integer(OFFSET), INPUT_SIZE));
        elsif rising_edge(clk) then
            dec_i_valid <= '1';
            if dec_i_valid = '1' then
                sine_real_dec := OFFSET + AMPLITUDE * sin(2.0 * PI * F_SIGNAL_HZ * (real(dec_cyc_count) * t_full));
                sine_int_dec  := integer(sine_real_dec  );
                if sine_int_dec < 0 then
                    sine_int_dec := 0;
                end if;
                if sine_int_dec > max_input_code then
                    sine_int_dec := max_input_code;
                end if;
                dec_i_data    <= std_logic_vector(to_unsigned(sine_int_dec, INPUT_SIZE));
                dec_cyc_count <= dec_cyc_count + 1;
            end if;
        end if;
    end process dec_driver;

    -- Interpolator
    int_driver : process(clk, rst_n)
        variable sine_real_int : real;
        variable sine_int_int  : integer;
    begin
        if rst_n = '0' then
            int_cyc_count        <= 0;
            int_low_sample_count <= 0;
            int_i_valid          <= '0';
            int_i_data           <= std_logic_vector(to_unsigned(integer(OFFSET), INPUT_SIZE));
        elsif rising_edge(clk) then
            int_i_valid <= '1';
            if int_i_valid = '1' then
                if (int_cyc_count mod OSR_INT) = 0 then
                    sine_real_int := OFFSET + AMPLITUDE * sin(2.0 * PI * F_SIGNAL_HZ * (real(int_low_sample_count) * t_int_low));
                    sine_int_int  := integer(sine_real_int  );
                    if sine_int_int < 0 then
                        sine_int_int := 0;
                    end if;
                    if sine_int_int > max_input_code then
                        sine_int_int := max_input_code;
                    end if;
                    int_i_data            <= std_logic_vector(to_unsigned(sine_int_int, INPUT_SIZE));
                    int_low_sample_count  <= int_low_sample_count + 1;
                end if;
                int_cyc_count <= int_cyc_count + 1;
            end if;
        end if;
    end process int_driver;


      ------------------
     -- CSV logging  --
    ------------------

    gen_csv : if CSV_ENABLE generate

        -- Open the file and write the header once
        csv_open : process
            variable l : line;
        begin
            file_open(csv_fh, CSV_FILE, write_mode);
            write(l, string'("time_ns,dec_i_data,dec_o_valid,dec_o_data,int_i_data,interp_o_valid,interp_o_data"));
            writeline(csv_fh, l);
            wait;
        end process csv_open;

        -- Log one row per clock while out of reset
        csv_log : process(clk)
            variable l : line;
        begin
            if rising_edge(clk) then
                if rst_n = '1' then
                    -- File is guaranteed to be open by csv_open process
                    write(l, integer(now / 1 ns));
                    write(l, string'(","));
                    write(l, to_integer(unsigned(dec_i_data)));
                    write(l, string'(","));
                    write(l, to_integer(unsigned'(0 => dec_o_valid)));
                    write(l, string'(","));
                    write(l, to_integer(unsigned(dec_o_data)));
                    write(l, string'(","));
                    write(l, to_integer(unsigned(int_i_data)));
                    write(l, string'(","));
                    write(l, to_integer(unsigned'(0 => interp_o_valid)));
                    write(l, string'(","));
                    write(l, to_integer(unsigned(interp_o_data)));
                    writeline(csv_fh, l);
                end if;
            end if;
        end process csv_log;

    end generate gen_csv;
	

    --======================================================================
    -- Run sequence
    --======================================================================

    run_seq : process
    begin
        wait until rst_n = '1';
        for k in 1 to run_cycles loop
            wait until rising_edge(clk);
        end loop;
        wait for 0 ns;   -- let csv_log's process (also woken by this same
                          -- clock edge) finish writing the last row first

        if CSV_ENABLE then
            file_close(csv_fh);
        end if;
        report "[tb_CIC] simulation done.";
        std.env.finish;
    end process run_seq;

end architecture tb;
