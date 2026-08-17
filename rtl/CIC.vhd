--============================================================================--
-- File:        CIC.vhd
-- Author:      Ayoub el idrissi Achraf
-- Description: Implements an N-stage CIC filter, configurable as either a
--              decimator or an interpolator via the CONFIG generic.
--============================================================================--

-- Using IEEE standard libraries
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

  ------------------------
 -- ENTITY DECLARATION --
------------------------

entity CIC is
    generic (
        N           : integer := 3;               -- Order of the filter
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
end entity CIC;


architecture rtl of CIC is

    -- Function replicating Verilog's $clog2 (ceiling of log2)
    function clog2(value : integer) return integer is
        variable temp   : integer := value - 1;
        variable result : integer := 0;
    begin
        while temp > 0 loop
            result := result + 1;
            temp    := temp / 2;
        end loop;
        return result;
    end function clog2;
	
	-- Compute register size based on configuration
	function calc_reg_size(N, LOG_OSR, INPUT_SIZE : integer; CONFIG : string) return integer is
	begin
		if CONFIG = "interpolator" then
			return (N - 1) * LOG_OSR + INPUT_SIZE;
		else
			return N * LOG_OSR + INPUT_SIZE;
		end if;
	end function;

	-- Compute LSB offset for output truncation
	function calc_lsb_out(REG_SIZE, OUTPUT_SIZE : integer) return integer is
	begin
		if REG_SIZE > OUTPUT_SIZE then
			return REG_SIZE - OUTPUT_SIZE;
		else
			return 0;
		end if;
	end function;

      ------------------
     -- LOCALPARAMS  --
    ------------------

	constant LOG_OSR  : integer := clog2(OSR);
	constant REG_SIZE : integer := calc_reg_size(N, LOG_OSR, INPUT_SIZE, CONFIG);
	constant LSB_OUT  : integer := calc_lsb_out(REG_SIZE, OUTPUT_SIZE);

      --------------------
     -- WIRES AND REGS --
    --------------------

    signal samples_counter : unsigned(LOG_OSR-1 downto 0);              -- Counter, OSR periodic
    signal final_stage     : std_logic_vector(N*REG_SIZE-1 downto 0);   -- Output
    signal condition_valid : std_logic;
    signal accept_sample   : std_logic;

    signal Integ_stor : std_logic_vector(N*REG_SIZE-1 downto 0);   -- Integrator(s)
    signal Integrator  : std_logic_vector(N*REG_SIZE-1 downto 0);
    signal Comb_stor   : std_logic_vector(N*REG_SIZE-1 downto 0);  -- Comb(s)
    signal Comb        : std_logic_vector(N*REG_SIZE-1 downto 0);

begin

    --============================================================================--
    -- Code
    --============================================================================--

    o_ready <= (i_rst_n and accept_sample) when CONFIG = "interpolator" else i_rst_n;

    accept_sample <= '1' when (samples_counter = (samples_counter'range => '1')) and i_valid = '1' else '0';

    process(i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            samples_counter <= (others => '0');
        elsif rising_edge(i_clk) then
            if i_valid = '1' then
                samples_counter <= samples_counter + 1;
            end if;
        end if;
    end process;


    gen_interpolator : if CONFIG = "interpolator" generate

        signal stuffed_sample : std_logic_vector(REG_SIZE-1 downto 0);

    begin

          ------------------
         -- Combs Stages --
        ------------------

        Comb(REG_SIZE-1 downto 0) <= std_logic_vector(resize(unsigned(i_data), REG_SIZE) -
                                                        unsigned(Comb_stor(REG_SIZE-1 downto 0)));

        process(i_clk, i_rst_n)
        begin
            if i_rst_n = '0' then
                Comb_stor(REG_SIZE-1 downto 0) <= (others => '0');
            elsif rising_edge(i_clk) then
                if accept_sample = '1' then
                    Comb_stor(REG_SIZE-1 downto 0) <= std_logic_vector(resize(unsigned(i_data), REG_SIZE));
                end if;
            end if;
        end process;

        gen_combs : for i in 1 to N-1 generate
            Comb((i+1)*REG_SIZE-1 downto i*REG_SIZE) <=
                std_logic_vector(unsigned(Comb(i*REG_SIZE-1 downto (i-1)*REG_SIZE)) -
                                  unsigned(Comb_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE)));

            process(i_clk, i_rst_n)
            begin
                if i_rst_n = '0' then
                    Comb_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE) <= (others => '0');
                elsif rising_edge(i_clk) then
                    if accept_sample = '1' then
                        Comb_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE) <= Comb(i*REG_SIZE-1 downto (i-1)*REG_SIZE);
                    end if;
                end if;
            end process;
        end generate gen_combs;


          -----------------------
         -- Integrators Stages --
        -----------------------

        stuffed_sample <= Comb(N*REG_SIZE-1 downto (N-1)*REG_SIZE) when accept_sample = '1' else
                           (others => '0');

        Integrator(REG_SIZE-1 downto 0) <= std_logic_vector(unsigned(stuffed_sample) +
                                                              unsigned(Integ_stor(REG_SIZE-1 downto 0)));

        process(i_clk, i_rst_n)
        begin
            if i_rst_n = '0' then
                Integ_stor(REG_SIZE-1 downto 0) <= (others => '0');
            elsif rising_edge(i_clk) then
                if i_valid = '1' then
                    Integ_stor(REG_SIZE-1 downto 0) <= Integrator(REG_SIZE-1 downto 0);
                end if;
            end if;
        end process;

        gen_integrators : for i in 1 to N-1 generate
            Integrator((i+1)*REG_SIZE-1 downto i*REG_SIZE) <=
                std_logic_vector(unsigned(Integrator(i*REG_SIZE-1 downto (i-1)*REG_SIZE)) +
                                  unsigned(Integ_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE)));

            process(i_clk, i_rst_n)
            begin
                if i_rst_n = '0' then
                    Integ_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE) <= (others => '0');
                elsif rising_edge(i_clk) then
                    if i_valid = '1' then
                        Integ_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE) <= Integrator((i+1)*REG_SIZE-1 downto i*REG_SIZE);
                    end if;
                end if;
            end process;
        end generate gen_integrators;

    end generate gen_interpolator;


    gen_decimator : if CONFIG /= "interpolator" generate

          -----------------------
         -- Integrators Stages --
        -----------------------

        Integrator(REG_SIZE-1 downto 0) <= std_logic_vector(resize(unsigned(i_data), REG_SIZE) +
                                                              unsigned(Integ_stor(REG_SIZE-1 downto 0)));

        process(i_clk, i_rst_n)
        begin
            if i_rst_n = '0' then
                Integ_stor(REG_SIZE-1 downto 0) <= (others => '0');
            elsif rising_edge(i_clk) then
                if i_valid = '1' then
                    Integ_stor(REG_SIZE-1 downto 0) <= Integrator(REG_SIZE-1 downto 0);
                end if;
            end if;
        end process;

        gen_integrators : for i in 1 to N-1 generate
            Integrator((i+1)*REG_SIZE-1 downto i*REG_SIZE) <=
                std_logic_vector(unsigned(Integrator(i*REG_SIZE-1 downto (i-1)*REG_SIZE)) +
                                  unsigned(Integ_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE)));

            process(i_clk, i_rst_n)
            begin
                if i_rst_n = '0' then
                    Integ_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE) <= (others => '0');
                elsif rising_edge(i_clk) then
                    if i_valid = '1' then
                        Integ_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE) <= Integrator((i+1)*REG_SIZE-1 downto i*REG_SIZE);
                    end if;
                end if;
            end process;
        end generate gen_integrators;


          ------------------
         -- Combs Stages --
        ------------------

        Comb(REG_SIZE-1 downto 0) <= std_logic_vector(unsigned(Integrator(N*REG_SIZE-1 downto (N-1)*REG_SIZE)) -
                                                        unsigned(Comb_stor(REG_SIZE-1 downto 0)));

        process(i_clk, i_rst_n)
        begin
            if i_rst_n = '0' then
                Comb_stor(REG_SIZE-1 downto 0) <= (others => '0');
            elsif rising_edge(i_clk) then
                if accept_sample = '1' then
                    Comb_stor(REG_SIZE-1 downto 0) <= Integrator(N*REG_SIZE-1 downto (N-1)*REG_SIZE);
                end if;
            end if;
        end process;

        gen_combs : for i in 1 to N-1 generate
            Comb((i+1)*REG_SIZE-1 downto i*REG_SIZE) <=
                std_logic_vector(unsigned(Comb(i*REG_SIZE-1 downto (i-1)*REG_SIZE)) -
                                  unsigned(Comb_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE)));

            process(i_clk, i_rst_n)
            begin
                if i_rst_n = '0' then
                    Comb_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE) <= (others => '0');
                elsif rising_edge(i_clk) then
                    if accept_sample = '1' then
                        Comb_stor((i+1)*REG_SIZE-1 downto i*REG_SIZE) <= Comb(i*REG_SIZE-1 downto (i-1)*REG_SIZE);
                    end if;
                end if;
            end process;
        end generate gen_combs;

    end generate gen_decimator;


      -------------
     -- Outputs --
    -------------

    final_stage     <= Integrator when CONFIG = "interpolator" else Comb;
    condition_valid <= i_valid    when CONFIG = "interpolator" else accept_sample;

    process(i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            o_data <= (others => '0');
        elsif rising_edge(i_clk) then
            if condition_valid = '1' then
                o_data <= final_stage(N*REG_SIZE-1 downto LSB_OUT+(N-1)*REG_SIZE);
            end if;
        end if;
    end process;

    process(i_clk, i_rst_n)
    begin
        if i_rst_n = '0' then
            o_valid <= '0';
        elsif rising_edge(i_clk) then
            o_valid <= condition_valid;
        end if;
    end process;


end architecture rtl;
