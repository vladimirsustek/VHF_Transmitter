--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   14:16:13 02/02/2022
-- Design Name:   
-- Module Name:   Z:/ise_repo/VHF_Transmitter/tb_adc_interface.vhd
-- Project Name:  VHF-Transmitter
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: adc_interface
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY tb_adc_interface IS
END tb_adc_interface;
 
ARCHITECTURE behavior OF tb_adc_interface IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT adc_interface
    PORT(
         clk_i : IN  std_logic;
         reset_i : IN  std_logic;
         miso_i : IN  std_logic;
         mosi_o : OUT  std_logic;
         sclk_o : OUT  std_logic;
         ncs_o : OUT  std_logic;
         data_o : OUT  std_logic_vector(15 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal clk_i : std_logic := '0';
   signal reset_i : std_logic := '0';
   signal miso_i : std_logic := '1';

 	--Outputs
   signal mosi_o : std_logic;
   signal sclk_o : std_logic;
   signal ncs_o : std_logic;
   signal data_o : std_logic_vector(15 downto 0);

   -- Clock period definitions
   constant clk_i_period : time := 250 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: adc_interface PORT MAP (
          clk_i => clk_i,
          reset_i => reset_i,
          miso_i => miso_i,
          mosi_o => mosi_o,
          sclk_o => sclk_o,
          ncs_o => ncs_o,
          data_o => data_o
        );

   -- Clock process definitions
   clk_i_process :process
   begin
		clk_i <= '0';
		wait for clk_i_period/2;
		clk_i <= '1';
		wait for clk_i_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
		wait for clk_i_period;
      -- insert stimulus here 

      wait;
   end process;

END;
