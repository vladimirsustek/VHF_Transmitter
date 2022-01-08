--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   10:41:59 01/08/2022
-- Design Name:   
-- Module Name:   C:/VHF_Transmitter/tb_stereo_adc.vhd
-- Project Name:  VHF-Transmitter
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: stereo_adc
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
 
ENTITY tb_stereo_adc IS
END tb_stereo_adc;
 
ARCHITECTURE behavior OF tb_stereo_adc IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT stereo_adc
    PORT(
         clk_i : IN  std_logic;
         areset_i : IN  std_logic;
         adc0_sclk_o : OUT  std_logic;
         adc0_sdi_i : IN  std_logic;
         adc0_ncs_o : OUT  std_logic;
         adc1_sclk_o : OUT  std_logic;
         adc1_sdi_i : IN  std_logic;
         adc1_ncs_o : OUT  std_logic;
         en_sampl_i : IN  std_logic;
         lpr_o : OUT  std_logic_vector(15 downto 0);
         lmr_o : OUT  std_logic_vector(15 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal clk_i : std_logic := '0';
   signal areset_i : std_logic := '0';
   signal adc0_sdi_i : std_logic := '1';
   signal adc1_sdi_i : std_logic := '1';
   signal en_sampl_i : std_logic := '1';

 	--Outputs
   signal adc0_sclk_o : std_logic;
   signal adc0_ncs_o : std_logic;
   signal adc1_sclk_o : std_logic;
   signal adc1_ncs_o : std_logic;
   signal lpr_o : std_logic_vector(15 downto 0);
   signal lmr_o : std_logic_vector(15 downto 0);

   -- Clock period definitions
   constant clk_i_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: stereo_adc PORT MAP (
          clk_i => clk_i,
          areset_i => areset_i,
          adc0_sclk_o => adc0_sclk_o,
          adc0_sdi_i => adc0_sdi_i,
          adc0_ncs_o => adc0_ncs_o,
          adc1_sclk_o => adc1_sclk_o,
          adc1_sdi_i => adc1_sdi_i,
          adc1_ncs_o => adc1_ncs_o,
          en_sampl_i => en_sampl_i,
          lpr_o => lpr_o,
          lmr_o => lmr_o
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
      wait for 100 ns;	

      wait for clk_i_period*2;
		adc0_sdi_i <= '0';
		adc1_sdi_i <= '0';
		wait for clk_i_period*16;
      -- insert stimulus here 

      wait;
   end process;

END;
