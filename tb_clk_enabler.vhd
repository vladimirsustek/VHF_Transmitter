--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   13:08:31 01/08/2022
-- Design Name:   
-- Module Name:   C:/VHF_Transmitter/tb_clk_enabler.vhd
-- Project Name:  VHF-Transmitter
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: clk_enabler
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
 
ENTITY tb_clk_enabler IS
END tb_clk_enabler;
 
ARCHITECTURE behavior OF tb_clk_enabler IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT clk_enabler
    PORT(
         clk_i : IN  std_logic;
         areset_i : IN  std_logic;
         en16x57600_o : OUT  std_logic;
         en1ms_o : OUT  std_logic;
         en44kHz_o : OUT  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal clk_i : std_logic := '0';
   signal areset_i : std_logic := '0';

 	--Outputs
   signal en16x57600_o : std_logic;
   signal en1ms_o : std_logic;
   signal en44kHz_o : std_logic;

   -- Clock period definitions
   constant clk_i_period : time := 62500 ps;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: clk_enabler PORT MAP (
          clk_i => clk_i,
          areset_i => areset_i,
          en16x57600_o => en16x57600_o,
          en1ms_o => en1ms_o,
          en44kHz_o => en44kHz_o
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

      wait for clk_i_period*10;

      -- insert stimulus here 

      wait;
   end process;

END;
