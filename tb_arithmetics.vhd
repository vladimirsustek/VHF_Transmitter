--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   16:06:37 02/16/2022
-- Design Name:   
-- Module Name:   /home/ise/ISE_SHARED/VHF_Transmitter/tb_arithmetics.vhd
-- Project Name:  VHF-Transmitter
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: arithemetics
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
 
ENTITY tb_arithmetics IS
END tb_arithmetics;
 
ARCHITECTURE behavior OF tb_arithmetics IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT arithemetics
    PORT(
         clk : IN  std_logic;
         output0 : out  std_logic_vector(7 downto 0);
         output1 : out  std_logic_vector(7 downto 0);
         output2 : out  std_logic_vector(7 downto 0);
         output3 : out  std_logic_vector(7 downto 0);
         output4 : out  std_logic_vector(7 downto 0);
         output5 : out  std_logic_vector(7 downto 0);
         output6 : out  std_logic_vector(7 downto 0);
         output7 : out  std_logic_vector(7 downto 0);
         output8 : out  std_logic_vector(7 downto 0);
         output9 : out  std_logic_vector(7 downto 0);
         output10 : out  std_logic_vector(7 downto 0);
         output11 : out  std_logic_vector(7 downto 0));
    END COMPONENT;
    

   --Inputs
   signal clk : std_logic := '0';

 	--Outputs
   signal output0 : std_logic_vector(7 downto 0);
   signal output1 : std_logic_vector(7 downto 0);
   signal output2 : std_logic_vector(7 downto 0);
   signal output3 : std_logic_vector(7 downto 0);
   signal output4 : std_logic_vector(7 downto 0);
   signal output5 : std_logic_vector(7 downto 0);
   signal output6 : std_logic_vector(7 downto 0);
   signal output7 : std_logic_vector(7 downto 0);
   signal output8 : std_logic_vector(7 downto 0);
   signal output9 : std_logic_vector(7 downto 0);
   signal output10 : std_logic_vector(7 downto 0);
   signal output11 : std_logic_vector(7 downto 0);
   -- Clock period definitions
   constant clk_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: arithemetics PORT MAP (
          clk => clk,
          output0 => output0,
          output1 => output1,
          output2 => output2,
          output3 => output3,
          output4 => output4,
          output5 => output5,
          output6 => output6,
          output7 => output7,
          output8 => output8,
          output9 => output9,
          output10 => output10,
          output11 => output11
        );

   -- Clock process definitions
   clk_process :process
   begin
		clk <= '0';
		wait for clk_period/2;
		clk <= '1';
		wait for clk_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
      wait for 100 ns;	

      wait for clk_period*10;

      -- insert stimulus here 

      wait;
   end process;

END;
