--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   23:05:33 11/24/2021
-- Design Name:   
-- Module Name:   C:/VHF_Transmitter/tb_top.vhd
-- Project Name:  VHF-Transmitter
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: top
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
 
ENTITY tb_top IS
END tb_top;
 
ARCHITECTURE behavior OF tb_top IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT top
    PORT(
         CLK : IN  std_logic;
         BTNS : IN  std_logic_vector(1 downto 0);
         LEDS : OUT  std_logic_vector(3 downto 0);
         UART_TX : OUT  std_logic;
         UART_RX : IN  std_logic;
         PORTD : IN  std_logic_vector(7 downto 6);
         PORTE : IN  std_logic_vector(0 downto 0);
         DAC1_NLDAC : OUT  std_logic;
         DAC1_SCLK : OUT  std_logic;
         DAC1_NCS : OUT  std_logic;
         DAC1_MOSI : OUT  std_logic;
         DAC1_MISO : IN  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal CLK : std_logic := '0';
   signal BTNS : std_logic_vector(1 downto 0) := (others => '0');
   signal UART_RX : std_logic := '0';
   signal PORTD : std_logic_vector(7 downto 6) := (others => '0');
   signal PORTE : std_logic_vector(0 downto 0) := (others => '0');
   signal DAC1_MISO : std_logic := '0';

 	--Outputs
   signal LEDS : std_logic_vector(3 downto 0);
   signal UART_TX : std_logic;
   signal DAC1_NLDAC : std_logic;
   signal DAC1_SCLK : std_logic;
   signal DAC1_NCS : std_logic;
   signal DAC1_MOSI : std_logic;

   -- Clock period definitions
   constant CLK_period : time := 125 ns;
   constant DAC1_SCLK_period : time := 10 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: top PORT MAP (
          CLK => CLK,
          BTNS => BTNS,
          LEDS => LEDS,
          UART_TX => UART_TX,
          UART_RX => UART_RX,
          PORTD => PORTD,
          PORTE => PORTE,
          DAC1_NLDAC => DAC1_NLDAC,
          DAC1_SCLK => DAC1_SCLK,
          DAC1_NCS => DAC1_NCS,
          DAC1_MOSI => DAC1_MOSI,
          DAC1_MISO => DAC1_MISO
        );

   -- Clock process definitions
   CLK_process :process
   begin
		CLK <= '0';
		wait for CLK_period/2;
		CLK <= '1';
		wait for CLK_period/2;
   end process;
 
   -- Stimulus process
   stim_proc: process
   begin		
      -- hold reset state for 100 ns.
      wait for 100 ns;	

      wait for CLK_period*10;

      -- insert stimulus here 

      wait;
   end process;

END;
