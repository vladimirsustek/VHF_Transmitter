--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   03:43:41 02/02/2022
-- Design Name:   
-- Module Name:   Z:/ise_repo/VHF_Transmitter/tb_top.vhd
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
         CLK_LFC : IN  std_logic;
         BTNS : IN  std_logic_vector(1 downto 0);
         LEDS : OUT  std_logic_vector(3 downto 0);
         UART_NRTS : IN  std_logic;
         UART_TX : OUT  std_logic;
         UART_RX : IN  std_logic;
         UART_NCTS : IN  std_logic;
         UART_GND : OUT  std_logic;
         DAC1_NRESET : IN  std_logic;
         DAC1_NCLEAR : IN  std_logic;
         DAC1_NALERT : IN  std_logic;
         DAC1_NLDAC : OUT  std_logic;
         DAC1_SCLK : OUT  std_logic;
         DAC1_NCS : OUT  std_logic;
         DAC1_MOSI : OUT  std_logic;
         DAC1_MISO : IN  std_logic;
         ADC0_SCLK : OUT  std_logic;
         ADC0_SDI : IN  std_logic;
         ADC0_NCS : OUT  std_logic;
         ADC1_SCLK : OUT  std_logic;
         ADC1_SDI : IN  std_logic;
         ADC1_NCS : OUT  std_logic;
         DAC0_DATA : OUT  std_logic_vector(9 downto 0);
         DAC0_CLK : OUT  std_logic
        );
    END COMPONENT;
    

   --Inputs
   signal CLK : std_logic := '0';
   signal CLK_LFC : std_logic := '0';
   signal BTNS : std_logic_vector(1 downto 0) := (others => '0');
   signal UART_NRTS : std_logic := '0';
   signal UART_RX : std_logic := '0';
   signal UART_NCTS : std_logic := '0';
   signal DAC1_NRESET : std_logic := '0';
   signal DAC1_NCLEAR : std_logic := '0';
   signal DAC1_NALERT : std_logic := '0';
   signal DAC1_MISO : std_logic := '0';
   signal ADC0_SDI : std_logic := '0';
   signal ADC1_SDI : std_logic := '0';

 	--Outputs
   signal LEDS : std_logic_vector(3 downto 0);
   signal UART_TX : std_logic;
   signal UART_GND : std_logic;
   signal DAC1_NLDAC : std_logic;
   signal DAC1_SCLK : std_logic;
   signal DAC1_NCS : std_logic;
   signal DAC1_MOSI : std_logic;
   signal ADC0_SCLK : std_logic;
   signal ADC0_NCS : std_logic;
   signal ADC1_SCLK : std_logic;
   signal ADC1_NCS : std_logic;
   signal DAC0_DATA : std_logic_vector(9 downto 0);
   signal DAC0_CLK : std_logic;

   -- Clock period definitions
   constant CLK_period : time := 125 ns;
 
BEGIN
 
	-- Instantiate the Unit Under Test (UUT)
   uut: top PORT MAP (
          CLK => CLK,
          CLK_LFC => CLK_LFC,
          BTNS => BTNS,
          LEDS => LEDS,
          UART_NRTS => UART_NRTS,
          UART_TX => UART_TX,
          UART_RX => UART_RX,
          UART_NCTS => UART_NCTS,
          UART_GND => UART_GND,
          DAC1_NRESET => DAC1_NRESET,
          DAC1_NCLEAR => DAC1_NCLEAR,
          DAC1_NALERT => DAC1_NALERT,
          DAC1_NLDAC => DAC1_NLDAC,
          DAC1_SCLK => DAC1_SCLK,
          DAC1_NCS => DAC1_NCS,
          DAC1_MOSI => DAC1_MOSI,
          DAC1_MISO => DAC1_MISO,
          ADC0_SCLK => ADC0_SCLK,
          ADC0_SDI => ADC0_SDI,
          ADC0_NCS => ADC0_NCS,
          ADC1_SCLK => ADC1_SCLK,
          ADC1_SDI => ADC1_SDI,
          ADC1_NCS => ADC1_NCS,
          DAC0_DATA => DAC0_DATA,
          DAC0_CLK => DAC0_CLK
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
