----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    09:46:14 01/08/2022 
-- Design Name: 
-- Module Name:    stereo_adc - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity stereo_adc is
    Port ( clk_i : in  STD_LOGIC;
           areset_i : in  STD_LOGIC;
           adc0_sclk_o : out  STD_LOGIC;
           adc0_sdi_i : in  STD_LOGIC;
           adc0_ncs_o : out  STD_LOGIC;
           adc1_sclk_o : out  STD_LOGIC;
           adc1_sdi_i : in  STD_LOGIC;
           adc1_ncs_o : out  STD_LOGIC;
           en_sampl_i : in  STD_LOGIC;
           lpr_o : out  STD_LOGIC_VECTOR(15 downto 0);
           lmr_o : out  STD_LOGIC_VECTOR(15 downto 0));
end stereo_adc;

architecture Behavioral of stereo_adc is

COMPONENT adc_interface
PORT(
	clk_i : IN std_logic;
	reset_i : IN std_logic;
	clk_en_i : IN std_logic;
	miso_i : IN std_logic;          
	mosi_o : OUT std_logic;
	sclk_o : OUT std_logic;
	ncs_o : OUT std_logic;
	data_o : OUT std_logic_vector(7 downto 0)
	);
END COMPONENT;

signal sLeft : std_logic_vector(7 downto 0):= (others => '0');
signal sRight : std_logic_vector(7 downto 0):= (others => '0');

signal sLpR : std_logic_vector(11 downto 0):= (others => '0');

begin
		-- sLpR = sLeft + sRight => maximally = 255 + 255 = 511
		sLpR <= std_logic_vector(X"000" + unsigned(sLeft) + unsigned(sRight));
		
		-- lpr_o = sLpR*8 + sLpR/4 => maximally = 510*8 + 510/4 + 510/64 = 4214
		-- why multiplied ? because of FM (DDS's: delta_f*lpr_o = 4214*3.57 = 15043Hz (mono)
		-- therefore, multiplication is customized for delta_f = 3.57Hz
		lpr_o <= std_logic_vector(X"0000" + shift_left(unsigned(sLpR), 3) + shift_right(unsigned(sLpR), 2) + shift_right(unsigned(sLpR), 6));

		-- lmr_o = TODO: add L-P as needed for the stereo broadcast
		lmr_o <= (others => '0');
		
	   left : adc_interface PORT MAP(
		clk_i => clk_i,
		reset_i => areset_i,
		clk_en_i => en_sampl_i,
		miso_i => adc0_sdi_i,
		mosi_o => open,
		sclk_o => adc0_sclk_o,
		ncs_o => adc0_ncs_o,
		data_o => sLeft
	);
	
		right : adc_interface PORT MAP(
		clk_i => clk_i,
		reset_i => areset_i,
		clk_en_i => en_sampl_i,
		miso_i => adc1_sdi_i,
		mosi_o => open,
		sclk_o => adc1_sclk_o,
		ncs_o => adc1_ncs_o,
		data_o => sRight
	);

end Behavioral;

