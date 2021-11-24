----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    11:56:53 11/07/2021 
-- Design Name: 
-- Module Name:    clk_enabler - Behavioral 
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
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity clk_enabler is
    Port ( clk_i : in  STD_LOGIC;
           areset_i : in  STD_LOGIC;
           en16x57600_o : out  STD_LOGIC;
           en1ms_o : out  STD_LOGIC;
           en44kHz_o : out  STD_LOGIC);
end clk_enabler;

architecture Behavioral of clk_enabler is

signal sEn16x57600 : STD_LOGIC := '0';
signal sEn1ms : STD_LOGIC := '0';
signal sEn44kHz : STD_LOGIC := '0';

begin


-- 16MHz/(17.5) = 57142kHz ~ 0.8% error 
en16xsEn16x57600 : process(clk_i, areset_i)

variable vCount : integer range 0 to 64 := 0;

begin
	if areset_i = '1' then
		vCount := 0;
		sEn16x57600 <= '0';
	elsif rising_edge(clk_i) then
		if vCount = 16 then -- till 17
			sEn16x57600 <= '1';
			vCount := vCount + 1;
		elsif vCount = 33 then
			sEn16x57600 <= '1';
			vCount := 0;
		else
			sEn16x57600 <= '0';
			vCount := vCount + 1;
		end if;
	end if;
	en16x57600_o <= sEn16x57600;
end process;

en1ms : process(clk_i, areset_i)
begin
en1ms_o <= sEn1ms;
end process;

en144kHz : process(clk_i, areset_i)
begin
en44kHz_o <= sEn44kHz;
end process;

end Behavioral;

