----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    15:56:12 02/16/2022 
-- Design Name: 
-- Module Name:    arithemetics - Behavioral 
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

entity arithemetics is
    Port ( clk : in  STD_LOGIC;
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
end arithemetics;

architecture Behavioral of arithemetics is

constant cHalf : std_logic_vector(7 downto 0):= X"80";

-- here completely works conversion of the:
-- 128*sin(x*2pi/12) + 128 => 0/255
-- 128*sin(x*2pi/12) => -128/127

-- should be used to convert ADC offset signal
-- into sinewave -max / +max

signal sig0 : std_logic_vector(7 downto 0) := X"80";
signal sig1 : std_logic_vector(7 downto 0) := X"bf";
signal sig2 : std_logic_vector(7 downto 0) := X"ee";
signal sig3 : std_logic_vector(7 downto 0) := X"ff";
signal sig4 : std_logic_vector(7 downto 0) := X"ee";
signal sig5 : std_logic_vector(7 downto 0) := X"c0";
signal sig6 : std_logic_vector(7 downto 0) := X"80";
signal sig7 : std_logic_vector(7 downto 0) := X"40";
signal sig8 : std_logic_vector(7 downto 0) := X"11";
signal sig9 : std_logic_vector(7 downto 0) := X"00";
signal sig10 : std_logic_vector(7 downto 0) := X"10";
signal sig11 : std_logic_vector(7 downto 0) := X"3f";

begin



output0 <= std_logic_vector(signed(sig0) - signed(cHalf));
output1 <= std_logic_vector(signed(sig1) - signed(cHalf));
output2 <= std_logic_vector(signed(sig2) - signed(cHalf));
output3 <= std_logic_vector(signed(sig3) - signed(cHalf));
output4 <= std_logic_vector(signed(sig4) - signed(cHalf));
output5 <= std_logic_vector(signed(sig5) - signed(cHalf));
output6 <= std_logic_vector(signed(sig6) - signed(cHalf));
output7 <= std_logic_vector(signed(sig7) - signed(cHalf));
output8 <= std_logic_vector(signed(sig8) - signed(cHalf));
output9 <= std_logic_vector(signed(sig9) - signed(cHalf));
output10 <= std_logic_vector(signed(sig10) - signed(cHalf));
output11 <= std_logic_vector(signed(sig11) - signed(cHalf));



end Behavioral;

