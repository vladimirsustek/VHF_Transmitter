--------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
--use IEEE.STD_LOGIC_ARITH.ALL;
--use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.NUMERIC_STD.ALL;
--------------------------------------------------------------------------------
ENTITY lcd_driver IS
  PORT(
    clk             : IN    STD_LOGIC;
	 rst_i			  : IN	 STD_LOGIC;
    lcd_e           : OUT   STD_LOGIC;
    lcd_rs          : OUT   STD_LOGIC;
    lcd_rw          : OUT   STD_LOGIC;
    lcd_db          : INOUT STD_LOGIC_VECTOR( 7 DOWNTO 4);

    line1_00        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_01        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_02        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_03        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_04        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_05        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_06        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_07        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_08        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_09        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_10        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_11        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_12        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_13        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_14        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line1_15        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
                                                     
    line2_00        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_01        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_02        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_03        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_04        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_05        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_06        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_07        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_08        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_09        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_10        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_11        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_12        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_13        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_14        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0);
    line2_15        : IN  STD_LOGIC_VECTOR( 7 DOWNTO 0));
END lcd_driver;
--------------------------------------------------------------------------------
ARCHITECTURE OpenCores OF lcd_driver IS
--------------------------------------------------------------------------------

  COMPONENT lcd16x2_ctrl
  GENERIC (
    CLK_PERIOD_NS   : POSITIVE := 40);    -- 25MHz
  PORT (
    clk             : IN  STD_LOGIC;
    rst             : IN  STD_LOGIC;
    lcd_e           : OUT STD_LOGIC;
    lcd_rs          : OUT STD_LOGIC;
    lcd_rw          : OUT STD_LOGIC;
    lcd_db          : OUT STD_LOGIC_VECTOR(  7 DOWNTO 4);
    line1_buffer    : IN  STD_LOGIC_VECTOR(127 DOWNTO 0);  -- 16x8bit
    line2_buffer    : IN  STD_LOGIC_VECTOR(127 DOWNTO 0)); 
  END COMPONENT;

  ------------------------------------------------------------------------------

  SIGNAL rst            : STD_LOGIC := '1';
  SIGNAL cnt_rst        : UNSIGNED(  7 DOWNTO 0)  := (OTHERS => '0');

  SIGNAL line1_buffer   : STD_LOGIC_VECTOR(127 DOWNTO 0);
  SIGNAL line2_buffer   : STD_LOGIC_VECTOR(127 DOWNTO 0);

--------------------------------------------------------------------------------
BEGIN
--------------------------------------------------------------------------------

  reset: PROCESS(clk, rst_i) BEGIN
    IF rising_edge(clk) THEN
      IF cnt_rst(cnt_rst'HIGH) = '0' and rst_i = '0' THEN
        cnt_rst <= cnt_rst + 1;
      END IF;
		if rst_i = '1' then
			rst <= '1';
			cnt_rst <= (others => '0');
		else
			rst <= NOT cnt_rst(cnt_rst'HIGH);
		end if;
    END IF;
  END PROCESS reset;

--  reset: PROCESS(clk) BEGIN
--    IF rising_edge(clk) THEN
--      IF cnt_rst(cnt_rst'HIGH) = '0' THEN
--        cnt_rst <= cnt_rst + 1;
--      END IF;
--		rst <= NOT cnt_rst(cnt_rst'HIGH);
--    END IF;
--  END PROCESS reset;
  
  ------------------------------------------------------------------------------

  lcd16x2_ctrl_i : lcd16x2_ctrl
  GENERIC MAP(
    CLK_PERIOD_NS   => 20)          -- 50MHz
  PORT MAP(
    clk             => clk,
    rst             => rst,
    lcd_e           => lcd_e,
    lcd_rs          => lcd_rs,
    lcd_rw          => lcd_rw,
    lcd_db          => lcd_db,
    line1_buffer    => line1_buffer,
    line2_buffer    => line2_buffer);

  ------------------------------------------------------------------------------

  line1_buffer(  7 DOWNTO   0) <= line1_15;
  line1_buffer( 15 DOWNTO   8) <= line1_14;
  line1_buffer( 23 DOWNTO  16) <= line1_13;
  line1_buffer( 31 DOWNTO  24) <= line1_12;
  line1_buffer( 39 DOWNTO  32) <= line1_11;
  line1_buffer( 47 DOWNTO  40) <= line1_10;
  line1_buffer( 55 DOWNTO  48) <= line1_09;
  line1_buffer( 63 DOWNTO  56) <= line1_08;
  line1_buffer( 71 DOWNTO  64) <= line1_07;
  line1_buffer( 79 DOWNTO  72) <= line1_06;
  line1_buffer( 87 DOWNTO  80) <= line1_05;
  line1_buffer( 95 DOWNTO  88) <= line1_04;
  line1_buffer(103 DOWNTO  96) <= line1_03;
  line1_buffer(111 DOWNTO 104) <= line1_02;
  line1_buffer(119 DOWNTO 112) <= line1_01;
  line1_buffer(127 DOWNTO 120) <= line1_00;
      
  line2_buffer(  7 DOWNTO   0) <= line2_15;
  line2_buffer( 15 DOWNTO   8) <= line2_14;
  line2_buffer( 23 DOWNTO  16) <= line2_13;
  line2_buffer( 31 DOWNTO  24) <= line2_12;
  line2_buffer( 39 DOWNTO  32) <= line2_11;
  line2_buffer( 47 DOWNTO  40) <= line2_10;
  line2_buffer( 55 DOWNTO  48) <= line2_09;
  line2_buffer( 63 DOWNTO  56) <= line2_08;
  line2_buffer( 71 DOWNTO  64) <= line2_07;
  line2_buffer( 79 DOWNTO  72) <= line2_06;
  line2_buffer( 87 DOWNTO  80) <= line2_05;
  line2_buffer( 95 DOWNTO  88) <= line2_04;
  line2_buffer(103 DOWNTO  96) <= line2_03;
  line2_buffer(111 DOWNTO 104) <= line2_02;
  line2_buffer(119 DOWNTO 112) <= line2_01;
  line2_buffer(127 DOWNTO 120) <= line2_00;

--------------------------------------------------------------------------------
END OpenCores;
--------------------------------------------------------------------------------
