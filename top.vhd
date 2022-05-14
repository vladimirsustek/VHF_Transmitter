----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    12:23:38 11/05/2021 
-- Design Name: 
-- Module Name:    top - Behavioral 
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
library UNISIM;
use UNISIM.VComponents.all;



entity top is
		Port (CLK : in  STD_LOGIC;
		      CLK_LFC : in STD_LOGIC;
		
				BTNS : in  STD_LOGIC_VECTOR (1 downto 0);
				LEDS : out  STD_LOGIC_VECTOR (3 downto 0);
				
				UART_NRTS : in STD_LOGIC;
				UART_TX : out STD_LOGIC;
				UART_RX : in STD_LOGIC;
				UART_NCTS : in STD_LOGIC;
				UART_GND : out STD_LOGIC;
				
				DAC1_NRESET : in std_logic;
				DAC1_NCLEAR : in std_logic;
				DAC1_NALERT: in std_logic;
				DAC1_NLDAC : out std_logic;
				DAC1_SCLK : out std_logic;
				DAC1_NCS : out std_logic;
				DAC1_MOSI : out std_logic;
				DAC1_MISO : in std_logic;

				ADC0_SCLK : out std_logic;
				ADC0_SDI : in std_logic;
				ADC0_NCS : out std_logic;
				
				ADC1_SCLK : out std_logic;
				ADC1_SDI : in std_logic;
				ADC1_NCS : out std_logic;
				
				LCD_RS : out std_logic;
				LCD_RW : out std_logic;
				LCD_E : out std_logic;
				LCD_DB : inout std_logic_vector(3 downto 0);

				RF_AMP : out std_logic;
				DAC0_DATA : out std_logic_vector(9 downto 0);
				DAC0_CLK : out std_logic);
end top;

architecture Behavioral of top is

	-----------------------------------------------------------------------------
	--								COMPONENT DECLARATIONS
	-----------------------------------------------------------------------------

	COMPONENT clk_enabler
	PORT(
		clk_i : IN std_logic;
		areset_i : IN std_logic;          
		en16x57600_o : OUT std_logic;
		en1ms_o : OUT std_logic;
		tick1ms_o : out STD_LOGIC_VECTOR(31 downto 0)
		);
	END COMPONENT;

	component clkup
		port
		 (-- Clock in ports
		  CLK_IN1           : in     std_logic;
		  -- Clock out ports
		  CLK_OUT1          : out    std_logic;
		  CLK_OUT2          : out    std_logic;
		  CLK_OUT3          : out    std_logic
		 );
	end component;
	
	COMPONENT lcd_driver
	PORT(
		clk : IN std_logic;
		rst_i : IN std_logic;
		line1_00 : IN std_logic_vector(7 downto 0);
		line1_01 : IN std_logic_vector(7 downto 0);
		line1_02 : IN std_logic_vector(7 downto 0);
		line1_03 : IN std_logic_vector(7 downto 0);
		line1_04 : IN std_logic_vector(7 downto 0);
		line1_05 : IN std_logic_vector(7 downto 0);
		line1_06 : IN std_logic_vector(7 downto 0);
		line1_07 : IN std_logic_vector(7 downto 0);
		line1_08 : IN std_logic_vector(7 downto 0);
		line1_09 : IN std_logic_vector(7 downto 0);
		line1_10 : IN std_logic_vector(7 downto 0);
		line1_11 : IN std_logic_vector(7 downto 0);
		line1_12 : IN std_logic_vector(7 downto 0);
		line1_13 : IN std_logic_vector(7 downto 0);
		line1_14 : IN std_logic_vector(7 downto 0);
		line1_15 : IN std_logic_vector(7 downto 0);
		line2_00 : IN std_logic_vector(7 downto 0);
		line2_01 : IN std_logic_vector(7 downto 0);
		line2_02 : IN std_logic_vector(7 downto 0);
		line2_03 : IN std_logic_vector(7 downto 0);
		line2_04 : IN std_logic_vector(7 downto 0);
		line2_05 : IN std_logic_vector(7 downto 0);
		line2_06 : IN std_logic_vector(7 downto 0);
		line2_07 : IN std_logic_vector(7 downto 0);
		line2_08 : IN std_logic_vector(7 downto 0);
		line2_09 : IN std_logic_vector(7 downto 0);
		line2_10 : IN std_logic_vector(7 downto 0);
		line2_11 : IN std_logic_vector(7 downto 0);
		line2_12 : IN std_logic_vector(7 downto 0);
		line2_13 : IN std_logic_vector(7 downto 0);
		line2_14 : IN std_logic_vector(7 downto 0);
		line2_15 : IN std_logic_vector(7 downto 0);    
		lcd_db : INOUT std_logic_vector(7 downto 4);      
		lcd_e : OUT std_logic;
		lcd_rs : OUT std_logic;
		lcd_rw : OUT std_logic
		);
	END COMPONENT;
	
  component kcpsm6
  generic(                 hwbuild : std_logic_vector(7 downto 0) := X"00";
                  interrupt_vector : std_logic_vector(11 downto 0) := X"3FF";
           scratch_pad_memory_size : integer := 256);
  port (                   address : out std_logic_vector(11 downto 0);
                       instruction : in std_logic_vector(17 downto 0);
                       bram_enable : out std_logic;
                           in_port : in std_logic_vector(7 downto 0);
                          out_port : out std_logic_vector(7 downto 0);
                           port_id : out std_logic_vector(7 downto 0);
                      write_strobe : out std_logic;
                    k_write_strobe : out std_logic;
                       read_strobe : out std_logic;
                         interrupt : in std_logic;
                     interrupt_ack : out std_logic;
                             sleep : in std_logic;
                             reset : in std_logic;
                               clk : in std_logic);
	end component;
	
  COMPONENT MAIN
  generic(             C_FAMILY : string := "S6"; 
              C_RAM_SIZE_KWORDS : integer := 4;
           C_JTAG_LOADER_ENABLE : integer := 1);
  Port (      address : in std_logic_vector(11 downto 0);
          instruction : out std_logic_vector(17 downto 0);
               enable : in std_logic;
                  rdl : out std_logic;                    
                  clk : in std_logic);
	END COMPONENT;	

	COMPONENT uart_tx6
	PORT(
		data_in : IN std_logic_vector(7 downto 0);
		en_16_x_baud : IN std_logic;
		buffer_write : IN std_logic;
		buffer_reset : IN std_logic;
		clk : IN std_logic;          
		serial_out : OUT std_logic;
		buffer_data_present : OUT std_logic;
		buffer_half_full : OUT std_logic;
		buffer_full : OUT std_logic
		);
	END COMPONENT;
	
	COMPONENT uart_rx6
	PORT(
		serial_in : IN std_logic;
		en_16_x_baud : IN std_logic;
		buffer_read : IN std_logic;
		buffer_reset : IN std_logic;
		clk : IN std_logic;          
		data_out : OUT std_logic_vector(7 downto 0);
		buffer_data_present : OUT std_logic;
		buffer_half_full : OUT std_logic;
		buffer_full : OUT std_logic
		);
	END COMPONENT;
	
	COMPONENT da1_interface
	PORT(
		clk_i : IN std_logic;
		reset_i : IN std_logic;
		miso_i : IN std_logic;
		voltage_i : IN std_logic_vector(15 downto 0);          
		mosi_o : OUT std_logic;
		sclk_o : OUT std_logic;
		ncs_o : OUT std_logic;
		nldac_o : OUT std_logic
		);
	END COMPONENT;
	
	COMPONENT DDS_RF
	PORT (
		 clk : IN STD_LOGIC;
		 pinc_in : IN STD_LOGIC_VECTOR(24 DOWNTO 0);
		 sine : OUT STD_LOGIC_VECTOR(8 DOWNTO 0)
	  );
	END COMPONENT;

	COMPONENT DDS_RF3
	  PORT (
		 clk : IN STD_LOGIC;
		 pinc_in : IN STD_LOGIC_VECTOR(24 DOWNTO 0);
		 sine : OUT STD_LOGIC_VECTOR(7 DOWNTO 0)
	  );
	END COMPONENT;
	
	COMPONENT DDS_RF_2
	  PORT (
		 clk : IN STD_LOGIC;
		 pinc_in : IN STD_LOGIC_VECTOR(24 DOWNTO 0);
		 sine : OUT STD_LOGIC_VECTOR(5 DOWNTO 0)
	  );
	END COMPONENT;
	
	COMPONENT DDS_LF
	  PORT (
		 clk : IN STD_LOGIC;
		 pinc_in : IN STD_LOGIC_VECTOR(23 DOWNTO 0);
		 sine : OUT STD_LOGIC_VECTOR(11 DOWNTO 0)
	  );
	END COMPONENT;

	COMPONENT adc_interface
	PORT(
		clk_i : IN std_logic;
		reset_i : IN std_logic;
		miso_i : IN std_logic;          
		mosi_o : OUT std_logic;
		sclk_o : OUT std_logic;
		ncs_o : OUT std_logic;
		data_o : OUT std_logic_vector(15 downto 0)
		);
	END COMPONENT;
	
	COMPONENT stereo_mplx
		  PORT (
			 clk : IN STD_LOGIC;
			 channel : OUT STD_LOGIC_VECTOR(0 DOWNTO 0);
			 sine : OUT STD_LOGIC_VECTOR(10 DOWNTO 0)
		  );
	END COMPONENT;
	
	COMPONENT signal_operation
	PORT (
		a : IN STD_LOGIC_VECTOR(9 DOWNTO 0);
		b : IN STD_LOGIC_VECTOR(9 DOWNTO 0);
		clk : IN STD_LOGIC;
		ce : IN STD_LOGIC;
		s : OUT STD_LOGIC_VECTOR(9 DOWNTO 0)
	);
	END COMPONENT;

type tones_t is array (7 downto 0) of std_logic_vector(23 downto 0);
constant cTones : tones_t := (X"000224", X"000267", X"0002B3", X"0002DB", X"000336", X"00039A", X"00040B", X"000449");

-- PICOBLAZE CONSTANTS -----------------------------------------
-- input ports
constant cBTNPort : std_logic_vector(7 downto 0) := X"00";
constant cUARTStatusPort : std_logic_vector(7 downto 0) := X"01";
constant cRxUARTDataPort : std_logic_vector(7 downto 0) := X"02";

constant sTickRead31_25Port : std_logic_vector(7 downto 0) := X"03";
constant sTickRead24_16Port : std_logic_vector(7 downto 0) := X"04";
constant sTickRead15_08Port : std_logic_vector(7 downto 0) := X"05";
constant sTickRead07_00Port : std_logic_vector(7 downto 0) := X"06";

-- output ports
constant cLEDPort : std_logic_vector(7 downto 0) := X"00";
constant cTxUARTDataPort : std_logic_vector(7 downto 0) := X"01";
constant cUARTRstPort : std_logic_vector(3 downto 0) := X"0";

constant cDAC1_07_00Port : std_logic_vector(7 downto 0) := X"03";
constant cDAC1_15_08Port : std_logic_vector(7 downto 0) := X"04";

constant cDAC0_07_00Port : std_logic_vector(7 downto 0) := X"05";
constant cDAC0_15_08Port : std_logic_vector(7 downto 0) := X"06";
constant cDAC0_23_16Port : std_logic_vector(7 downto 0) := X"07";
constant cDAC0_31_24Port : std_logic_vector(7 downto 0) := X"08";

constant cRFModePort : std_logic_vector(7 downto 0) := X"09";
constant cSystemResetPort : std_logic_vector(7 downto 0) := X"0A";

constant cLCDPort1_00 : std_logic_vector(7 downto 0) := X"10";
constant cLCDPort1_01 : std_logic_vector(7 downto 0) := X"11";
constant cLCDPort1_02 : std_logic_vector(7 downto 0) := X"12";
constant cLCDPort1_03 : std_logic_vector(7 downto 0) := X"13";
constant cLCDPort1_04 : std_logic_vector(7 downto 0) := X"14";
constant cLCDPort1_05 : std_logic_vector(7 downto 0) := X"15";
constant cLCDPort1_06 : std_logic_vector(7 downto 0) := X"16";
constant cLCDPort1_07 : std_logic_vector(7 downto 0) := X"17";
constant cLCDPort1_08 : std_logic_vector(7 downto 0) := X"18";
constant cLCDPort1_09 : std_logic_vector(7 downto 0) := X"19";
constant cLCDPort1_10 : std_logic_vector(7 downto 0) := X"1A";
constant cLCDPort1_11 : std_logic_vector(7 downto 0) := X"1B";
constant cLCDPort1_12 : std_logic_vector(7 downto 0) := X"1C";
constant cLCDPort1_13 : std_logic_vector(7 downto 0) := X"1D";
constant cLCDPort1_14 : std_logic_vector(7 downto 0) := X"1E";
constant cLCDPort1_15 : std_logic_vector(7 downto 0) := X"1F";

constant cLCDPort2_00 : std_logic_vector(7 downto 0) := X"20";
constant cLCDPort2_01 : std_logic_vector(7 downto 0) := X"21";
constant cLCDPort2_02 : std_logic_vector(7 downto 0) := X"22";
constant cLCDPort2_03 : std_logic_vector(7 downto 0) := X"23";
constant cLCDPort2_04 : std_logic_vector(7 downto 0) := X"24";
constant cLCDPort2_05 : std_logic_vector(7 downto 0) := X"25";
constant cLCDPort2_06 : std_logic_vector(7 downto 0) := X"26";
constant cLCDPort2_07 : std_logic_vector(7 downto 0) := X"27";
constant cLCDPort2_08 : std_logic_vector(7 downto 0) := X"28";
constant cLCDPort2_09 : std_logic_vector(7 downto 0) := X"29";
constant cLCDPort2_10 : std_logic_vector(7 downto 0) := X"2A";
constant cLCDPort2_11 : std_logic_vector(7 downto 0) := X"2B";
constant cLCDPort2_12 : std_logic_vector(7 downto 0) := X"2C";
constant cLCDPort2_13 : std_logic_vector(7 downto 0) := X"2D";
constant cLCDPort2_14 : std_logic_vector(7 downto 0) := X"2E";
constant cLCDPort2_15 : std_logic_vector(7 downto 0) := X"2F";
----------------------------------------------------------------

constant cDAC0_21_1MHz_phInc : std_logic_vector(24 downto 0) := "0010110100000011010110101";
constant cDAC0_30Hz_Offset : std_logic_vector(15 downto 0):= X"0008";

constant c38kHz : std_logic_vector(15 downto 0):= X"2981";
constant c19kHz : std_logic_vector(15 downto 0):= X"14C0";
constant c7k5Hz : std_logic_vector(15 downto 0):= X"0831";

constant cRFTestSine : std_logic_vector(7 downto 0) := X"01";
constant cRFTestSinePilot : std_logic_vector(7 downto 0) := X"02";
constant cRFMono : std_logic_vector(7 downto 0) := X"03";
constant cRFMonoPilot : std_logic_vector(7 downto 0) := X"04";
constant cRFMonoStereo : std_logic_vector(7 downto 0) := X"05";
constant cRFMonoStereoPilot : std_logic_vector(7 downto 0) := X"06";

constant cRFAmpStageOn : std_logic := '1';
constant cRFAmpStageOff : std_logic := '0';
	-----------------------------------------------------------------------------
	--								SIGNALS
	-----------------------------------------------------------------------------
	
-- Central signals
signal sCentralReset : std_logic := '0';
signal sCLK4MHz : std_logic := '0';
signal sCLK16MHz : std_logic := '0';
signal sCLK120MHz : std_logic := '0';
signal sNCLK120MHz : std_logic := '0';
signal sEn16x57600 : std_logic := '0';
signal sTick1ms : std_logic_vector(31 downto 0):= (others => '0');

-- Picoblaze
signal sMcuAddress : std_logic_vector(11 downto 0) := (others => '0');
signal sMcuInstruction : std_logic_vector(17 downto 0) := (others => '0');
signal sMcuBramEnable : std_logic := '0';
signal sMcuInPort : std_logic_vector(7 downto 0) := (others => '0');
signal sMcuOutPort : std_logic_vector(7 downto 0) := (others => '0');
signal sMcuPortId : std_logic_vector(7 downto 0) := (others => '0');
signal sMcuWriteStrobe : std_logic := '0';
signal sMcuKwriteStrobe : std_logic := '0';
signal sMcuReadStrobe : std_logic := '0';
signal sMcuInterrupt : std_logic := '0';
signal sMcuInterruptAck : std_logic := '0';
signal sMcuSleep : std_logic := '0';
signal sMcuReset : std_logic := '0';
signal sROMResetDuringLoad : std_logic := '0';

-- TX UART
signal sTxUARTreset : std_logic := '0';
signal sTxUARTdata : std_logic_vector(7 downto 0) := (others => '0');
signal sTxUARTOdata : std_logic_vector(7 downto 0) := (others => '0');
signal sTxUARTkOdata : std_logic_vector(7 downto 0) := (others => '0');
signal sTxUARTtxDataPresent : std_logic := '0';
signal sTxUARTHalfFull : std_logic := '0';
signal sTxUARTFull : std_logic := '0';
signal sTxUARTWrite : std_logic := '0';

-- RX UART
signal sRxUARTreset : std_logic := '0';
signal sRxUARTrxDataPresent : std_logic := '0';
signal sRxUARTHalfFull : std_logic := '0';
signal sRxUARTFull : std_logic := '0';
signal sRxUARTRead : std_logic := '0';
signal sRxUARTdata : std_logic_vector(7 downto 0) := (others => '0');

-- RF DAC1
signal sDAC1BufferedValue : std_logic_vector(15 downto 0) := (others => '0');
signal sDAC1ReadyValue : std_logic_vector(15 downto 0) := (others => '0');
signal sDAC1WritePortReq : std_logic := '0';
signal sDAC1WritePortAck : std_logic := '0';

-- RF DAC0
signal sDDSMonophInc : std_logic_vector(24 downto 0) := (others => '0');
signal sDDSPilotphInc : std_logic_vector(24 downto 0) := (others => '0');
signal sDDSStereophInc : std_logic_vector(24 downto 0) := (others => '0');
signal sDDSRDSphInc : std_logic_vector(24 downto 0) := (others => '0');
--signal sDDSphIncCtrl : std_logic_vector(24 downto 0) := cDAC0_21_1MHz_phInc;
signal sDDSphIncCtrl : std_logic_vector(24 downto 0) := (others => '0');
signal sDDSMono : std_logic_vector(8 downto 0) := (others => '0');
signal sDDSPilot : std_logic_vector(5 downto 0) := (others => '0');
signal sDDSStereo : std_logic_vector(7 downto 0) := (others => '0');
signal sDDSRDS : std_logic_vector(4 downto 0) := (others => '0');
signal sSumDDS : std_logic_vector(9 downto 0):= (others => '0');
signal sRFMode : std_logic_vector(7 downto 0) := (others => '0');

 -- AUDIO
signal sAudioL : std_logic_vector(15 downto 0):= (others => '0');
signal sAudioR : std_logic_vector(15 downto 0):= (others => '0');
signal sAudioLmR : std_logic_vector(15 downto 0):= (others => '0');
signal sAudioLpR : std_logic_vector(15 downto 0) := (others => '0');
signal sAudioSSBSC : std_logic_vector(31 downto 0) := (others => '0');
signal sAudioMplx32b : std_logic_vector(31 downto 0):= (others => '0');
signal sAudioMplx : std_logic_vector(12 downto 0):= (others => '0');
signal sSinexxKHz : std_logic_vector(10 downto 0):= (others => '0');
signal sSine38KHz : std_logic_vector(10 downto 0):= (others => '0');
signal sSine19KHz : std_logic_vector(10 downto 0):= (others => '0');
signal sSineChannel : std_logic_vector(0 downto 0):= (others => '0');
signal sAudioTestToneSine : std_logic_vector(11 downto 0) := (others => '0');
signal sAudioTestTonePhInc : std_logic_vector(23 downto 0) := (others => '0');

-- LCD (X"20" stands for "WHITESPACE")
signal sLCDLine1_00 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_01 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_02 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_03 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_04 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_05 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_06 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_07 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_08 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_09 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_10 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_11 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_12 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_13 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_14 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine1_15 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_00 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_01 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_02 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_03 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_04 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_05 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_06 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_07 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_08 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_09 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_10 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_11 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_12 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_13 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_14 : std_logic_vector(7 downto 0) := x"20";
signal sLCDLine2_15 : std_logic_vector(7 downto 0) := x"20";

begin

	-----------------------------------------------------------------------------
	-----------------------------------------------------------------------------
	--								CENTRAL COMBINATIONAL LOGIC
	-----------------------------------------------------------------------------
	-----------------------------------------------------------------------------

	sCentralReset <= '1' when (BTNS = "11") else '0';
	
	MCU_Reset : process(sCentralReset, sROMResetDuringLoad)
	begin
		if sCentralReset = '1' then
			sMcuReset <= '1';
		elsif sROMResetDuringLoad = '1' then
			sMcuReset <= '1';
		else
			sMcuReset <= '0';
		end if;
	end process;
	
	-----------------------------------------------------------------------------
	-----------------------------------------------------------------------------
	--								 		PICOBLAZE 
	-----------------------------------------------------------------------------
	-----------------------------------------------------------------------------
	
	-----------------------------------------------------------------------------
	-- 								 INSTANTIATION
	-----------------------------------------------------------------------------

	program_rom: MAIN PORT MAP(
		address => sMcuAddress,
		instruction => sMcuInstruction,
		enable => sMcuBramEnable,
		rdl => sROMResetDuringLoad ,
		clk => sCLK16MHz);
	
  processor: kcpsm6 generic map ( 
		hwbuild => X"00", 
		interrupt_vector => X"3FF",
		scratch_pad_memory_size => 128)
	port map(
		address => sMcuAddress,
		instruction => sMcuInstruction,
		bram_enable => sMcuBramEnable,
		port_id => sMcuPortId,
		write_strobe => sMcuWriteStrobe,
		k_write_strobe => sMcuKwriteStrobe,
		out_port => sMcuOutPort,
		read_strobe => sMcuReadStrobe,
		in_port => sMcuInPort,
		interrupt => sMcuInterrupt,
		interrupt_ack => sMcuInterruptAck,
		sleep => sTxUARTFull,
		reset => sMcuReset,
		clk => sCLK16MHz);
		
		lcd: lcd_driver PORT MAP(
		clk => sCLK4MHz,
		rst_i => sCentralReset,
		lcd_e => LCD_E,
		lcd_rs => LCD_RS,
		lcd_rw => LCD_RW,
		lcd_db => LCD_DB,
		line1_00 => sLCDLine1_00,
		line1_01 => sLCDLine1_01,
		line1_02 => sLCDLine1_02,
		line1_03 => sLCDLine1_03,
		line1_04 => sLCDLine1_04,
		line1_05 => sLCDLine1_05,
		line1_06 => sLCDLine1_06,
		line1_07 => sLCDLine1_07,
		line1_08 => sLCDLine1_08,
		line1_09 => sLCDLine1_09,
		line1_10 => sLCDLine1_10,
		line1_11 => sLCDLine1_11,
		line1_12 => sLCDLine1_12,
		line1_13 => sLCDLine1_13,
		line1_14 => sLCDLine1_14,
		line1_15 => sLCDLine1_15,
		line2_00 => sLCDLine2_00,
		line2_01 => sLCDLine2_01,
		line2_02 => sLCDLine2_02,
		line2_03 => sLCDLine2_03,
		line2_04 => sLCDLine2_04,
		line2_05 => sLCDLine2_05,
		line2_06 => sLCDLine2_06,
		line2_07 => sLCDLine2_07,
		line2_08 => sLCDLine2_08,
		line2_09 => sLCDLine2_09,
		line2_10 => sLCDLine2_10,
		line2_11 => sLCDLine2_11,
		line2_12 => sLCDLine2_12,
		line2_13 => sLCDLine2_13,
		line2_14 => sLCDLine2_14,
		line2_15 => sLCDLine2_15
	);
	-----------------------------------------------------------------------------
	-- 								 OUT/IN HANDLING
	-----------------------------------------------------------------------------							  
	output_ports : process(sCLK16MHz, sMcuPortId, sMcuOutPort, sMcuKwriteStrobe, sMcuwriteStrobe)
	begin
		if rising_edge(sCLK16MHz) then
		----------------------------------
		   if sMcuKwriteStrobe = '1' then
				case sMcuPortId(3 downto 0) is 
					when cUARTRstPort =>
						sTxUARTreset <= sMcuOutPort(0);
						sRxUARTreset <= sMcuOutPort(1);
					when cTxUARTDataPort(3 downto 0) =>
						sTxUARTdata <= sMcuOutPort;					
					when others =>
				end case;
			end if;
			--------------------------------
			if sMcuWriteStrobe = '1' then
				case sMcuPortId is
					when cLEDPort => 
						LEDS <= sMcuOutPort(7 downto 4);
					when cTxUARTDataPort => 
						sTxUARTdata <= sMcuOutPort;
					when cDAC1_15_08Port =>
						sDAC1BufferedValue(15 downto 8) <= sMcuOutPort(7 downto 0);
					when cDAC1_07_00Port =>
						sDAC1BufferedValue(7 downto 0) <= sMcuOutPort(7 downto 0);
						sDAC1WritePortReq <= '1';
					when cDAC0_31_24Port => 
						sDDSphIncCtrl(24) <= sMcuOutPort(0);
					when cDAC0_23_16Port =>
						sDDSphIncCtrl(23 downto 16) <= sMcuOutPort(7 downto 0);
					when cDAC0_15_08Port =>
						sDDSphIncCtrl(15 downto 8) <= sMcuOutPort(7 downto 0);
					when cDAC0_07_00Port =>
						sDDSphIncCtrl(7 downto 0) <= sMcuOutPort(7 downto 0);
					when cRFModePort =>
						sRFMode <= sMcuOutPort;
					when cLCDPort1_00 =>
						sLCDLine1_00 <= sMcuOutPort;
					when cLCDPort1_01 =>
						sLCDLine1_01 <= sMcuOutPort;
					when cLCDPort1_02 =>
						sLCDLine1_02 <= sMcuOutPort;
					when cLCDPort1_03 =>
						sLCDLine1_03 <= sMcuOutPort;
					when cLCDPort1_04 =>
						sLCDLine1_04 <= sMcuOutPort;
					when cLCDPort1_05 =>
						sLCDLine1_05 <= sMcuOutPort;
					when cLCDPort1_06 =>
						sLCDLine1_06 <= sMcuOutPort;
					when cLCDPort1_07 =>
						sLCDLine1_07 <= sMcuOutPort;
					when cLCDPort1_08 =>
						sLCDLine1_08 <= sMcuOutPort;
					when cLCDPort1_09 =>
						sLCDLine1_09 <= sMcuOutPort;
					when cLCDPort1_10 =>
						sLCDLine1_10 <= sMcuOutPort;
					when cLCDPort1_11 =>
						sLCDLine1_11 <= sMcuOutPort;
					when cLCDPort1_12 =>
						sLCDLine1_12 <= sMcuOutPort;
					when cLCDPort1_13 =>
						sLCDLine1_13 <= sMcuOutPort;
					when cLCDPort1_14 =>
						sLCDLine1_14 <= sMcuOutPort;
					when cLCDPort1_15 =>
						sLCDLine1_15 <= sMcuOutPort;
					when cLCDPort2_00 =>
						sLCDLine2_00 <= sMcuOutPort;
					when cLCDPort2_01 =>
						sLCDLine2_01 <= sMcuOutPort;
					when cLCDPort2_02 =>
						sLCDLine2_02 <= sMcuOutPort;
					when cLCDPort2_03 =>
						sLCDLine2_03 <= sMcuOutPort;
					when cLCDPort2_04 =>
						sLCDLine2_04 <= sMcuOutPort;
					when cLCDPort2_05 =>
						sLCDLine2_05 <= sMcuOutPort;
					when cLCDPort2_06 =>
						sLCDLine2_06 <= sMcuOutPort;
					when cLCDPort2_07 =>
						sLCDLine2_07 <= sMcuOutPort;
					when cLCDPort2_08 =>
						sLCDLine2_08 <= sMcuOutPort;
					when cLCDPort2_09 =>
						sLCDLine2_09 <= sMcuOutPort;
					when cLCDPort2_10 =>
						sLCDLine2_10 <= sMcuOutPort;
					when cLCDPort2_11 =>
						sLCDLine2_11 <= sMcuOutPort;
					when cLCDPort2_12 =>
						sLCDLine2_12 <= sMcuOutPort;
					when cLCDPort2_13 =>
						sLCDLine2_13 <= sMcuOutPort;
					when cLCDPort2_14 =>
						sLCDLine2_14 <= sMcuOutPort;
					when cLCDPort2_15 =>
						sLCDLine2_15 <= sMcuOutPort;
					when others =>
				end case;
			end if;
			--------------------------------
			if (sMcuWriteStrobe = '1' and sMcuPortId = cTxUARTDataPort) then
				sTxUARTWrite <= '1';
			elsif (sMcuKwriteStrobe = '1' and sMcuPortId(0) = '1') then
				sTxUARTWrite <= '1';
			else
				sTxUARTWrite <= '0';
			end if;
			if sDAC1WritePortAck = '1' then
				sDAC1WritePortReq <= '0';
			end if;
		end if;
	end process;
	
	input_ports : process(sCLK16MHz, sMcuPortId, BTNS(1 downto 0),  sTxUARTtxDataPresent, sTxUARTHalfFull, sTxUARTFull, sRxUARTrxDataPresent, sRxUARTHalfFull, sRxUARTFull, sRxUARTdata, sMcuReadStrobe)
	begin
		if rising_edge(sCLK16MHz) then
			case sMcuPortId is 
				when cBTNPort =>
					sMcuInPort(5 downto 0) <= (others => '0');
					sMcuInPort(6) <= BTNS(0);
					sMcuInPort(7) <= BTNS(1);
				when cUARTStatusPort =>
					sMcuInPort(7) <= sTxUARTtxDataPresent;
					sMcuInPort(6) <= sTxUARTHalfFull;
					sMcuInPort(5) <= sTxUARTFull;
					sMcuInPort(4) <= sRxUARTrxDataPresent;
					sMcuInPort(3) <= sRxUARTHalfFull;
					sMcuInPort(2) <= sRxUARTFull;
				when cRxUARTDataPort =>
					sMcuInPort <= sRxUARTdata;
				when sTickRead31_25Port =>
					sMcuInPort <= sTick1ms(31 downto 24);
				when sTickRead24_16Port =>
					sMcuInPort <= sTick1ms(23 downto 16);
				when sTickRead15_08Port =>
					sMcuInPort <= sTick1ms(15 downto 08);
				when sTickRead07_00Port =>
					sMcuInPort <= sTick1ms(07 downto 00);
				when others =>
					sMcuInPort <= (others => 'X');
			end  case;
			
			if sMcuReadStrobe = '1' and sMcuPortId = cRxUARTDataPort then
				sRxUARTRead <= '1';
			else
				sRxUARTRead <= '0';
			end if;
			
		end if;	
	end process;
	
	-----------------------------------------------------------------------------
	-- 								 UART INSTANTIATION
	-----------------------------------------------------------------------------	
	
	TxUART : uart_tx6 PORT MAP(
		data_in => sTxUARTdata,
		en_16_x_baud => sEn16x57600,
		serial_out => UART_TX,
		buffer_write => sTxUARTWrite,
		buffer_data_present => sTxUARTtxDataPresent,
		buffer_half_full => sTxUARTHalfFull,
		buffer_full => sTxUARTFull,
		buffer_reset => sTxUARTreset,
		clk => sCLK16MHz);
		
	RxUART: uart_rx6 PORT MAP(
		serial_in => UART_RX,
		en_16_x_baud => sEn16x57600,
		data_out => sRxUARTdata,
		buffer_read => sRxUARTRead,
		buffer_data_present => sRxUARTrxDataPresent,
		buffer_half_full => sRxUARTHalfFull,
		buffer_full => sRxUARTFull,
		buffer_reset => sRxUARTreset,
		clk => sCLK16MHz
	);
	
	UART_GND <= '0';
	
	-----------------------------------------------------------------------------
	-----------------------------------------------------------------------------
	--								 		CLOCK MANAGEMENT 
	-----------------------------------------------------------------------------
	-----------------------------------------------------------------------------
	 pll : clkup port map(
    CLK_IN1 => CLK,
    CLK_OUT1 => sCLK120MHz,
    CLK_OUT2 => sCLK16MHz,
	 CLK_OUT3 => sCLK4MHz);
	 
	clkdiv: clk_enabler PORT MAP(
		clk_i => sCLK16MHz,
		areset_i => sCentralReset,
		en16x57600_o => sEn16x57600,
		en1ms_o => open,  -- in future connect debouncer, picoblaze ISR
		tick1ms_o => sTick1ms);  -- connect audio ADC
		
	-----------------------------------------------------------------------------
	-----------------------------------------------------------------------------
	--								 		DAC 1 
	-----------------------------------------------------------------------------
	-----------------------------------------------------------------------------	
	
		-- VCO frequency control through DAC1 voltage 
		
		vco: da1_interface PORT MAP(
		clk_i => sCLK16MHz,
		reset_i => sCentralReset,
		miso_i => DAC1_MISO,
		voltage_i => sDAC1BufferedValue,
		mosi_o => DAC1_MOSI,
		sclk_o => DAC1_SCLK,
		ncs_o => DAC1_NCS,
		nldac_o => DAC1_NLDAC );
		
		buffer16bvalue : process(sCLK16MHz, sDAC1BufferedValue, sDAC1WritePortReq)
		begin
			if sCentralReset = '1' then
				sDAC1WritePortAck <= '0';
			else
				if rising_edge(sCLK16MHz) then
					if sDAC1WritePortReq = '1' then
						sDAC1ReadyValue <= sDAC1BufferedValue;
						sDAC1WritePortAck <= '1';
					else
						sDAC1WritePortAck <= '0';
					end if;
				end if;
			end if;
		end process;
				-----------------------------------------------------------------------------
		-----------------------------------------------------------------------------
		--								 		AUDIO
		-----------------------------------------------------------------------------
		-----------------------------------------------------------------------------	

	left_channel: adc_interface PORT MAP(
		clk_i => sCLK4MHz,
		reset_i => sCentralReset,
		miso_i => ADC0_SDI,
		mosi_o => open,
		sclk_o => ADC0_SCLK,
		ncs_o => ADC0_NCS,
		data_o => sAudioL
	);

	right_channel: adc_interface PORT MAP(
		clk_i => sCLK4MHz,
		reset_i => sCentralReset,
		miso_i => ADC1_SDI,
		mosi_o => open,
		sclk_o => ADC1_SCLK,
		ncs_o => ADC1_NCS,
		data_o => sAudioR
	);
	

  
   counter_of_cmajor_tones : process(sRFMode, CLK_LFC, sAudioTestTonePhInc)
	variable idx : integer range 0 to 7 := 0;
	begin
		if sRFMode = cRFTestSinePilot or sRFMode = cRFTestSine then
			idx := 0;
			sAudioTestTonePhInc <= (others => '0');
		else
			if rising_edge(CLK_LFC) then
				if idx = 0 then
					idx := 7;
				else
					idx := idx - 1;
				end if;
				sAudioTestTonePhInc <= cTones(idx);
			end if;
		end if;
	end process;
	
	tone_generator : DDS_LF
	  PORT MAP (
		clk => sCLK16MHz,
		pinc_in => sAudioTestTonePhInc,
		sine => sAudioTestToneSine
	  );
		-----------------------------------------------------------------------------
		-----------------------------------------------------------------------------
		--								 		DAC 0 
		-----------------------------------------------------------------------------
		-----------------------------------------------------------------------------			
		mono : DDS_RF
	  PORT MAP (
		 clk => sCLK120MHz,
		 pinc_in => sDDSMonophInc,
		 sine => sDDSMono
	  );

		pilot : DDS_RF_2
	  PORT MAP (
		 clk => sCLK120MHz,
		 pinc_in => sDDSPilotphInc,
		 sine => sDDSPilot
	  );
  
		stero : DDS_RF3
	  PORT MAP (
		 clk => sCLK120MHz,
		 pinc_in => sDDSStereophInc,
		 sine => sDDSStereo
	  );
	  
	    stereo_sub : stereo_mplx
	  PORT MAP (
		 clk => sCLK16MHz,
		 channel => sSineChannel,
		 sine => sSinexxKhz
	  );
	  
	  tone_demultiplex : process(sSineChannel, sSinexxKhz)
	  begin
			if (sSineChannel(0) = '0') then
				sSine38KHz <= sSinexxKhz;
			else
				sSine19KHz <= sSinexxKhz;
			end if;
	  end process;
	  
				
		-- XILINX primitive to buffer-out CLK bus signal
		sNCLK120MHz <= not(sCLK120MHz);
		DA0_CLK_O : ODDR2
		generic map(
		  DDR_ALIGNMENT => "NONE",
		  INIT => '0',
		  SRTYPE => "SYNC")
		port map (
		  Q => DAC0_CLK,
		  C0 => sCLK120MHz,
		  C1 => sNCLK120MHz,
		  CE => '1',
		  D0 => '1',
		  D1 => '0',
		  R => sCentralReset,
		  S =>'0'
		);
			
	--TBD: check signed/unsigned signal sum operations (result correctness)
	audio_source : process(sCentralReset, sRFMode, sAudioR, sAudioL, sAudioLpR, sAudioLmR, sDDSphIncCtrl, sDDSMono, sDDSPilot, sDDSStereo)
begin
	 if sCentralReset = '1' then
		 RF_AMP <= cRFAmpStageOff;
		 sAudioLpR <= (others => '0');
		 sAudioLmR <= (others => '0');
		 sAudioSSBSC <= (others => '0');
		 sAudioMplx <= (others => '0');
		 sDDSMonophInc <= (others => '0');
		 sDDSPilotphInc <= (others => '0');
		 sDDSStereophInc <= (others => '0');
		 sSumDDS <= (others => '0');
	 else
		 case sRFMode is 
			when cRFTestSine =>
				RF_AMP <= cRFAmpStageOn;
				sAudioLpR <= (others => '0');
				sAudioLmR <= (others => '0');
				sAudioSSBSC <= (others => '0');
				sAudioMplx <= (others => '0');
				sDDSMonophInc <= std_logic_vector(signed(sDDSphIncCtrl) + signed(cDAC0_30Hz_Offset) + (signed(sAudioTestToneSine) + signed(c7k5Hz)));
				sDDSPilotphInc <= (others => '0');
				sDDSStereophInc <= (others => '0');
				sSumDDS <= std_logic_vector("0000000000" + signed(sDDSMono));
			when cRFTestSinePilot =>
				RF_AMP <= cRFAmpStageOn;
				sAudioLpR <= (others => '0');
				sAudioLmR <= (others => '0');
				sAudioSSBSC <= (others => '0');
				sAudioMplx <= (others => '0');
				sDDSMonophInc <= std_logic_vector(signed(sDDSphIncCtrl) + signed(cDAC0_30Hz_Offset) + (signed(sAudioTestToneSine) + signed(c7k5Hz)));
				sDDSPilotphInc <= std_logic_vector(signed(sDDSphIncCtrl) + (signed(sSine19KHz)) + signed(c19kHz));
				sDDSStereophInc <= (others => '0');
				sSumDDS <= std_logic_vector("0000000000" + signed(sDDSMono) + signed(sDDSPilot));
			when cRFMono =>
				RF_AMP <= cRFAmpStageOn;
				sAudioLpR <= std_logic_vector(signed(X"0000" + unsigned(sAudioR) + unsigned(sAudioL)) - 2047);
				sAudioLmR <= (others => '0');
				sAudioSSBSC <= (others => '0');
				sAudioMplx <= (others => '0');
				sDDSMonophInc <= std_logic_vector(signed(sDDSphIncCtrl) + signed(cDAC0_30Hz_Offset) + signed(sAudioLpR) + signed(c7k5Hz) );
				sDDSPilotphInc <= (others => '0');
				sDDSStereophInc <= std_logic_vector(signed(sDDSphIncCtrl) + signed(c38kHz) + signed(sAudioMplx) + signed(cDAC0_30Hz_Offset));
				sSumDDS <= std_logic_vector("0000000000" + signed(sDDSMono));
			when cRFMonoPilot =>
				RF_AMP <= cRFAmpStageOn;
				sAudioLpR <= std_logic_vector(signed(X"0000" + unsigned(sAudioR) + unsigned(sAudioL)) - 2047);
				sAudioLmR <= (others => '0');
				sAudioSSBSC <= (others => '0');
				sAudioMplx <= (others => '0');
				sDDSMonophInc <= std_logic_vector(signed(sDDSphIncCtrl) + signed(sAudioLpR) + signed(c7k5Hz) + signed(cDAC0_30Hz_Offset));
				sDDSPilotphInc <= std_logic_vector(signed(sDDSphIncCtrl) + (signed(sSine19KHz)/4) + signed(c19kHz));			
				sDDSStereophInc <= (others => '0');
				sSumDDS <= std_logic_vector("0000000000" + signed(sDDSMono) + signed(sDDSPilot));
			when cRFMonoStereo =>
				RF_AMP <= cRFAmpStageOn;
				sAudioLpR <= std_logic_vector(signed(X"0000" + unsigned(sAudioR) + unsigned(sAudioL)) - 2047);
				sAudioLmR <= std_logic_vector(X"0000" + unsigned(sAudioR) - unsigned(sAudioL));
				sAudioSSBSC <= std_logic_vector(X"00000000" + signed(X"0000" + signed(sAudioLmR))*signed(shift_right(signed(sSine38KHz),1)));
				sAudioMplx32b <= std_logic_vector((signed(sAudioSSBSC)/256));
				sAudioMplx <= sAudioMplx32b(12 downto 0);
				sDDSMonophInc <= std_logic_vector(signed(sDDSphIncCtrl) + signed(cDAC0_30Hz_Offset) + signed(c7k5Hz) + signed(sAudioLpR));
				sDDSPilotphInc <= (others => '0');
				sDDSStereophInc <= std_logic_vector(signed(sDDSphIncCtrl) + signed(c38kHz) + signed(sAudioMplx));
				sSumDDS <= std_logic_vector("0000000000" + signed(sDDSMono) + signed(sDDSStereo));
			when cRFMonoStereoPilot =>
				RF_AMP <= cRFAmpStageOn;
				sAudioLpR <= std_logic_vector(signed(X"0000" + unsigned(sAudioR) + unsigned(sAudioL)) - 2047);
				sAudioLmR <= std_logic_vector(X"0000" + unsigned(sAudioR) - unsigned(sAudioL));
				sAudioSSBSC <= std_logic_vector(X"00000000" + signed(X"0000" + signed(sAudioLmR))*signed(shift_right(signed(sSine38KHz),1)));
				sAudioMplx32b <= std_logic_vector((signed(sAudioSSBSC)/256));
				sAudioMplx <= sAudioMplx32b(12 downto 0);
				sDDSMonophInc <= std_logic_vector(signed(sDDSphIncCtrl) + signed(cDAC0_30Hz_Offset) + signed(c7k5Hz) + signed(sAudioLpR));
				sDDSPilotphInc <= std_logic_vector(signed(sDDSphIncCtrl) + signed(c19kHz) + (signed(sSine19KHz)));
				sDDSStereophInc <= std_logic_vector(signed(sDDSphIncCtrl) + signed(c38kHz) + signed(sAudioMplx));
				sSumDDS <= std_logic_vector("0000000000" + signed(sDDSMono) + signed(sDDSPilot) + signed(sDDSStereo));
			when others =>
				RF_AMP <= cRFAmpStageOff;
				sAudioLpR <= (others => '0');
				sAudioLmR <= (others => '0');
				sAudioSSBSC <= (others => '0');
				sAudioMplx <= (others => '0');
				sDDSMonophInc <= (others => '0');
				sDDSPilotphInc <= (others => '0');
				sDDSStereophInc <= (others => '0');
				sSumDDS <= (others => '0');
		end case;
	end if;
	DAC0_DATA <= sSumDDS;
end process;
	

end Behavioral;



