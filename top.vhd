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
		
				BTNS : in  STD_LOGIC_VECTOR (1 downto 0);
				LEDS : out  STD_LOGIC_VECTOR (3 downto 0);
				
				UART_TX : out STD_LOGIC;
				UART_RX : in STD_LOGIC;
				
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
		en44kHz_o : OUT std_logic
		);
	END COMPONENT;

	component clkup
	port
	 (-- Clock in ports
	  CLK_IN1           : in     std_logic;
	  -- Clock out ports
	  CLK_OUT1          : out    std_logic;
	  CLK_OUT2          : out    std_logic
	 );
	end component;

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
		 sine : OUT STD_LOGIC_VECTOR(9 DOWNTO 0)
	  );
	END COMPONENT;
	
	COMPONENT stereo_adc
	PORT(
		clk_i : IN std_logic;
		areset_i : IN std_logic;
		adc0_sdi_i : IN std_logic;
		adc1_sdi_i : IN std_logic;
		en_sampl_i : IN std_logic;          
		adc0_sclk_o : OUT std_logic;
		adc0_ncs_o : OUT std_logic;
		adc1_sclk_o : OUT std_logic;
		adc1_ncs_o : OUT std_logic;
		lpr_o : OUT std_logic_vector(15 downto 0);
		lmr_o : OUT std_logic_vector(15 downto 0)
		);
	END COMPONENT;

	
constant cLEDPort : std_logic_vector(7 downto 0) := X"00";
constant cBTNPort : std_logic_vector(7 downto 0) := X"00";

constant cTxUARTDataPort : std_logic_vector(7 downto 0) := X"01";
constant cRxUARTDataPort : std_logic_vector(7 downto 0) := X"02";

constant cUARTStatusPort : std_logic_vector(7 downto 0) := X"01";
constant cUARTRstPort : std_logic_vector(3 downto 0) := X"0";

constant cDAC1_07_00Port : std_logic_vector(7 downto 0) := X"03";
constant cDAC1_15_08Port : std_logic_vector(7 downto 0) := X"04";

constant cDAC0_07_00Port : std_logic_vector(7 downto 0) := X"05";
constant cDAC0_15_08Port : std_logic_vector(7 downto 0) := X"06";
constant cDAC0_23_16Port : std_logic_vector(7 downto 0) := X"07";
constant cDAC0_31_24Port : std_logic_vector(7 downto 0) := X"08";

constant cDAC0_21_1MHz_phInc : std_logic_vector(24 downto 0) := "0010110100000011010110101";
constant cDAC0_30Hz_Offset : std_logic_vector(15 downto 0):= X"0008";
	-----------------------------------------------------------------------------
	--								SIGNALS
	-----------------------------------------------------------------------------
	
	-- Central signals
signal sCentralReset : std_logic := '0';
signal sCLK16MHz : std_logic := '0';
signal sCLK120MHz : std_logic := '0';
signal sNCLK120MHz : std_logic := '0';
signal sEn16x57600 : std_logic := '0';
signal sEn44kHz : std_logic := '0';

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
  -- DAC1
signal sDAC1BufferedValue : std_logic_vector(15 downto 0) := (others => '0');
signal sDAC1ReadyValue : std_logic_vector(15 downto 0) := (others => '0');
signal sDAC1WritePortReq : std_logic := '0';
signal sDAC1WritePortAck : std_logic := '0';

 -- DAC0
signal sDDSphInc : std_logic_vector(24 downto 0) := (others => '0');
--signal sDDSphIncCtrl : std_logic_vector(24 downto 0) := (others => '0');
signal sDDSphIncCtrl : std_logic_vector(24 downto 0) := cDAC0_21_1MHz_phInc;

 -- AUDIO
signal sAudioLpR : std_logic_vector(15 downto 0):= (others => '0');
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
	
	-----------------------------------------------------------------------------
	-----------------------------------------------------------------------------
	--								 		CLOCK MANAGEMENT 
	-----------------------------------------------------------------------------
	-----------------------------------------------------------------------------
	 pll : clkup port map(
    CLK_IN1 => CLK,
    CLK_OUT1 => sCLK120MHz,
    CLK_OUT2 => sCLK16MHz);
	 
	clkdiv: clk_enabler PORT MAP(
		clk_i => sCLK16MHz,
		areset_i => sCentralReset,
		en16x57600_o => sEn16x57600,
		en1ms_o => open,  -- connect debouncer, picoblaze ISR
		en44kHz_o => sEn44kHz);  -- connect audio ADC
		
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
			if rising_edge(sCLK16MHz) then
				if sDAC1WritePortReq = '1' then
					sDAC1ReadyValue <= sDAC1BufferedValue;
					sDAC1WritePortAck <= '1';
				else
					sDAC1WritePortAck <= '0';
				end if;
			end if;
		end process;
		
		-----------------------------------------------------------------------------
		-----------------------------------------------------------------------------
		--								 		DAC 0 
		-----------------------------------------------------------------------------
		-----------------------------------------------------------------------------	
		
		-- phInc expression: Preset Broadcast FM + Mono L+R + 30Hz offset => FM mono
		sDDSphInc <= std_logic_vector(unsigned(sDDSphIncCtrl) + unsigned(sAudioLpR) + unsigned(cDAC0_30Hz_Offset));

		-- main DDS FM modulator
		FM_Mono : DDS_RF
	  PORT MAP (
		 clk => sCLK120MHz,
		 pinc_in => sDDSphInc,
		 sine => DAC0_DATA
	  );
		
		
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
		
		
	
		-----------------------------------------------------------------------------
		-----------------------------------------------------------------------------
		--								 		AUDIO
		-----------------------------------------------------------------------------
		-----------------------------------------------------------------------------	
		
		audio: stereo_adc PORT MAP(
		clk_i => sCLK16MHz,
		areset_i => sCentralReset,
		adc0_sclk_o => ADC0_SCLK,
		adc0_sdi_i => ADC0_SDI,
		adc0_ncs_o => ADC0_NCS,
		adc1_sclk_o => ADC1_SCLK,
		adc1_sdi_i => ADC1_SDI,
		adc1_ncs_o => ADC1_NCS,
		en_sampl_i => sEn44kHz,
		lpr_o => sAudioLpR,
		lmr_o => open
	);
		
		
end Behavioral;

