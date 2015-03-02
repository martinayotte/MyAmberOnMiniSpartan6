----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    16:56:06 02/15/2015 
-- Design Name: 
-- Module Name:    SDRAM_WB_CTRL - Behavioral 
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

entity SDRAM_WB_CTRL is
	port (
		WB_CLK_I: in std_logic;
		WB_RST_I: in std_logic;
		WB_DATA_O: out std_logic_vector(31 downto 0);
		WB_DATA_I: in std_logic_vector(31 downto 0);
		WB_ADR_I: in std_logic_vector(31 downto 0);
		WB_WE_I: in std_logic;
		WB_CYC_I: in std_logic;
		WB_STB_I: in std_logic;
		WB_SEL_I: in std_logic_vector(3 downto 0);
		WB_ACK_O: out std_logic;
		wb_stall_o: out std_logic;
		-- extra clocking
--		clk_off_3ns: in std_logic;
		-- SDRAM signals
		SDRAM_ADDR : OUT STD_LOGIC_VECTOR (12 downto 0);
		SDRAM_BA : OUT STD_LOGIC_VECTOR (1 downto 0);
		SDRAM_CAS : OUT STD_LOGIC;
		SDRAM_CKE : OUT STD_LOGIC;
		SDRAM_CLK : OUT STD_LOGIC;
		SDRAM_CS : OUT STD_LOGIC;
		SDRAM_DATA : INOUT STD_LOGIC_VECTOR(15 downto 0);
		SDRAM_DQM : OUT STD_LOGIC_VECTOR(1 downto 0);
		SDRAM_RAS : OUT STD_LOGIC;
		SDRAM_WE : OUT STD_LOGIC
		);

end entity SDRAM_WB_CTRL;

architecture Behavioral of SDRAM_WB_CTRL is

	component sdram_controller is
		generic (
			HIGH_BIT: integer := 24
		);
		PORT (
			clk: in std_logic;
--			clock_100_delayed_3ns: in std_logic;
			reset: in std_logic;
			-- Signals to/from the SDRAM chip
			SDRAM_ADDR : OUT STD_LOGIC_VECTOR (12 downto 0);
			SDRAM_BA : OUT STD_LOGIC_VECTOR (1 downto 0);
			SDRAM_CAS : OUT STD_LOGIC;
			SDRAM_CKE : OUT STD_LOGIC;
			SDRAM_CLK : OUT STD_LOGIC;
			SDRAM_CS : OUT STD_LOGIC;
			SDRAM_DATA : INOUT STD_LOGIC_VECTOR(15 downto 0);
			SDRAM_DQM : OUT STD_LOGIC_VECTOR(1 downto 0);
			SDRAM_RAS : OUT STD_LOGIC;
			SDRAM_WE : OUT STD_LOGIC;
--			pending: out std_logic;
			--- Inputs from rest of the system
			cmd_address : IN STD_LOGIC_VECTOR (HIGH_BIT downto 2);
			cmd_enable : IN STD_LOGIC;
			cmd_wr : IN STD_LOGIC;
			data_out : OUT STD_LOGIC_VECTOR (31 downto 0);
			data_out_ready : OUT STD_LOGIC;
			cmd_data_in : IN STD_LOGIC_VECTOR (31 downto 0);
			cmd_byte_enable : in std_logic_vector(3 downto 0)
		);
	end component;
	
	signal sdr_address: STD_LOGIC_VECTOR (24 downto 2);
	signal sdr_req_read : STD_LOGIC;
	signal sdr_req_write : STD_LOGIC;
	signal sdr_data_out : STD_LOGIC_VECTOR (31 downto 0);
	signal sdr_data_out_valid : STD_LOGIC;
	signal sdr_data_in : STD_LOGIC_VECTOR (31 downto 0);
	signal sdr_data_mask: std_logic_vector(3 downto 0);
--	signal pending: std_logic;
	
begin
	ctrl: sdram_controller
		generic map (
			HIGH_BIT => 24
		)
		port map (
			clk => wb_clk_i,
--			clock_100_delayed_3ns => clk_off_3ns,
			reset => wb_rst_i,
			SDRAM_ADDR => SDRAM_ADDR,
			SDRAM_BA => SDRAM_BA,
			SDRAM_CAS => SDRAM_CAS,
			SDRAM_CKE => SDRAM_CKE,
			SDRAM_CLK => SDRAM_CLK,
			SDRAM_CS => SDRAM_CS,
			SDRAM_DATA => SDRAM_DATA,
			SDRAM_DQM => SDRAM_DQM,
			SDRAM_RAS => SDRAM_RAS,
			SDRAM_WE => SDRAM_WE,
--			pending => pending,
			cmd_address => sdr_address,
			cmd_enable => sdr_req_read,
			cmd_wr => sdr_req_write,
			data_out => sdr_data_out,
			data_out_ready => sdr_data_out_valid,
			cmd_data_in => sdr_data_in,
			cmd_byte_enable => sdr_data_mask
		);
		
	sdr_address(24 downto 2) <= WB_ADR_I(24 downto 2);
	
	sdr_req_read<='1' when WB_CYC_I='1' and WB_STB_I='1' and WB_WE_I='0' else '0';
	sdr_req_write<='1' when WB_CYC_I='1' and WB_STB_I='1' and WB_WE_I='1' else '0';
	
	sdr_data_in <= WB_DATA_I;
	sdr_data_mask <= WB_SEL_I;
	
--	wb_stall_o <= '1' when pending='1' else '0';
	
	process(WB_CLK_I)
	begin
		if rising_edge(WB_CLK_I) then
			WB_ACK_O <= sdr_data_out_valid or sdr_req_write;
			WB_DATA_O <= sdr_data_out;
		end if;
	end process;

end Behavioral;

