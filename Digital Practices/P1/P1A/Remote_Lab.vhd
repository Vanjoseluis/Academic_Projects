----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    20:46:18 04/12/2017 
-- Design Name: Remote_Lab Module
-- Module Name:    Remote_Lab - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- The component implements EPP byte registers for virtual IOs on a FPGA target: 

-- Name			Epp	Dir	Explain
--					Address 
-- regsw				0x00	in		8 switches
-- regbtn			0x01	in		5 buttons
-- Leds1 and 0	0x02	out	8 LEDs
-- Leds3 and 2	0x03	out	8 LEDs
-- Leds5 and 4	0x03	out	8 LEDs
-- Leds7 and 6	0x05	out	8 LEDs
-- Display0		0x06	out	Punto + 7 segmentos Display 0
-- Display1		0x07	out	Punto + 7 segmentos Display 1
-- Display2		0x08	out	Punto + 7 segmentos Display 2
-- Display3		0x09	out	Punto + 7 segmentos Display 3

-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity Remote_Lab is
	port (
		Clk		: in std_logic;        -- Master Clock 100MHz
		-- Epp-like bus signals
		EppASTB		: in std_logic;        -- Address strobe
		EppDSTB		: in std_logic;        -- Data strobe
		EppWRITE		: in std_logic;        -- Port write signal
		EppDB			: inout std_logic_vector(7 downto 0); -- port data bus
		EppWAIT		: out std_logic;        -- Port wait signal
		-- user extended signals 
		sw				: in std_logic_vector(7 downto 0);
		swInternal	: out std_logic_vector(7 downto 0);
		btn			: in std_logic_vector(4 downto 0);
		btnInternal	: out std_logic_vector(4 downto 0);
		LedInternal	: in std_logic_vector(7 downto 0);
		Led			: out std_logic_vector(7 downto 0);
		RsRx			: in std_logic;
		RxInternal	: out std_logic;
		RsTx			: out std_logic;
		TxInternal	: in std_logic;
		SevenSeg3	: in std_logic_vector(7 downto 0);   
		SevenSeg2	: in std_logic_vector(7 downto 0);   
		SevenSeg1	: in std_logic_vector(7 downto 0);   
		SevenSeg0	: in std_logic_vector(7 downto 0);  
		seg			: out std_logic_vector(7 downto 0); -- Punto decimal y segmentos de los 4 displays 7 segmentos
		an				: out std_logic_vector(3 downto 0)  -- Anodos de los 4 displays 7 segmentos
	);
end Remote_Lab;

architecture Behavioral of Remote_Lab is

	component Modulo
		port( led : in  STD_LOGIC;
           clk : in  STD_LOGIC;
           tono : out  STD_LOGIC_VECTOR (3 downto 0));
	end component;

	signal regEppAdr			: std_logic_vector (7 downto 0); -- Epp address 
	signal regVer				: std_logic_vector(7 downto 0); --  0x00    I/O returns the complement of written value
	signal busEppInternal	: std_logic_vector(7 downto 0);  -- internal bus (before tristate)
	signal regsw				: std_logic_vector(7 downto 0) := "00000000";
	signal regbtn				: std_logic_vector(7 downto 0) := "00000000";
	signal cnt					: std_logic_vector (16 downto 0) := "00000000000000000";
	signal seg_int				: std_logic_vector(7 downto 0); 
	signal sel					: std_logic_vector (1 downto 0);
	signal tono0				: std_logic_vector (3 downto 0);
	signal tono1				: std_logic_vector (3 downto 0);
	signal tono2				: std_logic_vector (3 downto 0);
	signal tono3				: std_logic_vector (3 downto 0);
	signal tono4				: std_logic_vector (3 downto 0);
	signal tono5				: std_logic_vector (3 downto 0);
	signal tono6				: std_logic_vector (3 downto 0);
	signal tono7 				: std_logic_vector (3 downto 0);
	signaL tono1_0 			: std_logic_vector (7 downto 0);
	signaL tono3_2 			: std_logic_vector (7 downto 0);
	signaL tono5_4 			: std_logic_vector (7 downto 0);
	signaL tono7_6 			: std_logic_vector (7 downto 0);

begin

	swInternal	<= sw or regsw; 
	btnInternal	<= btn or regbtn(4 downto 0);
	Led			<= LedInternal;
	RxInternal	<= RsRx;
	RsTx			<= TxInternal;

	U1: Modulo
	port map (
		led => LedInternal(0),
		clk => Clk,
		tono => tono0
	);
	
	U2: Modulo
	port map (
		led => LedInternal(1),
		clk => Clk,
		tono => tono1
	);
	
	U3: Modulo
	port map (
		led => LedInternal(2),
		clk => Clk,
		tono => tono2
	);
	
	U4: Modulo
	port map (
		led => LedInternal(3),
		clk => Clk,
		tono => tono3
	);
	
	U5: Modulo
	port map (
		led => LedInternal(4),
		clk => Clk,
		tono => tono4
	);
	
	U6: Modulo
	port map (
		led => LedInternal(5),
		clk => Clk,
		tono => tono5
	);
	
	U7: Modulo
	port map (
		led => LedInternal(6),
		clk => Clk,
		tono => tono6
	);
	
	U8: Modulo
	port map (
		led => LedInternal(7),
		clk => Clk,
		tono => tono7
	);
	
	tono1_0 <= tono1 & tono0;
	tono3_2 <= tono3 & tono2;
	tono5_4 <= tono5 & tono4;
	tono7_6 <= tono7 & tono6;
	
	sel <= cnt(16 downto 15);
	seg <= not(seg_int);

	process (Clk)
	begin
		if Clk'event and Clk = '1' then
			cnt <= cnt + 1;
		end if;
	end process;

	process (sel, SevenSeg3, SevenSeg2, SevenSeg1, SevenSeg0)
	begin
		case (sel) is
			when "00" => 
				an <= "0111";
				seg_int <= SevenSeg3;
			when "01" =>
				an <= "1011";
				seg_int <= SevenSeg2;
			when "10" =>
				an <= "1101";
				seg_int <= SevenSeg1;
			when others =>
				an <= "1110";
				seg_int <= SevenSeg0;
		end case;
	end process;

	-- Epp signals
   -- Port signals
   EppWAIT <= '1' when EppASTB = '0' or EppDSTB = '0' else '0';
             -- asynchronous Wait assering (maximal Epp speed)

   EppDB <= busEppInternal when (EppWRITE = '1') else "ZZZZZZZZ";

   busEppInternal <= 
       regEppAdr	when (EppASTB = '0') else
       regsw		when (regEppAdr = x"00") else
       regbtn		when (regEppAdr = x"01") else
		 tono1_0		when (regEppAdr = x"02") else
		 tono3_2		when (regEppAdr = x"03") else
		 tono5_4		when (regEppAdr = x"04") else
		 tono7_6		when (regEppAdr = x"05") else
       SevenSeg0	when (regEppAdr = x"06") else
       SevenSeg1	when (regEppAdr = x"07") else
       SevenSeg2	when (regEppAdr = x"08") else
       SevenSeg3	when (regEppAdr = x"09") else
       "00000000";

	-- EPP Address register
	process (EppASTB)
	begin
	if rising_edge(EppASTB) then  -- Astb end edge
	  if EppWRITE = '0' then -- Epp Addr write cycle
		 regEppAdr <= EppDB;          -- Epp Address register update
	  end if;
	end if;
	end process;

  -- EPP Write registers register
	process (EppDSTB)
	begin
		if rising_edge(EppDSTB) then  -- Astb end edge
			if EppWRITE = '0' then -- Epp Addr write cycle
				if regEppAdr = X"00" then 
					regsw <= EppDB;       
				elsif regEppAdr = X"01" then 
					regbtn(7 downto 0) <= EppDB; 
				end if;
			end if;
		end if;
	end process;

end Behavioral;

---------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.std_logic_unsigned.all;
use IEEE.NUMERIC_STD.ALL;

entity Modulo is
	port(
		led : in  STD_LOGIC;
		clk : in  STD_LOGIC;
		tono : out  STD_LOGIC_VECTOR (3 downto 0)
	);
end Modulo;

architecture Behavioral of Modulo is

component Division
	port(
		tiempo_on : in STD_LOGIC_VECTOR(21 downto 0);
		periodo : in STD_LOGIC_VECTOR(21 downto 0);
		div : out STD_LOGIC_VECTOR(3 downto 0)
	);
end component;

signal counter : STD_LOGIC_VECTOR(21 downto 0) := (others => '0');
signal led_reg1 : STD_LOGIC;
signal led_reg2 : STD_LOGIC;
signal flanco_subida : STD_LOGIC;
signal flanco_bajada : STD_LOGIC;
signal tiempo_on : STD_LOGIC_VECTOR(21 downto 0) := (others => '0');
signal periodo : STD_LOGIC_VECTOR(21 downto 0) := (others => '0');
signal div : STD_LOGIC_VECTOR(3 downto 0);
signal limite: STD_LOGIC;
constant TERMINAL_COUNT : STD_LOGIC_VECTOR(21 downto 0) := (others => '1');
signal guardaDivision : STD_LOGIC;

begin

	U1: Division
		port map (
			tiempo_on => tiempo_on,
			periodo => periodo,
			div => div
		);

	process (clk) 
	begin
		if clk='1' and clk'event then
			if flanco_subida = '1' then 
				counter <= (others => '0');
			else
				counter <= counter + 1;
			end if;
		end if;
	end process;

	limite <= '1' when (counter = TERMINAL_COUNT) else '0'; 

	process (clk) 
	begin
		if clk='1' and clk'event then
			led_reg1 <= led;
			led_reg2 <= led_reg1;  
			guardaDivision <= flanco_subida;
		end if;
	end process;

	process(led_reg1, led_reg2)
	begin
		flanco_subida <= (not led_reg2) and led_reg1;
		flanco_bajada <= (not led_reg1) and led_reg2;
	end process;

	process (clk) 
	begin
		if clk='1' and clk'event then
			if flanco_bajada = '1' then
				tiempo_on <= counter;
			end if;
		end if;
	end process;

	process (clk) 
	begin
		if clk='1' and clk'event then
			if flanco_subida = '1' then
				periodo <= counter;
			end if;
		end if;
	end process;

	process (clk) 
	begin
		if clk='1' and clk'event then
			if limite='1' and led='0' then
				tono <= "0000";
			elsif limite='1' and led='1' then
				tono <= "1111";
			else
				if guardaDivision = '1' then
					tono <= div;
				end if;	
			end if;
		end if;
	end process;	

end Behavioral;
-----------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;

entity Division is
	port(
		tiempo_on : in STD_LOGIC_VECTOR(21 downto 0);
		periodo : in STD_LOGIC_VECTOR(21 downto 0);
		div : out STD_LOGIC_VECTOR(3 downto 0)
	);
end Division;

architecture Behavioral of Division is

component Genera_tono
	port( 
		Periodo: in std_logic_vector(3 downto 0);
		TiempoOn: in std_logic_vector(3 downto 0);
		Tono: out std_logic_vector(3 downto 0)
	);
end component;

signal t_o : STD_LOGIC_VECTOR(3 downto 0);
signal p : STD_LOGIC_VECTOR(3 downto 0);

begin

	U1: Genera_Tono
		port map (
			TiempoOn => t_o,
			Periodo => p,
			Tono => div
		);

	process (periodo, tiempo_on)
	begin
--		if (periodo(23)='1') then
--			p<=periodo(23 downto 20);
--			t_o<=tiempo_on(23 downto 20);
--		elsif (periodo(22)='1') then
--			p<=periodo(22 downto 19);
--			t_o<=tiempo_on(22 downto 19);
--		elsif (periodo(21)='1') then
		if (periodo(21)='1') then
			p<=periodo(21 downto 18);
			t_o<=tiempo_on(21 downto 18);
		elsif (periodo (20)='1') then
			p<=periodo(20 downto 17);
			t_o<=tiempo_on(20 downto 17);
		elsif (periodo(19)='1') then
			p<=periodo(19 downto 16);
			t_o<=tiempo_on(19 downto 16);
		elsif (periodo(18)='1') then
			p<=periodo(18 downto 15);
			t_o<=tiempo_on(18 downto 15);
		elsif (periodo (17)='1') then
			p<=periodo(17 downto 14);
			t_o<=tiempo_on(17 downto 14);
		elsif (periodo(16)='1') then
			p<=periodo(16 downto 13);
			t_o<=tiempo_on(16 downto 13);
		elsif (periodo(15)='1') then
			p<=periodo(15 downto 12);
			t_o<=tiempo_on(15 downto 12);
		elsif (periodo (14)='1') then
			p<=periodo(14 downto 11);
			t_o<=tiempo_on(14 downto 11);
		elsif (periodo(13)='1') then
			p<=periodo(13 downto 10);
			t_o<=tiempo_on(13 downto 10);
		elsif (periodo(12)='1') then
			p<=periodo(12 downto 9);
			t_o<=tiempo_on(12 downto 9);
		elsif (periodo (11)='1') then
			p<=periodo(11 downto 8);
			t_o<=tiempo_on(11 downto 8);
		elsif (periodo(10)='1') then
			p<=periodo(10 downto 7);
			t_o<=tiempo_on(10 downto 7);
		elsif (periodo (9)='1') then
			p<=periodo(9 downto 6);
			t_o<=tiempo_on(9 downto 6);
		elsif (periodo(8)='1') then
			p<=periodo(8 downto 5);
			t_o<=tiempo_on(8 downto 5);
		elsif (periodo(7)='1') then
			p<=periodo(7 downto 4);
			t_o<=tiempo_on(7 downto 4);
		elsif (periodo (6)='1') then
			p<=periodo(6 downto 3);
			t_o<=tiempo_on(6 downto 3);
		elsif (periodo(5)='1') then
			p<=periodo(5 downto 2);
			t_o<=tiempo_on(5 downto 2);
		elsif (periodo(4)='1') then
			p<=periodo(4 downto 1);
			t_o<=tiempo_on(4 downto 1);
		else
			p<=periodo(3 downto 0);
			t_o<=tiempo_on(3 downto 0);
		end if;	
	end process;
	
end Behavioral;

--------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.std_logic_unsigned.all;

entity Genera_Tono is
	Port (
		Periodo: in std_logic_vector(3 downto 0);
		TiempoOn: in std_logic_vector(3 downto 0);
		Tono: out std_logic_vector(3 downto 0)
	);
end Genera_Tono;

architecture behavioral of Genera_Tono is
begin

	Tono(3) <= ((not(Periodo(2)) and not(Periodo(1)) and TiempoOn(2) and TiempoOn(0)) 
		or (not(Periodo(1)) and TiempoOn(2) and TiempoOn(1) and TiempoOn(0)) 
		or (not(Periodo(2)) and TiempoOn(2) and TiempoOn(1)) or (TiempoOn(3)));
		
	Tono(2) <= ((not(Periodo(2)) and not(TiempoOn(2)) and TiempoOn(1) and TiempoOn(0)) 
		or (not(Periodo(2)) and not(Periodo(1)) and not(Periodo(0)) and TiempoOn(1) and TiempoOn(0)) 
		or (TiempoOn(2) and not(TiempoOn(1)) and not(TiempoOn(0))) 
		or (not(Periodo(1)) and not(Periodo(0)) and TiempoOn(3) and TiempoOn(1)) 
		or (not(Periodo(2)) and not(Periodo(0)) and TiempoOn(3)) 
		or (not(Periodo(2)) and not(Periodo(1)) and TiempoOn(3)) 
		or (not(Periodo(1)) and TiempoOn(3) and TiempoOn(1) and TiempoOn(0)) 
		or (not(Periodo(2)) and TiempoOn(3) and TiempoOn(0)) 
		or (not(Periodo(2)) and TiempoOn(3) and TiempoOn(1)) 
		or (Periodo(2) and TiempoOn(2) and not(TiempoOn(0))) 
		or (Periodo(1) and TiempoOn(2) and not(TiempoOn(1))) 
		or (Periodo(2) and TiempoOn(2) and not(TiempoOn(1))) 
		or (Periodo(2) and Periodo(1) and TiempoOn(2)));
		
	Tono(1) <= ((not(TiempoOn(3)) and not(TiempoOn(2)) and TiempoOn(1) and not(TiempoOn(0))) 
		or (not(Periodo(2)) and not(Periodo(1)) and TiempoOn(2) and not(TiempoOn(0))) 
		or (not(Periodo(1)) and not(Periodo(0)) and TiempoOn(3) and not(TiempoOn(1))) 
		or (Periodo(2) and not(TiempoOn(3)) and not(TiempoOn(2)) and TiempoOn(1)) 
		or (not(Periodo(1)) and TiempoOn(3) and not(TiempoOn(1)) and TiempoOn(0)) 
		or (Periodo(2) and not(Periodo(1)) and not(Periodo(0)) and TiempoOn(2) and not(TiempoOn(1)) and TiempoOn(0)) 
		or (not(Periodo(2)) and not(Periodo(1)) and Periodo(0) and TiempoOn(2) and TiempoOn(1)) 
		or (not(Periodo(2)) and Periodo(1) and TiempoOn(2) and not(TiempoOn(1)) and TiempoOn(0)) 
		or (not(Periodo(2)) and Periodo(1) and Periodo(0) and TiempoOn(3) and not(TiempoOn(1)) and not(TiempoOn(0))) 
		or (Periodo(2) and Periodo(1) and TiempoOn(1)) 
		or (not(Periodo(1)) and not(TiempoOn(3)) and TiempoOn(1) and not(TiempoOn(0))) 
		or (not(Periodo(2)) and not(Periodo(0)) and TiempoOn(2) and not(TiempoOn(1)) and not(TiempoOn(0))) 
		or (not(Periodo(2)) and not(Periodo(0)) and not(TiempoOn(2)) and TiempoOn(1) and not(TiempoOn(0))) 
		or (not(Periodo(1)) and Periodo(0) and TiempoOn(1) and not(TiempoOn(0))) 
		or (not(Periodo(2)) and TiempoOn(3) and TiempoOn(1) and TiempoOn(0)) 
		or (not(Periodo(2)) and Periodo(1) and not(Periodo(0)) and TiempoOn(2) and TiempoOn(0)));
		
	Tono(0) <= ((not(TiempoOn(3)) and not(TiempoOn(2)) and not(TiempoOn(1)) and TiempoOn(0)) 
		or (not(Periodo(1)) and not(Periodo(0)) and TiempoOn(2) and not(TiempoOn(0))) 
		or (not(Periodo(2)) and Periodo(1) and Periodo(0) and TiempoOn(2) and not(TiempoOn(1)) and not(TiempoOn(0))) 
		or (not(Periodo(2)) and not(Periodo(0)) and TiempoOn(1) and not(TiempoOn(0))) 
		or (not(Periodo(2)) and not(Periodo(1)) and not(TiempoOn(2)) and TiempoOn(1)) 
		or (not(Periodo(2)) and not(Periodo(1)) and TiempoOn(3)) 
		or (not(Periodo(2)) and TiempoOn(3) and TiempoOn(1)) 
		or (not(Periodo(0)) and TiempoOn(3) and TiempoOn(2) and TiempoOn(1)) 
		or (not(Periodo(1)) and Periodo(0) and TiempoOn(3) and not(TiempoOn(0))) 
		or (Periodo(2) and Periodo(1) and TiempoOn(0)) 
		or (Periodo(2) and Periodo(0) and TiempoOn(2) and not(TiempoOn(1)) and TiempoOn(0)) 
		or (not(Periodo(1)) and not(TiempoOn(3)) and not(TiempoOn(2)) and TiempoOn(0)) 
		or (not(Periodo(1)) and not(Periodo(0)) and not(TiempoOn(2)) and TiempoOn(0)) 
		or (not(Periodo(2)) and not(Periodo(1)) and not(Periodo(0)) and TiempoOn(2)) 
		or (not(Periodo(2)) and Periodo(0) and TiempoOn(2) and TiempoOn(1) and TiempoOn(0)) 
		or (not(Periodo(2)) and not(Periodo(0)) and not(TiempoOn(1)) and TiempoOn(0)));
		
end behavioral;
