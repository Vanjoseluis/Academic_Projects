-- Vhdl test bench created from schematic C:\Users\edie\Desktop\RLab_Plant_Jose_Raul_25-3-26\Cuenta4votos.sch - Thu Mar 26 10:30:43 2026
--
-- Notes: 
-- 1) This testbench template has been automatically generated using types
-- std_logic and std_logic_vector for the ports of the unit under test.
-- Xilinx recommends that these types always be used for the top-level
-- I/O of a design in order to guarantee that the testbench will bind
-- correctly to the timing (post-route) simulation model.
-- 2) To use this template as your testbench, change the filename to any
-- name of your choice with the extension .vhd, and use the "Source->Add"
-- menu in Project Navigator to import the testbench. Then
-- edit the user defined section below, adding code to generate the 
-- stimulus for your design.
--
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
USE ieee.std_logic_arith.ALL;
USE ieee.numeric_std.ALL;
LIBRARY UNISIM;
USE UNISIM.Vcomponents.ALL;
ENTITY Cuenta4votos_Cuenta4votos_sch_tb IS
END Cuenta4votos_Cuenta4votos_sch_tb;
ARCHITECTURE behavioral OF Cuenta4votos_Cuenta4votos_sch_tb IS 

   COMPONENT Cuenta4votos
   PORT( M3 : IN STD_LOGIC;
			M2 : IN STD_LOGIC;
			M1 : IN STD_LOGIC;
			M0 : IN STD_LOGIC;
			S2 : OUT STD_LOGIC;
			S1 : OUT STD_LOGIC;
			S0 : OUT STD_LOGIC);
   END COMPONENT;
	
	SIGNAL M3 : STD_LOGIC;
	SIGNAL M2 : STD_LOGIC;
	SIGNAL M1 : STD_LOGIC;
	SIGNAL M0 : STD_LOGIC;
	SIGNAL S2 : STD_LOGIC;
	SIGNAL S1 : STD_LOGIC;
	SIGNAL S0 : STD_LOGIC;
	
	



BEGIN

   UUT: Cuenta4votos PORT MAP(
		M0 => M0,
		M1 => M1,
		M2 => M2,
		M3 => M3,
		S0 => S0,
		S1 => S1,
		S2 => S2	
   );

-- *** Test Bench - User Defined Section ***
  -- Forma de onda de la señal A
	clock1: process
	begin
	M0 <= '0';
	loop
	wait for CKSemiPeriod;
	M0 <= not M0;
	end loop;
	end process clock1;

-- Forma de onda de la señal B
	clock2: process
		begin
		M1 <= '0';
	loop
	wait for (2*CKSemiPeriod);
	M1 <= not M1;
	end loop;
	end process clock2;

-- Forma de onda de la señal C
	clock3: process
		begin
		M2 <= '0';
	loop
	wait for (4*CKSemiPeriod);
	M2 <= not M2;
	end loop;
	end process clock3;

-- Forma de onda de la señal D
	clock4: process
		begin
		M3 <= '0';
	loop
	wait for (8*CKSemiPeriod);
	M3 <= not M3;
	end loop;
	end process clock4;

-- *** End Test Bench - User Defined Section

END;


END;
