-- Vhdl test bench created from schematic C:\Users\juanm\Desktop\P1 digi\Complemento_a2.sch - Sun Mar 29 16:56:33 2026
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
USE ieee.numeric_std.ALL;
LIBRARY UNISIM;
USE UNISIM.Vcomponents.ALL;
ENTITY Complemento_a2_Complemento_a2_sch_tb IS
END Complemento_a2_Complemento_a2_sch_tb;
ARCHITECTURE behavioral OF Complemento_a2_Complemento_a2_sch_tb IS 
constant CkSemiPeriod : TIME := 50 ns;

   COMPONENT Complemento_a2
   PORT( S0	:	OUT	STD_LOGIC; 
          S1	:	OUT	STD_LOGIC; 
          S2	:	OUT	STD_LOGIC; 
          S3	:	OUT	STD_LOGIC; 
          S4	:	OUT	STD_LOGIC; 
          D0	:	IN	STD_LOGIC; 
          D1	:	IN	STD_LOGIC; 
          D3	:	IN	STD_LOGIC; 
          D2	:	IN	STD_LOGIC);
   END COMPONENT;

   SIGNAL S0	:	STD_LOGIC;
   SIGNAL S1	:	STD_LOGIC;
   SIGNAL S2	:	STD_LOGIC;
   SIGNAL S3	:	STD_LOGIC;
   SIGNAL S4	:	STD_LOGIC;
   SIGNAL D0	:	STD_LOGIC;
   SIGNAL D1	:	STD_LOGIC;
   SIGNAL D3	:	STD_LOGIC;
   SIGNAL D2	:	STD_LOGIC;

BEGIN

   UUT: Complemento_a2 PORT MAP(
		S0 => S0, 
		S1 => S1, 
		S2 => S2, 
		S3 => S3, 
		S4 => S4, 
		D0 => D0, 
		D1 => D1, 
		D3 => D3, 
		D2 => D2
   );

-- *** Test Bench - User Defined Section ***
    clock1: process
begin
    D0 <= '0';
    loop
        wait for CkSemiPeriod;
        D0 <= not D0;
    end loop;
end process clock1;

-- Forma de onda de la señal D1 
clock2: process
begin
    D1 <= '0';
    loop
        wait for (2*CkSemiPeriod);
        D1 <= not D1;
    end loop;
end process clock2;

-- Forma de onda de la señal D2 
clock3: process
begin
    D2 <= '0';
    loop
        wait for (4*CkSemiPeriod);
        D2 <= not D2;
    end loop;
end process clock3;

-- Forma de onda de la señal D3 
clock4: process
begin
    D3 <= '0'; 
    loop
        wait for (8*CkSemiPeriod);
        D3 <= not D3;
    end loop;
end process clock4;

-- *** End Test Bench - User Defined Section ***
END;

