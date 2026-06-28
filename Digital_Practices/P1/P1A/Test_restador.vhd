-- Vhdl test bench created from schematic C:\Users\juanm\Desktop\P1 digi\Restador.sch - Sun Mar 29 16:50:14 2026
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
ENTITY Restador_Restador_sch_tb IS
END Restador_Restador_sch_tb;
ARCHITECTURE behavioral OF Restador_Restador_sch_tb IS 

   COMPONENT Restador
   PORT( B0	:	IN	STD_LOGIC; 
          B1	:	IN	STD_LOGIC; 
          B2	:	IN	STD_LOGIC; 
          B3	:	IN	STD_LOGIC; 
          C0	:	IN	STD_LOGIC; 
          A0	:	IN	STD_LOGIC; 
          A1	:	IN	STD_LOGIC; 
          A2	:	IN	STD_LOGIC; 
          A3	:	IN	STD_LOGIC; 
          S0	:	OUT	STD_LOGIC; 
          S1	:	OUT	STD_LOGIC; 
          S2	:	OUT	STD_LOGIC; 
          S3	:	OUT	STD_LOGIC; 
          S4	:	OUT	STD_LOGIC);
   END COMPONENT;

   SIGNAL B0	:	STD_LOGIC;
   SIGNAL B1	:	STD_LOGIC;
   SIGNAL B2	:	STD_LOGIC;
   SIGNAL B3	:	STD_LOGIC;
   SIGNAL C0	:	STD_LOGIC;
   SIGNAL A0	:	STD_LOGIC;
   SIGNAL A1	:	STD_LOGIC;
   SIGNAL A2	:	STD_LOGIC;
   SIGNAL A3	:	STD_LOGIC;
   SIGNAL S0	:	STD_LOGIC;
   SIGNAL S1	:	STD_LOGIC;
   SIGNAL S2	:	STD_LOGIC;
   SIGNAL S3	:	STD_LOGIC;
   SIGNAL S4	:	STD_LOGIC;

BEGIN

   UUT: Restador PORT MAP(
		B0 => B0, 
		B1 => B1, 
		B2 => B2, 
		B3 => B3, 
		C0 => C0, 
		A0 => A0, 
		A1 => A1, 
		A2 => A2, 
		A3 => A3, 
		S0 => S0, 
		S1 => S1, 
		S2 => S2, 
		S3 => S3, 
		S4 => S4
   );

-- *** Test Bench - User Defined Section ***
tb : PROCESS
    variable v : unsigned(8 downto 0);
BEGIN
    -- Recorremos las 512 combinaciones posibles (2^9)
    for i in 0 to 511 loop
        
        -- Convertimos el entero 'i' a un vector binario de 9 bits
        v := to_unsigned(i, 9);

        -- Entradas A (bits pares)
        A0 <= v(0);
        A1 <= v(2);
        A2 <= v(4);
        A3 <= v(6);

        -- Entradas B (bits impares)
        B0 <= v(1);
        B1 <= v(3);
        B2 <= v(5);
        B3 <= v(7);

        -- Acarreo de entrada
        C0 <= v(8);

        -- Tiempo entre combinaciones
        wait for 20 ns;
    end loop;

    -- Finalización de la simulación
    assert false report "Simulación de las 512 combinaciones terminada" severity failure;

    wait; -- Espera infinita
END PROCESS;
-- *** End Test Bench - User Defined Section ***


END;
