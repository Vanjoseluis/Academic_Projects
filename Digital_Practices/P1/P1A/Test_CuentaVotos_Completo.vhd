-- Vhdl test bench created from schematic C:\Users\juanm\Desktop\P1 digi\Cuenta_Votos_COMPLETO.sch - Sun Mar 29 17:27:53 2026
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
ENTITY Cuenta_Votos_COMPLETO_Cuenta_Votos_COMPLETO_sch_tb IS
END Cuenta_Votos_COMPLETO_Cuenta_Votos_COMPLETO_sch_tb;
ARCHITECTURE behavioral OF 
      Cuenta_Votos_COMPLETO_Cuenta_Votos_COMPLETO_sch_tb IS 

   COMPONENT Cuenta_Votos_COMPLETO
   PORT( SVF	:	OUT	STD_LOGIC_VECTOR (3 DOWNTO 0); 
          SVC	:	OUT	STD_LOGIC_VECTOR (3 DOWNTO 0); 
          M	:	IN	STD_LOGIC_VECTOR (7 DOWNTO 0));
   END COMPONENT;

   SIGNAL SVF	:	STD_LOGIC_VECTOR (3 DOWNTO 0);
   SIGNAL SVC	:	STD_LOGIC_VECTOR (3 DOWNTO 0);
   SIGNAL M	:	STD_LOGIC_VECTOR (7 DOWNTO 0);

BEGIN

   UUT: Cuenta_Votos_COMPLETO PORT MAP(
		SVF => SVF, 
		SVC => SVC, 
		M => M
   );

-- *** Test Bench - User Defined Section ***
   tb : PROCESS
   BEGIN
      
      -- Bucle que cuenta desde el 0 hasta el 255
      for i in 0 to 255 loop
         
         -- Convertimos el número 'i' a un vector binario de 8 bits
         -- y se lo asignamos de golpe a toda la entrada M
         M <= std_logic_vector(to_unsigned(i, 8));

         -- Esperamos 20 ns para ver la onda en la gráfica
         wait for 20 ns;
         
      end loop;

      -- Mensaje para detener la simulación al terminar
      assert false report "Simulacion del Contador de Votos terminada" severity failure;
      
      WAIT; -- Espera infinita
   END PROCESS;
-- *** End Test Bench - User Defined Section ***
	

END;
