-- Vhdl test bench created from schematic C:\Users\rjnavas\Documents\doc\ElecDig-GIERM\Practicas_LR\Door_LR\Control.sch - Thu Apr 08 18:47:18 2021
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
ENTITY Control_Control_sch_tb IS
END Control_Control_sch_tb;
ARCHITECTURE behavioral OF Control_Control_sch_tb IS 

   COMPONENT Control
   PORT( S	:	OUT	STD_LOGIC; 
          P	:	IN	STD_LOGIC; 
          A	:	IN	STD_LOGIC; 
          DP	:	IN	STD_LOGIC; 
          Ck	:	IN	STD_LOGIC; 
          CLR	:	IN	STD_LOGIC);
   END COMPONENT;

   SIGNAL S	:	STD_LOGIC;
   SIGNAL P	:	STD_LOGIC;
   SIGNAL A	:	STD_LOGIC;
   SIGNAL DP	:	STD_LOGIC;
   SIGNAL Ck	:	STD_LOGIC;
   SIGNAL CLR	:	STD_LOGIC;
	constant CkSemiperiod : TIME := 50ns;

BEGIN

   UUT: Control PORT MAP(
		S => S, 
		P => P, 
		A => A, 
		DP => DP, 
		Ck => Ck, 
		CLR => CLR
   );

-- *** Test Bench - User Defined Section ***
   clock: process
		begin
		Ck <= '0';
		loop
		wait for CkSemiPeriod;
		Ck <= not Ck;
		end loop;
	end process clock;
   tb : PROCESS
   BEGIN
	   CLR <= '1';
		P <= '0';
		A <= '0';
		DP <= 'U';
		wait for (CkSemiPeriod);
		CLR <= '0';
		wait for (CkSemiPeriod);
		CLR <= '1';
		wait for (2*CkSemiPeriod);
		A <= '1';
		DP <= '0';
		wait for (1*CkSemiPeriod);
		P <= '1';
		wait for (2*CkSemiPeriod);
		P <= '0';
		wait for (2*CkSemiPeriod);
		P <= '1';
		wait for (2*CkSemiPeriod);
		P <= '0';
		wait for (2*CkSemiPeriod);
		P <= '1';
		DP <='1';
		wait for (2*CkSemiPeriod);
		P <= '0';
		wait for (2*CkSemiPeriod);
		DP <='0';
		-- wait for (1*CkSemiPeriod);
		P <= '1';
		wait for (2*CkSemiPeriod);
		P <= '0';
		wait for (2*CkSemiPeriod);
		DP <='1';
		wait for (2*CkSemiPeriod);
		DP <='0';
      WAIT; -- will wait forever
   END PROCESS;
-- *** End Test Bench - User Defined Section ***

END;
