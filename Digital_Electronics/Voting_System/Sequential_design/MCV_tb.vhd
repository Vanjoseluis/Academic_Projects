-- Vhdl test bench created from schematic C:\Users\rjnavas\Documents\doc\ElecDig-GIERM\Pract_LRV22\LR_ModCV\MCV.sch - Sat Mar 11 12:36:02 2023
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
USE ieee.std_logic_arith.ALL;
LIBRARY UNISIM;
USE UNISIM.Vcomponents.ALL;
ENTITY MCV_MCV_sch_tb IS
END MCV_MCV_sch_tb;
ARCHITECTURE behavioral OF MCV_MCV_sch_tb IS 

   COMPONENT MCV
   PORT( M	:	IN	STD_LOGIC_VECTOR (7 DOWNTO 0); 
          VC	:	OUT	STD_LOGIC_VECTOR (3 DOWNTO 0); 
          VF	:	OUT	STD_LOGIC_VECTOR (3 DOWNTO 0));
   END COMPONENT;

   SIGNAL M	:	STD_LOGIC_VECTOR (7 DOWNTO 0);
   SIGNAL VC	:	STD_LOGIC_VECTOR (3 DOWNTO 0);
   SIGNAL VF	:	STD_LOGIC_VECTOR (3 DOWNTO 0);
	constant CkPeriod : TIME := 50 ns;

BEGIN

   UUT: MCV PORT MAP(
		M => M, 
		VC => VC, 
		VF => VF
   );

-- *** Test Bench - User Defined Section ***
   tb : PROCESS
	variable i : integer;
	variable INPUT : std_logic_vector(7 DOWNTO 0);
   BEGIN
			for i in 0 to 255 loop
				INPUT := conv_std_logic_vector(i,8);
				M <= INPUT;
				wait for CkPeriod;
			end loop;
				INPUT := "00000000";
				M <= INPUT;
      WAIT; -- will wait forever
   END PROCESS;
-- *** End Test Bench - User Defined Section ***

END;
