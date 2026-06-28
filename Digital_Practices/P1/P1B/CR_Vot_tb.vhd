-- Vhdl test bench created from schematic C:\Users\rjnavas\Documents\doc\ElecDig-GIERM\Pract_LRV22\LR_ModCV\CR_Vot.sch - Sat Mar 11 13:17:16 2023
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
ENTITY CR_Vot_CR_Vot_sch_tb IS
END CR_Vot_CR_Vot_sch_tb;
ARCHITECTURE behavioral OF CR_Vot_CR_Vot_sch_tb IS 

   COMPONENT CR_Vot
   PORT( VVal	:	IN	STD_LOGIC; 
          SVF	:	IN	STD_LOGIC_VECTOR (3 DOWNTO 0); 
          RVF	:	OUT	STD_LOGIC; 
          RNA	:	OUT	STD_LOGIC; 
          RVC	:	OUT	STD_LOGIC);
   END COMPONENT;

   SIGNAL VVal	:	STD_LOGIC;
   SIGNAL SVF	:	STD_LOGIC_VECTOR (3 DOWNTO 0);
   SIGNAL RVF	:	STD_LOGIC;
   SIGNAL RNA	:	STD_LOGIC;
   SIGNAL RVC	:	STD_LOGIC;
	constant CkPeriod : TIME := 50 ns;

BEGIN

   UUT: CR_Vot PORT MAP(
		VVal => VVal, 
		SVF => SVF, 
		RVF => RVF, 
		RNA => RNA, 
		RVC => RVC
   );

-- *** Test Bench - User Defined Section ***
   tb : PROCESS
	variable i : integer;
	variable INPUT : std_logic_vector(3 DOWNTO 0);
   BEGIN 
		INPUT := "0000";
		SVF <= INPUT; 
		Vval <= '0';
		wait for 10*CkPeriod;
		Vval <= '1';
		for i in 0 to 15 loop
				INPUT := conv_std_logic_vector(i,4);
				SVF <= INPUT;
				wait for CkPeriod;
			end loop;
				INPUT := "0000";
				SVF <= INPUT;
      WAIT; -- will wait forever
   END PROCESS;
-- *** End Test Bench - User Defined Section ***

END;
