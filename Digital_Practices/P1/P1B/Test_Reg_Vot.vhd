-- Vhdl test bench created from schematic C:\Users\Usuario_UMA\Downloads\RLab_Plant_Jose_Raul_07-05-26_1B\Reg_Vot.sch - Thu May 07 12:27:44 2026
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
ENTITY Reg_Vot_Reg_Vot_sch_tb IS
END Reg_Vot_Reg_Vot_sch_tb;

ARCHITECTURE behavioral OF Reg_Vot_Reg_Vot_sch_tb IS 
constant CKsemiperiod : TIME := 20 ns;
   COMPONENT Reg_Vot
   PORT( CK	   :	IN		STD_LOGIC; 
         CLR	:	IN		STD_LOGIC;
			ENA	:	IN		STD_LOGIC;
			VF		:	IN		STD_LOGIC;
			VC		:	IN		STD_LOGIC; 
			VF_C	:	OUT	STD_LOGIC; 
         V_R	:	OUT	STD_LOGIC; 
         Led_V	:	OUT	STD_LOGIC); 
   END COMPONENT;

   SIGNAL CK			:	STD_LOGIC;
   SIGNAL CLR			:	STD_LOGIC;
   SIGNAL ENA			:	STD_LOGIC;
   SIGNAL VF			:	STD_LOGIC;
   SIGNAL VC			:	STD_LOGIC;
   SIGNAL VF_C			:	STD_LOGIC;
   SIGNAL V_R			:	STD_LOGIC;
   SIGNAL Led_V		:	STD_LOGIC;

BEGIN

   UUT: Reg_Vot PORT MAP(
		VF_C => VF_C, 
		V_R => V_R, 
		Led_V => Led_V, 
		ENA => ENA, 
		VC => VC, 
		VF => VF, 
		CK => CK, 
		CLR => CLR
   );

-- *** Test Bench - User Defined Section ***
   tb : PROCESS
   BEGIN
      CLR <= '0'; 
		ENA <= '0'; 
		VF <= '0'; 
		VC <= '0';
		
		for i in 0 to 13 loop
			CK <= '1';
			wait for CKsemiperiod;
			
			case i is
				when 1 =>
					CLR <= '1';
				when 2 =>
					CLR <= '0'; 
				when 3 =>
					ENA <= '1';
				when 4 =>
					VF <= '1'; VC <= '1';
				when 5 =>
					VC <= '0';
				when 6 =>
					VF <= '0'; VC <= '1';
				when 7 =>
					ENA <= '0';
				when 8 =>
					CLR <= '1';
				when 9 =>
					CLR <= '0'; ENA <= '1'; VF <= '0'; VC <= '0';
				when 10 =>
					ENA <= '0';
				when 11 =>
					ENA <= '1';
				when 12 =>
					VC <= '1';
				when 13 =>
					ENA <= '0'; VF <= '1';
				when others => 
					null;
			end case;
			
			CK <= '0';
			wait for CKsemiperiod;
		end loop;
		
		CK <= '1';
		wait for CKsemiperiod;
		WAIT; -- will wait forever
   END PROCESS;
-- *** End Test Bench - User Defined Section ***

END;
