--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   09:54:25 05/14/2026
-- Design Name:   
-- Module Name:   C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Reg_Vot_VHDL_Exc_TEST.vhd
-- Project Name:  RL_23Pract1B
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: Reg_Vot_VHDL_Exc
-- 
-- Dependencies:
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
--
-- Notes: 
-- This testbench has been automatically generated using types std_logic and
-- std_logic_vector for the ports of the unit under test.  Xilinx recommends
-- that these types always be used for the top-level I/O of a design in order
-- to guarantee that the testbench will bind correctly to the post-implementation 
-- simulation model.
--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--USE ieee.numeric_std.ALL;
 
ENTITY Reg_Vot_VHDL_Exc_TEST IS
END Reg_Vot_VHDL_Exc_TEST;
 
ARCHITECTURE behavioral OF Reg_Vot_VHDL_Exc_TEST IS 
constant CKsemiperiod : TIME := 20 ns;
   COMPONENT Reg_Vot_VHDL_Exc
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

   UUT: Reg_Vot_VHDL_Exc PORT MAP(
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
