-- Vhdl test bench created from schematic C:\Users\edie\Desktop\Practica2_JoseLuis_Raul_26_05_26\Control.sch - Thu May 28 09:46:02 2026
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
constant CKsemiperiod : TIME := 20 ns;

   COMPONENT Control
   PORT(  Ck	:	IN	STD_LOGIC; 
          CLR	:	IN	STD_LOGIC; 
          R	:	IN	STD_LOGIC; 
          U	:	IN	STD_LOGIC; 
          S	:	OUT	STD_LOGIC; 
          EST	:	OUT	STD_LOGIC_VECTOR (2 DOWNTO 0));
   END COMPONENT;

   SIGNAL Ck	:	STD_LOGIC;
   SIGNAL CLR	:	STD_LOGIC;
   SIGNAL R	:	STD_LOGIC;
   SIGNAL U	:	STD_LOGIC;
   SIGNAL S	:	STD_LOGIC;
   SIGNAL EST	:	STD_LOGIC_VECTOR (2 DOWNTO 0);

BEGIN

   UUT: Control PORT MAP(
		Ck => Ck, 
		CLR => CLR, 
		R => R, 
		U => U, 
		S => S, 
		EST => EST
   );

-- *** Test Bench - User Defined Section ***
   tb : PROCESS
   BEGIN
        CLR <= '0'; 
        R <= '0'; 
        U <= '0';
        
        for i in 0 to 18 loop
            CK <= '1';
            wait for CKsemiperiod;
            
            case i is
                --reset
                     when 1 =>
                    CLR <= '1';
                when 2 =>
                    CLR <= '0';
                          
                -- apertura con error
                     when 3 =>
                    U <= '1';
                when 4 =>
                    U <= '0';
                when 5 =>
                    U <= '1';
                when 6 =>
                    U <= '0';
                when 7 =>
                    R <= '1';  -- error
                when 8 =>
                    R <= '0'; 

	  -- apertura correcta
                     when 9 =>
                    U <= '1';
                when 10 =>
                    U <= '0';
                when 11 =>
                    U <= '1';
                when 12 =>
                    U <= '0';
                when 13 =>
                    U <= '1';  -- error
                when 14 =>
                    U <= '0'; 
                          
                    -- cierre
                when 15 =>
                    U <= '1';
                when 16 =>
                    U <= '0';
                when 17 =>
                    U <= '1';
                when 18 =>
                    U <= '0';
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

