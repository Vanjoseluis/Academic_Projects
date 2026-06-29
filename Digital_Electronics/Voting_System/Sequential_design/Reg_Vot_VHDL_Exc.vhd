----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    10:02:37 05/07/2026 
-- Design Name: 
-- Module Name:    Reg_Vot_VHDL_Exc - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity Reg_Vot_VHDL_Exc is
    Port ( VF : in  STD_LOGIC;
           VC : in  STD_LOGIC;
           ENA : in  STD_LOGIC;
           CLR : in  STD_LOGIC;
			  CK : in  STD_LOGIC;
           VF_C : out  STD_LOGIC;
           V_R : out  STD_LOGIC;
           Led_V : out  STD_LOGIC);
end Reg_Vot_VHDL_Exc;

architecture Behavioral of Reg_Vot_VHDL_Exc is
	SIGNAL intVF_C :  STD_LOGIC;
	SIGNAL intV_R :  STD_LOGIC;
	SIGNAL intLed_V :  STD_LOGIC;
	SIGNAL D0      
	:  
	STD_LOGIC;
	SIGNAL Q0_reg  :  STD_LOGIC; 
	SIGNAL D1      
	:  
	STD_LOGIC;
	SIGNAL Q1_reg  :  STD_LOGIC; 
begin
	D0 <=  (NOT(Q0_reg) AND NOT(Q1_reg) AND ENA) OR ((VF OR not(VC)) AND ENA and
				NOT(Q1_reg) AND Q0_reg) OR (Q0_reg AND Q1_reg and ENA);
	D1 <=  (NOT(Q1_reg) AND Q0_reg AND (VF xor VC) AND ENA) OR (Q1_reg AND
				NOT(Q0_reg) and ENA) OR (Q0_reg AND Q1_reg and ENA);
	intVF_C <= Q0_reg AND Q1_reg;
	intV_R <= Q1_reg;
	intLed_V<= not(Q1_reg) AND Q0_reg;
	
	FFD: process (CK, CLR)     -- BIESTABLES 
		begin
			if CLR = '1' then-- OJO que el clear es activo a nivel alto 
				Q0_reg <= '0';
				Q1_reg <= '0';
			elsif (rising_edge(ck) and ENA = '1') then
				Q0_reg <= D0;    Q1_reg <= D1;
			end if;
		end process;
		
		-- Asignación de salidas (lógica combinacional)
		VF_C    <= intVF_C;
		V_R     <= intV_R;
		Led_V <= intLed_V;
end Behavioral;

