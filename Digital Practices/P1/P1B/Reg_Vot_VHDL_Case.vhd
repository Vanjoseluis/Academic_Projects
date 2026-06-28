----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    10:29:15 05/07/2026 
-- Design Name: 
-- Module Name:    Reg_Vot_VHDL_Case - Behavioral 
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

entity Reg_Vot_VHDL_Case is
    Port ( VF : in  STD_LOGIC;
           VC : in  STD_LOGIC;
           ENA : in  STD_LOGIC;
           CLR : in  STD_LOGIC;
           CK : in  STD_LOGIC;
           VF_C : out  STD_LOGIC;
           V_R : out  STD_LOGIC;
           led_V : out  STD_LOGIC);
end Reg_Vot_VHDL_Case;

architecture Behavioral of Reg_Vot_VHDL_Case is
signal EST_actual : STD_LOGIC_VECTOR (1 DOWNTO 0);
signal EST_proximo : STD_LOGIC_VECTOR (1 DOWNTO 0);
begin

-- registro tipo D
FFD: process (CK, CLR) -- biestables tipo D
    begin
        if (CLR = '1') then
            EST_actual <= (others => '0');
        else
            if rising_edge(CK) then
                EST_actual <= EST_proximo;
            end if;
        end if;
    end process;

-- A PARTIR DE AQUI SIGUE LA LÓGICA COMBINACIONAL PARA EL CALCULO
-- DEL ESTADO SIGUIENTE
-- AVISO 1: Implementar procesos, no mezclar con instrucciones if, case, etc.
-- AVISO 2: Si distintas instrucciones asignan valores a una misma señal
-- puede existir el error de señal afectada por múltiples drivers, es decir,
-- dos procesos intentan modificar la misma señal...

process (ENA, VC, VF)
    begin
        if (ENA = '1') then
            case EST_actual is
                when "00" => EST_proximo <= "01";
                when "01" => if (VC = '1' and VF = '0') then EST_proximo <= "10"; end if;
                             if (VF = '1' and VC = '0') then EST_proximo <= "11"; end if;
									  if (VF = '1' and VC = '1') then EST_proximo <= "01"; end if;
                             if (VF = '0' and VC = '0') then EST_proximo <= "01"; end if;
                when others => EST_proximo <= EST_actual;
            end case;
			else
			  if (EST_actual = "11" or EST_actual = "10") then
					EST_proximo <= EST_actual;
			  else
					EST_proximo <= "00";
			  end if;
		 end if;
end process;

-- lógica combinacional de salida
VF_C <= EST_actual(0) and EST_actual(1);
V_R <= EST_actual(1);
led_V <= not(EST_actual(1)) and (EST_actual(0));

end BEHAVIORAL;

-- synopsys translate_off
configuration CFG_Reg_Vot_VHDL_CASE of Reg_Vot_VHDL_CASE is
    for BEHAVIORAL
    end for;
end CFG_Reg_Vot_VHDL_CASE;
-- synopsys translate_on