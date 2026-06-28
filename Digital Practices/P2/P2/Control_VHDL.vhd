----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    15:37:23 05/26/2026 
-- Design Name: 
-- Module Name:    Control_VHDL - Behavioral 
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

entity Control_VHDL is
    Port ( U : in  STD_LOGIC;
           R : in  STD_LOGIC;
           CLR : in  STD_LOGIC;
           Ck : in  STD_LOGIC;
           Q : out  STD_LOGIC_VECTOR (2 downto 0);
           S : out  STD_LOGIC);
end Control_VHDL;

architecture Behavioral of Control_VHDL is
signal EST_actual : STD_LOGIC_VECTOR (2 DOWNTO 0);
signal EST_proximo : STD_LOGIC_VECTOR (2 DOWNTO 0);
begin

    Q <= EST_actual;
-- registro tipo D
FFD: process (CK, CLR) -- biestables tipo D
    begin
        if (CLR = '1') then
            EST_actual <= (others => '0');
        else
            if falling_edge(CK) then --Flanco bajada
                EST_actual <= EST_proximo;
            end if;
        end if;
    end process;
	 
process (U,R, EST_actual)
    begin
	 --Apertura
            case EST_actual is
                when "000" => if (U = '0' and R= '0') then EST_proximo <= "000"; end if;
										if (U = '1' and R= '0') then EST_proximo <= "001"; end if;
										if (R='1') then EST_proximo<= "000"; end if;

										
                when "001" => if (U = '0' and R= '0') then EST_proximo <= "001"; end if;
										if (U = '1' and R= '0') then EST_proximo <= "011"; end if;
										if (R='1') then EST_proximo<= "000"; end if;
										

                when "011" => if (U = '0' and R= '0') then EST_proximo <= "011"; end if;
										if (U = '1' and R= '0') then EST_proximo <= "111"; end if;
										if (R='1') then EST_proximo<= "000"; end if;

	--Cerrar
                when "111" => if (U = '0' and R= '0') then EST_proximo <= "111"; end if;
										if (U = '1' and R= '0') then EST_proximo <= "110"; end if;
										if (R='1') then EST_proximo<= "111"; end if;
										

                when "110" => if (U = '0' and R= '0') then EST_proximo <= "110"; end if;
										if (U = '1' and R= '0') then EST_proximo <= "000"; end if;
										if (R='1') then EST_proximo<= "111"; end if;
										
                when others => EST_proximo <= EST_actual;
            end case;
end process;

S <= (not EST_actual(2) and not EST_actual(1)) or (not EST_actual(2) and EST_actual(1) and EST_actual(0));
end Behavioral;


