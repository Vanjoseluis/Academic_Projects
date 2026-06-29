----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    10:11:30 04/16/2026 
-- Design Name: 
-- Module Name:    Cuenta_4votos - Behavioral 
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

entity Cuenta_4votosVHDL is
    Port ( M : in  STD_LOGIC_VECTOR (3 downto 0);
           S : out  STD_LOGIC_VECTOR (2 downto 0));
end Cuenta_4votosVHDL;

architecture Behavioral of Cuenta_4votosVHDL is

begin -- Modulo cuenta 4 votos
	process (M)
	begin
		
	-- VERSION ECUACION LOGICA	
		--S(0) <= M(0) XOR M(1) XOR M(2) XOR M(3);
		
		
		--S(1) <= ((NOT(M(3))) AND M(2) AND M(0)) OR 
				--	((NOT(M(2))) AND M(0) AND M(1)) OR
				--	((NOT(M(0))) AND M(1) AND M(2)) OR
				--	((NOT(M(2))) AND M(3) AND M(1)) OR 
				--	((NOT(M(3))) AND M(0) AND M(2)) OR
				-- ((NOT(M(1))) AND M(3) AND M(2)) OR
				-- ((NOT(M(1))) AND M(3) AND M(0));

	--	S(2) <= M(3) AND M(2) AND M(1) AND M(0); 
	
	
	-- VERSION IF
		--if (M(0) XOR M(1) XOR M(2) XOR M(3)) = '1' then -- Igualar a uno para devolver valor booleano
		--	S(0) <= '1';
		--else
		--	S(0) <= '0';
		--end if;
		
		--if (((NOT(M(3))) AND M(2) AND M(0)) OR 
		--	((NOT(M(2))) AND M(0) AND M(1)) OR
		--	((NOT(M(0))) AND M(1) AND M(2)) OR
		--	((NOT(M(2))) AND M(3) AND M(1)) OR 
		--	((NOT(M(3))) AND M(0) AND M(2)) OR
		--	((NOT(M(1))) AND M(3) AND M(2)) OR
		--	((NOT(M(1))) AND M(3) AND M(0))) = '1' then
		--	S(1) <= '1';
		--else
		--	S(1) <= '0';
		--end if;
		
		--if (M(0) AND M(1) AND M(2) AND M(3)) = '1' then
		--	S(2) <= '1';
		--else
		--	S(2) <= '0';
		--end if;
		

-- Version Case
		case M(3 downto 0)is
			when "0000" => S <= "000";
			when "0001" => S <= "001";
			when "0010" => S <= "001";
			when "0011" => S <= "010";
			when "0100" => S <= "001";
			when "0101" => S <= "010";
			when "0110" => S <= "010";
			when "0111" => S <= "011";
			when "1000" => S <= "001";
			when "1001" => S <= "010";
			when "1010" => S <= "010";
			when "1011" => S <= "011";
			when "1100" => S <= "010";
			when "1101" => S <= "011";
			when "1110" => S <= "011";
			when "1111" => S <= "100";
			when others => S <= "000";
		end case;
		
	end process;
end Behavioral;

