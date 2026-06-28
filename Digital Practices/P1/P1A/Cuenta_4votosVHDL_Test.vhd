--------------------------------------------------------------------------------
-- Company: 
-- Engineer:
--
-- Create Date:   12:13:43 04/16/2026
-- Design Name:   
-- Module Name:   C:/Users/Usuario_UMA/Downloads/RLab_Plant_Jose_Raul_16-04-26/Cuenta_4votosVHDL_Test.vhd
-- Project Name:  RLab_Plant
-- Target Device:  
-- Tool versions:  
-- Description:   
-- 
-- VHDL Test Bench Created by ISE for module: Cuenta_4votosVHDL
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
 
ENTITY Cuenta_4votosVHDL_Test IS
END Cuenta_4votosVHDL_Test;

ARCHITECTURE behavior OF Cuenta_4votosVHDL_Test IS

    constant CkSemiPeriod : TIME := 50 ns;

    COMPONENT Cuenta_4votosVHDL
    PORT(
         M : IN  std_logic_vector(3 downto 0);
         S : OUT std_logic_vector(2 downto 0)
    );
    END COMPONENT;

    signal M : std_logic_vector(3 downto 0) := (others => '0');
    signal S : std_logic_vector(2 downto 0);

BEGIN

    UUT: Cuenta_4votosVHDL
        port map (
            M => M,
            S => S
        );

    clock1: process
    begin
        M(0) <= '0';
        loop
            wait for CKSemiPeriod;
            M(0) <= not M(0);
        end loop;
    end process;

    clock2: process
    begin
        M(1) <= '0';
        loop
            wait for 2*CKSemiPeriod;
            M(1) <= not M(1);
        end loop;
    end process;

    clock3: process
    begin
        M(2) <= '0';
        loop
            wait for 4*CKSemiPeriod;
            M(2) <= not M(2);
        end loop;
    end process;

    clock4: process
    begin
        M(3) <= '0';
        loop
            wait for 8*CKSemiPeriod;
            M(3) <= not M(3);
        end loop;
    end process;

END behavior;
