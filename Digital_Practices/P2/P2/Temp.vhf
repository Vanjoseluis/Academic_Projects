--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Temp.vhf
-- /___/   /\     Timestamp : 05/26/2026 15:41:29
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/Usuario_UMA/Downloads/Practica2_JoseLuis_Raul_25_05_26/Temp.vhf -w C:/Users/Usuario_UMA/Downloads/Practica2_JoseLuis_Raul_25_05_26/Temp.sch
--Design Name: Temp
--Device: spartan6
--Purpose:
--    This vhdl netlist is translated from an ECS schematic. It can be 
--    synthesized and simulated, but it should not be modified. 
--
----- CELL CC8RE_HXILINX_Temp -----

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity CC8RE_HXILINX_Temp is
port (
    CEO : out STD_LOGIC;
    Q   : out STD_LOGIC_VECTOR(7 downto 0);
    TC  : out STD_LOGIC;
    C   : in STD_LOGIC;
    CE  : in STD_LOGIC;
    R   : in STD_LOGIC
    );
end CC8RE_HXILINX_Temp;

architecture CC8RE_HXILINX_Temp_V of CC8RE_HXILINX_Temp is

  signal COUNT : STD_LOGIC_VECTOR(7 downto 0) := (others => '0');
  constant TERMINAL_COUNT : STD_LOGIC_VECTOR(7 downto 0) := (others => '1');

begin

process(C)
begin
  if (C'event and C ='1') then
    if (R='1') then
      COUNT <= (others => '0');
    elsif (CE='1') then 
      COUNT <= COUNT+1;
    end if;
  end if;
end process;

TC <= '0' when (R='1') else
      '1' when (COUNT = TERMINAL_COUNT) else '0'; 
CEO <= '1' when ((COUNT = TERMINAL_COUNT) and CE='1') else '0'; 
Q <= COUNT;

end CC8RE_HXILINX_Temp_V;
----- CELL M2_1_HXILINX_Temp -----
  
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity M2_1_HXILINX_Temp is
  
port(
    O   : out std_logic;

    D0  : in std_logic;
    D1  : in std_logic;
    S0  : in std_logic
  );
end M2_1_HXILINX_Temp;

architecture M2_1_HXILINX_Temp_V of M2_1_HXILINX_Temp is
begin
  process (D0, D1, S0)
  begin
    case S0 is
    when '0' => O <= D0;
    when '1' => O <= D1;
    when others => NULL;
    end case;
    end process; 
end M2_1_HXILINX_Temp_V;

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Temp is
   port ( Ck  : in    std_logic; 
          CLR : in    std_logic; 
          EST : in    std_logic_vector (2 downto 0); 
          FC  : out   std_logic; 
          Q   : out   std_logic_vector (7 downto 0));
end Temp;

architecture BEHAVIORAL of Temp is
   attribute HU_SET     : string ;
   attribute BOX_TYPE   : string ;
   signal XLXN_7  : std_logic;
   signal XLXN_18 : std_logic;
   signal XLXN_25 : std_logic;
   component CC8RE_HXILINX_Temp
      port ( C   : in    std_logic; 
             CE  : in    std_logic; 
             R   : in    std_logic; 
             CEO : out   std_logic; 
             Q   : out   std_logic_vector (7 downto 0); 
             TC  : out   std_logic);
   end component;
   
   component INV
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of INV : component is "BLACK_BOX";
   
   component M2_1_HXILINX_Temp
      port ( D0 : in    std_logic; 
             D1 : in    std_logic; 
             S0 : in    std_logic; 
             O  : out   std_logic);
   end component;
   
   attribute HU_SET of XLXI_1 : label is "XLXI_1_11";
   attribute HU_SET of XLXI_9 : label is "XLXI_9_12";
begin
   XLXI_1 : CC8RE_HXILINX_Temp
      port map (C=>Ck,
                CE=>XLXN_7,
                R=>XLXN_18,
                CEO=>open,
                Q(7 downto 0)=>Q(7 downto 0),
                TC=>FC);
   
   XLXI_7 : INV
      port map (I=>CLR,
                O=>XLXN_18);
   
   XLXI_9 : M2_1_HXILINX_Temp
      port map (D0=>EST(1),
                D1=>XLXN_25,
                S0=>EST(0),
                O=>XLXN_7);
   
   XLXI_10 : INV
      port map (I=>EST(2),
                O=>XLXN_25);
   
end BEHAVIORAL;


