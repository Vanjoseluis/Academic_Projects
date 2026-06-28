--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : p_fin.vhf
-- /___/   /\     Timestamp : 05/26/2026 15:41:29
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/Usuario_UMA/Downloads/Practica2_JoseLuis_Raul_25_05_26/p_fin.vhf -w C:/Users/Usuario_UMA/Downloads/Practica2_JoseLuis_Raul_25_05_26/p_fin.sch
--Design Name: p_fin
--Device: spartan6
--Purpose:
--    This vhdl netlist is translated from an ECS schematic. It can be 
--    synthesized and simulated, but it should not be modified. 
--

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity p_fin is
   port ( A  : in    std_logic; 
          ck : in    std_logic; 
          S  : out   std_logic);
end p_fin;

architecture BEHAVIORAL of p_fin is
   attribute BOX_TYPE   : string ;
   signal af : std_logic;
   signal bf : std_logic;
   component FD
      generic( INIT : bit :=  '0');
      port ( C : in    std_logic; 
             D : in    std_logic; 
             Q : out   std_logic);
   end component;
   attribute BOX_TYPE of FD : component is "BLACK_BOX";
   
   component AND2B1
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND2B1 : component is "BLACK_BOX";
   
begin
   XLXI_1 : FD
      port map (C=>ck,
                D=>A,
                Q=>af);
   
   XLXI_17 : AND2B1
      port map (I0=>af,
                I1=>bf,
                O=>S);
   
   XLXI_19 : FD
      port map (C=>ck,
                D=>af,
                Q=>bf);
   
end BEHAVIORAL;


