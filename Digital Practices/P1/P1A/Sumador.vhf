--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Sumador.vhf
-- /___/   /\     Timestamp : 04/23/2026 09:38:04
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_16-04-26/Sumador.vhf -w C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_16-04-26/Sumador.sch
--Design Name: Sumador
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

entity Sumador is
   port ( A  : in    std_logic; 
          B  : in    std_logic; 
          C0 : in    std_logic; 
          C1 : out   std_logic; 
          S  : out   std_logic);
end Sumador;

architecture BEHAVIORAL of Sumador is
   attribute BOX_TYPE   : string ;
   signal XLXN_6 : std_logic;
   signal XLXN_7 : std_logic;
   signal XLXN_8 : std_logic;
   component XOR2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of XOR2 : component is "BLACK_BOX";
   
   component AND2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND2 : component is "BLACK_BOX";
   
   component OR2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR2 : component is "BLACK_BOX";
   
begin
   XLXI_1 : XOR2
      port map (I0=>B,
                I1=>A,
                O=>XLXN_8);
   
   XLXI_2 : XOR2
      port map (I0=>C0,
                I1=>XLXN_8,
                O=>S);
   
   XLXI_3 : AND2
      port map (I0=>B,
                I1=>A,
                O=>XLXN_6);
   
   XLXI_4 : AND2
      port map (I0=>C0,
                I1=>XLXN_8,
                O=>XLXN_7);
   
   XLXI_5 : OR2
      port map (I0=>XLXN_6,
                I1=>XLXN_7,
                O=>C1);
   
end BEHAVIORAL;


