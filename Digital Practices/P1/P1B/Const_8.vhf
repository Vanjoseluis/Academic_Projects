--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Const_8.vhf
-- /___/   /\     Timestamp : 05/14/2026 10:13:10
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Const_8.vhf -w C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Const_8.sch
--Design Name: Const_8
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

entity Const_8 is
   port ( Const_8 : out   std_logic_vector (3 downto 0));
end Const_8;

architecture BEHAVIORAL of Const_8 is
   attribute BOX_TYPE   : string ;
   component VCC
      port ( P : out   std_logic);
   end component;
   attribute BOX_TYPE of VCC : component is "BLACK_BOX";
   
   component GND
      port ( G : out   std_logic);
   end component;
   attribute BOX_TYPE of GND : component is "BLACK_BOX";
   
begin
   XLXI_10 : VCC
      port map (P=>Const_8(3));
   
   XLXI_11 : GND
      port map (G=>Const_8(2));
   
   XLXI_12 : GND
      port map (G=>Const_8(1));
   
   XLXI_13 : GND
      port map (G=>Const_8(0));
   
end BEHAVIORAL;


