--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Puls_On_Off.vhf
-- /___/   /\     Timestamp : 05/14/2026 10:13:08
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Puls_On_Off.vhf -w C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Puls_On_Off.sch
--Design Name: Puls_On_Off
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

entity Puls_On_Off is
   port ( ck  : in    std_logic; 
          EN  : in    std_logic; 
          ENS : out   std_logic);
end Puls_On_Off;

architecture BEHAVIORAL of Puls_On_Off is
   attribute BOX_TYPE   : string ;
   signal XLXN_6    : std_logic;
   signal XLXN_16   : std_logic;
   signal XLXN_26   : std_logic;
   signal XLXN_27   : std_logic;
   signal XLXN_33   : std_logic;
   signal ENS_DUMMY : std_logic;
   component FD
      generic( INIT : bit :=  '0');
      port ( C : in    std_logic; 
             D : in    std_logic; 
             Q : out   std_logic);
   end component;
   attribute BOX_TYPE of FD : component is "BLACK_BOX";
   
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
   
   component INV
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of INV : component is "BLACK_BOX";
   
begin
   ENS <= ENS_DUMMY;
   XLXI_1 : FD
      port map (C=>ck,
                D=>EN,
                Q=>XLXN_16);
   
   XLXI_2 : FD
      port map (C=>ck,
                D=>XLXN_6,
                Q=>ENS_DUMMY);
   
   XLXI_4 : XOR2
      port map (I0=>XLXN_33,
                I1=>ENS_DUMMY,
                O=>XLXN_6);
   
   XLXI_7 : AND2
      port map (I0=>XLXN_27,
                I1=>XLXN_26,
                O=>XLXN_33);
   
   XLXI_12 : INV
      port map (I=>XLXN_16,
                O=>XLXN_26);
   
   XLXI_13 : FD
      port map (C=>ck,
                D=>XLXN_16,
                Q=>XLXN_27);
   
end BEHAVIORAL;


