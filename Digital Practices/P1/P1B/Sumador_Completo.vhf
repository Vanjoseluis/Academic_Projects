--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Sumador_Completo.vhf
-- /___/   /\     Timestamp : 05/14/2026 10:13:15
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Sumador_Completo.vhf -w C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Sumador_Completo.sch
--Design Name: Sumador_Completo
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

entity Sumador_MUSER_Sumador_Completo is
   port ( A  : in    std_logic; 
          B  : in    std_logic; 
          C0 : in    std_logic; 
          C1 : out   std_logic; 
          S  : out   std_logic);
end Sumador_MUSER_Sumador_Completo;

architecture BEHAVIORAL of Sumador_MUSER_Sumador_Completo is
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



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Sumador_Completo is
   port ( A0 : in    std_logic; 
          A1 : in    std_logic; 
          A2 : in    std_logic; 
          A3 : in    std_logic; 
          B0 : in    std_logic; 
          B1 : in    std_logic; 
          B2 : in    std_logic; 
          B3 : in    std_logic; 
          C0 : in    std_logic; 
          C3 : out   std_logic; 
          C4 : out   std_logic; 
          S0 : out   std_logic; 
          S1 : out   std_logic; 
          S2 : out   std_logic; 
          S3 : out   std_logic);
end Sumador_Completo;

architecture BEHAVIORAL of Sumador_Completo is
   signal C1       : std_logic;
   signal C2       : std_logic;
   signal C3_DUMMY : std_logic;
   component Sumador_MUSER_Sumador_Completo
      port ( A  : in    std_logic; 
             B  : in    std_logic; 
             C0 : in    std_logic; 
             S  : out   std_logic; 
             C1 : out   std_logic);
   end component;
   
begin
   C3 <= C3_DUMMY;
   XLXI_1 : Sumador_MUSER_Sumador_Completo
      port map (A=>A0,
                B=>B0,
                C0=>C0,
                C1=>C1,
                S=>S0);
   
   XLXI_2 : Sumador_MUSER_Sumador_Completo
      port map (A=>A1,
                B=>B1,
                C0=>C1,
                C1=>C2,
                S=>S1);
   
   XLXI_3 : Sumador_MUSER_Sumador_Completo
      port map (A=>A2,
                B=>B2,
                C0=>C2,
                C1=>C3_DUMMY,
                S=>S2);
   
   XLXI_4 : Sumador_MUSER_Sumador_Completo
      port map (A=>A3,
                B=>B3,
                C0=>C3_DUMMY,
                C1=>C4,
                S=>S3);
   
end BEHAVIORAL;


