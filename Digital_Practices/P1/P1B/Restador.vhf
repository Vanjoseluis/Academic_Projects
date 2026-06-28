--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Restador.vhf
-- /___/   /\     Timestamp : 05/14/2026 10:13:16
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Restador.vhf -w C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Restador.sch
--Design Name: Restador
--Device: spartan6
--Purpose:
--    This vhdl netlist is translated from an ECS schematic. It can be 
--    synthesized and simulated, but it should not be modified. 
--
----- CELL INV4_HXILINX_Restador -----
  
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity INV4_HXILINX_Restador is
  
port(
    O0  : out std_logic;
    O1  : out std_logic;
    O2  : out std_logic;
    O3  : out std_logic;

    I0  : in std_logic;
    I1  : in std_logic;
    I2  : in std_logic;
    I3  : in std_logic
  );
end INV4_HXILINX_Restador;

architecture INV4_HXILINX_Restador_V of INV4_HXILINX_Restador is
begin
  O0 <= not I0 ;
  O1 <= not I1 ;
  O2 <= not I2 ;
  O3 <= not I3 ;
end INV4_HXILINX_Restador_V;

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Sumador_MUSER_Restador is
   port ( A  : in    std_logic; 
          B  : in    std_logic; 
          C0 : in    std_logic; 
          C1 : out   std_logic; 
          S  : out   std_logic);
end Sumador_MUSER_Restador;

architecture BEHAVIORAL of Sumador_MUSER_Restador is
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

entity Sumador_Completo_MUSER_Restador is
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
end Sumador_Completo_MUSER_Restador;

architecture BEHAVIORAL of Sumador_Completo_MUSER_Restador is
   signal C1       : std_logic;
   signal C2       : std_logic;
   signal C3_DUMMY : std_logic;
   component Sumador_MUSER_Restador
      port ( A  : in    std_logic; 
             B  : in    std_logic; 
             C0 : in    std_logic; 
             S  : out   std_logic; 
             C1 : out   std_logic);
   end component;
   
begin
   C3 <= C3_DUMMY;
   XLXI_1 : Sumador_MUSER_Restador
      port map (A=>A0,
                B=>B0,
                C0=>C0,
                C1=>C1,
                S=>S0);
   
   XLXI_2 : Sumador_MUSER_Restador
      port map (A=>A1,
                B=>B1,
                C0=>C1,
                C1=>C2,
                S=>S1);
   
   XLXI_3 : Sumador_MUSER_Restador
      port map (A=>A2,
                B=>B2,
                C0=>C2,
                C1=>C3_DUMMY,
                S=>S2);
   
   XLXI_4 : Sumador_MUSER_Restador
      port map (A=>A3,
                B=>B3,
                C0=>C3_DUMMY,
                C1=>C4,
                S=>S3);
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Complemento_a2_MUSER_Restador is
   port ( D0 : in    std_logic; 
          D1 : in    std_logic; 
          D2 : in    std_logic; 
          D3 : in    std_logic; 
          S0 : out   std_logic; 
          S1 : out   std_logic; 
          S2 : out   std_logic; 
          S3 : out   std_logic; 
          S4 : out   std_logic);
end Complemento_a2_MUSER_Restador;

architecture BEHAVIORAL of Complemento_a2_MUSER_Restador is
   attribute HU_SET     : string ;
   attribute BOX_TYPE   : string ;
   signal A0      : std_logic;
   signal A1      : std_logic;
   signal A2      : std_logic;
   signal A3      : std_logic;
   signal XLXN_21 : std_logic;
   signal XLXN_22 : std_logic;
   component INV4_HXILINX_Restador
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O0 : out   std_logic; 
             O1 : out   std_logic; 
             O2 : out   std_logic; 
             O3 : out   std_logic);
   end component;
   
   component GND
      port ( G : out   std_logic);
   end component;
   attribute BOX_TYPE of GND : component is "BLACK_BOX";
   
   component VCC
      port ( P : out   std_logic);
   end component;
   attribute BOX_TYPE of VCC : component is "BLACK_BOX";
   
   component Sumador_Completo_MUSER_Restador
      port ( A0 : in    std_logic; 
             A1 : in    std_logic; 
             A2 : in    std_logic; 
             A3 : in    std_logic; 
             B0 : in    std_logic; 
             B1 : in    std_logic; 
             B2 : in    std_logic; 
             B3 : in    std_logic; 
             C0 : in    std_logic; 
             C4 : out   std_logic; 
             S0 : out   std_logic; 
             S1 : out   std_logic; 
             S2 : out   std_logic; 
             S3 : out   std_logic);
   end component;
   
   attribute HU_SET of XLXI_6 : label is "XLXI_6_77";
begin
   XLXI_6 : INV4_HXILINX_Restador
      port map (I0=>D3,
                I1=>D2,
                I2=>D1,
                I3=>D0,
                O0=>A3,
                O1=>A2,
                O2=>A1,
                O3=>A0);
   
   XLXI_14 : GND
      port map (G=>XLXN_22);
   
   XLXI_18 : VCC
      port map (P=>XLXN_21);
   
   XLXI_19 : Sumador_Completo_MUSER_Restador
      port map (A0=>A0,
                A1=>A1,
                A2=>A2,
                A3=>A3,
                B0=>XLXN_21,
                B1=>XLXN_22,
                B2=>XLXN_22,
                B3=>XLXN_22,
                C0=>XLXN_22,
                C4=>S4,
                S0=>S0,
                S1=>S1,
                S2=>S2,
                S3=>S3);
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Restador is
   port ( A0 : in    std_logic; 
          A1 : in    std_logic; 
          A2 : in    std_logic; 
          A3 : in    std_logic; 
          B0 : in    std_logic; 
          B1 : in    std_logic; 
          B2 : in    std_logic; 
          B3 : in    std_logic; 
          C0 : in    std_logic; 
          S0 : out   std_logic; 
          S1 : out   std_logic; 
          S2 : out   std_logic; 
          S3 : out   std_logic; 
          S4 : out   std_logic);
end Restador;

architecture BEHAVIORAL of Restador is
   signal XLXN_114 : std_logic;
   signal XLXN_115 : std_logic;
   signal XLXN_116 : std_logic;
   signal XLXN_117 : std_logic;
   component Complemento_a2_MUSER_Restador
      port ( D0 : in    std_logic; 
             D1 : in    std_logic; 
             D2 : in    std_logic; 
             D3 : in    std_logic; 
             S0 : out   std_logic; 
             S1 : out   std_logic; 
             S2 : out   std_logic; 
             S3 : out   std_logic);
   end component;
   
   component Sumador_Completo_MUSER_Restador
      port ( A0 : in    std_logic; 
             A1 : in    std_logic; 
             A2 : in    std_logic; 
             A3 : in    std_logic; 
             B0 : in    std_logic; 
             B1 : in    std_logic; 
             B2 : in    std_logic; 
             B3 : in    std_logic; 
             C0 : in    std_logic; 
             C4 : out   std_logic; 
             S0 : out   std_logic; 
             S1 : out   std_logic; 
             S2 : out   std_logic; 
             S3 : out   std_logic);
   end component;
   
begin
   XLXI_17 : Complemento_a2_MUSER_Restador
      port map (D0=>B0,
                D1=>B1,
                D2=>B2,
                D3=>B3,
                S0=>XLXN_114,
                S1=>XLXN_115,
                S2=>XLXN_116,
                S3=>XLXN_117);
   
   XLXI_18 : Sumador_Completo_MUSER_Restador
      port map (A0=>A0,
                A1=>A1,
                A2=>A2,
                A3=>A3,
                B0=>XLXN_114,
                B1=>XLXN_115,
                B2=>XLXN_116,
                B3=>XLXN_117,
                C0=>C0,
                C4=>S4,
                S0=>S0,
                S1=>S1,
                S2=>S2,
                S3=>S3);
   
end BEHAVIORAL;


