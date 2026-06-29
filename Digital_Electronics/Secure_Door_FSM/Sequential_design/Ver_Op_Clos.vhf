--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Ver_Op_Clos.vhf
-- /___/   /\     Timestamp : 05/26/2026 15:41:29
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/Usuario_UMA/Downloads/Practica2_JoseLuis_Raul_25_05_26/Ver_Op_Clos.vhf -w C:/Users/Usuario_UMA/Downloads/Practica2_JoseLuis_Raul_25_05_26/Ver_Op_Clos.sch
--Design Name: Ver_Op_Clos
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

entity Ver_Op_Clos is
   port ( Close      : in    std_logic; 
          Op         : in    std_logic; 
          Seven_Seg0 : out   std_logic_vector (7 downto 0); 
          Seven_Seg1 : out   std_logic_vector (7 downto 0); 
          Seven_Seg2 : out   std_logic_vector (7 downto 0); 
          Seven_Seg3 : out   std_logic_vector (7 downto 0));
end Ver_Op_Clos;

architecture BEHAVIORAL of Ver_Op_Clos is
   attribute BOX_TYPE   : string ;
   signal XLXN_15          : std_logic;
   signal XLXN_19          : std_logic;
   signal XLXN_32          : std_logic;
   signal XLXN_34          : std_logic;
   signal XLXN_39          : std_logic;
   signal XLXN_42          : std_logic;
   signal XLXN_53          : std_logic;
   signal XLXN_58          : std_logic;
   signal Seven_Seg3_DUMMY : std_logic_vector (7 downto 0);
   component OR2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR2 : component is "BLACK_BOX";
   
   component OR2B1
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR2B1 : component is "BLACK_BOX";
   
   component BUF
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of BUF : component is "BLACK_BOX";
   
   component AND2B2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND2B2 : component is "BLACK_BOX";
   
   component GND
      port ( G : out   std_logic);
   end component;
   attribute BOX_TYPE of GND : component is "BLACK_BOX";
   
   component VCC
      port ( P : out   std_logic);
   end component;
   attribute BOX_TYPE of VCC : component is "BLACK_BOX";
   
   component INV
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of INV : component is "BLACK_BOX";
   
begin
   Seven_Seg3(7 downto 0) <= Seven_Seg3_DUMMY(7 downto 0);
   XLXI_1 : OR2
      port map (I0=>Close,
                I1=>Op,
                O=>Seven_Seg3_DUMMY(0));
   
   XLXI_2 : OR2B1
      port map (I0=>Close,
                I1=>Op,
                O=>Seven_Seg3_DUMMY(1));
   
   XLXI_3 : BUF
      port map (I=>XLXN_15,
                O=>Seven_Seg3_DUMMY(3));
   
   XLXI_4 : BUF
      port map (I=>XLXN_15,
                O=>Seven_Seg3_DUMMY(4));
   
   XLXI_5 : BUF
      port map (I=>Seven_Seg3_DUMMY(1),
                O=>Seven_Seg3_DUMMY(2));
   
   XLXI_6 : BUF
      port map (I=>Seven_Seg3_DUMMY(0),
                O=>Seven_Seg3_DUMMY(5));
   
   XLXI_7 : BUF
      port map (I=>XLXN_19,
                O=>Seven_Seg3_DUMMY(7));
   
   XLXI_8 : AND2B2
      port map (I0=>Close,
                I1=>Op,
                O=>Seven_Seg3_DUMMY(6));
   
   XLXI_9 : GND
      port map (G=>XLXN_19);
   
   XLXI_10 : VCC
      port map (P=>XLXN_15);
   
   XLXI_11 : BUF
      port map (I=>Op,
                O=>Seven_Seg2(0));
   
   XLXI_12 : BUF
      port map (I=>Op,
                O=>Seven_Seg2(1));
   
   XLXI_13 : BUF
      port map (I=>Seven_Seg3_DUMMY(6),
                O=>Seven_Seg2(2));
   
   XLXI_14 : BUF
      port map (I=>XLXN_32,
                O=>Seven_Seg2(4));
   
   XLXI_16 : BUF
      port map (I=>XLXN_34,
                O=>Seven_Seg2(7));
   
   XLXI_17 : INV
      port map (I=>Op,
                O=>Seven_Seg2(3));
   
   XLXI_18 : BUF
      port map (I=>Seven_Seg3_DUMMY(0),
                O=>Seven_Seg2(5));
   
   XLXI_19 : VCC
      port map (P=>XLXN_32);
   
   XLXI_20 : GND
      port map (G=>XLXN_34);
   
   XLXI_21 : BUF
      port map (I=>Seven_Seg3_DUMMY(0),
                O=>Seven_Seg1(0));
   
   XLXI_22 : BUF
      port map (I=>Close,
                O=>Seven_Seg1(1));
   
   XLXI_24 : BUF
      port map (I=>XLXN_39,
                O=>Seven_Seg1(3));
   
   XLXI_25 : BUF
      port map (I=>XLXN_39,
                O=>Seven_Seg1(4));
   
   XLXI_26 : BUF
      port map (I=>Seven_Seg3_DUMMY(0),
                O=>Seven_Seg1(5));
   
   XLXI_28 : BUF
      port map (I=>XLXN_42,
                O=>Seven_Seg1(7));
   
   XLXI_29 : BUF
      port map (I=>Close,
                O=>Seven_Seg0(0));
   
   XLXI_30 : BUF
      port map (I=>XLXN_53,
                O=>Seven_Seg0(1));
   
   XLXI_31 : BUF
      port map (I=>Seven_Seg3_DUMMY(0),
                O=>Seven_Seg0(2));
   
   XLXI_32 : BUF
      port map (I=>Close,
                O=>Seven_Seg0(3));
   
   XLXI_34 : BUF
      port map (I=>Close,
                O=>Seven_Seg0(5));
   
   XLXI_35 : BUF
      port map (I=>XLXN_58,
                O=>Seven_Seg0(6));
   
   XLXI_36 : BUF
      port map (I=>XLXN_53,
                O=>Seven_Seg0(7));
   
   XLXI_37 : VCC
      port map (P=>XLXN_39);
   
   XLXI_38 : GND
      port map (G=>XLXN_42);
   
   XLXI_39 : GND
      port map (G=>XLXN_53);
   
   XLXI_40 : INV
      port map (I=>Close,
                O=>Seven_Seg1(6));
   
   XLXI_41 : INV
      port map (I=>Op,
                O=>Seven_Seg1(2));
   
   XLXI_42 : VCC
      port map (P=>XLXN_58);
   
   XLXI_43 : INV
      port map (I=>Close,
                O=>Seven_Seg2(6));
   
   XLXI_44 : INV
      port map (I=>Close,
                O=>Seven_Seg0(4));
   
end BEHAVIORAL;


