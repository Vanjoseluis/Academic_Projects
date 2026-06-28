--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Control.vhf
-- /___/   /\     Timestamp : 05/28/2026 09:31:44
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/Practica2_JoseLuis_Raul_26_05_26/Control.vhf -w C:/Users/edie/Desktop/Practica2_JoseLuis_Raul_26_05_26/Control.sch
--Design Name: Control
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

entity Control_Combinacional_MUSER_Control is
   port ( q : in    std_logic_vector (2 downto 0); 
          R : in    std_logic; 
          U : in    std_logic; 
          D : out   std_logic_vector (2 downto 0));
end Control_Combinacional_MUSER_Control;

architecture BEHAVIORAL of Control_Combinacional_MUSER_Control is
   attribute BOX_TYPE   : string ;
   signal XLXN_49  : std_logic;
   signal XLXN_50  : std_logic;
   signal XLXN_51  : std_logic;
   signal XLXN_121 : std_logic;
   signal XLXN_139 : std_logic;
   signal XLXN_140 : std_logic;
   signal XLXN_141 : std_logic;
   signal XLXN_142 : std_logic;
   signal XLXN_154 : std_logic;
   signal XLXN_155 : std_logic;
   signal XLXN_161 : std_logic;
   component AND4B3
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND4B3 : component is "BLACK_BOX";
   
   component AND3B2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND3B2 : component is "BLACK_BOX";
   
   component AND3
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND3 : component is "BLACK_BOX";
   
   component AND4B1
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND4B1 : component is "BLACK_BOX";
   
   component OR4
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR4 : component is "BLACK_BOX";
   
   component AND3B1
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND3B1 : component is "BLACK_BOX";
   
   component AND4B2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND4B2 : component is "BLACK_BOX";
   
   component OR3
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR3 : component is "BLACK_BOX";
   
begin
   XLXI_1 : AND4B3
      port map (I0=>q(1),
                I1=>q(2),
                I2=>R,
                I3=>U,
                O=>XLXN_49);
   
   XLXI_2 : AND3B2
      port map (I0=>q(2),
                I1=>R,
                I2=>q(0),
                O=>XLXN_51);
   
   XLXI_7 : AND3
      port map (I0=>q(1),
                I1=>q(2),
                I2=>R,
                O=>XLXN_50);
   
   XLXI_8 : AND4B1
      port map (I0=>U,
                I1=>q(0),
                I2=>q(1),
                I3=>q(2),
                O=>XLXN_121);
   
   XLXI_9 : OR4
      port map (I0=>XLXN_50,
                I1=>XLXN_51,
                I2=>XLXN_121,
                I3=>XLXN_49,
                O=>D(0));
   
   XLXI_28 : AND3B1
      port map (I0=>R,
                I1=>q(1),
                I2=>q(0),
                O=>XLXN_140);
   
   XLXI_29 : AND3
      port map (I0=>q(1),
                I1=>q(2),
                I2=>R,
                O=>XLXN_142);
   
   XLXI_33 : AND3B1
      port map (I0=>U,
                I1=>q(1),
                I2=>q(2),
                O=>XLXN_141);
   
   XLXI_35 : AND4B2
      port map (I0=>R,
                I1=>q(2),
                I2=>q(0),
                I3=>U,
                O=>XLXN_139);
   
   XLXI_37 : AND3B1
      port map (I0=>U,
                I1=>q(1),
                I2=>q(2),
                O=>XLXN_155);
   
   XLXI_39 : AND3
      port map (I0=>q(1),
                I1=>q(2),
                I2=>R,
                O=>XLXN_161);
   
   XLXI_41 : AND4B1
      port map (I0=>R,
                I1=>U,
                I2=>q(0),
                I3=>q(1),
                O=>XLXN_154);
   
   XLXI_44 : OR4
      port map (I0=>XLXN_142,
                I1=>XLXN_141,
                I2=>XLXN_140,
                I3=>XLXN_139,
                O=>D(1));
   
   XLXI_45 : OR3
      port map (I0=>XLXN_161,
                I1=>XLXN_155,
                I2=>XLXN_154,
                O=>D(2));
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Control is
   port ( Ck  : in    std_logic; 
          CLR : in    std_logic; 
          R   : in    std_logic; 
          U   : in    std_logic; 
          EST : out   std_logic_vector (2 downto 0); 
          S   : out   std_logic);
end Control;

architecture BEHAVIORAL of Control is
   attribute BOX_TYPE   : string ;
   signal D         : std_logic_vector (2 downto 0);
   signal XLXN_42   : std_logic;
   signal XLXN_44   : std_logic;
   signal EST_DUMMY : std_logic_vector (2 downto 0);
   component Control_Combinacional_MUSER_Control
      port ( D : out   std_logic_vector (2 downto 0); 
             q : in    std_logic_vector (2 downto 0); 
             R : in    std_logic; 
             U : in    std_logic);
   end component;
   
   component FDC
      port ( C   : in    std_logic; 
             CLR : in    std_logic; 
             D   : in    std_logic; 
             Q   : out   std_logic);
   end component;
   attribute BOX_TYPE of FDC : component is "BLACK_BOX";
   
   component AND3B1
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND3B1 : component is "BLACK_BOX";
   
   component AND2B2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND2B2 : component is "BLACK_BOX";
   
   component OR2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR2 : component is "BLACK_BOX";
   
begin
   EST(2 downto 0) <= EST_DUMMY(2 downto 0);
   XLXI_8 : Control_Combinacional_MUSER_Control
      port map (q(2 downto 0)=>EST_DUMMY(2 downto 0),
                R=>R,
                U=>U,
                D(2 downto 0)=>D(2 downto 0));
   
   XLXI_9 : FDC
      port map (C=>Ck,
                CLR=>CLR,
                D=>D(0),
                Q=>EST_DUMMY(0));
   
   XLXI_10 : FDC
      port map (C=>Ck,
                CLR=>CLR,
                D=>D(1),
                Q=>EST_DUMMY(1));
   
   XLXI_11 : FDC
      port map (C=>Ck,
                CLR=>CLR,
                D=>D(2),
                Q=>EST_DUMMY(2));
   
   XLXI_12 : AND3B1
      port map (I0=>EST_DUMMY(2),
                I1=>EST_DUMMY(1),
                I2=>EST_DUMMY(0),
                O=>XLXN_42);
   
   XLXI_15 : AND2B2
      port map (I0=>EST_DUMMY(2),
                I1=>EST_DUMMY(1),
                O=>XLXN_44);
   
   XLXI_16 : OR2
      port map (I0=>XLXN_44,
                I1=>XLXN_42,
                O=>S);
   
end BEHAVIORAL;


