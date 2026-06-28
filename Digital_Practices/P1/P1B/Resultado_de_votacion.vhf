--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Resultado_de_votacion.vhf
-- /___/   /\     Timestamp : 05/14/2026 10:13:09
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Resultado_de_votacion.vhf -w C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Resultado_de_votacion.sch
--Design Name: Resultado_de_votacion
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

entity Resultado_de_votacion is
   port ( SVC  : in    std_logic_vector (3 downto 0); 
          VVal : in    std_logic; 
          NA   : out   std_logic_vector (1 downto 0); 
          VC   : out   std_logic_vector (2 downto 0); 
          VF   : out   std_logic_vector (2 downto 0));
end Resultado_de_votacion;

architecture BEHAVIORAL of Resultado_de_votacion is
   attribute BOX_TYPE   : string ;
   signal XLXN_2   : std_logic;
   signal XLXN_3   : std_logic;
   signal XLXN_4   : std_logic;
   signal XLXN_15  : std_logic;
   signal XLXN_17  : std_logic;
   signal XLXN_25  : std_logic;
   signal XLXN_26  : std_logic;
   signal XLXN_27  : std_logic;
   signal XLXN_28  : std_logic;
   signal XLXN_31  : std_logic;
   signal XLXN_39  : std_logic;
   signal XLXN_42  : std_logic;
   signal XLXN_44  : std_logic;
   signal XLXN_45  : std_logic;
   signal XLXN_46  : std_logic;
   signal XLXN_49  : std_logic;
   signal XLXN_50  : std_logic;
   signal XLXN_51  : std_logic;
   signal XLXN_52  : std_logic;
   signal VC_DUMMY : std_logic_vector (2 downto 0);
   signal VF_DUMMY : std_logic_vector (2 downto 0);
   signal NA_DUMMY : std_logic_vector (1 downto 0);
   component AND3
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND3 : component is "BLACK_BOX";
   
   component INV
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of INV : component is "BLACK_BOX";
   
   component OR2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR2 : component is "BLACK_BOX";
   
   component AND4
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND4 : component is "BLACK_BOX";
   
   component AND2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND2 : component is "BLACK_BOX";
   
   component BUF
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of BUF : component is "BLACK_BOX";
   
begin
   NA(1 downto 0) <= NA_DUMMY(1 downto 0);
   VC(2 downto 0) <= VC_DUMMY(2 downto 0);
   VF(2 downto 0) <= VF_DUMMY(2 downto 0);
   XLXI_1 : AND3
      port map (I0=>XLXN_4,
                I1=>XLXN_3,
                I2=>XLXN_2,
                O=>XLXN_50);
   
   XLXI_2 : INV
      port map (I=>SVC(3),
                O=>XLXN_2);
   
   XLXI_3 : INV
      port map (I=>SVC(1),
                O=>XLXN_3);
   
   XLXI_4 : INV
      port map (I=>SVC(2),
                O=>XLXN_4);
   
   XLXI_5 : OR2
      port map (I0=>XLXN_31,
                I1=>XLXN_28,
                O=>XLXN_51);
   
   XLXI_6 : AND4
      port map (I0=>XLXN_27,
                I1=>XLXN_26,
                I2=>XLXN_25,
                I3=>SVC(3),
                O=>XLXN_31);
   
   XLXI_8 : OR2
      port map (I0=>SVC(1),
                I1=>SVC(0),
                O=>XLXN_15);
   
   XLXI_15 : AND3
      port map (I0=>XLXN_15,
                I1=>SVC(2),
                I2=>XLXN_17,
                O=>XLXN_28);
   
   XLXI_16 : INV
      port map (I=>SVC(3),
                O=>XLXN_17);
   
   XLXI_18 : INV
      port map (I=>SVC(1),
                O=>XLXN_27);
   
   XLXI_19 : INV
      port map (I=>SVC(2),
                O=>XLXN_25);
   
   XLXI_20 : INV
      port map (I=>SVC(0),
                O=>XLXN_26);
   
   XLXI_21 : INV
      port map (I=>SVC(0),
                O=>XLXN_45);
   
   XLXI_23 : INV
      port map (I=>SVC(2),
                O=>XLXN_39);
   
   XLXI_24 : INV
      port map (I=>SVC(3),
                O=>XLXN_44);
   
   XLXI_25 : AND3
      port map (I0=>XLXN_44,
                I1=>SVC(1),
                I2=>XLXN_39,
                O=>XLXN_49);
   
   XLXI_26 : AND4
      port map (I0=>XLXN_42,
                I1=>XLXN_44,
                I2=>XLXN_45,
                I3=>SVC(2),
                O=>XLXN_46);
   
   XLXI_27 : OR2
      port map (I0=>XLXN_49,
                I1=>XLXN_46,
                O=>XLXN_52);
   
   XLXI_28 : INV
      port map (I=>SVC(1),
                O=>XLXN_42);
   
   XLXI_29 : AND2
      port map (I0=>XLXN_50,
                I1=>VVal,
                O=>VF_DUMMY(0));
   
   XLXI_30 : AND2
      port map (I0=>XLXN_51,
                I1=>VVal,
                O=>VC_DUMMY(0));
   
   XLXI_31 : AND2
      port map (I0=>XLXN_52,
                I1=>VVal,
                O=>NA_DUMMY(0));
   
   XLXI_32 : BUF
      port map (I=>VF_DUMMY(0),
                O=>VF_DUMMY(1));
   
   XLXI_33 : BUF
      port map (I=>VF_DUMMY(0),
                O=>VF_DUMMY(2));
   
   XLXI_37 : BUF
      port map (I=>VC_DUMMY(0),
                O=>VC_DUMMY(1));
   
   XLXI_38 : BUF
      port map (I=>VC_DUMMY(0),
                O=>VC_DUMMY(2));
   
   XLXI_45 : BUF
      port map (I=>NA_DUMMY(0),
                O=>NA_DUMMY(1));
   
end BEHAVIORAL;


