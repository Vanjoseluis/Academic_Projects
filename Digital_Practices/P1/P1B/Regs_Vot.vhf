--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Regs_Vot.vhf
-- /___/   /\     Timestamp : 05/14/2026 10:13:13
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Regs_Vot.vhf -w C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Regs_Vot.sch
--Design Name: Regs_Vot
--Device: spartan6
--Purpose:
--    This vhdl netlist is translated from an ECS schematic. It can be 
--    synthesized and simulated, but it should not be modified. 
--
----- CELL NOR7_HXILINX_Regs_Vot -----
  
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity NOR7_HXILINX_Regs_Vot is
  
port(
    O  : out std_logic;

    I0  : in std_logic;
    I1  : in std_logic;
    I2  : in std_logic;
    I3  : in std_logic;
    I4  : in std_logic;
    I5  : in std_logic;
    I6  : in std_logic
  );
end NOR7_HXILINX_Regs_Vot;

architecture NOR7_HXILINX_Regs_Vot_V of NOR7_HXILINX_Regs_Vot is
begin
  O <= not (I0 or I1 or I2 or I3 or I4 or I5 or I6);
end NOR7_HXILINX_Regs_Vot_V;
----- CELL AND8_HXILINX_Regs_Vot -----
  
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity AND8_HXILINX_Regs_Vot is
  
port(
    O  : out std_logic;

    I0  : in std_logic;
    I1  : in std_logic;
    I2  : in std_logic;
    I3  : in std_logic;
    I4  : in std_logic;
    I5  : in std_logic;
    I6  : in std_logic;
    I7  : in std_logic
  );
end AND8_HXILINX_Regs_Vot;

architecture AND8_HXILINX_Regs_Vot_V of AND8_HXILINX_Regs_Vot is
begin
  O <= I0 and I1 and I2 and I3 and I4 and I5 and I6 and I7;
end AND8_HXILINX_Regs_Vot_V;

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity ENA_GEN_MUSER_Regs_Vot is
   port ( Sel : in    std_logic_vector (7 downto 0); 
          Ena : out   std_logic_vector (7 downto 0));
end ENA_GEN_MUSER_Regs_Vot;

architecture BEHAVIORAL of ENA_GEN_MUSER_Regs_Vot is
   attribute BOX_TYPE   : string ;
   attribute HU_SET     : string ;
   signal XLXN_141 : std_logic;
   signal XLXN_154 : std_logic;
   signal XLXN_155 : std_logic;
   signal XLXN_156 : std_logic;
   signal XLXN_157 : std_logic;
   signal XLXN_158 : std_logic;
   signal XLXN_290 : std_logic;
   signal XLXN_300 : std_logic;
   component AND2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND2 : component is "BLACK_BOX";
   
   component NOR7_HXILINX_Regs_Vot
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             I4 : in    std_logic; 
             I5 : in    std_logic; 
             I6 : in    std_logic; 
             O  : out   std_logic);
   end component;
   
   attribute HU_SET of XLXI_37 : label is "XLXI_37_65";
   attribute HU_SET of XLXI_38 : label is "XLXI_38_70";
   attribute HU_SET of XLXI_39 : label is "XLXI_39_67";
   attribute HU_SET of XLXI_40 : label is "XLXI_40_66";
   attribute HU_SET of XLXI_41 : label is "XLXI_41_69";
   attribute HU_SET of XLXI_42 : label is "XLXI_42_68";
   attribute HU_SET of XLXI_70 : label is "XLXI_70_71";
   attribute HU_SET of XLXI_72 : label is "XLXI_72_72";
begin
   XLXI_2 : AND2
      port map (I0=>XLXN_156,
                I1=>Sel(3),
                O=>Ena(3));
   
   XLXI_4 : AND2
      port map (I0=>XLXN_157,
                I1=>Sel(4),
                O=>Ena(4));
   
   XLXI_14 : AND2
      port map (I0=>XLXN_158,
                I1=>Sel(5),
                O=>Ena(5));
   
   XLXI_20 : AND2
      port map (I0=>XLXN_141,
                I1=>Sel(0),
                O=>Ena(0));
   
   XLXI_22 : AND2
      port map (I0=>XLXN_155,
                I1=>Sel(1),
                O=>Ena(1));
   
   XLXI_24 : AND2
      port map (I0=>XLXN_154,
                I1=>Sel(2),
                O=>Ena(2));
   
   XLXI_37 : NOR7_HXILINX_Regs_Vot
      port map (I0=>Sel(7),
                I1=>Sel(6),
                I2=>Sel(5),
                I3=>Sel(4),
                I4=>Sel(3),
                I5=>Sel(2),
                I6=>Sel(1),
                O=>XLXN_141);
   
   XLXI_38 : NOR7_HXILINX_Regs_Vot
      port map (I0=>Sel(7),
                I1=>Sel(5),
                I2=>Sel(4),
                I3=>Sel(3),
                I4=>Sel(2),
                I5=>Sel(6),
                I6=>Sel(0),
                O=>XLXN_155);
   
   XLXI_39 : NOR7_HXILINX_Regs_Vot
      port map (I0=>Sel(7),
                I1=>Sel(6),
                I2=>Sel(5),
                I3=>Sel(4),
                I4=>Sel(3),
                I5=>Sel(1),
                I6=>Sel(0),
                O=>XLXN_154);
   
   XLXI_40 : NOR7_HXILINX_Regs_Vot
      port map (I0=>Sel(7),
                I1=>Sel(1),
                I2=>Sel(6),
                I3=>Sel(5),
                I4=>Sel(4),
                I5=>Sel(2),
                I6=>Sel(0),
                O=>XLXN_156);
   
   XLXI_41 : NOR7_HXILINX_Regs_Vot
      port map (I0=>Sel(7),
                I1=>Sel(5),
                I2=>Sel(3),
                I3=>Sel(2),
                I4=>Sel(1),
                I5=>Sel(0),
                I6=>Sel(6),
                O=>XLXN_157);
   
   XLXI_42 : NOR7_HXILINX_Regs_Vot
      port map (I0=>Sel(7),
                I1=>Sel(0),
                I2=>Sel(1),
                I3=>Sel(2),
                I4=>Sel(3),
                I5=>Sel(6),
                I6=>Sel(4),
                O=>XLXN_158);
   
   XLXI_69 : AND2
      port map (I0=>XLXN_290,
                I1=>Sel(6),
                O=>Ena(6));
   
   XLXI_70 : NOR7_HXILINX_Regs_Vot
      port map (I0=>Sel(7),
                I1=>Sel(5),
                I2=>Sel(4),
                I3=>Sel(3),
                I4=>Sel(2),
                I5=>Sel(1),
                I6=>Sel(0),
                O=>XLXN_290);
   
   XLXI_71 : AND2
      port map (I0=>XLXN_300,
                I1=>Sel(7),
                O=>Ena(7));
   
   XLXI_72 : NOR7_HXILINX_Regs_Vot
      port map (I0=>Sel(6),
                I1=>Sel(5),
                I2=>Sel(4),
                I3=>Sel(3),
                I4=>Sel(2),
                I5=>Sel(1),
                I6=>Sel(0),
                O=>XLXN_300);
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Regs_Vot is
   port ( CK    : in    std_logic; 
          CLR   : in    std_logic; 
          Sel   : in    std_logic_vector (7 downto 0); 
          VC    : in    std_logic; 
          VF    : in    std_logic; 
          Led_V : out   std_logic_vector (7 downto 0); 
          VF_C  : out   std_logic_vector (7 downto 0); 
          V_R   : out   std_logic_vector (7 downto 0); 
          V_RR  : out   std_logic);
end Regs_Vot;

architecture BEHAVIORAL of Regs_Vot is
   attribute HU_SET     : string ;
   signal Ena       : std_logic_vector (7 downto 0);
   signal V_R_DUMMY : std_logic_vector (7 downto 0);
   component AND8_HXILINX_Regs_Vot
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             I4 : in    std_logic; 
             I5 : in    std_logic; 
             I6 : in    std_logic; 
             I7 : in    std_logic; 
             O  : out   std_logic);
   end component;
   
   component ENA_GEN_MUSER_Regs_Vot
      port ( Ena : out   std_logic_vector (7 downto 0); 
             Sel : in    std_logic_vector (7 downto 0));
   end component;
   
   component Reg_Vot_VHDL_Exc
      port ( VF    : in    std_logic; 
             VC    : in    std_logic; 
             ENA   : in    std_logic; 
             VF_C  : out   std_logic; 
             CK    : in    std_logic; 
             CLR   : in    std_logic; 
             Led_V : out   std_logic; 
             V_R   : out   std_logic);
   end component;
   
   component Reg_Vot_VHDL_Case
      port ( VF    : in    std_logic; 
             VC    : in    std_logic; 
             ENA   : in    std_logic; 
             CLR   : in    std_logic; 
             CK    : in    std_logic; 
             VF_C  : out   std_logic; 
             V_R   : out   std_logic; 
             led_V : out   std_logic);
   end component;
   
   attribute HU_SET of XLXI_33 : label is "XLXI_33_73";
begin
   V_R(7 downto 0) <= V_R_DUMMY(7 downto 0);
   XLXI_33 : AND8_HXILINX_Regs_Vot
      port map (I0=>V_R_DUMMY(7),
                I1=>V_R_DUMMY(6),
                I2=>V_R_DUMMY(5),
                I3=>V_R_DUMMY(4),
                I4=>V_R_DUMMY(3),
                I5=>V_R_DUMMY(2),
                I6=>V_R_DUMMY(1),
                I7=>V_R_DUMMY(0),
                O=>V_RR);
   
   XLXI_34 : ENA_GEN_MUSER_Regs_Vot
      port map (Sel(7 downto 0)=>Sel(7 downto 0),
                Ena(7 downto 0)=>Ena(7 downto 0));
   
   XLXI_40 : Reg_Vot_VHDL_Exc
      port map (CK=>CK,
                CLR=>CLR,
                ENA=>Ena(0),
                VC=>VC,
                VF=>VF,
                Led_V=>Led_V(0),
                VF_C=>VF_C(0),
                V_R=>V_R_DUMMY(0));
   
   XLXI_41 : Reg_Vot_VHDL_Exc
      port map (CK=>CK,
                CLR=>CLR,
                ENA=>Ena(1),
                VC=>VC,
                VF=>VF,
                Led_V=>Led_V(1),
                VF_C=>VF_C(1),
                V_R=>V_R_DUMMY(1));
   
   XLXI_42 : Reg_Vot_VHDL_Exc
      port map (CK=>CK,
                CLR=>CLR,
                ENA=>Ena(2),
                VC=>VC,
                VF=>VF,
                Led_V=>Led_V(2),
                VF_C=>VF_C(2),
                V_R=>V_R_DUMMY(2));
   
   XLXI_45 : Reg_Vot_VHDL_Exc
      port map (CK=>CK,
                CLR=>CLR,
                ENA=>Ena(5),
                VC=>VC,
                VF=>VF,
                Led_V=>Led_V(5),
                VF_C=>VF_C(5),
                V_R=>V_R_DUMMY(5));
   
   XLXI_46 : Reg_Vot_VHDL_Exc
      port map (CK=>CK,
                CLR=>CLR,
                ENA=>Ena(4),
                VC=>VC,
                VF=>VF,
                Led_V=>Led_V(4),
                VF_C=>VF_C(4),
                V_R=>V_R_DUMMY(4));
   
   XLXI_47 : Reg_Vot_VHDL_Exc
      port map (CK=>CK,
                CLR=>CLR,
                ENA=>Ena(3),
                VC=>VC,
                VF=>VF,
                Led_V=>Led_V(3),
                VF_C=>VF_C(3),
                V_R=>V_R_DUMMY(3));
   
   XLXI_52 : Reg_Vot_VHDL_Case
      port map (CK=>CK,
                CLR=>CLR,
                ENA=>Ena(6),
                VC=>VC,
                VF=>VF,
                led_V=>Led_V(6),
                VF_C=>VF_C(6),
                V_R=>V_R_DUMMY(6));
   
   XLXI_54 : Reg_Vot_VHDL_Case
      port map (CK=>CK,
                CLR=>CLR,
                ENA=>Ena(7),
                VC=>VC,
                VF=>VF,
                led_V=>Led_V(7),
                VF_C=>VF_C(7),
                V_R=>V_R_DUMMY(7));
   
end BEHAVIORAL;


