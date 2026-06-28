--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Reg_Vot.vhf
-- /___/   /\     Timestamp : 05/07/2026 13:10:56
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/Usuario_UMA/Downloads/RLab_Plant_Jose_Raul_07-05-26_1B/Reg_Vot.vhf -w C:/Users/Usuario_UMA/Downloads/RLab_Plant_Jose_Raul_07-05-26_1B/Reg_Vot.sch
--Design Name: Reg_Vot
--Device: spartan6
--Purpose:
--    This vhdl netlist is translated from an ECS schematic. It can be 
--    synthesized and simulated, but it should not be modified. 
--
----- CELL M4_1E_HXILINX_Reg_Vot -----
  
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity M4_1E_HXILINX_Reg_Vot is
  
port(
    O   : out std_logic;

    D0  : in std_logic;
    D1  : in std_logic;
    D2  : in std_logic;
    D3  : in std_logic;
    E   : in std_logic;
    S0  : in std_logic;
    S1  : in std_logic
  );
end M4_1E_HXILINX_Reg_Vot;

architecture M4_1E_HXILINX_Reg_Vot_V of M4_1E_HXILINX_Reg_Vot is
begin
  process (D0, D1, D2, D3, E, S0, S1)
  variable sel : std_logic_vector(1 downto 0);
  begin
    sel := S1&S0;
    if( E = '0') then
    O <= '0';
    else
      case sel is
      when "00" => O <= D0;
      when "01" => O <= D1;
      when "10" => O <= D2;
      when "11" => O <= D3;
      when others => NULL;
      end case;
    end if;
    end process; 
end M4_1E_HXILINX_Reg_Vot_V;

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Reg_Vot is
   port ( CK    : in    std_logic; 
          CLR   : in    std_logic; 
          ENA   : in    std_logic; 
          VC    : in    std_logic; 
          VF    : in    std_logic; 
          Led_V : out   std_logic; 
          VF_C  : out   std_logic; 
          V_R   : out   std_logic);
end Reg_Vot;

architecture BEHAVIORAL of Reg_Vot is
   attribute BOX_TYPE   : string ;
   attribute HU_SET     : string ;
   signal D0      : std_logic;
   signal D1      : std_logic;
   signal q0      : std_logic;
   signal q1      : std_logic;
   signal XLXN_20 : std_logic;
   signal XLXN_21 : std_logic;
   signal XLXN_30 : std_logic;
   signal XLXN_34 : std_logic;
   signal XLXN_45 : std_logic;
   signal XLXN_47 : std_logic;
   component FDC
      port ( C   : in    std_logic; 
             CLR : in    std_logic; 
             D   : in    std_logic; 
             Q   : out   std_logic);
   end component;
   attribute BOX_TYPE of FDC : component is "BLACK_BOX";
   
   component AND2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND2 : component is "BLACK_BOX";
   
   component AND2B1
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND2B1 : component is "BLACK_BOX";
   
   component BUF
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of BUF : component is "BLACK_BOX";
   
   component M4_1E_HXILINX_Reg_Vot
      port ( D0 : in    std_logic; 
             D1 : in    std_logic; 
             D2 : in    std_logic; 
             D3 : in    std_logic; 
             E  : in    std_logic; 
             S0 : in    std_logic; 
             S1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   
   component XOR2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of XOR2 : component is "BLACK_BOX";
   
   component GND
      port ( G : out   std_logic);
   end component;
   attribute BOX_TYPE of GND : component is "BLACK_BOX";
   
   component VCC
      port ( P : out   std_logic);
   end component;
   attribute BOX_TYPE of VCC : component is "BLACK_BOX";
   
   component OR2B1
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR2B1 : component is "BLACK_BOX";
   
   attribute HU_SET of XLXI_6 : label is "XLXI_6_0";
   attribute HU_SET of XLXI_7 : label is "XLXI_7_1";
begin
   XLXI_1 : FDC
      port map (C=>CK,
                CLR=>CLR,
                D=>D1,
                Q=>q1);
   
   XLXI_2 : FDC
      port map (C=>CK,
                CLR=>CLR,
                D=>D0,
                Q=>q0);
   
   XLXI_3 : AND2
      port map (I0=>q0,
                I1=>q1,
                O=>VF_C);
   
   XLXI_4 : AND2B1
      port map (I0=>q1,
                I1=>q0,
                O=>Led_V);
   
   XLXI_5 : BUF
      port map (I=>q1,
                O=>V_R);
   
   XLXI_6 : M4_1E_HXILINX_Reg_Vot
      port map (D0=>ENA,
                D1=>XLXN_45,
                D2=>XLXN_21,
                D3=>XLXN_20,
                E=>XLXN_20,
                S0=>q0,
                S1=>q1,
                O=>D0);
   
   XLXI_7 : M4_1E_HXILINX_Reg_Vot
      port map (D0=>XLXN_21,
                D1=>XLXN_34,
                D2=>XLXN_20,
                D3=>XLXN_20,
                E=>XLXN_20,
                S0=>q0,
                S1=>q1,
                O=>D1);
   
   XLXI_8 : XOR2
      port map (I0=>VC,
                I1=>VF,
                O=>XLXN_30);
   
   XLXI_9 : GND
      port map (G=>XLXN_21);
   
   XLXI_10 : VCC
      port map (P=>XLXN_20);
   
   XLXI_15 : AND2
      port map (I0=>ENA,
                I1=>XLXN_30,
                O=>XLXN_34);
   
   XLXI_19 : AND2
      port map (I0=>XLXN_47,
                I1=>ENA,
                O=>XLXN_45);
   
   XLXI_20 : OR2B1
      port map (I0=>VC,
                I1=>VF,
                O=>XLXN_47);
   
end BEHAVIORAL;


