--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : VCodAc.vhf
-- /___/   /\     Timestamp : 05/26/2026 15:41:29
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/Usuario_UMA/Downloads/Practica2_JoseLuis_Raul_25_05_26/VCodAc.vhf -w C:/Users/Usuario_UMA/Downloads/Practica2_JoseLuis_Raul_25_05_26/VCodAc.sch
--Design Name: VCodAc
--Device: spartan6
--Purpose:
--    This vhdl netlist is translated from an ECS schematic. It can be 
--    synthesized and simulated, but it should not be modified. 
--
----- CELL M16_1E_HXILINX_VCodAc -----
  
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity M16_1E_HXILINX_VCodAc is
  
port(
    O    : out std_logic;

    D0   : in std_logic;
    D1   : in std_logic;
    D2   : in std_logic;
    D3   : in std_logic;
    D4   : in std_logic;
    D5   : in std_logic;
    D6   : in std_logic;
    D7   : in std_logic;
    D8   : in std_logic;
    D9   : in std_logic;
    D10  : in std_logic;
    D11  : in std_logic;
    D12  : in std_logic;
    D13  : in std_logic;
    D14  : in std_logic;
    D15  : in std_logic;
    E    : in std_logic;
    S0   : in std_logic;
    S1   : in std_logic;
    S2   : in std_logic;
    S3   : in std_logic
  );
end M16_1E_HXILINX_VCodAc;

architecture M16_1E_HXILINX_VCodAc_V of M16_1E_HXILINX_VCodAc is
begin
  process (D0, D1, D2, D3, D4, D5, D6, D7, D8, D9, D10, D11, D12, D13, D14, D15, E, S0, S1, S2, S3)
  variable sel : std_logic_vector(3 downto 0);
  begin
    sel := S3&S2&S1&S0;
    if( E = '0') then
    O <= '0';
    else
      case sel is
      when "0000" => O <= D0;
      when "0001" => O <= D1;
      when "0010" => O <= D2;
      when "0011" => O <= D3;
      when "0100" => O <= D4;
      when "0101" => O <= D5;
      when "0110" => O <= D6;
      when "0111" => O <= D7;
      when "1000" => O <= D8;
      when "1001" => O <= D9;
      when "1010" => O <= D10;
      when "1011" => O <= D11;
      when "1100" => O <= D12;
      when "1101" => O <= D13;
      when "1110" => O <= D14;
      when "1111" => O <= D15;
      when others => NULL;
      end case;
    end if;
    end process; 
end M16_1E_HXILINX_VCodAc_V;

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity VCodAc is
   port ( EST : in    std_logic_vector (2 downto 0); 
          P   : in    std_logic_vector (3 downto 0); 
          USU : in    std_logic_vector (2 downto 1); 
          F   : out   std_logic; 
          V   : out   std_logic);
end VCodAc;

architecture BEHAVIORAL of VCodAc is
   attribute HU_SET     : string ;
   attribute BOX_TYPE   : string ;
   signal C       : std_logic;
   signal En      : std_logic;
   signal VA      : std_logic;
   signal XLXN_5  : std_logic;
   signal XLXN_78 : std_logic;
   component M16_1E_HXILINX_VCodAc
      port ( D0  : in    std_logic; 
             D1  : in    std_logic; 
             D10 : in    std_logic; 
             D11 : in    std_logic; 
             D12 : in    std_logic; 
             D13 : in    std_logic; 
             D14 : in    std_logic; 
             D15 : in    std_logic; 
             D2  : in    std_logic; 
             D3  : in    std_logic; 
             D4  : in    std_logic; 
             D5  : in    std_logic; 
             D6  : in    std_logic; 
             D7  : in    std_logic; 
             D8  : in    std_logic; 
             D9  : in    std_logic; 
             E   : in    std_logic; 
             S0  : in    std_logic; 
             S1  : in    std_logic; 
             S2  : in    std_logic; 
             S3  : in    std_logic; 
             O   : out   std_logic);
   end component;
   
   component OR4
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR4 : component is "BLACK_BOX";
   
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
   
   attribute HU_SET of XLXI_3 : label is "XLXI_3_10";
begin
   XLXI_3 : M16_1E_HXILINX_VCodAc
      port map (D0=>P(0),
                D1=>P(2),
                D2=>XLXN_78,
                D3=>P(1),
                D4=>XLXN_78,
                D5=>XLXN_78,
                D6=>P(1),
                D7=>P(2),
                D8=>P(1),
                D9=>P(3),
                D10=>XLXN_78,
                D11=>P(2),
                D12=>XLXN_78,
                D13=>XLXN_78,
                D14=>P(0),
                D15=>P(2),
                E=>En,
                S0=>EST(0),
                S1=>EST(1),
                S2=>EST(2),
                S3=>C,
                O=>VA);
   
   XLXI_4 : OR4
      port map (I0=>P(3),
                I1=>P(2),
                I2=>P(1),
                I3=>P(0),
                O=>XLXN_5);
   
   XLXI_5 : AND2
      port map (I0=>VA,
                I1=>XLXN_5,
                O=>V);
   
   XLXI_6 : AND2B1
      port map (I0=>VA,
                I1=>XLXN_5,
                O=>F);
   
   XLXI_10 : XOR2
      port map (I0=>USU(2),
                I1=>USU(1),
                O=>En);
   
   XLXI_11 : AND2B1
      port map (I0=>USU(1),
                I1=>USU(2),
                O=>C);
   
   XLXI_12 : GND
      port map (G=>XLXN_78);
   
end BEHAVIORAL;


