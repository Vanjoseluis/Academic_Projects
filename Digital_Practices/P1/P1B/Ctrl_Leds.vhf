--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Ctrl_Leds.vhf
-- /___/   /\     Timestamp : 05/14/2026 10:13:06
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Ctrl_Leds.vhf -w C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Ctrl_Leds.sch
--Design Name: Ctrl_Leds
--Device: spartan6
--Purpose:
--    This vhdl netlist is translated from an ECS schematic. It can be 
--    synthesized and simulated, but it should not be modified. 
--
----- CELL M4_1E_HXILINX_Ctrl_Leds -----
  
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity M4_1E_HXILINX_Ctrl_Leds is
  
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
end M4_1E_HXILINX_Ctrl_Leds;

architecture M4_1E_HXILINX_Ctrl_Leds_V of M4_1E_HXILINX_Ctrl_Leds is
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
end M4_1E_HXILINX_Ctrl_Leds_V;

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Ctrl_Leds is
   port ( On_Off : in    std_logic; 
          VerVot : in    std_logic; 
          VLed   : in    std_logic_vector (7 downto 0); 
          VRes   : in    std_logic_vector (7 downto 0); 
          V_R    : in    std_logic_vector (7 downto 0); 
          V_RR   : in    std_logic; 
          Leds   : out   std_logic_vector (7 downto 0));
end Ctrl_Leds;

architecture BEHAVIORAL of Ctrl_Leds is
   attribute HU_SET     : string ;
   attribute BOX_TYPE   : string ;
   signal XLXN_33 : std_logic;
   signal XLXN_41 : std_logic_vector (7 downto 0);
   component M4_1E_HXILINX_Ctrl_Leds
      port ( D0 : in    std_logic; 
             D1 : in    std_logic; 
             D2 : in    std_logic; 
             D3 : in    std_logic; 
             E  : in    std_logic; 
             S0 : in    std_logic; 
             S1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   
   component VCC
      port ( P : out   std_logic);
   end component;
   attribute BOX_TYPE of VCC : component is "BLACK_BOX";
   
   component OR2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR2 : component is "BLACK_BOX";
   
   attribute HU_SET of XLXI_251_0 : label is "XLXI_251_0_24";
   attribute HU_SET of XLXI_251_1 : label is "XLXI_251_1_23";
   attribute HU_SET of XLXI_251_2 : label is "XLXI_251_2_22";
   attribute HU_SET of XLXI_251_3 : label is "XLXI_251_3_21";
   attribute HU_SET of XLXI_251_4 : label is "XLXI_251_4_20";
   attribute HU_SET of XLXI_251_5 : label is "XLXI_251_5_19";
   attribute HU_SET of XLXI_251_6 : label is "XLXI_251_6_18";
   attribute HU_SET of XLXI_251_7 : label is "XLXI_251_7_17";
begin
   XLXI_251_0 : M4_1E_HXILINX_Ctrl_Leds
      port map (D0=>VLed(0),
                D1=>VRes(0),
                D2=>V_R(0),
                D3=>V_RR,
                E=>XLXN_33,
                S0=>V_RR,
                S1=>VerVot,
                O=>XLXN_41(0));
   
   XLXI_251_1 : M4_1E_HXILINX_Ctrl_Leds
      port map (D0=>VLed(1),
                D1=>VRes(1),
                D2=>V_R(1),
                D3=>V_RR,
                E=>XLXN_33,
                S0=>V_RR,
                S1=>VerVot,
                O=>XLXN_41(1));
   
   XLXI_251_2 : M4_1E_HXILINX_Ctrl_Leds
      port map (D0=>VLed(2),
                D1=>VRes(2),
                D2=>V_R(2),
                D3=>V_RR,
                E=>XLXN_33,
                S0=>V_RR,
                S1=>VerVot,
                O=>XLXN_41(2));
   
   XLXI_251_3 : M4_1E_HXILINX_Ctrl_Leds
      port map (D0=>VLed(3),
                D1=>VRes(3),
                D2=>V_R(3),
                D3=>V_RR,
                E=>XLXN_33,
                S0=>V_RR,
                S1=>VerVot,
                O=>XLXN_41(3));
   
   XLXI_251_4 : M4_1E_HXILINX_Ctrl_Leds
      port map (D0=>VLed(4),
                D1=>VRes(4),
                D2=>V_R(4),
                D3=>V_RR,
                E=>XLXN_33,
                S0=>V_RR,
                S1=>VerVot,
                O=>XLXN_41(4));
   
   XLXI_251_5 : M4_1E_HXILINX_Ctrl_Leds
      port map (D0=>VLed(5),
                D1=>VRes(5),
                D2=>V_R(5),
                D3=>V_RR,
                E=>XLXN_33,
                S0=>V_RR,
                S1=>VerVot,
                O=>XLXN_41(5));
   
   XLXI_251_6 : M4_1E_HXILINX_Ctrl_Leds
      port map (D0=>VLed(6),
                D1=>VRes(6),
                D2=>V_R(6),
                D3=>V_RR,
                E=>XLXN_33,
                S0=>V_RR,
                S1=>VerVot,
                O=>XLXN_41(6));
   
   XLXI_251_7 : M4_1E_HXILINX_Ctrl_Leds
      port map (D0=>VLed(7),
                D1=>VRes(7),
                D2=>V_R(7),
                D3=>V_RR,
                E=>XLXN_33,
                S0=>V_RR,
                S1=>VerVot,
                O=>XLXN_41(7));
   
   XLXI_252 : VCC
      port map (P=>XLXN_33);
   
   XLXI_253_0 : OR2
      port map (I0=>On_Off,
                I1=>XLXN_41(0),
                O=>Leds(0));
   
   XLXI_253_1 : OR2
      port map (I0=>On_Off,
                I1=>XLXN_41(1),
                O=>Leds(1));
   
   XLXI_253_2 : OR2
      port map (I0=>On_Off,
                I1=>XLXN_41(2),
                O=>Leds(2));
   
   XLXI_253_3 : OR2
      port map (I0=>On_Off,
                I1=>XLXN_41(3),
                O=>Leds(3));
   
   XLXI_253_4 : OR2
      port map (I0=>On_Off,
                I1=>XLXN_41(4),
                O=>Leds(4));
   
   XLXI_253_5 : OR2
      port map (I0=>On_Off,
                I1=>XLXN_41(5),
                O=>Leds(5));
   
   XLXI_253_6 : OR2
      port map (I0=>On_Off,
                I1=>XLXN_41(6),
                O=>Leds(6));
   
   XLXI_253_7 : OR2
      port map (I0=>On_Off,
                I1=>XLXN_41(7),
                O=>Leds(7));
   
end BEHAVIORAL;


