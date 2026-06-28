--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Visualiza_Cuenta.vhf
-- /___/   /\     Timestamp : 04/23/2026 09:38:07
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_16-04-26/Visualiza_Cuenta.vhf -w C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_16-04-26/Visualiza_Cuenta.sch
--Design Name: Visualiza_Cuenta
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

entity Const_C_MUSER_Visualiza_Cuenta is
   port ( C : out   std_logic_vector (3 downto 0));
end Const_C_MUSER_Visualiza_Cuenta;

architecture BEHAVIORAL of Const_C_MUSER_Visualiza_Cuenta is
   attribute BOX_TYPE   : string ;
   component VCC
      port ( P : out   std_logic);
   end component;
   attribute BOX_TYPE of VCC : component is "BLACK_BOX";
   
   component GND
      port ( G : out   std_logic);
   end component;
   attribute BOX_TYPE of GND : component is "BLACK_BOX";
   
begin
   XLXI_7 : VCC
      port map (P=>C(2));
   
   XLXI_10 : VCC
      port map (P=>C(3));
   
   XLXI_11 : GND
      port map (G=>C(1));
   
   XLXI_12 : GND
      port map (G=>C(0));
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Const_F_MUSER_Visualiza_Cuenta is
   port ( F : out   std_logic_vector (3 downto 0));
end Const_F_MUSER_Visualiza_Cuenta;

architecture BEHAVIORAL of Const_F_MUSER_Visualiza_Cuenta is
   attribute BOX_TYPE   : string ;
   component VCC
      port ( P : out   std_logic);
   end component;
   attribute BOX_TYPE of VCC : component is "BLACK_BOX";
   
begin
   XLXI_7 : VCC
      port map (P=>F(2));
   
   XLXI_8 : VCC
      port map (P=>F(1));
   
   XLXI_9 : VCC
      port map (P=>F(0));
   
   XLXI_10 : VCC
      port map (P=>F(3));
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Hexadecimal_7Segmentos_MUSER_Visualiza_Cuenta is
   port ( enable    : in    std_logic; 
          x         : in    std_logic_vector (3 downto 0); 
          segmentos : out   std_logic_vector (7 downto 0));
end Hexadecimal_7Segmentos_MUSER_Visualiza_Cuenta;

architecture BEHAVIORAL of Hexadecimal_7Segmentos_MUSER_Visualiza_Cuenta is
   attribute BOX_TYPE   : string ;
   signal XLXN_331  : std_logic;
   signal XLXN_332  : std_logic;
   signal XLXN_333  : std_logic;
   signal XLXN_520  : std_logic;
   signal XLXN_523  : std_logic;
   signal XLXN_524  : std_logic;
   signal XLXN_525  : std_logic;
   signal XLXN_537  : std_logic;
   signal XLXN_538  : std_logic;
   signal XLXN_539  : std_logic;
   signal XLXN_540  : std_logic;
   signal XLXN_555  : std_logic;
   signal XLXN_556  : std_logic;
   signal XLXN_557  : std_logic;
   signal XLXN_569  : std_logic;
   signal XLXN_570  : std_logic;
   signal XLXN_571  : std_logic;
   signal XLXN_572  : std_logic;
   signal XLXN_583  : std_logic;
   signal XLXN_584  : std_logic;
   signal XLXN_585  : std_logic;
   signal XLXN_586  : std_logic;
   signal XLXN_630  : std_logic;
   signal XLXN_631  : std_logic;
   signal XLXN_632  : std_logic;
   signal XLXN_725  : std_logic;
   signal XLXN_736  : std_logic;
   signal XLXN_739  : std_logic;
   signal XLXN_746  : std_logic;
   signal XLXN_764  : std_logic;
   signal XLXN_775  : std_logic;
   signal XLXN_1399 : std_logic;
   component AND4B3
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND4B3 : component is "BLACK_BOX";
   
   component OR4
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR4 : component is "BLACK_BOX";
   
   component OR3
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR3 : component is "BLACK_BOX";
   
   component AND3B2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND3B2 : component is "BLACK_BOX";
   
   component AND2B1
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND2B1 : component is "BLACK_BOX";
   
   component AND3B1
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND3B1 : component is "BLACK_BOX";
   
   component AND4B1
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND4B1 : component is "BLACK_BOX";
   
   component AND3B3
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND3B3 : component is "BLACK_BOX";
   
   component AND4B2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND4B2 : component is "BLACK_BOX";
   
   component AND3
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND3 : component is "BLACK_BOX";
   
   component GND
      port ( G : out   std_logic);
   end component;
   attribute BOX_TYPE of GND : component is "BLACK_BOX";
   
begin
   XLXI_1 : AND4B3
      port map (I0=>x(1),
                I1=>x(2),
                I2=>x(3),
                I3=>x(0),
                O=>XLXN_524);
   
   XLXI_2 : AND4B3
      port map (I0=>x(0),
                I1=>x(1),
                I2=>x(3),
                I3=>x(2),
                O=>XLXN_523);
   
   XLXI_11 : OR4
      port map (I0=>XLXN_539,
                I1=>XLXN_538,
                I2=>XLXN_537,
                I3=>XLXN_540,
                O=>XLXN_736);
   
   XLXI_32 : OR3
      port map (I0=>XLXN_557,
                I1=>XLXN_555,
                I2=>XLXN_556,
                O=>XLXN_739);
   
   XLXI_34 : OR3
      port map (I0=>XLXN_333,
                I1=>XLXN_331,
                I2=>XLXN_332,
                O=>XLXN_764);
   
   XLXI_35 : AND4B3
      port map (I0=>x(1),
                I1=>x(2),
                I2=>x(3),
                I3=>x(0),
                O=>XLXN_572);
   
   XLXI_36 : AND4B3
      port map (I0=>x(0),
                I1=>x(1),
                I2=>x(3),
                I3=>x(2),
                O=>XLXN_569);
   
   XLXI_37 : AND3B2
      port map (I0=>x(1),
                I1=>x(3),
                I2=>x(2),
                O=>XLXN_331);
   
   XLXI_38 : AND3B2
      port map (I0=>x(1),
                I1=>x(2),
                I2=>x(0),
                O=>XLXN_333);
   
   XLXI_39 : AND2B1
      port map (I0=>x(3),
                I1=>x(0),
                O=>XLXN_332);
   
   XLXI_40 : AND3B2
      port map (I0=>x(2),
                I1=>x(3),
                I2=>x(0),
                O=>XLXN_585);
   
   XLXI_41 : AND3B2
      port map (I0=>x(2),
                I1=>x(3),
                I2=>x(1),
                O=>XLXN_583);
   
   XLXI_44 : AND3B1
      port map (I0=>x(3),
                I1=>x(0),
                I2=>x(1),
                O=>XLXN_584);
   
   XLXI_46 : AND4B1
      port map (I0=>x(3),
                I1=>x(0),
                I2=>x(1),
                I3=>x(2),
                O=>XLXN_630);
   
   XLXI_47 : AND3B3
      port map (I0=>x(1),
                I1=>x(2),
                I2=>x(3),
                O=>XLXN_631);
   
   XLXI_68 : AND4B1
      port map (I0=>x(1),
                I1=>x(0),
                I2=>x(2),
                I3=>x(3),
                O=>XLXN_520);
   
   XLXI_103 : AND4B1
      port map (I0=>x(2),
                I1=>x(0),
                I2=>x(1),
                I3=>x(3),
                O=>XLXN_525);
   
   XLXI_104 : OR4
      port map (I0=>XLXN_525,
                I1=>XLXN_520,
                I2=>XLXN_523,
                I3=>XLXN_524,
                O=>XLXN_725);
   
   XLXI_107 : AND4B2
      port map (I0=>x(1),
                I1=>x(3),
                I2=>x(0),
                I3=>x(2),
                O=>XLXN_540);
   
   XLXI_108 : AND3B1
      port map (I0=>x(0),
                I1=>x(1),
                I2=>x(2),
                O=>XLXN_537);
   
   XLXI_109 : AND3B1
      port map (I0=>x(0),
                I1=>x(2),
                I2=>x(3),
                O=>XLXN_538);
   
   XLXI_110 : AND3
      port map (I0=>x(0),
                I1=>x(1),
                I2=>x(3),
                O=>XLXN_539);
   
   XLXI_113 : AND3B1
      port map (I0=>x(0),
                I1=>x(2),
                I2=>x(3),
                O=>XLXN_555);
   
   XLXI_114 : AND3
      port map (I0=>x(1),
                I1=>x(2),
                I2=>x(3),
                O=>XLXN_557);
   
   XLXI_115 : AND4B3
      port map (I0=>x(0),
                I1=>x(2),
                I2=>x(3),
                I3=>x(1),
                O=>XLXN_556);
   
   XLXI_132 : AND4B2
      port map (I0=>x(0),
                I1=>x(2),
                I2=>x(1),
                I3=>x(3),
                O=>XLXN_570);
   
   XLXI_133 : AND3
      port map (I0=>x(0),
                I1=>x(1),
                I2=>x(2),
                O=>XLXN_571);
   
   XLXI_134 : OR4
      port map (I0=>XLXN_571,
                I1=>XLXN_570,
                I2=>XLXN_569,
                I3=>XLXN_572,
                O=>XLXN_746);
   
   XLXI_136 : AND4B1
      port map (I0=>x(1),
                I1=>x(0),
                I2=>x(2),
                I3=>x(3),
                O=>XLXN_586);
   
   XLXI_137 : OR4
      port map (I0=>XLXN_586,
                I1=>XLXN_584,
                I2=>XLXN_583,
                I3=>XLXN_585,
                O=>XLXN_775);
   
   XLXI_141 : AND4B2
      port map (I0=>x(0),
                I1=>x(1),
                I2=>x(2),
                I3=>x(3),
                O=>XLXN_632);
   
   XLXI_155 : OR3
      port map (I0=>XLXN_632,
                I1=>XLXN_630,
                I2=>XLXN_631,
                O=>XLXN_1399);
   
   XLXI_245 : AND2B1
      port map (I0=>XLXN_725,
                I1=>enable,
                O=>segmentos(0));
   
   XLXI_481 : AND2B1
      port map (I0=>XLXN_736,
                I1=>enable,
                O=>segmentos(1));
   
   XLXI_482 : AND2B1
      port map (I0=>XLXN_739,
                I1=>enable,
                O=>segmentos(2));
   
   XLXI_483 : AND2B1
      port map (I0=>XLXN_775,
                I1=>enable,
                O=>segmentos(5));
   
   XLXI_484 : AND2B1
      port map (I0=>XLXN_764,
                I1=>enable,
                O=>segmentos(4));
   
   XLXI_485 : AND2B1
      port map (I0=>XLXN_746,
                I1=>enable,
                O=>segmentos(3));
   
   XLXI_486 : AND2B1
      port map (I0=>XLXN_1399,
                I1=>enable,
                O=>segmentos(6));
   
   XLXI_487 : GND
      port map (G=>segmentos(7));
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Visualiza_Cuenta is
   port ( SVC       : in    std_logic_vector (3 downto 0); 
          SVF       : in    std_logic_vector (3 downto 0); 
          VVal      : in    std_logic; 
          SevenSeg0 : out   std_logic_vector (7 downto 0); 
          SevenSeg1 : out   std_logic_vector (7 downto 0); 
          SevenSeg2 : out   std_logic_vector (7 downto 0); 
          SevenSeg3 : out   std_logic_vector (7 downto 0));
end Visualiza_Cuenta;

architecture BEHAVIORAL of Visualiza_Cuenta is
   signal XLXN_13   : std_logic_vector (3 downto 0);
   signal XLXN_14   : std_logic_vector (3 downto 0);
   component Hexadecimal_7Segmentos_MUSER_Visualiza_Cuenta
      port ( enable    : in    std_logic; 
             segmentos : out   std_logic_vector (7 downto 0); 
             x         : in    std_logic_vector (3 downto 0));
   end component;
   
   component Const_F_MUSER_Visualiza_Cuenta
      port ( F : out   std_logic_vector (3 downto 0));
   end component;
   
   component Const_C_MUSER_Visualiza_Cuenta
      port ( C : out   std_logic_vector (3 downto 0));
   end component;
   
begin
   XLXI_1 : Hexadecimal_7Segmentos_MUSER_Visualiza_Cuenta
      port map (enable=>VVal,
                x(3 downto 0)=>XLXN_14(3 downto 0),
                segmentos(7 downto 0)=>SevenSeg3(7 downto 0));
   
   XLXI_2 : Hexadecimal_7Segmentos_MUSER_Visualiza_Cuenta
      port map (enable=>VVal,
                x(3 downto 0)=>SVC(3 downto 0),
                segmentos(7 downto 0)=>SevenSeg2(7 downto 0));
   
   XLXI_3 : Hexadecimal_7Segmentos_MUSER_Visualiza_Cuenta
      port map (enable=>VVal,
                x(3 downto 0)=>XLXN_13(3 downto 0),
                segmentos(7 downto 0)=>SevenSeg1(7 downto 0));
   
   XLXI_4 : Hexadecimal_7Segmentos_MUSER_Visualiza_Cuenta
      port map (enable=>VVal,
                x(3 downto 0)=>SVF(3 downto 0),
                segmentos(7 downto 0)=>SevenSeg0(7 downto 0));
   
   XLXI_5 : Const_F_MUSER_Visualiza_Cuenta
      port map (F(3 downto 0)=>XLXN_13(3 downto 0));
   
   XLXI_6 : Const_C_MUSER_Visualiza_Cuenta
      port map (C(3 downto 0)=>XLXN_14(3 downto 0));
   
end BEHAVIORAL;


