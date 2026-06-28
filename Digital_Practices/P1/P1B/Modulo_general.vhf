--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Modulo_general.vhf
-- /___/   /\     Timestamp : 05/14/2026 10:13:13
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Modulo_general.vhf -w C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_07-05-26_1B/Modulo_general.sch
--Design Name: Modulo_general
--Device: spartan6
--Purpose:
--    This vhdl netlist is translated from an ECS schematic. It can be 
--    synthesized and simulated, but it should not be modified. 
--
----- CELL INV4_HXILINX_Modulo_general -----
  
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity INV4_HXILINX_Modulo_general is
  
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
end INV4_HXILINX_Modulo_general;

architecture INV4_HXILINX_Modulo_general_V of INV4_HXILINX_Modulo_general is
begin
  O0 <= not I0 ;
  O1 <= not I1 ;
  O2 <= not I2 ;
  O3 <= not I3 ;
end INV4_HXILINX_Modulo_general_V;

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Resultado_de_votacion_MUSER_Modulo_general is
   port ( SVC  : in    std_logic_vector (3 downto 0); 
          VVal : in    std_logic; 
          NA   : out   std_logic_vector (1 downto 0); 
          VC   : out   std_logic_vector (2 downto 0); 
          VF   : out   std_logic_vector (2 downto 0));
end Resultado_de_votacion_MUSER_Modulo_general;

architecture BEHAVIORAL of Resultado_de_votacion_MUSER_Modulo_general is
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



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Const_C_MUSER_Modulo_general is
   port ( C : out   std_logic_vector (3 downto 0));
end Const_C_MUSER_Modulo_general;

architecture BEHAVIORAL of Const_C_MUSER_Modulo_general is
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

entity Const_F_MUSER_Modulo_general is
   port ( F : out   std_logic_vector (3 downto 0));
end Const_F_MUSER_Modulo_general;

architecture BEHAVIORAL of Const_F_MUSER_Modulo_general is
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

entity Hexadecimal_7Segmentos_MUSER_Modulo_general is
   port ( enable    : in    std_logic; 
          x         : in    std_logic_vector (3 downto 0); 
          segmentos : out   std_logic_vector (7 downto 0));
end Hexadecimal_7Segmentos_MUSER_Modulo_general;

architecture BEHAVIORAL of Hexadecimal_7Segmentos_MUSER_Modulo_general is
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

entity Visualiza_Cuenta_MUSER_Modulo_general is
   port ( SVC       : in    std_logic_vector (3 downto 0); 
          SVF       : in    std_logic_vector (3 downto 0); 
          VVal      : in    std_logic; 
          SevenSeg0 : out   std_logic_vector (7 downto 0); 
          SevenSeg1 : out   std_logic_vector (7 downto 0); 
          SevenSeg2 : out   std_logic_vector (7 downto 0); 
          SevenSeg3 : out   std_logic_vector (7 downto 0));
end Visualiza_Cuenta_MUSER_Modulo_general;

architecture BEHAVIORAL of Visualiza_Cuenta_MUSER_Modulo_general is
   signal XLXN_13   : std_logic_vector (3 downto 0);
   signal XLXN_14   : std_logic_vector (3 downto 0);
   component Hexadecimal_7Segmentos_MUSER_Modulo_general
      port ( enable    : in    std_logic; 
             segmentos : out   std_logic_vector (7 downto 0); 
             x         : in    std_logic_vector (3 downto 0));
   end component;
   
   component Const_F_MUSER_Modulo_general
      port ( F : out   std_logic_vector (3 downto 0));
   end component;
   
   component Const_C_MUSER_Modulo_general
      port ( C : out   std_logic_vector (3 downto 0));
   end component;
   
begin
   XLXI_1 : Hexadecimal_7Segmentos_MUSER_Modulo_general
      port map (enable=>VVal,
                x(3 downto 0)=>XLXN_14(3 downto 0),
                segmentos(7 downto 0)=>SevenSeg3(7 downto 0));
   
   XLXI_2 : Hexadecimal_7Segmentos_MUSER_Modulo_general
      port map (enable=>VVal,
                x(3 downto 0)=>SVC(3 downto 0),
                segmentos(7 downto 0)=>SevenSeg2(7 downto 0));
   
   XLXI_3 : Hexadecimal_7Segmentos_MUSER_Modulo_general
      port map (enable=>VVal,
                x(3 downto 0)=>XLXN_13(3 downto 0),
                segmentos(7 downto 0)=>SevenSeg1(7 downto 0));
   
   XLXI_4 : Hexadecimal_7Segmentos_MUSER_Modulo_general
      port map (enable=>VVal,
                x(3 downto 0)=>SVF(3 downto 0),
                segmentos(7 downto 0)=>SevenSeg0(7 downto 0));
   
   XLXI_5 : Const_F_MUSER_Modulo_general
      port map (F(3 downto 0)=>XLXN_13(3 downto 0));
   
   XLXI_6 : Const_C_MUSER_Modulo_general
      port map (C(3 downto 0)=>XLXN_14(3 downto 0));
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Const_8_MUSER_Modulo_general is
   port ( Const_8 : out   std_logic_vector (3 downto 0));
end Const_8_MUSER_Modulo_general;

architecture BEHAVIORAL of Const_8_MUSER_Modulo_general is
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
   XLXI_10 : VCC
      port map (P=>Const_8(3));
   
   XLXI_11 : GND
      port map (G=>Const_8(2));
   
   XLXI_12 : GND
      port map (G=>Const_8(1));
   
   XLXI_13 : GND
      port map (G=>Const_8(0));
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Sumador_MUSER_Modulo_general is
   port ( A  : in    std_logic; 
          B  : in    std_logic; 
          C0 : in    std_logic; 
          C1 : out   std_logic; 
          S  : out   std_logic);
end Sumador_MUSER_Modulo_general;

architecture BEHAVIORAL of Sumador_MUSER_Modulo_general is
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

entity Sumador_Completo_MUSER_Modulo_general is
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
end Sumador_Completo_MUSER_Modulo_general;

architecture BEHAVIORAL of Sumador_Completo_MUSER_Modulo_general is
   signal C1       : std_logic;
   signal C2       : std_logic;
   signal C3_DUMMY : std_logic;
   component Sumador_MUSER_Modulo_general
      port ( A  : in    std_logic; 
             B  : in    std_logic; 
             C0 : in    std_logic; 
             S  : out   std_logic; 
             C1 : out   std_logic);
   end component;
   
begin
   C3 <= C3_DUMMY;
   XLXI_1 : Sumador_MUSER_Modulo_general
      port map (A=>A0,
                B=>B0,
                C0=>C0,
                C1=>C1,
                S=>S0);
   
   XLXI_2 : Sumador_MUSER_Modulo_general
      port map (A=>A1,
                B=>B1,
                C0=>C1,
                C1=>C2,
                S=>S1);
   
   XLXI_3 : Sumador_MUSER_Modulo_general
      port map (A=>A2,
                B=>B2,
                C0=>C2,
                C1=>C3_DUMMY,
                S=>S2);
   
   XLXI_4 : Sumador_MUSER_Modulo_general
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

entity Complemento_a2_MUSER_Modulo_general is
   port ( D0 : in    std_logic; 
          D1 : in    std_logic; 
          D2 : in    std_logic; 
          D3 : in    std_logic; 
          S0 : out   std_logic; 
          S1 : out   std_logic; 
          S2 : out   std_logic; 
          S3 : out   std_logic; 
          S4 : out   std_logic);
end Complemento_a2_MUSER_Modulo_general;

architecture BEHAVIORAL of Complemento_a2_MUSER_Modulo_general is
   attribute HU_SET     : string ;
   attribute BOX_TYPE   : string ;
   signal A0      : std_logic;
   signal A1      : std_logic;
   signal A2      : std_logic;
   signal A3      : std_logic;
   signal XLXN_21 : std_logic;
   signal XLXN_22 : std_logic;
   component INV4_HXILINX_Modulo_general
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
   
   component Sumador_Completo_MUSER_Modulo_general
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
   
   attribute HU_SET of XLXI_6 : label is "XLXI_6_74";
begin
   XLXI_6 : INV4_HXILINX_Modulo_general
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
   
   XLXI_19 : Sumador_Completo_MUSER_Modulo_general
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

entity Restador_MUSER_Modulo_general is
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
end Restador_MUSER_Modulo_general;

architecture BEHAVIORAL of Restador_MUSER_Modulo_general is
   signal XLXN_114 : std_logic;
   signal XLXN_115 : std_logic;
   signal XLXN_116 : std_logic;
   signal XLXN_117 : std_logic;
   component Complemento_a2_MUSER_Modulo_general
      port ( D0 : in    std_logic; 
             D1 : in    std_logic; 
             D2 : in    std_logic; 
             D3 : in    std_logic; 
             S0 : out   std_logic; 
             S1 : out   std_logic; 
             S2 : out   std_logic; 
             S3 : out   std_logic);
   end component;
   
   component Sumador_Completo_MUSER_Modulo_general
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
   XLXI_17 : Complemento_a2_MUSER_Modulo_general
      port map (D0=>B0,
                D1=>B1,
                D2=>B2,
                D3=>B3,
                S0=>XLXN_114,
                S1=>XLXN_115,
                S2=>XLXN_116,
                S3=>XLXN_117);
   
   XLXI_18 : Sumador_Completo_MUSER_Modulo_general
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



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Cuenta_Votos_COMPLETO_MUSER_Modulo_general is
   port ( M   : in    std_logic_vector (7 downto 0); 
          SVC : out   std_logic_vector (3 downto 0); 
          SVF : out   std_logic_vector (3 downto 0));
end Cuenta_Votos_COMPLETO_MUSER_Modulo_general;

architecture BEHAVIORAL of Cuenta_Votos_COMPLETO_MUSER_Modulo_general is
   attribute BOX_TYPE   : string ;
   signal Const_8   : std_logic_vector (3 downto 0);
   signal SA        : std_logic_vector (2 downto 0);
   signal SB        : std_logic_vector (2 downto 0);
   signal XLXN_27   : std_logic;
   signal XLXN_80   : std_logic;
   signal XLXN_81   : std_logic;
   signal XLXN_88   : std_logic;
   signal SVF_DUMMY : std_logic_vector (3 downto 0);
   component GND
      port ( G : out   std_logic);
   end component;
   attribute BOX_TYPE of GND : component is "BLACK_BOX";
   
   component Sumador_Completo_MUSER_Modulo_general
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
   
   component Restador_MUSER_Modulo_general
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
   end component;
   
   component Const_8_MUSER_Modulo_general
      port ( Const_8 : out   std_logic_vector (3 downto 0));
   end component;
   
   component Cuenta_4votosVHDL
      port ( M : in    std_logic_vector (3 downto 0); 
             S : out   std_logic_vector (2 downto 0));
   end component;
   
begin
   SVF(3 downto 0) <= SVF_DUMMY(3 downto 0);
   XLXI_12 : GND
      port map (G=>XLXN_27);
   
   XLXI_39 : GND
      port map (G=>XLXN_80);
   
   XLXI_40 : GND
      port map (G=>XLXN_81);
   
   XLXI_41 : GND
      port map (G=>XLXN_88);
   
   XLXI_43 : Sumador_Completo_MUSER_Modulo_general
      port map (A0=>SA(0),
                A1=>SA(1),
                A2=>SA(2),
                A3=>XLXN_88,
                B0=>SB(0),
                B1=>SB(1),
                B2=>SB(2),
                B3=>XLXN_80,
                C0=>XLXN_81,
                C4=>open,
                S0=>SVF_DUMMY(0),
                S1=>SVF_DUMMY(1),
                S2=>SVF_DUMMY(2),
                S3=>SVF_DUMMY(3));
   
   XLXI_44 : Restador_MUSER_Modulo_general
      port map (A0=>Const_8(0),
                A1=>Const_8(1),
                A2=>Const_8(2),
                A3=>Const_8(3),
                B0=>SVF_DUMMY(0),
                B1=>SVF_DUMMY(1),
                B2=>SVF_DUMMY(2),
                B3=>SVF_DUMMY(3),
                C0=>XLXN_27,
                S0=>SVC(0),
                S1=>SVC(1),
                S2=>SVC(2),
                S3=>SVC(3),
                S4=>open);
   
   XLXI_45 : Const_8_MUSER_Modulo_general
      port map (Const_8(3 downto 0)=>Const_8(3 downto 0));
   
   XLXI_46 : Cuenta_4votosVHDL
      port map (M(3 downto 0)=>M(7 downto 4),
                S(2 downto 0)=>SB(2 downto 0));
   
   XLXI_47 : Cuenta_4votosVHDL
      port map (M(3 downto 0)=>M(3 downto 0),
                S(2 downto 0)=>SA(2 downto 0));
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Modulo_general is
   port ( M           : in    std_logic_vector (7 downto 0); 
          VVal        : in    std_logic; 
          NA          : out   std_logic_vector (1 downto 0); 
          SevenSeg_C  : out   std_logic_vector (7 downto 0); 
          SevenSeg_F  : out   std_logic_vector (7 downto 0); 
          SevenSeg_VC : out   std_logic_vector (7 downto 0); 
          SevenSeg_VF : out   std_logic_vector (7 downto 0); 
          SVC         : out   std_logic_vector (3 downto 0); 
          SVF         : out   std_logic_vector (3 downto 0); 
          VC          : out   std_logic_vector (2 downto 0); 
          VF          : out   std_logic_vector (2 downto 0));
end Modulo_general;

architecture BEHAVIORAL of Modulo_general is
   signal SVC_DUMMY   : std_logic_vector (3 downto 0);
   signal SVF_DUMMY   : std_logic_vector (3 downto 0);
   component Cuenta_Votos_COMPLETO_MUSER_Modulo_general
      port ( M   : in    std_logic_vector (7 downto 0); 
             SVC : out   std_logic_vector (3 downto 0); 
             SVF : out   std_logic_vector (3 downto 0));
   end component;
   
   component Visualiza_Cuenta_MUSER_Modulo_general
      port ( SevenSeg0 : out   std_logic_vector (7 downto 0); 
             SevenSeg1 : out   std_logic_vector (7 downto 0); 
             SevenSeg2 : out   std_logic_vector (7 downto 0); 
             SevenSeg3 : out   std_logic_vector (7 downto 0); 
             SVC       : in    std_logic_vector (3 downto 0); 
             SVF       : in    std_logic_vector (3 downto 0); 
             VVal      : in    std_logic);
   end component;
   
   component Resultado_de_votacion_MUSER_Modulo_general
      port ( NA   : out   std_logic_vector (1 downto 0); 
             SVC  : in    std_logic_vector (3 downto 0); 
             VC   : out   std_logic_vector (2 downto 0); 
             VF   : out   std_logic_vector (2 downto 0); 
             VVal : in    std_logic);
   end component;
   
begin
   SVC(3 downto 0) <= SVC_DUMMY(3 downto 0);
   SVF(3 downto 0) <= SVF_DUMMY(3 downto 0);
   XLXI_2 : Cuenta_Votos_COMPLETO_MUSER_Modulo_general
      port map (M(7 downto 0)=>M(7 downto 0),
                SVC(3 downto 0)=>SVC_DUMMY(3 downto 0),
                SVF(3 downto 0)=>SVF_DUMMY(3 downto 0));
   
   XLXI_9 : Visualiza_Cuenta_MUSER_Modulo_general
      port map (SVC(3 downto 0)=>SVC_DUMMY(3 downto 0),
                SVF(3 downto 0)=>SVF_DUMMY(3 downto 0),
                VVal=>VVal,
                SevenSeg0(7 downto 0)=>SevenSeg_VF(7 downto 0),
                SevenSeg1(7 downto 0)=>SevenSeg_F(7 downto 0),
                SevenSeg2(7 downto 0)=>SevenSeg_VC(7 downto 0),
                SevenSeg3(7 downto 0)=>SevenSeg_C(7 downto 0));
   
   XLXI_12 : Resultado_de_votacion_MUSER_Modulo_general
      port map (SVC(3 downto 0)=>SVC_DUMMY(3 downto 0),
                VVal=>VVal,
                NA(1 downto 0)=>NA(1 downto 0),
                VC(2 downto 0)=>VC(2 downto 0),
                VF(2 downto 0)=>VF(2 downto 0));
   
end BEHAVIORAL;


