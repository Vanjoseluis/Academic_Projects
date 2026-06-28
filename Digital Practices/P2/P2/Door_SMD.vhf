--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : Door_SMD.vhf
-- /___/   /\     Timestamp : 05/26/2026 15:41:31
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/Usuario_UMA/Downloads/Practica2_JoseLuis_Raul_25_05_26/Door_SMD.vhf -w C:/Users/Usuario_UMA/Downloads/Practica2_JoseLuis_Raul_25_05_26/Door_SMD.sch
--Design Name: Door_SMD
--Device: spartan6
--Purpose:
--    This vhdl netlist is translated from an ECS schematic. It can be 
--    synthesized and simulated, but it should not be modified. 
--
----- CELL SR4CLED_HXILINX_Door_SMD -----


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity SR4CLED_HXILINX_Door_SMD is
port (
    Q0   : out STD_LOGIC;
    Q1   : out STD_LOGIC;
    Q2   : out STD_LOGIC;
    Q3   : out STD_LOGIC;
    C    : in STD_LOGIC;
    CE   : in STD_LOGIC;
    CLR  : in STD_LOGIC;
    D0   : in STD_LOGIC;
    D1   : in STD_LOGIC;
    D2   : in STD_LOGIC;
    D3   : in STD_LOGIC;
    L    : in STD_LOGIC;
    LEFT : in STD_LOGIC;
    SLI  : in STD_LOGIC;
    SRI  : in STD_LOGIC
    );
end SR4CLED_HXILINX_Door_SMD;

architecture Behavioral of SR4CLED_HXILINX_Door_SMD is
signal q_tmp : std_logic_vector(3 downto 0);
begin

process(C, CLR)
begin
  if (CLR='1') then
    q_tmp <= "0000";
  elsif (C'event and C = '1') then
    if (L= '1') then
      q_tmp <= D3&D2&D1&D0;
    elsif (CE='1') then 
      if (LEFT= '1') then
        q_tmp <= ( q_tmp(2 downto 0) & SLI );
      else
        q_tmp <= ( SRI & q_tmp(3 downto 1) );
      end if;
    end if;
  end if;
end process;

Q3 <= q_tmp(3);
Q2 <= q_tmp(2);
Q1 <= q_tmp(1);
Q0 <= q_tmp(0);


end Behavioral;


library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Ver_Op_Clos_MUSER_Door_SMD is
   port ( Close      : in    std_logic; 
          Op         : in    std_logic; 
          Seven_Seg0 : out   std_logic_vector (7 downto 0); 
          Seven_Seg1 : out   std_logic_vector (7 downto 0); 
          Seven_Seg2 : out   std_logic_vector (7 downto 0); 
          Seven_Seg3 : out   std_logic_vector (7 downto 0));
end Ver_Op_Clos_MUSER_Door_SMD;

architecture BEHAVIORAL of Ver_Op_Clos_MUSER_Door_SMD is
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



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Door_SMD is
   port ( Ck         : in    std_logic; 
          Clr        : in    std_logic; 
          D          : in    std_logic; 
          Out0       : out   std_logic; 
          Out1       : out   std_logic; 
          Out2       : out   std_logic; 
          Out3       : out   std_logic; 
          Seven_Seg0 : out   std_logic_vector (7 downto 0); 
          Seven_Seg1 : out   std_logic_vector (7 downto 0); 
          Seven_Seg2 : out   std_logic_vector (7 downto 0); 
          Seven_Seg3 : out   std_logic_vector (7 downto 0));
end Door_SMD;

architecture BEHAVIORAL of Door_SMD is
   attribute HU_SET     : string ;
   attribute BOX_TYPE   : string ;
   signal Cero       : std_logic;
   signal Clos       : std_logic;
   signal Op         : std_logic;
   signal XLXN_1     : std_logic;
   signal XLXN_64    : std_logic;
   signal XLXN_65    : std_logic;
   signal Out0_DUMMY : std_logic;
   signal Out1_DUMMY : std_logic;
   signal Out2_DUMMY : std_logic;
   signal Out3_DUMMY : std_logic;
   component SR4CLED_HXILINX_Door_SMD
      port ( C    : in    std_logic; 
             CE   : in    std_logic; 
             CLR  : in    std_logic; 
             D0   : in    std_logic; 
             D1   : in    std_logic; 
             D2   : in    std_logic; 
             D3   : in    std_logic; 
             L    : in    std_logic; 
             LEFT : in    std_logic; 
             SLI  : in    std_logic; 
             SRI  : in    std_logic; 
             Q0   : out   std_logic; 
             Q1   : out   std_logic; 
             Q2   : out   std_logic; 
             Q3   : out   std_logic);
   end component;
   
   component INV
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of INV : component is "BLACK_BOX";
   
   component GND
      port ( G : out   std_logic);
   end component;
   attribute BOX_TYPE of GND : component is "BLACK_BOX";
   
   component Ver_Op_Clos_MUSER_Door_SMD
      port ( Close      : in    std_logic; 
             Op         : in    std_logic; 
             Seven_Seg0 : out   std_logic_vector (7 downto 0); 
             Seven_Seg1 : out   std_logic_vector (7 downto 0); 
             Seven_Seg2 : out   std_logic_vector (7 downto 0); 
             Seven_Seg3 : out   std_logic_vector (7 downto 0));
   end component;
   
   component NOR4
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             I3 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of NOR4 : component is "BLACK_BOX";
   
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
   
   attribute HU_SET of XLXI_1 : label is "XLXI_1_24";
begin
   Out0 <= Out0_DUMMY;
   Out1 <= Out1_DUMMY;
   Out2 <= Out2_DUMMY;
   Out3 <= Out3_DUMMY;
   XLXI_1 : SR4CLED_HXILINX_Door_SMD
      port map (C=>Ck,
                CE=>Clr,
                CLR=>XLXN_1,
                D0=>Cero,
                D1=>Cero,
                D2=>Cero,
                D3=>Cero,
                L=>Cero,
                LEFT=>D,
                SLI=>Clr,
                SRI=>Cero,
                Q0=>Out0_DUMMY,
                Q1=>Out1_DUMMY,
                Q2=>Out2_DUMMY,
                Q3=>Out3_DUMMY);
   
   XLXI_2 : INV
      port map (I=>Clr,
                O=>XLXN_1);
   
   XLXI_5 : GND
      port map (G=>Cero);
   
   XLXI_10 : Ver_Op_Clos_MUSER_Door_SMD
      port map (Close=>Clos,
                Op=>Op,
                Seven_Seg0(7 downto 0)=>Seven_Seg0(7 downto 0),
                Seven_Seg1(7 downto 0)=>Seven_Seg1(7 downto 0),
                Seven_Seg2(7 downto 0)=>Seven_Seg2(7 downto 0),
                Seven_Seg3(7 downto 0)=>Seven_Seg3(7 downto 0));
   
   XLXI_11 : NOR4
      port map (I0=>Out3_DUMMY,
                I1=>Out2_DUMMY,
                I2=>Out1_DUMMY,
                I3=>Out0_DUMMY,
                O=>XLXN_64);
   
   XLXI_12 : AND4
      port map (I0=>Out3_DUMMY,
                I1=>Out2_DUMMY,
                I2=>Out1_DUMMY,
                I3=>Out0_DUMMY,
                O=>XLXN_65);
   
   XLXI_13 : AND2
      port map (I0=>XLXN_64,
                I1=>Clr,
                O=>Op);
   
   XLXI_14 : AND2
      port map (I0=>Clr,
                I1=>XLXN_65,
                O=>Clos);
   
end BEHAVIORAL;


