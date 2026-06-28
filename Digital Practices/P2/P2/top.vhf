--------------------------------------------------------------------------------
-- Copyright (c) 1995-2013 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     Version : 14.7
--  \   \         Application : sch2hdl
--  /   /         Filename : top.vhf
-- /___/   /\     Timestamp : 05/28/2026 09:41:00
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: sch2hdl -intstyle ise -family spartan6 -flat -suppress -vhdl C:/Users/edie/Desktop/Practica2_JoseLuis_Raul_26_05_26/top.vhf -w C:/Users/edie/Desktop/Practica2_JoseLuis_Raul_26_05_26/top.sch
--Design Name: top
--Device: spartan6
--Purpose:
--    This vhdl netlist is translated from an ECS schematic. It can be 
--    synthesized and simulated, but it should not be modified. 
--
----- CELL M2_1E_HXILINX_top -----
  
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity M2_1E_HXILINX_top is
  
port(
    O   : out std_logic;

    D0  : in std_logic;
    D1  : in std_logic;
    E   : in std_logic;
    S0  : in std_logic
  );
end M2_1E_HXILINX_top;

architecture M2_1E_HXILINX_top_V of M2_1E_HXILINX_top is
begin
  process (D0, D1, E, S0)
  begin
    if( E = '0') then
    O <= '0';
    else
      case S0 is
      when '0' => O <= D0;
      when '1' => O <= D1;
      when others => NULL;
      end case;
    end if;
    end process; 
end M2_1E_HXILINX_top_V;
----- CELL CC8RE_HXILINX_top -----

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity CC8RE_HXILINX_top is
port (
    CEO : out STD_LOGIC;
    Q   : out STD_LOGIC_VECTOR(7 downto 0);
    TC  : out STD_LOGIC;
    C   : in STD_LOGIC;
    CE  : in STD_LOGIC;
    R   : in STD_LOGIC
    );
end CC8RE_HXILINX_top;

architecture CC8RE_HXILINX_top_V of CC8RE_HXILINX_top is

  signal COUNT : STD_LOGIC_VECTOR(7 downto 0) := (others => '0');
  constant TERMINAL_COUNT : STD_LOGIC_VECTOR(7 downto 0) := (others => '1');

begin

process(C)
begin
  if (C'event and C ='1') then
    if (R='1') then
      COUNT <= (others => '0');
    elsif (CE='1') then 
      COUNT <= COUNT+1;
    end if;
  end if;
end process;

TC <= '0' when (R='1') else
      '1' when (COUNT = TERMINAL_COUNT) else '0'; 
CEO <= '1' when ((COUNT = TERMINAL_COUNT) and CE='1') else '0'; 
Q <= COUNT;

end CC8RE_HXILINX_top_V;
----- CELL SR4CLED_HXILINX_top -----


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity SR4CLED_HXILINX_top is
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
end SR4CLED_HXILINX_top;

architecture Behavioral of SR4CLED_HXILINX_top is
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

----- CELL M16_1E_HXILINX_top -----
  
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity M16_1E_HXILINX_top is
  
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
end M16_1E_HXILINX_top;

architecture M16_1E_HXILINX_top_V of M16_1E_HXILINX_top is
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
end M16_1E_HXILINX_top_V;
----- CELL M2_1_HXILINX_top -----
  
library IEEE;
use IEEE.STD_LOGIC_1164.all;

entity M2_1_HXILINX_top is
  
port(
    O   : out std_logic;

    D0  : in std_logic;
    D1  : in std_logic;
    S0  : in std_logic
  );
end M2_1_HXILINX_top;

architecture M2_1_HXILINX_top_V of M2_1_HXILINX_top is
begin
  process (D0, D1, S0)
  begin
    case S0 is
    when '0' => O <= D0;
    when '1' => O <= D1;
    when others => NULL;
    end case;
    end process; 
end M2_1_HXILINX_top_V;

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity p_fin_MUSER_top is
   port ( A  : in    std_logic; 
          ck : in    std_logic; 
          S  : out   std_logic);
end p_fin_MUSER_top;

architecture BEHAVIORAL of p_fin_MUSER_top is
   attribute BOX_TYPE   : string ;
   signal af : std_logic;
   signal bf : std_logic;
   component FD
      generic( INIT : bit :=  '0');
      port ( C : in    std_logic; 
             D : in    std_logic; 
             Q : out   std_logic);
   end component;
   attribute BOX_TYPE of FD : component is "BLACK_BOX";
   
   component AND2B1
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of AND2B1 : component is "BLACK_BOX";
   
begin
   XLXI_1 : FD
      port map (C=>ck,
                D=>A,
                Q=>af);
   
   XLXI_17 : AND2B1
      port map (I0=>af,
                I1=>bf,
                O=>S);
   
   XLXI_19 : FD
      port map (C=>ck,
                D=>af,
                Q=>bf);
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Ver_Op_Clos_MUSER_top is
   port ( Close      : in    std_logic; 
          Op         : in    std_logic; 
          Seven_Seg0 : out   std_logic_vector (7 downto 0); 
          Seven_Seg1 : out   std_logic_vector (7 downto 0); 
          Seven_Seg2 : out   std_logic_vector (7 downto 0); 
          Seven_Seg3 : out   std_logic_vector (7 downto 0));
end Ver_Op_Clos_MUSER_top;

architecture BEHAVIORAL of Ver_Op_Clos_MUSER_top is
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

entity Door_SMD_MUSER_top is
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
end Door_SMD_MUSER_top;

architecture BEHAVIORAL of Door_SMD_MUSER_top is
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
   component SR4CLED_HXILINX_top
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
   
   component Ver_Op_Clos_MUSER_top
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
   
   attribute HU_SET of XLXI_1 : label is "XLXI_1_5";
begin
   Out0 <= Out0_DUMMY;
   Out1 <= Out1_DUMMY;
   Out2 <= Out2_DUMMY;
   Out3 <= Out3_DUMMY;
   XLXI_1 : SR4CLED_HXILINX_top
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
   
   XLXI_10 : Ver_Op_Clos_MUSER_top
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



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity Temp_MUSER_top is
   port ( Ck  : in    std_logic; 
          CLR : in    std_logic; 
          EST : in    std_logic_vector (2 downto 0); 
          FC  : out   std_logic; 
          Q   : out   std_logic_vector (7 downto 0));
end Temp_MUSER_top;

architecture BEHAVIORAL of Temp_MUSER_top is
   attribute HU_SET     : string ;
   attribute BOX_TYPE   : string ;
   signal XLXN_7  : std_logic;
   signal XLXN_18 : std_logic;
   signal XLXN_25 : std_logic;
   component CC8RE_HXILINX_top
      port ( C   : in    std_logic; 
             CE  : in    std_logic; 
             R   : in    std_logic; 
             CEO : out   std_logic; 
             Q   : out   std_logic_vector (7 downto 0); 
             TC  : out   std_logic);
   end component;
   
   component INV
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of INV : component is "BLACK_BOX";
   
   component M2_1_HXILINX_top
      port ( D0 : in    std_logic; 
             D1 : in    std_logic; 
             S0 : in    std_logic; 
             O  : out   std_logic);
   end component;
   
   attribute HU_SET of XLXI_1 : label is "XLXI_1_6";
   attribute HU_SET of XLXI_9 : label is "XLXI_9_7";
begin
   XLXI_1 : CC8RE_HXILINX_top
      port map (C=>Ck,
                CE=>XLXN_7,
                R=>XLXN_18,
                CEO=>open,
                Q(7 downto 0)=>Q(7 downto 0),
                TC=>FC);
   
   XLXI_7 : INV
      port map (I=>CLR,
                O=>XLXN_18);
   
   XLXI_9 : M2_1_HXILINX_top
      port map (D0=>EST(1),
                D1=>XLXN_25,
                S0=>EST(0),
                O=>XLXN_7);
   
   XLXI_10 : INV
      port map (I=>EST(2),
                O=>XLXN_25);
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity VCodAc_MUSER_top is
   port ( EST : in    std_logic_vector (2 downto 0); 
          P   : in    std_logic_vector (3 downto 0); 
          USU : in    std_logic_vector (2 downto 1); 
          F   : out   std_logic; 
          V   : out   std_logic);
end VCodAc_MUSER_top;

architecture BEHAVIORAL of VCodAc_MUSER_top is
   attribute HU_SET     : string ;
   attribute BOX_TYPE   : string ;
   signal C       : std_logic;
   signal En      : std_logic;
   signal VA      : std_logic;
   signal XLXN_5  : std_logic;
   signal XLXN_78 : std_logic;
   component M16_1E_HXILINX_top
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
   
   attribute HU_SET of XLXI_3 : label is "XLXI_3_8";
begin
   XLXI_3 : M16_1E_HXILINX_top
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



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity MyDesign_MUSER_top is
   port ( btn       : in    std_logic_vector (4 downto 0); 
          Ck        : in    std_logic; 
          sw        : in    std_logic_vector (7 downto 0); 
          Leds      : out   std_logic_vector (7 downto 0); 
          SevenSeg0 : out   std_logic_vector (7 downto 0); 
          SevenSeg1 : out   std_logic_vector (7 downto 0); 
          SevenSeg2 : out   std_logic_vector (7 downto 0); 
          SevenSeg3 : out   std_logic_vector (7 downto 0));
end MyDesign_MUSER_top;

architecture BEHAVIORAL of MyDesign_MUSER_top is
   attribute BOX_TYPE   : string ;
   attribute HU_SET     : string ;
   signal Ck_4Hz    : std_logic;
   signal cnt       : std_logic_vector (7 downto 0);
   signal EST       : std_logic_vector (2 downto 0);
   signal F         : std_logic;
   signal FC        : std_logic;
   signal F0        : std_logic;
   signal U         : std_logic;
   signal U0        : std_logic;
   signal XLXN_65   : std_logic;
   signal XLXN_93   : std_logic;
   signal XLXN_94   : std_logic;
   signal XLXN_99   : std_logic;
   signal XLXN_107  : std_logic;
   component clk_div
      port ( clk_in  : in    std_logic; 
             div_1hz : out   std_logic; 
             div_4hz : out   std_logic);
   end component;
   
   component BUF
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of BUF : component is "BLACK_BOX";
   
   component VCodAc_MUSER_top
      port ( EST : in    std_logic_vector (2 downto 0); 
             F   : out   std_logic; 
             P   : in    std_logic_vector (3 downto 0); 
             USU : in    std_logic_vector (2 downto 1); 
             V   : out   std_logic);
   end component;
   
   component OR3
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             I2 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR3 : component is "BLACK_BOX";
   
   component Temp_MUSER_top
      port ( Ck  : in    std_logic; 
             CLR : in    std_logic; 
             EST : in    std_logic_vector (2 downto 0); 
             FC  : out   std_logic; 
             Q   : out   std_logic_vector (7 downto 0));
   end component;
   
   component M2_1E_HXILINX_top
      port ( D0 : in    std_logic; 
             D1 : in    std_logic; 
             E  : in    std_logic; 
             S0 : in    std_logic; 
             O  : out   std_logic);
   end component;
   
   component OR2
      port ( I0 : in    std_logic; 
             I1 : in    std_logic; 
             O  : out   std_logic);
   end component;
   attribute BOX_TYPE of OR2 : component is "BLACK_BOX";
   
   component Door_SMD_MUSER_top
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
   end component;
   
   component p_fin_MUSER_top
      port ( A  : in    std_logic; 
             ck : in    std_logic; 
             S  : out   std_logic);
   end component;
   
   component INV
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of INV : component is "BLACK_BOX";
   
   component Control_VHDL
      port ( U   : in    std_logic; 
             R   : in    std_logic; 
             CLR : in    std_logic; 
             Ck  : in    std_logic; 
             S   : out   std_logic; 
             Q   : out   std_logic_vector (2 downto 0));
   end component;
   
   attribute HU_SET of XLXI_42 : label is "XLXI_42_9";
begin
   XLXI_4 : clk_div
      port map (clk_in=>Ck,
                div_1hz=>open,
                div_4hz=>Ck_4Hz);
   
   XLXI_7 : BUF
      port map (I=>EST(0),
                O=>Leds(0));
   
   XLXI_13 : BUF
      port map (I=>EST(1),
                O=>Leds(1));
   
   XLXI_14 : BUF
      port map (I=>EST(2),
                O=>Leds(2));
   
   XLXI_20 : VCodAc_MUSER_top
      port map (EST(2 downto 0)=>EST(2 downto 0),
                P(3 downto 0)=>btn(4 downto 1),
                USU(2 downto 1)=>sw(2 downto 1),
                F=>F0,
                V=>U0);
   
   XLXI_27 : BUF
      port map (I=>XLXN_94,
                O=>Leds(3));
   
   XLXI_36 : OR3
      port map (I0=>FC,
                I1=>F,
                I2=>U,
                O=>XLXN_93);
   
   XLXI_37 : Temp_MUSER_top
      port map (Ck=>Ck_4Hz,
                CLR=>sw(0),
                EST(2 downto 0)=>EST(2 downto 0),
                FC=>FC,
                Q(7 downto 0)=>cnt(7 downto 0));
   
   XLXI_42 : M2_1E_HXILINX_top
      port map (D0=>XLXN_93,
                D1=>cnt(0),
                E=>sw(3),
                S0=>btn(0),
                O=>XLXN_94);
   
   XLXI_43 : OR2
      port map (I0=>FC,
                I1=>F,
                O=>XLXN_99);
   
   XLXI_46 : Door_SMD_MUSER_top
      port map (Ck=>Ck_4Hz,
                Clr=>sw(0),
                D=>XLXN_65,
                Out0=>Leds(7),
                Out1=>Leds(6),
                Out2=>Leds(5),
                Out3=>Leds(4),
                Seven_Seg0(7 downto 0)=>SevenSeg0(7 downto 0),
                Seven_Seg1(7 downto 0)=>SevenSeg1(7 downto 0),
                Seven_Seg2(7 downto 0)=>SevenSeg2(7 downto 0),
                Seven_Seg3(7 downto 0)=>SevenSeg3(7 downto 0));
   
   XLXI_47 : p_fin_MUSER_top
      port map (A=>U0,
                ck=>Ck_4Hz,
                S=>U);
   
   XLXI_48 : p_fin_MUSER_top
      port map (A=>F0,
                ck=>Ck_4Hz,
                S=>F);
   
   XLXI_49 : INV
      port map (I=>sw(0),
                O=>XLXN_107);
   
   XLXI_50 : Control_VHDL
      port map (Ck=>Ck_4Hz,
                CLR=>XLXN_107,
                R=>XLXN_99,
                U=>U,
                Q(2 downto 0)=>EST(2 downto 0),
                S=>XLXN_65);
   
end BEHAVIORAL;



library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity top is
   port ( btn      : in    std_logic_vector (4 downto 0); 
          clk      : in    std_logic; 
          EppASTB  : in    std_logic; 
          EppDSTB  : in    std_logic; 
          EppWRITE : in    std_logic; 
          RsRx     : in    std_logic; 
          sw       : in    std_logic_vector (7 downto 0); 
          an       : out   std_logic_vector (3 downto 0); 
          EppWAIT  : out   std_logic; 
          Led      : out   std_logic_vector (7 downto 0); 
          RsTx     : out   std_logic; 
          seg      : out   std_logic_vector (7 downto 0); 
          DB       : inout std_logic_vector (7 downto 0));
end top;

architecture BEHAVIORAL of top is
   attribute BOX_TYPE   : string ;
   signal btnInternal : std_logic_vector (4 downto 0);
   signal clk_out     : std_logic;
   signal LedInternal : std_logic_vector (7 downto 0);
   signal RxInternal  : std_logic;
   signal seven_seg0  : std_logic_vector (7 downto 0);
   signal seven_seg1  : std_logic_vector (7 downto 0);
   signal seven_seg2  : std_logic_vector (7 downto 0);
   signal seven_seg3  : std_logic_vector (7 downto 0);
   signal swInternal  : std_logic_vector (7 downto 0);
   signal XLXN_289    : std_logic;
   component BUFG
      port ( I : in    std_logic; 
             O : out   std_logic);
   end component;
   attribute BOX_TYPE of BUFG : component is "BLACK_BOX";
   
   component MyDesign_MUSER_top
      port ( btn       : in    std_logic_vector (4 downto 0); 
             Ck        : in    std_logic; 
             Leds      : out   std_logic_vector (7 downto 0); 
             SevenSeg0 : out   std_logic_vector (7 downto 0); 
             SevenSeg1 : out   std_logic_vector (7 downto 0); 
             SevenSeg2 : out   std_logic_vector (7 downto 0); 
             SevenSeg3 : out   std_logic_vector (7 downto 0); 
             sw        : in    std_logic_vector (7 downto 0));
   end component;
   
   component VCC
      port ( P : out   std_logic);
   end component;
   attribute BOX_TYPE of VCC : component is "BLACK_BOX";
   
   component Remote_Lab
      port ( Clk         : in    std_logic; 
             EppASTB     : in    std_logic; 
             EppDSTB     : in    std_logic; 
             RsTx        : out   std_logic; 
             EppWRITE    : in    std_logic; 
             sw          : in    std_logic_vector (7 downto 0); 
             btn         : in    std_logic_vector (4 downto 0); 
             RsRx        : in    std_logic; 
             LedInternal : in    std_logic_vector (7 downto 0); 
             SevenSeg3   : in    std_logic_vector (7 downto 0); 
             SevenSeg2   : in    std_logic_vector (7 downto 0); 
             SevenSeg1   : in    std_logic_vector (7 downto 0); 
             SevenSeg0   : in    std_logic_vector (7 downto 0); 
             TxInternal  : in    std_logic; 
             seg         : out   std_logic_vector (7 downto 0); 
             an          : out   std_logic_vector (3 downto 0); 
             RxInternal  : out   std_logic; 
             swInternal  : out   std_logic_vector (7 downto 0); 
             Led         : out   std_logic_vector (7 downto 0); 
             EppWAIT     : out   std_logic; 
             EppDB       : inout std_logic_vector (7 downto 0); 
             btnInternal : out   std_logic_vector (4 downto 0));
   end component;
   
begin
   XLXI_29 : BUFG
      port map (I=>clk,
                O=>clk_out);
   
   XLXI_241 : MyDesign_MUSER_top
      port map (btn(4 downto 0)=>btnInternal(4 downto 0),
                Ck=>clk_out,
                sw(7 downto 0)=>swInternal(7 downto 0),
                Leds(7 downto 0)=>LedInternal(7 downto 0),
                SevenSeg0(7 downto 0)=>seven_seg0(7 downto 0),
                SevenSeg1(7 downto 0)=>seven_seg1(7 downto 0),
                SevenSeg2(7 downto 0)=>seven_seg2(7 downto 0),
                SevenSeg3(7 downto 0)=>seven_seg3(7 downto 0));
   
   XLXI_262 : VCC
      port map (P=>XLXN_289);
   
   XLXI_264 : Remote_Lab
      port map (btn(4 downto 0)=>btn(4 downto 0),
                Clk=>clk_out,
                EppASTB=>EppASTB,
                EppDSTB=>EppDSTB,
                EppWRITE=>EppWRITE,
                LedInternal(7 downto 0)=>LedInternal(7 downto 0),
                RsRx=>RsRx,
                SevenSeg0(7 downto 0)=>seven_seg0(7 downto 0),
                SevenSeg1(7 downto 0)=>seven_seg1(7 downto 0),
                SevenSeg2(7 downto 0)=>seven_seg2(7 downto 0),
                SevenSeg3(7 downto 0)=>seven_seg3(7 downto 0),
                sw(7 downto 0)=>sw(7 downto 0),
                TxInternal=>XLXN_289,
                an(3 downto 0)=>an(3 downto 0),
                btnInternal(4 downto 0)=>btnInternal(4 downto 0),
                EppWAIT=>EppWAIT,
                Led(7 downto 0)=>Led(7 downto 0),
                RsTx=>RsTx,
                RxInternal=>RxInternal,
                seg(7 downto 0)=>seg(7 downto 0),
                swInternal(7 downto 0)=>swInternal(7 downto 0),
                EppDB(7 downto 0)=>DB(7 downto 0));
   
end BEHAVIORAL;


