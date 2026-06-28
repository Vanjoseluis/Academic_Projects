--------------------------------------------------------------------------------
-- Copyright (c) 1995-2007 Xilinx, Inc.  All rights reserved.
--------------------------------------------------------------------------------
--   ____  ____ 
--  /   /\/   / 
-- /___/  \  /    Vendor: Xilinx 
-- \   \   \/     
--  \   \         Application : 
--  /   /         Filename : xil_56920_15
-- /___/   /\     Timestamp : 02/22/2007 13:36:13
-- \   \  /  \ 
--  \___\/\___\ 
--
--Command: 
--Design Name: wtut
--

library ieee;
use ieee.std_logic_1164.ALL;
use ieee.numeric_std.ALL;
library UNISIM;
use UNISIM.Vcomponents.ALL;

entity clk_div is
   port ( clk_in	: in    std_logic; 
          div_1hz : out   std_logic;
			 div_4hz : out   std_logic);
end clk_div;

architecture Behavioral of clk_div is
signal cnt1 : integer := 0;
signal cnt2 : integer := 0;
signal div1_temp : std_logic := '0';
signal div2_temp : std_logic := '0';

begin
	process (clk_in) begin
	if (clk_in'event and clk_in = '1') then
		if cnt1 >= 50000000 then
			div1_temp <= not(div1_temp);
			cnt1 <= 0;
		else	
			div1_temp <= div1_temp;
			cnt1 <= cnt1 + 1;
		end if;
		div_1hz <= div1_temp;
	end if;
	end process;

	process (clk_in) begin
	if (clk_in'event and clk_in = '1') then
		if cnt2 >= 12500000 then
			div2_temp <= not(div2_temp);
			cnt2 <= 0;
		else	
			div2_temp <= div2_temp;
			cnt2 <= cnt2 + 1;
		end if;
		div_4hz <= div2_temp;
	end if;
	end process;
	
end Behavioral;