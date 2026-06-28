<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="SevenSeg3(7:0)" />
        <signal name="SevenSeg2(7:0)" />
        <signal name="SevenSeg1(7:0)" />
        <signal name="SevenSeg0(7:0)" />
        <signal name="Ck" />
        <signal name="sw(0)" />
        <signal name="Ck_4Hz" />
        <signal name="sw(7:0)" />
        <signal name="btn(4:0)" />
        <signal name="Leds(7:0)" />
        <signal name="U0" />
        <signal name="btn(4:1)" />
        <signal name="EST(2:0)" />
        <signal name="U" />
        <signal name="F" />
        <signal name="FC" />
        <signal name="F0" />
        <signal name="XLXN_65" />
        <signal name="sw(2:1)" />
        <signal name="XLXN_99" />
        <signal name="cnt(7:0)" />
        <signal name="Leds(7)" />
        <signal name="Leds(6)" />
        <signal name="Leds(5)" />
        <signal name="Leds(4)" />
        <signal name="Leds(3)" />
        <signal name="XLXN_94" />
        <signal name="XLXN_93" />
        <signal name="cnt(0)" />
        <signal name="btn(0)" />
        <signal name="sw(3)" />
        <signal name="Leds(1)" />
        <signal name="EST(1)" />
        <signal name="Leds(0)" />
        <signal name="EST(0)" />
        <signal name="Leds(2)" />
        <signal name="EST(2)" />
        <signal name="XLXN_107" />
        <port polarity="Output" name="SevenSeg3(7:0)" />
        <port polarity="Output" name="SevenSeg2(7:0)" />
        <port polarity="Output" name="SevenSeg1(7:0)" />
        <port polarity="Output" name="SevenSeg0(7:0)" />
        <port polarity="Input" name="Ck" />
        <port polarity="Input" name="sw(7:0)" />
        <port polarity="Input" name="btn(4:0)" />
        <port polarity="Output" name="Leds(7:0)" />
        <blockdef name="clk_div">
            <timestamp>2014-3-10T11:8:13</timestamp>
            <rect width="256" x="64" y="-128" height="128" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
            <line x2="384" y1="-32" y2="-32" x1="320" />
        </blockdef>
        <blockdef name="buf">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="128" y1="-32" y2="-32" x1="224" />
            <line x2="128" y1="0" y2="-32" x1="64" />
            <line x2="64" y1="-32" y2="-64" x1="128" />
            <line x2="64" y1="-64" y2="0" x1="64" />
        </blockdef>
        <blockdef name="VCodAc">
            <timestamp>2022-4-2T13:18:9</timestamp>
            <rect width="256" x="64" y="-192" height="192" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <rect width="64" x="0" y="-44" height="24" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <rect width="64" x="0" y="-172" height="24" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <rect width="64" x="0" y="-108" height="24" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
        </blockdef>
        <blockdef name="or3">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="48" y1="-64" y2="-64" x1="0" />
            <line x2="72" y1="-128" y2="-128" x1="0" />
            <line x2="48" y1="-192" y2="-192" x1="0" />
            <line x2="192" y1="-128" y2="-128" x1="256" />
            <arc ex="192" ey="-128" sx="112" sy="-80" r="88" cx="116" cy="-168" />
            <arc ex="48" ey="-176" sx="48" sy="-80" r="56" cx="16" cy="-128" />
            <line x2="48" y1="-64" y2="-80" x1="48" />
            <line x2="48" y1="-192" y2="-176" x1="48" />
            <line x2="48" y1="-80" y2="-80" x1="112" />
            <arc ex="112" ey="-176" sx="192" sy="-128" r="88" cx="116" cy="-88" />
            <line x2="48" y1="-176" y2="-176" x1="112" />
        </blockdef>
        <blockdef name="Temp">
            <timestamp>2022-4-2T17:20:30</timestamp>
            <rect width="256" x="64" y="-192" height="192" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <rect width="64" x="0" y="-172" height="24" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
            <rect width="64" x="320" y="-108" height="24" />
        </blockdef>
        <blockdef name="m2_1e">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="96" y1="-96" y2="-96" x1="0" />
            <line x2="96" y1="-32" y2="-32" x1="0" />
            <line x2="92" y1="-32" y2="-32" x1="208" />
            <line x2="208" y1="-152" y2="-32" x1="208" />
            <line x2="96" y1="-96" y2="-96" x1="144" />
            <line x2="144" y1="-136" y2="-96" x1="144" />
            <line x2="96" y1="-128" y2="-256" x1="96" />
            <line x2="96" y1="-160" y2="-128" x1="256" />
            <line x2="256" y1="-224" y2="-160" x1="256" />
            <line x2="256" y1="-256" y2="-224" x1="96" />
            <line x2="256" y1="-192" y2="-192" x1="320" />
            <line x2="96" y1="-224" y2="-224" x1="0" />
            <line x2="96" y1="-160" y2="-160" x1="0" />
        </blockdef>
        <blockdef name="or2">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-64" x1="0" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="192" y1="-96" y2="-96" x1="256" />
            <arc ex="192" ey="-96" sx="112" sy="-48" r="88" cx="116" cy="-136" />
            <arc ex="48" ey="-144" sx="48" sy="-48" r="56" cx="16" cy="-96" />
            <line x2="48" y1="-144" y2="-144" x1="112" />
            <arc ex="112" ey="-144" sx="192" sy="-96" r="88" cx="116" cy="-56" />
            <line x2="48" y1="-48" y2="-48" x1="112" />
        </blockdef>
        <blockdef name="Door_SMD">
            <timestamp>2021-1-23T23:44:14</timestamp>
            <rect width="256" x="64" y="-512" height="512" />
            <line x2="0" y1="-416" y2="-416" x1="64" />
            <line x2="0" y1="-352" y2="-352" x1="64" />
            <line x2="0" y1="-480" y2="-480" x1="64" />
            <line x2="384" y1="-480" y2="-480" x1="320" />
            <line x2="384" y1="-416" y2="-416" x1="320" />
            <line x2="384" y1="-352" y2="-352" x1="320" />
            <line x2="384" y1="-288" y2="-288" x1="320" />
            <line x2="384" y1="-32" y2="-32" x1="320" />
            <rect width="64" x="320" y="-44" height="24" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
            <rect width="64" x="320" y="-108" height="24" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
            <rect width="64" x="320" y="-172" height="24" />
            <line x2="384" y1="-224" y2="-224" x1="320" />
            <rect width="64" x="320" y="-236" height="24" />
        </blockdef>
        <blockdef name="p_fin">
            <timestamp>2014-11-21T9:29:49</timestamp>
            <rect width="128" x="64" y="-128" height="128" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="256" y1="-96" y2="-96" x1="192" />
        </blockdef>
        <blockdef name="inv">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="160" y1="-32" y2="-32" x1="224" />
            <line x2="128" y1="-64" y2="-32" x1="64" />
            <line x2="64" y1="-32" y2="0" x1="128" />
            <line x2="64" y1="0" y2="-64" x1="64" />
            <circle r="16" cx="144" cy="-32" />
        </blockdef>
        <blockdef name="Control_VHDL">
            <timestamp>2026-5-26T13:40:57</timestamp>
            <rect width="256" x="64" y="-256" height="256" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="384" y1="-224" y2="-224" x1="320" />
            <rect width="64" x="320" y="-172" height="24" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
        </blockdef>
        <blockdef name="Control">
            <timestamp>2026-5-26T4:33:26</timestamp>
            <rect width="256" x="64" y="-256" height="256" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
            <rect width="64" x="320" y="-172" height="24" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <line x2="384" y1="-224" y2="-224" x1="320" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
        </blockdef>
        <block symbolname="VCodAc" name="XLXI_20">
            <blockpin signalname="EST(2:0)" name="EST(2:0)" />
            <blockpin signalname="F0" name="F" />
            <blockpin signalname="btn(4:1)" name="P(3:0)" />
            <blockpin signalname="sw(2:1)" name="USU(2:1)" />
            <blockpin signalname="U0" name="V" />
        </block>
        <block symbolname="clk_div" name="XLXI_4">
            <blockpin signalname="Ck" name="clk_in" />
            <blockpin name="div_1hz" />
            <blockpin signalname="Ck_4Hz" name="div_4hz" />
        </block>
        <block symbolname="or2" name="XLXI_43">
            <blockpin signalname="FC" name="I0" />
            <blockpin signalname="F" name="I1" />
            <blockpin signalname="XLXN_99" name="O" />
        </block>
        <block symbolname="Temp" name="XLXI_37">
            <blockpin signalname="Ck_4Hz" name="Ck" />
            <blockpin signalname="sw(0)" name="CLR" />
            <blockpin signalname="EST(2:0)" name="EST(2:0)" />
            <blockpin signalname="FC" name="FC" />
            <blockpin signalname="cnt(7:0)" name="Q(7:0)" />
        </block>
        <block symbolname="buf" name="XLXI_27">
            <blockpin signalname="XLXN_94" name="I" />
            <blockpin signalname="Leds(3)" name="O" />
        </block>
        <block symbolname="or3" name="XLXI_36">
            <blockpin signalname="FC" name="I0" />
            <blockpin signalname="F" name="I1" />
            <blockpin signalname="U" name="I2" />
            <blockpin signalname="XLXN_93" name="O" />
        </block>
        <block symbolname="m2_1e" name="XLXI_42">
            <blockpin signalname="XLXN_93" name="D0" />
            <blockpin signalname="cnt(0)" name="D1" />
            <blockpin signalname="sw(3)" name="E" />
            <blockpin signalname="btn(0)" name="S0" />
            <blockpin signalname="XLXN_94" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_13">
            <blockpin signalname="EST(1)" name="I" />
            <blockpin signalname="Leds(1)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_7">
            <blockpin signalname="EST(0)" name="I" />
            <blockpin signalname="Leds(0)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_14">
            <blockpin signalname="EST(2)" name="I" />
            <blockpin signalname="Leds(2)" name="O" />
        </block>
        <block symbolname="Door_SMD" name="XLXI_46">
            <blockpin signalname="Ck_4Hz" name="Ck" />
            <blockpin signalname="sw(0)" name="Clr" />
            <blockpin signalname="XLXN_65" name="D" />
            <blockpin signalname="Leds(7)" name="Out0" />
            <blockpin signalname="Leds(6)" name="Out1" />
            <blockpin signalname="Leds(5)" name="Out2" />
            <blockpin signalname="Leds(4)" name="Out3" />
            <blockpin signalname="SevenSeg0(7:0)" name="Seven_Seg0(7:0)" />
            <blockpin signalname="SevenSeg1(7:0)" name="Seven_Seg1(7:0)" />
            <blockpin signalname="SevenSeg2(7:0)" name="Seven_Seg2(7:0)" />
            <blockpin signalname="SevenSeg3(7:0)" name="Seven_Seg3(7:0)" />
        </block>
        <block symbolname="p_fin" name="XLXI_47">
            <blockpin signalname="U0" name="A" />
            <blockpin signalname="Ck_4Hz" name="ck" />
            <blockpin signalname="U" name="S" />
        </block>
        <block symbolname="p_fin" name="XLXI_48">
            <blockpin signalname="F0" name="A" />
            <blockpin signalname="Ck_4Hz" name="ck" />
            <blockpin signalname="F" name="S" />
        </block>
        <block symbolname="inv" name="XLXI_49">
            <blockpin signalname="sw(0)" name="I" />
            <blockpin signalname="XLXN_107" name="O" />
        </block>
        <block symbolname="Control_VHDL" name="XLXI_50">
            <blockpin signalname="U" name="U" />
            <blockpin signalname="XLXN_99" name="R" />
            <blockpin signalname="XLXN_107" name="CLR" />
            <blockpin signalname="Ck_4Hz" name="Ck" />
            <blockpin signalname="XLXN_65" name="S" />
            <blockpin signalname="EST(2:0)" name="Q(2:0)" />
        </block>
        <block symbolname="Control" name="XLXI_51">
            <blockpin name="Ck" />
            <blockpin name="CLR" />
            <blockpin name="EST(2:0)" />
            <blockpin name="R" />
            <blockpin name="S" />
            <blockpin name="U" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <attr value="Inch" name="LengthUnitName" />
        <attr value="10" name="GridsPerUnit" />
        <branch name="sw(7:0)">
            <wire x2="704" y1="64" y2="64" x1="560" />
        </branch>
        <branch name="btn(4:0)">
            <wire x2="720" y1="144" y2="144" x1="560" />
        </branch>
        <branch name="Leds(7:0)">
            <wire x2="960" y1="64" y2="64" x1="800" />
        </branch>
        <branch name="SevenSeg3(7:0)">
            <wire x2="960" y1="144" y2="144" x1="800" />
        </branch>
        <branch name="SevenSeg2(7:0)">
            <wire x2="960" y1="224" y2="224" x1="800" />
        </branch>
        <branch name="SevenSeg1(7:0)">
            <wire x2="960" y1="304" y2="304" x1="800" />
        </branch>
        <branch name="SevenSeg0(7:0)">
            <wire x2="960" y1="384" y2="384" x1="800" />
        </branch>
        <branch name="Ck">
            <wire x2="720" y1="208" y2="208" x1="560" />
        </branch>
        <iomarker fontsize="28" x="560" y="64" name="sw(7:0)" orien="R180" />
        <iomarker fontsize="28" x="560" y="144" name="btn(4:0)" orien="R180" />
        <iomarker fontsize="28" x="960" y="64" name="Leds(7:0)" orien="R0" />
        <iomarker fontsize="28" x="960" y="144" name="SevenSeg3(7:0)" orien="R0" />
        <iomarker fontsize="28" x="960" y="224" name="SevenSeg2(7:0)" orien="R0" />
        <iomarker fontsize="28" x="960" y="304" name="SevenSeg1(7:0)" orien="R0" />
        <iomarker fontsize="28" x="960" y="384" name="SevenSeg0(7:0)" orien="R0" />
        <iomarker fontsize="28" x="560" y="208" name="Ck" orien="R180" />
        <branch name="Ck_4Hz">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1008" y="928" type="branch" />
            <wire x2="1136" y1="928" y2="928" x1="1008" />
            <wire x2="1168" y1="928" y2="928" x1="1136" />
            <wire x2="1168" y1="656" y2="656" x1="1136" />
            <wire x2="1136" y1="656" y2="928" x1="1136" />
        </branch>
        <instance x="384" y="1072" name="XLXI_4" orien="R0">
        </instance>
        <branch name="Ck">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="336" y="976" type="branch" />
            <wire x2="384" y1="976" y2="976" x1="336" />
        </branch>
        <branch name="Ck_4Hz">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="832" y="1040" type="branch" />
            <wire x2="832" y1="1040" y2="1040" x1="768" />
        </branch>
        <branch name="U">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="1632" y="592" type="branch" />
            <wire x2="1632" y1="592" y2="592" x1="1424" />
            <wire x2="1856" y1="592" y2="592" x1="1632" />
            <wire x2="1856" y1="592" y2="640" x1="1856" />
            <wire x2="1920" y1="640" y2="640" x1="1856" />
        </branch>
        <branch name="Ck_4Hz">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1424" y="1056" type="branch" />
            <wire x2="1632" y1="1056" y2="1056" x1="1424" />
            <wire x2="1920" y1="1056" y2="1056" x1="1632" />
            <wire x2="1632" y1="832" y2="1056" x1="1632" />
            <wire x2="1920" y1="832" y2="832" x1="1632" />
        </branch>
        <branch name="EST(2:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1840" y="992" type="branch" />
            <wire x2="1920" y1="992" y2="992" x1="1840" />
        </branch>
        <branch name="U0">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="1040" y="592" type="branch" />
            <wire x2="1040" y1="592" y2="592" x1="1024" />
            <wire x2="1168" y1="592" y2="592" x1="1040" />
        </branch>
        <branch name="XLXN_65">
            <wire x2="2656" y1="640" y2="640" x1="2304" />
            <wire x2="2672" y1="624" y2="624" x1="2656" />
            <wire x2="2656" y1="624" y2="640" x1="2656" />
        </branch>
        <branch name="EST(2:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2368" y="704" type="branch" />
            <wire x2="2368" y1="704" y2="704" x1="2304" />
        </branch>
        <branch name="FC">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1568" y="720" type="branch" />
            <wire x2="1584" y1="720" y2="720" x1="1568" />
            <wire x2="1584" y1="704" y2="720" x1="1584" />
        </branch>
        <branch name="F">
            <attrtext style="alignment:SOFT-TVCENTER;fontsize:28;fontname:Arial" attrname="Name" x="1504" y="656" type="branch" />
            <wire x2="1504" y1="864" y2="864" x1="1424" />
            <wire x2="1584" y1="640" y2="640" x1="1504" />
            <wire x2="1504" y1="640" y2="656" x1="1504" />
            <wire x2="1504" y1="656" y2="864" x1="1504" />
        </branch>
        <branch name="FC">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2384" y="992" type="branch" />
            <wire x2="2384" y1="992" y2="992" x1="2304" />
        </branch>
        <branch name="cnt(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2384" y="1056" type="branch" />
            <wire x2="2384" y1="1056" y2="1056" x1="2304" />
        </branch>
        <branch name="Ck_4Hz">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2640" y="688" type="branch" />
            <wire x2="2672" y1="688" y2="688" x1="2640" />
        </branch>
        <branch name="sw(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2640" y="752" type="branch" />
            <wire x2="2672" y1="752" y2="752" x1="2640" />
        </branch>
        <branch name="SevenSeg3(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3200" y="880" type="branch" />
            <wire x2="3200" y1="880" y2="880" x1="3056" />
        </branch>
        <branch name="SevenSeg2(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3200" y="944" type="branch" />
            <wire x2="3200" y1="944" y2="944" x1="3056" />
        </branch>
        <branch name="SevenSeg1(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3200" y="1008" type="branch" />
            <wire x2="3200" y1="1008" y2="1008" x1="3056" />
        </branch>
        <branch name="SevenSeg0(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3216" y="1072" type="branch" />
            <wire x2="3216" y1="1072" y2="1072" x1="3056" />
        </branch>
        <branch name="Leds(7)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3216" y="624" type="branch" />
            <wire x2="3216" y1="624" y2="624" x1="3056" />
        </branch>
        <branch name="Leds(6)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3232" y="688" type="branch" />
            <wire x2="3232" y1="688" y2="688" x1="3056" />
        </branch>
        <branch name="Leds(5)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3232" y="752" type="branch" />
            <wire x2="3232" y1="752" y2="752" x1="3056" />
        </branch>
        <branch name="Leds(4)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3232" y="816" type="branch" />
            <wire x2="3232" y1="816" y2="816" x1="3056" />
        </branch>
        <instance x="1920" y="1152" name="XLXI_37" orien="R0">
        </instance>
        <instance x="1280" y="1264" name="XLXI_27" orien="R0" />
        <branch name="Leds(3)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1616" y="1232" type="branch" />
            <wire x2="1616" y1="1232" y2="1232" x1="1504" />
        </branch>
        <branch name="XLXN_94">
            <wire x2="1280" y1="1232" y2="1232" x1="1040" />
        </branch>
        <branch name="U">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="288" y="1136" type="branch" />
            <wire x2="416" y1="1136" y2="1136" x1="288" />
        </branch>
        <branch name="F">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="288" y="1200" type="branch" />
            <wire x2="416" y1="1200" y2="1200" x1="288" />
        </branch>
        <branch name="FC">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="304" y="1264" type="branch" />
            <wire x2="416" y1="1264" y2="1264" x1="304" />
        </branch>
        <instance x="416" y="1328" name="XLXI_36" orien="R0" />
        <branch name="XLXN_93">
            <wire x2="720" y1="1200" y2="1200" x1="672" />
        </branch>
        <branch name="cnt(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="672" y="1264" type="branch" />
            <wire x2="720" y1="1264" y2="1264" x1="672" />
        </branch>
        <instance x="720" y="1424" name="XLXI_42" orien="R0" />
        <branch name="btn(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="352" y="1328" type="branch" />
            <wire x2="720" y1="1328" y2="1328" x1="352" />
        </branch>
        <branch name="sw(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="352" y="1392" type="branch" />
            <wire x2="720" y1="1392" y2="1392" x1="352" />
        </branch>
        <instance x="1280" y="1408" name="XLXI_13" orien="R0" />
        <branch name="Leds(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1616" y="1376" type="branch" />
            <wire x2="1616" y1="1376" y2="1376" x1="1504" />
        </branch>
        <branch name="EST(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1152" y="1376" type="branch" />
            <wire x2="1280" y1="1376" y2="1376" x1="1152" />
        </branch>
        <instance x="1280" y="1488" name="XLXI_7" orien="R0" />
        <branch name="Leds(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1616" y="1456" type="branch" />
            <wire x2="1616" y1="1456" y2="1456" x1="1504" />
        </branch>
        <branch name="EST(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1168" y="1456" type="branch" />
            <wire x2="1280" y1="1456" y2="1456" x1="1168" />
        </branch>
        <branch name="Leds(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1616" y="1312" type="branch" />
            <wire x2="1616" y1="1312" y2="1312" x1="1504" />
        </branch>
        <branch name="EST(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1184" y="1312" type="branch" />
            <wire x2="1280" y1="1312" y2="1312" x1="1184" />
        </branch>
        <instance x="1280" y="1344" name="XLXI_14" orien="R0" />
        <instance x="2672" y="1104" name="XLXI_46" orien="R0">
        </instance>
        <instance x="1168" y="960" name="XLXI_48" orien="R0">
        </instance>
        <instance x="1168" y="688" name="XLXI_47" orien="R0">
        </instance>
        <branch name="sw(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1424" y="1120" type="branch" />
            <wire x2="1568" y1="1120" y2="1120" x1="1424" />
            <wire x2="1920" y1="1120" y2="1120" x1="1568" />
            <wire x2="1568" y1="784" y2="1120" x1="1568" />
        </branch>
        <branch name="XLXN_99">
            <wire x2="1904" y1="672" y2="672" x1="1840" />
            <wire x2="1904" y1="672" y2="704" x1="1904" />
            <wire x2="1920" y1="704" y2="704" x1="1904" />
        </branch>
        <instance x="1584" y="768" name="XLXI_43" orien="R0" />
        <instance x="1568" y="816" name="XLXI_49" orien="R0" />
        <branch name="XLXN_107">
            <wire x2="1856" y1="784" y2="784" x1="1792" />
            <wire x2="1856" y1="768" y2="784" x1="1856" />
            <wire x2="1920" y1="768" y2="768" x1="1856" />
        </branch>
        <instance x="1920" y="864" name="XLXI_50" orien="R0">
        </instance>
        <instance x="2128" y="384" name="XLXI_51" orien="R0">
        </instance>
        <branch name="F0">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="1072" y="656" type="branch" />
            <wire x2="1072" y1="656" y2="656" x1="1024" />
            <wire x2="1072" y1="656" y2="864" x1="1072" />
            <wire x2="1168" y1="864" y2="864" x1="1072" />
        </branch>
        <branch name="EST(2:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="352" y="720" type="branch" />
            <wire x2="368" y1="720" y2="720" x1="352" />
            <wire x2="640" y1="720" y2="720" x1="368" />
        </branch>
        <branch name="sw(2:1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="384" y="656" type="branch" />
            <wire x2="400" y1="656" y2="656" x1="384" />
            <wire x2="640" y1="656" y2="656" x1="400" />
        </branch>
        <branch name="btn(4:1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="336" y="592" type="branch" />
            <wire x2="352" y1="592" y2="592" x1="336" />
            <wire x2="640" y1="592" y2="592" x1="352" />
        </branch>
        <instance x="640" y="752" name="XLXI_20" orien="R0">
        </instance>
    </sheet>
</drawing>