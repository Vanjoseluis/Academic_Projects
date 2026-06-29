<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="q1" />
        <signal name="q0" />
        <signal name="VF_C" />
        <signal name="V_R" />
        <signal name="Led_V" />
        <signal name="D1" />
        <signal name="D0" />
        <signal name="XLXN_20" />
        <signal name="XLXN_21" />
        <signal name="XLXN_30" />
        <signal name="XLXN_34" />
        <signal name="ENA" />
        <signal name="VC" />
        <signal name="VF" />
        <signal name="XLXN_45" />
        <signal name="XLXN_47" />
        <signal name="CK" />
        <signal name="CLR" />
        <port polarity="Output" name="VF_C" />
        <port polarity="Output" name="V_R" />
        <port polarity="Output" name="Led_V" />
        <port polarity="Input" name="ENA" />
        <port polarity="Input" name="VC" />
        <port polarity="Input" name="VF" />
        <port polarity="Input" name="CK" />
        <port polarity="Input" name="CLR" />
        <blockdef name="fdc">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="320" y1="-256" y2="-256" x1="384" />
            <rect width="256" x="64" y="-320" height="256" />
            <line x2="80" y1="-112" y2="-128" x1="64" />
            <line x2="64" y1="-128" y2="-144" x1="80" />
            <line x2="192" y1="-64" y2="-32" x1="192" />
            <line x2="64" y1="-32" y2="-32" x1="192" />
        </blockdef>
        <blockdef name="and2">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-64" x1="0" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="192" y1="-96" y2="-96" x1="256" />
            <arc ex="144" ey="-144" sx="144" sy="-48" r="48" cx="144" cy="-96" />
            <line x2="64" y1="-48" y2="-48" x1="144" />
            <line x2="144" y1="-144" y2="-144" x1="64" />
            <line x2="64" y1="-48" y2="-144" x1="64" />
        </blockdef>
        <blockdef name="and2b1">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-48" y2="-144" x1="64" />
            <line x2="144" y1="-144" y2="-144" x1="64" />
            <line x2="64" y1="-48" y2="-48" x1="144" />
            <arc ex="144" ey="-144" sx="144" sy="-48" r="48" cx="144" cy="-96" />
            <line x2="192" y1="-96" y2="-96" x1="256" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
        </blockdef>
        <blockdef name="buf">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="128" y1="-32" y2="-32" x1="224" />
            <line x2="128" y1="0" y2="-32" x1="64" />
            <line x2="64" y1="-32" y2="-64" x1="128" />
            <line x2="64" y1="-64" y2="0" x1="64" />
        </blockdef>
        <blockdef name="m4_1e">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="96" y1="-416" y2="-416" x1="0" />
            <line x2="96" y1="-352" y2="-352" x1="0" />
            <line x2="96" y1="-288" y2="-288" x1="0" />
            <line x2="96" y1="-224" y2="-224" x1="0" />
            <line x2="96" y1="-32" y2="-32" x1="0" />
            <line x2="256" y1="-320" y2="-320" x1="320" />
            <line x2="96" y1="-160" y2="-160" x1="0" />
            <line x2="96" y1="-96" y2="-96" x1="0" />
            <line x2="96" y1="-96" y2="-96" x1="176" />
            <line x2="176" y1="-208" y2="-96" x1="176" />
            <line x2="96" y1="-32" y2="-32" x1="224" />
            <line x2="224" y1="-216" y2="-32" x1="224" />
            <line x2="96" y1="-224" y2="-192" x1="256" />
            <line x2="256" y1="-416" y2="-224" x1="256" />
            <line x2="256" y1="-448" y2="-416" x1="96" />
            <line x2="96" y1="-192" y2="-448" x1="96" />
            <line x2="96" y1="-160" y2="-160" x1="128" />
            <line x2="128" y1="-200" y2="-160" x1="128" />
        </blockdef>
        <blockdef name="xor2">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-64" x1="0" />
            <line x2="60" y1="-128" y2="-128" x1="0" />
            <line x2="208" y1="-96" y2="-96" x1="256" />
            <arc ex="44" ey="-144" sx="48" sy="-48" r="56" cx="16" cy="-96" />
            <arc ex="64" ey="-144" sx="64" sy="-48" r="56" cx="32" cy="-96" />
            <line x2="64" y1="-144" y2="-144" x1="128" />
            <line x2="64" y1="-48" y2="-48" x1="128" />
            <arc ex="128" ey="-144" sx="208" sy="-96" r="88" cx="132" cy="-56" />
            <arc ex="208" ey="-96" sx="128" sy="-48" r="88" cx="132" cy="-136" />
        </blockdef>
        <blockdef name="gnd">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-96" x1="64" />
            <line x2="52" y1="-48" y2="-48" x1="76" />
            <line x2="60" y1="-32" y2="-32" x1="68" />
            <line x2="40" y1="-64" y2="-64" x1="88" />
            <line x2="64" y1="-64" y2="-80" x1="64" />
            <line x2="64" y1="-128" y2="-96" x1="64" />
        </blockdef>
        <blockdef name="vcc">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-64" x1="64" />
            <line x2="64" y1="0" y2="-32" x1="64" />
            <line x2="32" y1="-64" y2="-64" x1="96" />
        </blockdef>
        <blockdef name="or2b1">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="32" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="44" cy="-64" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="192" y1="-96" y2="-96" x1="256" />
            <line x2="48" y1="-48" y2="-48" x1="112" />
            <arc ex="112" ey="-144" sx="192" sy="-96" r="88" cx="116" cy="-56" />
            <line x2="48" y1="-144" y2="-144" x1="112" />
            <arc ex="48" ey="-144" sx="48" sy="-48" r="56" cx="16" cy="-96" />
            <arc ex="192" ey="-96" sx="112" sy="-48" r="88" cx="116" cy="-136" />
        </blockdef>
        <block symbolname="fdc" name="XLXI_1">
            <blockpin signalname="CK" name="C" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="D1" name="D" />
            <blockpin signalname="q1" name="Q" />
        </block>
        <block symbolname="fdc" name="XLXI_2">
            <blockpin signalname="CK" name="C" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="D0" name="D" />
            <blockpin signalname="q0" name="Q" />
        </block>
        <block symbolname="and2" name="XLXI_3">
            <blockpin signalname="q0" name="I0" />
            <blockpin signalname="q1" name="I1" />
            <blockpin signalname="VF_C" name="O" />
        </block>
        <block symbolname="and2b1" name="XLXI_4">
            <blockpin signalname="q1" name="I0" />
            <blockpin signalname="q0" name="I1" />
            <blockpin signalname="Led_V" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_5">
            <blockpin signalname="q1" name="I" />
            <blockpin signalname="V_R" name="O" />
        </block>
        <block symbolname="m4_1e" name="XLXI_6">
            <blockpin signalname="ENA" name="D0" />
            <blockpin signalname="XLXN_45" name="D1" />
            <blockpin signalname="XLXN_21" name="D2" />
            <blockpin signalname="XLXN_20" name="D3" />
            <blockpin signalname="XLXN_20" name="E" />
            <blockpin signalname="q0" name="S0" />
            <blockpin signalname="q1" name="S1" />
            <blockpin signalname="D0" name="O" />
        </block>
        <block symbolname="m4_1e" name="XLXI_7">
            <blockpin signalname="XLXN_21" name="D0" />
            <blockpin signalname="XLXN_34" name="D1" />
            <blockpin signalname="XLXN_20" name="D2" />
            <blockpin signalname="XLXN_20" name="D3" />
            <blockpin signalname="XLXN_20" name="E" />
            <blockpin signalname="q0" name="S0" />
            <blockpin signalname="q1" name="S1" />
            <blockpin signalname="D1" name="O" />
        </block>
        <block symbolname="gnd" name="XLXI_9">
            <blockpin signalname="XLXN_21" name="G" />
        </block>
        <block symbolname="vcc" name="XLXI_10">
            <blockpin signalname="XLXN_20" name="P" />
        </block>
        <block symbolname="and2" name="XLXI_15">
            <blockpin signalname="ENA" name="I0" />
            <blockpin signalname="XLXN_30" name="I1" />
            <blockpin signalname="XLXN_34" name="O" />
        </block>
        <block symbolname="xor2" name="XLXI_8">
            <blockpin signalname="VC" name="I0" />
            <blockpin signalname="VF" name="I1" />
            <blockpin signalname="XLXN_30" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_19">
            <blockpin signalname="XLXN_47" name="I0" />
            <blockpin signalname="ENA" name="I1" />
            <blockpin signalname="XLXN_45" name="O" />
        </block>
        <block symbolname="or2b1" name="XLXI_20">
            <blockpin signalname="VC" name="I0" />
            <blockpin signalname="VF" name="I1" />
            <blockpin signalname="XLXN_47" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <instance x="1872" y="1008" name="XLXI_1" orien="R0" />
        <instance x="1872" y="1440" name="XLXI_2" orien="R0" />
        <instance x="2592" y="880" name="XLXI_3" orien="R0" />
        <instance x="2592" y="1024" name="XLXI_5" orien="R0" />
        <branch name="q1">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2320" y="752" type="branch" />
            <wire x2="2320" y1="752" y2="752" x1="2256" />
            <wire x2="2528" y1="752" y2="752" x1="2320" />
            <wire x2="2592" y1="752" y2="752" x1="2528" />
            <wire x2="2528" y1="752" y2="992" x1="2528" />
            <wire x2="2592" y1="992" y2="992" x1="2528" />
            <wire x2="2528" y1="992" y2="1120" x1="2528" />
            <wire x2="2592" y1="1120" y2="1120" x1="2528" />
        </branch>
        <branch name="q0">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2320" y="1184" type="branch" />
            <wire x2="2320" y1="1184" y2="1184" x1="2256" />
            <wire x2="2416" y1="1184" y2="1184" x1="2320" />
            <wire x2="2592" y1="1184" y2="1184" x1="2416" />
            <wire x2="2416" y1="816" y2="1184" x1="2416" />
            <wire x2="2592" y1="816" y2="816" x1="2416" />
        </branch>
        <instance x="2592" y="1056" name="XLXI_4" orien="M180" />
        <branch name="VF_C">
            <wire x2="2960" y1="784" y2="784" x1="2848" />
        </branch>
        <branch name="V_R">
            <wire x2="2960" y1="992" y2="992" x1="2816" />
        </branch>
        <branch name="Led_V">
            <wire x2="2960" y1="1152" y2="1152" x1="2848" />
        </branch>
        <iomarker fontsize="28" x="2960" y="784" name="VF_C" orien="R0" />
        <iomarker fontsize="28" x="2960" y="992" name="V_R" orien="R0" />
        <iomarker fontsize="28" x="2960" y="1152" name="Led_V" orien="R0" />
        <instance x="1296" y="1008" name="XLXI_7" orien="R0" />
        <instance x="1280" y="1584" name="XLXI_6" orien="R0" />
        <branch name="D1">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="1776" y="752" type="branch" />
            <wire x2="1744" y1="688" y2="688" x1="1616" />
            <wire x2="1744" y1="688" y2="752" x1="1744" />
            <wire x2="1776" y1="752" y2="752" x1="1744" />
            <wire x2="1872" y1="752" y2="752" x1="1776" />
        </branch>
        <branch name="D0">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="1760" y="1184" type="branch" />
            <wire x2="1728" y1="1264" y2="1264" x1="1600" />
            <wire x2="1728" y1="1184" y2="1264" x1="1728" />
            <wire x2="1760" y1="1184" y2="1184" x1="1728" />
            <wire x2="1872" y1="1184" y2="1184" x1="1760" />
        </branch>
        <branch name="q0">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1184" y="848" type="branch" />
            <wire x2="1296" y1="848" y2="848" x1="1184" />
        </branch>
        <branch name="q1">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1184" y="912" type="branch" />
            <wire x2="1296" y1="912" y2="912" x1="1184" />
        </branch>
        <branch name="q0">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1184" y="1424" type="branch" />
            <wire x2="1280" y1="1424" y2="1424" x1="1184" />
        </branch>
        <branch name="q1">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1184" y="1488" type="branch" />
            <wire x2="1280" y1="1488" y2="1488" x1="1184" />
        </branch>
        <instance x="768" y="912" name="XLXI_10" orien="R0" />
        <branch name="XLXN_20">
            <wire x2="832" y1="912" y2="976" x1="832" />
            <wire x2="1072" y1="976" y2="976" x1="832" />
            <wire x2="1296" y1="976" y2="976" x1="1072" />
            <wire x2="1072" y1="976" y2="1360" x1="1072" />
            <wire x2="1072" y1="1360" y2="1552" x1="1072" />
            <wire x2="1280" y1="1552" y2="1552" x1="1072" />
            <wire x2="1280" y1="1360" y2="1360" x1="1072" />
            <wire x2="1072" y1="784" y2="976" x1="1072" />
            <wire x2="1216" y1="784" y2="784" x1="1072" />
            <wire x2="1296" y1="784" y2="784" x1="1216" />
            <wire x2="1296" y1="720" y2="720" x1="1216" />
            <wire x2="1216" y1="720" y2="784" x1="1216" />
        </branch>
        <branch name="XLXN_21">
            <wire x2="1024" y1="1008" y2="1008" x1="800" />
            <wire x2="1024" y1="1008" y2="1296" x1="1024" />
            <wire x2="1280" y1="1296" y2="1296" x1="1024" />
            <wire x2="800" y1="1008" y2="1024" x1="800" />
            <wire x2="1296" y1="592" y2="592" x1="1024" />
            <wire x2="1024" y1="592" y2="1008" x1="1024" />
        </branch>
        <instance x="736" y="1152" name="XLXI_9" orien="R0" />
        <branch name="XLXN_34">
            <wire x2="1296" y1="656" y2="656" x1="1008" />
        </branch>
        <instance x="752" y="752" name="XLXI_15" orien="R0" />
        <branch name="XLXN_30">
            <wire x2="752" y1="624" y2="624" x1="704" />
        </branch>
        <branch name="ENA">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="576" y="1168" type="branch" />
            <wire x2="688" y1="1168" y2="1168" x1="576" />
            <wire x2="1280" y1="1168" y2="1168" x1="688" />
            <wire x2="688" y1="1168" y2="1200" x1="688" />
            <wire x2="752" y1="1200" y2="1200" x1="688" />
            <wire x2="752" y1="688" y2="688" x1="688" />
            <wire x2="688" y1="688" y2="1168" x1="688" />
        </branch>
        <branch name="VC">
            <wire x2="384" y1="656" y2="656" x1="272" />
            <wire x2="448" y1="656" y2="656" x1="384" />
            <wire x2="384" y1="656" y2="1296" x1="384" />
            <wire x2="464" y1="1296" y2="1296" x1="384" />
        </branch>
        <branch name="VF">
            <wire x2="432" y1="592" y2="592" x1="272" />
            <wire x2="448" y1="592" y2="592" x1="432" />
            <wire x2="432" y1="592" y2="1232" x1="432" />
            <wire x2="464" y1="1232" y2="1232" x1="432" />
        </branch>
        <instance x="448" y="720" name="XLXI_8" orien="R0" />
        <instance x="752" y="1328" name="XLXI_19" orien="R0" />
        <branch name="XLXN_45">
            <wire x2="1280" y1="1232" y2="1232" x1="1008" />
        </branch>
        <instance x="464" y="1360" name="XLXI_20" orien="R0" />
        <branch name="XLXN_47">
            <wire x2="752" y1="1264" y2="1264" x1="720" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="800" y="1712" type="branch" />
            <wire x2="1808" y1="1712" y2="1712" x1="800" />
            <wire x2="1808" y1="976" y2="1408" x1="1808" />
            <wire x2="1808" y1="1408" y2="1712" x1="1808" />
            <wire x2="1872" y1="1408" y2="1408" x1="1808" />
            <wire x2="1872" y1="976" y2="976" x1="1808" />
        </branch>
        <branch name="CK">
            <wire x2="1840" y1="1856" y2="1856" x1="528" />
            <wire x2="1872" y1="880" y2="880" x1="1840" />
            <wire x2="1840" y1="880" y2="1312" x1="1840" />
            <wire x2="1840" y1="1312" y2="1856" x1="1840" />
            <wire x2="1872" y1="1312" y2="1312" x1="1840" />
        </branch>
        <branch name="ENA">
            <wire x2="544" y1="1680" y2="1680" x1="432" />
        </branch>
        <branch name="CLR">
            <wire x2="544" y1="1744" y2="1744" x1="432" />
        </branch>
        <iomarker fontsize="28" x="528" y="1856" name="CK" orien="R180" />
        <iomarker fontsize="28" x="432" y="1680" name="ENA" orien="R180" />
        <iomarker fontsize="28" x="432" y="1744" name="CLR" orien="R180" />
        <iomarker fontsize="28" x="272" y="656" name="VC" orien="R180" />
        <iomarker fontsize="28" x="272" y="592" name="VF" orien="R180" />
    </sheet>
</drawing>