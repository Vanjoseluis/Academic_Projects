<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="U" />
        <signal name="R" />
        <signal name="D(2:0)" />
        <signal name="q(2:0)" />
        <signal name="q(2)" />
        <signal name="q(1)" />
        <signal name="q(0)" />
        <signal name="XLXN_49" />
        <signal name="XLXN_50" />
        <signal name="D(0)" />
        <signal name="XLXN_51" />
        <signal name="XLXN_121" />
        <signal name="XLXN_139" />
        <signal name="XLXN_140" />
        <signal name="XLXN_141" />
        <signal name="XLXN_142" />
        <signal name="XLXN_154" />
        <signal name="XLXN_155" />
        <signal name="D(1)" />
        <signal name="D(2)" />
        <signal name="XLXN_161" />
        <port polarity="Input" name="U" />
        <port polarity="Input" name="R" />
        <port polarity="Output" name="D(2:0)" />
        <port polarity="Input" name="q(2:0)" />
        <blockdef name="and4b3">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="40" y1="-128" y2="-128" x1="0" />
            <circle r="12" cx="52" cy="-128" />
            <line x2="40" y1="-192" y2="-192" x1="0" />
            <circle r="12" cx="52" cy="-192" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="192" y1="-160" y2="-160" x1="256" />
            <line x2="64" y1="-64" y2="-256" x1="64" />
            <line x2="64" y1="-112" y2="-112" x1="144" />
            <arc ex="144" ey="-208" sx="144" sy="-112" r="48" cx="144" cy="-160" />
            <line x2="144" y1="-208" y2="-208" x1="64" />
        </blockdef>
        <blockdef name="and3b2">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="40" y1="-128" y2="-128" x1="0" />
            <circle r="12" cx="52" cy="-128" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="192" y1="-128" y2="-128" x1="256" />
            <line x2="64" y1="-64" y2="-192" x1="64" />
            <arc ex="144" ey="-176" sx="144" sy="-80" r="48" cx="144" cy="-128" />
            <line x2="64" y1="-80" y2="-80" x1="144" />
            <line x2="144" y1="-176" y2="-176" x1="64" />
        </blockdef>
        <blockdef name="and3">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-64" x1="0" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="192" y1="-128" y2="-128" x1="256" />
            <line x2="144" y1="-176" y2="-176" x1="64" />
            <line x2="64" y1="-80" y2="-80" x1="144" />
            <arc ex="144" ey="-176" sx="144" sy="-80" r="48" cx="144" cy="-128" />
            <line x2="64" y1="-64" y2="-192" x1="64" />
        </blockdef>
        <blockdef name="and4b1">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="192" y1="-160" y2="-160" x1="256" />
            <line x2="64" y1="-64" y2="-256" x1="64" />
            <line x2="64" y1="-112" y2="-112" x1="144" />
            <arc ex="144" ey="-208" sx="144" sy="-112" r="48" cx="144" cy="-160" />
            <line x2="144" y1="-208" y2="-208" x1="64" />
        </blockdef>
        <blockdef name="or4">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="48" y1="-64" y2="-64" x1="0" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="48" y1="-256" y2="-256" x1="0" />
            <line x2="192" y1="-160" y2="-160" x1="256" />
            <arc ex="112" ey="-208" sx="192" sy="-160" r="88" cx="116" cy="-120" />
            <line x2="48" y1="-208" y2="-208" x1="112" />
            <line x2="48" y1="-112" y2="-112" x1="112" />
            <line x2="48" y1="-256" y2="-208" x1="48" />
            <line x2="48" y1="-64" y2="-112" x1="48" />
            <arc ex="48" ey="-208" sx="48" sy="-112" r="56" cx="16" cy="-160" />
            <arc ex="192" ey="-160" sx="112" sy="-112" r="88" cx="116" cy="-200" />
        </blockdef>
        <blockdef name="and3b1">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="192" y1="-128" y2="-128" x1="256" />
            <line x2="64" y1="-64" y2="-192" x1="64" />
            <arc ex="144" ey="-176" sx="144" sy="-80" r="48" cx="144" cy="-128" />
            <line x2="64" y1="-80" y2="-80" x1="144" />
            <line x2="144" y1="-176" y2="-176" x1="64" />
        </blockdef>
        <blockdef name="and4b2">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="40" y1="-128" y2="-128" x1="0" />
            <circle r="12" cx="52" cy="-128" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="192" y1="-160" y2="-160" x1="256" />
            <line x2="144" y1="-208" y2="-208" x1="64" />
            <arc ex="144" ey="-208" sx="144" sy="-112" r="48" cx="144" cy="-160" />
            <line x2="64" y1="-64" y2="-256" x1="64" />
            <line x2="64" y1="-112" y2="-112" x1="144" />
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
        <block symbolname="and4b3" name="XLXI_1">
            <blockpin signalname="q(1)" name="I0" />
            <blockpin signalname="q(2)" name="I1" />
            <blockpin signalname="R" name="I2" />
            <blockpin signalname="U" name="I3" />
            <blockpin signalname="XLXN_49" name="O" />
        </block>
        <block symbolname="and3b2" name="XLXI_2">
            <blockpin signalname="q(2)" name="I0" />
            <blockpin signalname="R" name="I1" />
            <blockpin signalname="q(0)" name="I2" />
            <blockpin signalname="XLXN_51" name="O" />
        </block>
        <block symbolname="and4b1" name="XLXI_8">
            <blockpin signalname="U" name="I0" />
            <blockpin signalname="q(0)" name="I1" />
            <blockpin signalname="q(1)" name="I2" />
            <blockpin signalname="q(2)" name="I3" />
            <blockpin signalname="XLXN_121" name="O" />
        </block>
        <block symbolname="and3" name="XLXI_7">
            <blockpin signalname="q(1)" name="I0" />
            <blockpin signalname="q(2)" name="I1" />
            <blockpin signalname="R" name="I2" />
            <blockpin signalname="XLXN_50" name="O" />
        </block>
        <block symbolname="or4" name="XLXI_9">
            <blockpin signalname="XLXN_50" name="I0" />
            <blockpin signalname="XLXN_51" name="I1" />
            <blockpin signalname="XLXN_121" name="I2" />
            <blockpin signalname="XLXN_49" name="I3" />
            <blockpin signalname="D(0)" name="O" />
        </block>
        <block symbolname="and3b1" name="XLXI_28">
            <blockpin signalname="R" name="I0" />
            <blockpin signalname="q(1)" name="I1" />
            <blockpin signalname="q(0)" name="I2" />
            <blockpin signalname="XLXN_140" name="O" />
        </block>
        <block symbolname="and3" name="XLXI_29">
            <blockpin signalname="q(1)" name="I0" />
            <blockpin signalname="q(2)" name="I1" />
            <blockpin signalname="R" name="I2" />
            <blockpin signalname="XLXN_142" name="O" />
        </block>
        <block symbolname="and3b1" name="XLXI_33">
            <blockpin signalname="U" name="I0" />
            <blockpin signalname="q(1)" name="I1" />
            <blockpin signalname="q(2)" name="I2" />
            <blockpin signalname="XLXN_141" name="O" />
        </block>
        <block symbolname="and4b2" name="XLXI_35">
            <blockpin signalname="R" name="I0" />
            <blockpin signalname="q(2)" name="I1" />
            <blockpin signalname="q(0)" name="I2" />
            <blockpin signalname="U" name="I3" />
            <blockpin signalname="XLXN_139" name="O" />
        </block>
        <block symbolname="and3b1" name="XLXI_37">
            <blockpin signalname="U" name="I0" />
            <blockpin signalname="q(1)" name="I1" />
            <blockpin signalname="q(2)" name="I2" />
            <blockpin signalname="XLXN_155" name="O" />
        </block>
        <block symbolname="and4b1" name="XLXI_41">
            <blockpin signalname="R" name="I0" />
            <blockpin signalname="U" name="I1" />
            <blockpin signalname="q(0)" name="I2" />
            <blockpin signalname="q(1)" name="I3" />
            <blockpin signalname="XLXN_154" name="O" />
        </block>
        <block symbolname="or4" name="XLXI_44">
            <blockpin signalname="XLXN_142" name="I0" />
            <blockpin signalname="XLXN_141" name="I1" />
            <blockpin signalname="XLXN_140" name="I2" />
            <blockpin signalname="XLXN_139" name="I3" />
            <blockpin signalname="D(1)" name="O" />
        </block>
        <block symbolname="or3" name="XLXI_45">
            <blockpin signalname="XLXN_161" name="I0" />
            <blockpin signalname="XLXN_155" name="I1" />
            <blockpin signalname="XLXN_154" name="I2" />
            <blockpin signalname="D(2)" name="O" />
        </block>
        <block symbolname="and3" name="XLXI_39">
            <blockpin signalname="q(1)" name="I0" />
            <blockpin signalname="q(2)" name="I1" />
            <blockpin signalname="R" name="I2" />
            <blockpin signalname="XLXN_161" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <branch name="U">
            <wire x2="880" y1="128" y2="128" x1="736" />
        </branch>
        <branch name="D(2:0)">
            <wire x2="1200" y1="192" y2="192" x1="1024" />
        </branch>
        <iomarker fontsize="28" x="1200" y="192" name="D(2:0)" orien="R0" />
        <iomarker fontsize="28" x="736" y="128" name="U" orien="R180" />
        <branch name="q(2:0)">
            <wire x2="896" y1="256" y2="256" x1="768" />
        </branch>
        <iomarker fontsize="28" x="768" y="256" name="q(2:0)" orien="R180" />
        <branch name="R">
            <wire x2="896" y1="192" y2="192" x1="736" />
        </branch>
        <iomarker fontsize="28" x="736" y="192" name="R" orien="R180" />
        <instance x="432" y="784" name="XLXI_1" orien="R0" />
        <branch name="U">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="368" y="528" type="branch" />
            <wire x2="432" y1="528" y2="528" x1="368" />
        </branch>
        <branch name="R">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="368" y="592" type="branch" />
            <wire x2="432" y1="592" y2="592" x1="368" />
        </branch>
        <branch name="q(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="368" y="656" type="branch" />
            <wire x2="432" y1="656" y2="656" x1="368" />
        </branch>
        <branch name="q(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="368" y="720" type="branch" />
            <wire x2="432" y1="720" y2="720" x1="368" />
        </branch>
        <instance x="432" y="1328" name="XLXI_2" orien="R0" />
        <branch name="q(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="368" y="1136" type="branch" />
            <wire x2="432" y1="1136" y2="1136" x1="368" />
        </branch>
        <branch name="R">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="368" y="1200" type="branch" />
            <wire x2="432" y1="1200" y2="1200" x1="368" />
        </branch>
        <branch name="q(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="368" y="1264" type="branch" />
            <wire x2="432" y1="1264" y2="1264" x1="368" />
        </branch>
        <branch name="R">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="352" y="1376" type="branch" />
            <wire x2="432" y1="1376" y2="1376" x1="352" />
        </branch>
        <branch name="q(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="352" y="1440" type="branch" />
            <wire x2="432" y1="1440" y2="1440" x1="352" />
        </branch>
        <branch name="q(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="352" y="1504" type="branch" />
            <wire x2="432" y1="1504" y2="1504" x1="352" />
        </branch>
        <instance x="432" y="1104" name="XLXI_8" orien="R0" />
        <branch name="q(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="368" y="848" type="branch" />
            <wire x2="432" y1="848" y2="848" x1="368" />
        </branch>
        <branch name="q(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="368" y="912" type="branch" />
            <wire x2="432" y1="912" y2="912" x1="368" />
        </branch>
        <branch name="q(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="368" y="976" type="branch" />
            <wire x2="432" y1="976" y2="976" x1="368" />
        </branch>
        <branch name="U">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="368" y="1040" type="branch" />
            <wire x2="432" y1="1040" y2="1040" x1="368" />
        </branch>
        <branch name="XLXN_49">
            <wire x2="768" y1="624" y2="624" x1="688" />
            <wire x2="768" y1="624" y2="976" x1="768" />
        </branch>
        <branch name="D(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1056" y="1072" type="branch" />
            <wire x2="1056" y1="1072" y2="1072" x1="1024" />
        </branch>
        <instance x="432" y="1568" name="XLXI_7" orien="R0" />
        <instance x="768" y="1232" name="XLXI_9" orien="R0" />
        <branch name="XLXN_51">
            <wire x2="704" y1="1200" y2="1200" x1="688" />
            <wire x2="768" y1="1104" y2="1104" x1="704" />
            <wire x2="704" y1="1104" y2="1200" x1="704" />
        </branch>
        <branch name="XLXN_121">
            <wire x2="704" y1="944" y2="944" x1="688" />
            <wire x2="704" y1="944" y2="1040" x1="704" />
            <wire x2="768" y1="1040" y2="1040" x1="704" />
        </branch>
        <branch name="XLXN_50">
            <wire x2="768" y1="1440" y2="1440" x1="688" />
            <wire x2="768" y1="1168" y2="1440" x1="768" />
        </branch>
        <instance x="1312" y="1376" name="XLXI_29" orien="R0" />
        <instance x="1312" y="976" name="XLXI_28" orien="R0" />
        <instance x="1312" y="1168" name="XLXI_33" orien="R0" />
        <branch name="q(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="1312" type="branch" />
            <wire x2="1312" y1="1312" y2="1312" x1="1280" />
        </branch>
        <branch name="q(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="1248" type="branch" />
            <wire x2="1312" y1="1248" y2="1248" x1="1280" />
        </branch>
        <branch name="R">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="1184" type="branch" />
            <wire x2="1296" y1="1184" y2="1184" x1="1280" />
            <wire x2="1312" y1="1184" y2="1184" x1="1296" />
        </branch>
        <branch name="U">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="1104" type="branch" />
            <wire x2="1312" y1="1104" y2="1104" x1="1280" />
        </branch>
        <branch name="q(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="1040" type="branch" />
            <wire x2="1312" y1="1040" y2="1040" x1="1280" />
        </branch>
        <branch name="q(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="976" type="branch" />
            <wire x2="1312" y1="976" y2="976" x1="1280" />
        </branch>
        <branch name="R">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="912" type="branch" />
            <wire x2="1312" y1="912" y2="912" x1="1280" />
        </branch>
        <branch name="q(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="848" type="branch" />
            <wire x2="1312" y1="848" y2="848" x1="1280" />
        </branch>
        <branch name="q(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="784" type="branch" />
            <wire x2="1312" y1="784" y2="784" x1="1280" />
        </branch>
        <branch name="R">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="704" type="branch" />
            <wire x2="1312" y1="704" y2="704" x1="1280" />
        </branch>
        <branch name="q(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="640" type="branch" />
            <wire x2="1312" y1="640" y2="640" x1="1280" />
        </branch>
        <branch name="q(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="576" type="branch" />
            <wire x2="1312" y1="576" y2="576" x1="1280" />
        </branch>
        <branch name="U">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1280" y="512" type="branch" />
            <wire x2="1312" y1="512" y2="512" x1="1280" />
        </branch>
        <branch name="XLXN_139">
            <wire x2="1648" y1="608" y2="608" x1="1568" />
            <wire x2="1648" y1="608" y2="912" x1="1648" />
        </branch>
        <branch name="XLXN_141">
            <wire x2="1648" y1="1040" y2="1040" x1="1568" />
        </branch>
        <branch name="XLXN_140">
            <wire x2="1584" y1="848" y2="848" x1="1568" />
            <wire x2="1584" y1="848" y2="976" x1="1584" />
            <wire x2="1648" y1="976" y2="976" x1="1584" />
        </branch>
        <branch name="XLXN_142">
            <wire x2="1584" y1="1248" y2="1248" x1="1568" />
            <wire x2="1584" y1="1104" y2="1248" x1="1584" />
            <wire x2="1632" y1="1104" y2="1104" x1="1584" />
            <wire x2="1648" y1="1104" y2="1104" x1="1632" />
        </branch>
        <instance x="1312" y="768" name="XLXI_35" orien="R0" />
        <instance x="2256" y="1008" name="XLXI_37" orien="R0" />
        <branch name="XLXN_154">
            <wire x2="2688" y1="624" y2="624" x1="2512" />
            <wire x2="2688" y1="624" y2="896" x1="2688" />
        </branch>
        <branch name="XLXN_155">
            <wire x2="2592" y1="880" y2="880" x1="2512" />
            <wire x2="2592" y1="880" y2="960" x1="2592" />
            <wire x2="2688" y1="960" y2="960" x1="2592" />
        </branch>
        <branch name="q(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2208" y="592" type="branch" />
            <wire x2="2256" y1="592" y2="592" x1="2208" />
        </branch>
        <branch name="q(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2208" y="528" type="branch" />
            <wire x2="2256" y1="528" y2="528" x1="2208" />
        </branch>
        <branch name="U">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2208" y="656" type="branch" />
            <wire x2="2256" y1="656" y2="656" x1="2208" />
        </branch>
        <branch name="R">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2208" y="720" type="branch" />
            <wire x2="2256" y1="720" y2="720" x1="2208" />
        </branch>
        <branch name="q(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2208" y="816" type="branch" />
            <wire x2="2256" y1="816" y2="816" x1="2208" />
        </branch>
        <branch name="q(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2208" y="880" type="branch" />
            <wire x2="2256" y1="880" y2="880" x1="2208" />
        </branch>
        <branch name="U">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2208" y="944" type="branch" />
            <wire x2="2256" y1="944" y2="944" x1="2208" />
        </branch>
        <instance x="2256" y="784" name="XLXI_41" orien="R0" />
        <instance x="1648" y="1168" name="XLXI_44" orien="R0" />
        <branch name="D(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1952" y="1008" type="branch" />
            <wire x2="1952" y1="1008" y2="1008" x1="1904" />
        </branch>
        <instance x="2688" y="1088" name="XLXI_45" orien="R0" />
        <branch name="D(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2976" y="960" type="branch" />
            <wire x2="2976" y1="960" y2="960" x1="2944" />
        </branch>
        <branch name="XLXN_161">
            <wire x2="2688" y1="1120" y2="1120" x1="2512" />
            <wire x2="2688" y1="1024" y2="1040" x1="2688" />
            <wire x2="2688" y1="1040" y2="1120" x1="2688" />
        </branch>
        <instance x="2256" y="1248" name="XLXI_39" orien="R0" />
        <branch name="R">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2208" y="1056" type="branch" />
            <wire x2="2256" y1="1056" y2="1056" x1="2208" />
        </branch>
        <branch name="q(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2208" y="1120" type="branch" />
            <wire x2="2256" y1="1120" y2="1120" x1="2208" />
        </branch>
        <branch name="q(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2208" y="1184" type="branch" />
            <wire x2="2256" y1="1184" y2="1184" x1="2208" />
        </branch>
    </sheet>
</drawing>