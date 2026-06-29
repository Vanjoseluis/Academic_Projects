<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="Seven_Seg3(7:0)" />
        <signal name="Seven_Seg2(7:0)" />
        <signal name="Seven_Seg1(7:0)" />
        <signal name="Seven_Seg0(7:0)" />
        <signal name="Op" />
        <signal name="Clos" />
        <signal name="XLXN_1" />
        <signal name="Clr" />
        <signal name="Ck" />
        <signal name="D" />
        <signal name="Cero" />
        <signal name="XLXN_56" />
        <signal name="Out0" />
        <signal name="XLXN_58" />
        <signal name="Out1" />
        <signal name="XLXN_60" />
        <signal name="Out2" />
        <signal name="Out3" />
        <signal name="XLXN_64" />
        <signal name="XLXN_65" />
        <port polarity="Output" name="Seven_Seg3(7:0)" />
        <port polarity="Output" name="Seven_Seg2(7:0)" />
        <port polarity="Output" name="Seven_Seg1(7:0)" />
        <port polarity="Output" name="Seven_Seg0(7:0)" />
        <port polarity="Input" name="Clr" />
        <port polarity="Input" name="Ck" />
        <port polarity="Input" name="D" />
        <port polarity="Output" name="Out0" />
        <port polarity="Output" name="Out1" />
        <port polarity="Output" name="Out2" />
        <port polarity="Output" name="Out3" />
        <blockdef name="sr4cled">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <rect width="256" x="64" y="-768" height="704" />
            <line x2="64" y1="-320" y2="-320" x1="0" />
            <line x2="320" y1="-448" y2="-448" x1="384" />
            <line x2="320" y1="-512" y2="-512" x1="384" />
            <line x2="320" y1="-576" y2="-576" x1="384" />
            <line x2="320" y1="-640" y2="-640" x1="384" />
            <line x2="64" y1="-448" y2="-448" x1="0" />
            <line x2="64" y1="-512" y2="-512" x1="0" />
            <line x2="64" y1="-576" y2="-576" x1="0" />
            <line x2="64" y1="-640" y2="-640" x1="0" />
            <line x2="64" y1="-704" y2="-704" x1="0" />
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="64" y1="-32" y2="-32" x1="192" />
            <line x2="192" y1="-64" y2="-32" x1="192" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-128" y2="-144" x1="80" />
            <line x2="80" y1="-112" y2="-128" x1="64" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="64" y1="-384" y2="-384" x1="0" />
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
        <blockdef name="gnd">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-96" x1="64" />
            <line x2="52" y1="-48" y2="-48" x1="76" />
            <line x2="60" y1="-32" y2="-32" x1="68" />
            <line x2="40" y1="-64" y2="-64" x1="88" />
            <line x2="64" y1="-64" y2="-80" x1="64" />
            <line x2="64" y1="-128" y2="-96" x1="64" />
        </blockdef>
        <blockdef name="and4">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-112" y2="-112" x1="144" />
            <arc ex="144" ey="-208" sx="144" sy="-112" r="48" cx="144" cy="-160" />
            <line x2="144" y1="-208" y2="-208" x1="64" />
            <line x2="64" y1="-64" y2="-256" x1="64" />
            <line x2="192" y1="-160" y2="-160" x1="256" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-64" y2="-64" x1="0" />
        </blockdef>
        <blockdef name="nor4">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="48" y1="-64" y2="-64" x1="0" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="48" y1="-256" y2="-256" x1="0" />
            <line x2="216" y1="-160" y2="-160" x1="256" />
            <circle r="12" cx="204" cy="-160" />
            <line x2="48" y1="-208" y2="-208" x1="112" />
            <arc ex="112" ey="-208" sx="192" sy="-160" r="88" cx="116" cy="-120" />
            <line x2="48" y1="-112" y2="-112" x1="112" />
            <line x2="48" y1="-256" y2="-208" x1="48" />
            <line x2="48" y1="-64" y2="-112" x1="48" />
            <arc ex="48" ey="-208" sx="48" sy="-112" r="56" cx="16" cy="-160" />
            <arc ex="192" ey="-160" sx="112" sy="-112" r="88" cx="116" cy="-200" />
        </blockdef>
        <blockdef name="Ver_Op_Clos">
            <timestamp>2021-1-23T19:43:57</timestamp>
            <rect width="256" x="64" y="-256" height="256" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <line x2="384" y1="-32" y2="-32" x1="320" />
            <rect width="64" x="320" y="-44" height="24" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
            <rect width="64" x="320" y="-108" height="24" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
            <rect width="64" x="320" y="-172" height="24" />
            <line x2="384" y1="-224" y2="-224" x1="320" />
            <rect width="64" x="320" y="-236" height="24" />
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
        <block symbolname="Ver_Op_Clos" name="XLXI_10">
            <blockpin signalname="Clos" name="Close" />
            <blockpin signalname="Op" name="Op" />
            <blockpin signalname="Seven_Seg0(7:0)" name="Seven_Seg0(7:0)" />
            <blockpin signalname="Seven_Seg1(7:0)" name="Seven_Seg1(7:0)" />
            <blockpin signalname="Seven_Seg2(7:0)" name="Seven_Seg2(7:0)" />
            <blockpin signalname="Seven_Seg3(7:0)" name="Seven_Seg3(7:0)" />
        </block>
        <block symbolname="and2" name="XLXI_13">
            <blockpin signalname="XLXN_64" name="I0" />
            <blockpin signalname="Clr" name="I1" />
            <blockpin signalname="Op" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_14">
            <blockpin signalname="Clr" name="I0" />
            <blockpin signalname="XLXN_65" name="I1" />
            <blockpin signalname="Clos" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_2">
            <blockpin signalname="Clr" name="I" />
            <blockpin signalname="XLXN_1" name="O" />
        </block>
        <block symbolname="sr4cled" name="XLXI_1">
            <blockpin signalname="Ck" name="C" />
            <blockpin signalname="Clr" name="CE" />
            <blockpin signalname="XLXN_1" name="CLR" />
            <blockpin signalname="Cero" name="D0" />
            <blockpin signalname="Cero" name="D1" />
            <blockpin signalname="Cero" name="D2" />
            <blockpin signalname="Cero" name="D3" />
            <blockpin signalname="Cero" name="L" />
            <blockpin signalname="D" name="LEFT" />
            <blockpin signalname="Clr" name="SLI" />
            <blockpin signalname="Cero" name="SRI" />
            <blockpin signalname="Out0" name="Q0" />
            <blockpin signalname="Out1" name="Q1" />
            <blockpin signalname="Out2" name="Q2" />
            <blockpin signalname="Out3" name="Q3" />
        </block>
        <block symbolname="gnd" name="XLXI_5">
            <blockpin signalname="Cero" name="G" />
        </block>
        <block symbolname="nor4" name="XLXI_11">
            <blockpin signalname="Out3" name="I0" />
            <blockpin signalname="Out2" name="I1" />
            <blockpin signalname="Out1" name="I2" />
            <blockpin signalname="Out0" name="I3" />
            <blockpin signalname="XLXN_64" name="O" />
        </block>
        <block symbolname="and4" name="XLXI_12">
            <blockpin signalname="Out3" name="I0" />
            <blockpin signalname="Out2" name="I1" />
            <blockpin signalname="Out1" name="I2" />
            <blockpin signalname="Out0" name="I3" />
            <blockpin signalname="XLXN_65" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <branch name="Seven_Seg3(7:0)">
            <wire x2="3024" y1="528" y2="528" x1="2896" />
        </branch>
        <branch name="Seven_Seg2(7:0)">
            <wire x2="3040" y1="592" y2="592" x1="2896" />
        </branch>
        <branch name="Seven_Seg1(7:0)">
            <wire x2="3056" y1="656" y2="656" x1="2896" />
        </branch>
        <branch name="Seven_Seg0(7:0)">
            <wire x2="3056" y1="720" y2="720" x1="2896" />
        </branch>
        <iomarker fontsize="28" x="3024" y="528" name="Seven_Seg3(7:0)" orien="R0" />
        <iomarker fontsize="28" x="3040" y="592" name="Seven_Seg2(7:0)" orien="R0" />
        <iomarker fontsize="28" x="3056" y="656" name="Seven_Seg1(7:0)" orien="R0" />
        <iomarker fontsize="28" x="3056" y="720" name="Seven_Seg0(7:0)" orien="R0" />
        <instance x="2512" y="752" name="XLXI_10" orien="R0">
        </instance>
        <branch name="Op">
            <attrtext style="alignment:SOFT-TVCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2400" y="416" type="branch" />
            <wire x2="2400" y1="384" y2="384" x1="2384" />
            <wire x2="2400" y1="384" y2="416" x1="2400" />
            <wire x2="2400" y1="416" y2="528" x1="2400" />
            <wire x2="2512" y1="528" y2="528" x1="2400" />
        </branch>
        <branch name="Clos">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2400" y="592" type="branch" />
            <wire x2="2400" y1="784" y2="784" x1="2384" />
            <wire x2="2512" y1="592" y2="592" x1="2400" />
            <wire x2="2400" y1="592" y2="752" x1="2400" />
            <wire x2="2400" y1="752" y2="784" x1="2400" />
        </branch>
        <instance x="576" y="1072" name="XLXI_2" orien="R0" />
        <branch name="XLXN_1">
            <wire x2="848" y1="1040" y2="1040" x1="800" />
        </branch>
        <branch name="Clr">
            <wire x2="576" y1="1040" y2="1040" x1="512" />
        </branch>
        <branch name="Ck">
            <wire x2="848" y1="944" y2="944" x1="512" />
        </branch>
        <branch name="D">
            <wire x2="848" y1="816" y2="816" x1="528" />
        </branch>
        <instance x="848" y="1072" name="XLXI_1" orien="R0" />
        <branch name="Cero">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="256" y="432" type="branch" />
            <wire x2="848" y1="432" y2="432" x1="256" />
            <wire x2="256" y1="432" y2="496" x1="256" />
            <wire x2="848" y1="496" y2="496" x1="256" />
            <wire x2="256" y1="496" y2="560" x1="256" />
            <wire x2="848" y1="560" y2="560" x1="256" />
            <wire x2="256" y1="560" y2="624" x1="256" />
            <wire x2="848" y1="624" y2="624" x1="256" />
            <wire x2="256" y1="624" y2="688" x1="256" />
            <wire x2="848" y1="688" y2="688" x1="256" />
            <wire x2="256" y1="688" y2="752" x1="256" />
            <wire x2="256" y1="752" y2="800" x1="256" />
            <wire x2="848" y1="752" y2="752" x1="256" />
        </branch>
        <instance x="192" y="928" name="XLXI_5" orien="R0" />
        <instance x="1840" y="576" name="XLXI_11" orien="R0" />
        <instance x="1840" y="912" name="XLXI_12" orien="R0" />
        <branch name="Out0">
            <wire x2="1536" y1="432" y2="432" x1="1232" />
            <wire x2="1536" y1="432" y2="656" x1="1536" />
            <wire x2="1840" y1="656" y2="656" x1="1536" />
            <wire x2="1536" y1="192" y2="320" x1="1536" />
            <wire x2="1536" y1="320" y2="432" x1="1536" />
            <wire x2="1840" y1="320" y2="320" x1="1536" />
            <wire x2="1616" y1="192" y2="192" x1="1536" />
        </branch>
        <branch name="Out1">
            <wire x2="1552" y1="496" y2="496" x1="1232" />
            <wire x2="1552" y1="496" y2="720" x1="1552" />
            <wire x2="1840" y1="720" y2="720" x1="1552" />
            <wire x2="1552" y1="240" y2="384" x1="1552" />
            <wire x2="1552" y1="384" y2="496" x1="1552" />
            <wire x2="1840" y1="384" y2="384" x1="1552" />
            <wire x2="1616" y1="240" y2="240" x1="1552" />
        </branch>
        <branch name="Out2">
            <wire x2="1568" y1="560" y2="560" x1="1232" />
            <wire x2="1568" y1="560" y2="784" x1="1568" />
            <wire x2="1840" y1="784" y2="784" x1="1568" />
            <wire x2="1568" y1="288" y2="448" x1="1568" />
            <wire x2="1568" y1="448" y2="560" x1="1568" />
            <wire x2="1840" y1="448" y2="448" x1="1568" />
            <wire x2="1616" y1="288" y2="288" x1="1568" />
        </branch>
        <branch name="Out3">
            <wire x2="1600" y1="624" y2="624" x1="1232" />
            <wire x2="1840" y1="624" y2="624" x1="1600" />
            <wire x2="1600" y1="624" y2="848" x1="1600" />
            <wire x2="1840" y1="848" y2="848" x1="1600" />
            <wire x2="1600" y1="848" y2="928" x1="1600" />
            <wire x2="1680" y1="928" y2="928" x1="1600" />
            <wire x2="1840" y1="512" y2="624" x1="1840" />
        </branch>
        <branch name="Clr">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="352" y="368" type="branch" />
            <wire x2="656" y1="368" y2="368" x1="352" />
            <wire x2="848" y1="368" y2="368" x1="656" />
            <wire x2="656" y1="368" y2="880" x1="656" />
            <wire x2="848" y1="880" y2="880" x1="656" />
        </branch>
        <iomarker fontsize="28" x="512" y="1040" name="Clr" orien="R180" />
        <iomarker fontsize="28" x="512" y="944" name="Ck" orien="R180" />
        <iomarker fontsize="28" x="528" y="816" name="D" orien="R180" />
        <iomarker fontsize="28" x="1616" y="192" name="Out0" orien="R0" />
        <iomarker fontsize="28" x="1616" y="240" name="Out1" orien="R0" />
        <iomarker fontsize="28" x="1616" y="288" name="Out2" orien="R0" />
        <iomarker fontsize="28" x="1680" y="928" name="Out3" orien="R0" />
        <instance x="2128" y="480" name="XLXI_13" orien="R0" />
        <instance x="2128" y="880" name="XLXI_14" orien="R0" />
        <branch name="XLXN_64">
            <wire x2="2112" y1="416" y2="416" x1="2096" />
            <wire x2="2128" y1="416" y2="416" x1="2112" />
        </branch>
        <branch name="XLXN_65">
            <wire x2="2112" y1="752" y2="752" x1="2096" />
            <wire x2="2128" y1="752" y2="752" x1="2112" />
        </branch>
        <branch name="Clr">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2096" y="816" type="branch" />
            <wire x2="2128" y1="816" y2="816" x1="2096" />
        </branch>
        <branch name="Clr">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2096" y="352" type="branch" />
            <wire x2="2128" y1="352" y2="352" x1="2096" />
        </branch>
    </sheet>
</drawing>