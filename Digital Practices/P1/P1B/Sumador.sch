<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="A" />
        <signal name="B" />
        <signal name="XLXN_6" />
        <signal name="XLXN_7" />
        <signal name="XLXN_8" />
        <signal name="S" />
        <signal name="C1" />
        <signal name="C0" />
        <port polarity="Input" name="A" />
        <port polarity="Input" name="B" />
        <port polarity="Output" name="S" />
        <port polarity="Output" name="C1" />
        <port polarity="Input" name="C0" />
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
        <block symbolname="xor2" name="XLXI_1">
            <blockpin signalname="B" name="I0" />
            <blockpin signalname="A" name="I1" />
            <blockpin signalname="XLXN_8" name="O" />
        </block>
        <block symbolname="xor2" name="XLXI_2">
            <blockpin signalname="C0" name="I0" />
            <blockpin signalname="XLXN_8" name="I1" />
            <blockpin signalname="S" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_3">
            <blockpin signalname="B" name="I0" />
            <blockpin signalname="A" name="I1" />
            <blockpin signalname="XLXN_6" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_4">
            <blockpin signalname="C0" name="I0" />
            <blockpin signalname="XLXN_8" name="I1" />
            <blockpin signalname="XLXN_7" name="O" />
        </block>
        <block symbolname="or2" name="XLXI_5">
            <blockpin signalname="XLXN_6" name="I0" />
            <blockpin signalname="XLXN_7" name="I1" />
            <blockpin signalname="C1" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <instance x="1824" y="976" name="XLXI_2" orien="R0" />
        <instance x="1040" y="1488" name="XLXI_3" orien="R0" />
        <instance x="2064" y="1232" name="XLXI_5" orien="R0" />
        <branch name="A">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="752" y="816" type="branch" />
            <wire x2="928" y1="816" y2="816" x1="752" />
        </branch>
        <branch name="B">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="768" y="880" type="branch" />
            <wire x2="928" y1="880" y2="880" x1="768" />
        </branch>
        <branch name="A">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="784" y="1360" type="branch" />
            <wire x2="1040" y1="1360" y2="1360" x1="784" />
        </branch>
        <branch name="B">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="784" y="1424" type="branch" />
            <wire x2="1040" y1="1424" y2="1424" x1="784" />
        </branch>
        <branch name="XLXN_6">
            <wire x2="1760" y1="1392" y2="1392" x1="1296" />
            <wire x2="1760" y1="1168" y2="1392" x1="1760" />
            <wire x2="2064" y1="1168" y2="1168" x1="1760" />
        </branch>
        <branch name="XLXN_7">
            <wire x2="2064" y1="1104" y2="1104" x1="1744" />
        </branch>
        <branch name="XLXN_8">
            <wire x2="1504" y1="848" y2="848" x1="1184" />
            <wire x2="1824" y1="848" y2="848" x1="1504" />
            <wire x2="1504" y1="848" y2="992" x1="1504" />
            <wire x2="1408" y1="992" y2="1072" x1="1408" />
            <wire x2="1488" y1="1072" y2="1072" x1="1408" />
            <wire x2="1504" y1="992" y2="992" x1="1408" />
        </branch>
        <branch name="C0">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1248" y="1136" type="branch" />
            <wire x2="1488" y1="1136" y2="1136" x1="1248" />
        </branch>
        <branch name="C0">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1664" y="912" type="branch" />
            <wire x2="1824" y1="912" y2="912" x1="1664" />
        </branch>
        <branch name="S">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2256" y="880" type="branch" />
            <wire x2="2256" y1="880" y2="880" x1="2080" />
        </branch>
        <branch name="C1">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2416" y="1136" type="branch" />
            <wire x2="2416" y1="1136" y2="1136" x1="2320" />
        </branch>
        <instance x="1488" y="1200" name="XLXI_4" orien="R0" />
        <instance x="928" y="944" name="XLXI_1" orien="R0" />
        <branch name="A">
            <wire x2="880" y1="400" y2="480" x1="880" />
        </branch>
        <branch name="B">
            <wire x2="912" y1="400" y2="480" x1="912" />
        </branch>
        <branch name="C0">
            <wire x2="944" y1="400" y2="480" x1="944" />
        </branch>
        <branch name="C1">
            <wire x2="976" y1="400" y2="480" x1="976" />
        </branch>
        <branch name="S">
            <wire x2="1008" y1="400" y2="480" x1="1008" />
        </branch>
        <iomarker fontsize="28" x="880" y="400" name="A" orien="R270" />
        <iomarker fontsize="28" x="912" y="400" name="B" orien="R270" />
        <iomarker fontsize="28" x="944" y="400" name="C0" orien="R270" />
        <iomarker fontsize="28" x="976" y="400" name="C1" orien="R270" />
        <iomarker fontsize="28" x="1008" y="400" name="S" orien="R270" />
    </sheet>
</drawing>