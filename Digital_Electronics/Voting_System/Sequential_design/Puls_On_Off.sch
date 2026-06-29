<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="ck" />
        <signal name="XLXN_16" />
        <signal name="EN" />
        <signal name="XLXN_6" />
        <signal name="ENS" />
        <signal name="XLXN_26" />
        <signal name="XLXN_27" />
        <signal name="XLXN_33" />
        <port polarity="Input" name="ck" />
        <port polarity="Input" name="EN" />
        <port polarity="Output" name="ENS" />
        <blockdef name="fd">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <rect width="256" x="64" y="-320" height="256" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="320" y1="-256" y2="-256" x1="384" />
            <line x2="64" y1="-128" y2="-144" x1="80" />
            <line x2="80" y1="-112" y2="-128" x1="64" />
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
        <blockdef name="inv">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="160" y1="-32" y2="-32" x1="224" />
            <line x2="128" y1="-64" y2="-32" x1="64" />
            <line x2="64" y1="-32" y2="0" x1="128" />
            <line x2="64" y1="0" y2="-64" x1="64" />
            <circle r="16" cx="144" cy="-32" />
        </blockdef>
        <block symbolname="and2" name="XLXI_7">
            <blockpin signalname="XLXN_27" name="I0" />
            <blockpin signalname="XLXN_26" name="I1" />
            <blockpin signalname="XLXN_33" name="O" />
        </block>
        <block symbolname="fd" name="XLXI_1">
            <blockpin signalname="ck" name="C" />
            <blockpin signalname="EN" name="D" />
            <blockpin signalname="XLXN_16" name="Q" />
        </block>
        <block symbolname="inv" name="XLXI_12">
            <blockpin signalname="XLXN_16" name="I" />
            <blockpin signalname="XLXN_26" name="O" />
        </block>
        <block symbolname="fd" name="XLXI_2">
            <blockpin signalname="ck" name="C" />
            <blockpin signalname="XLXN_6" name="D" />
            <blockpin signalname="ENS" name="Q" />
        </block>
        <block symbolname="xor2" name="XLXI_4">
            <blockpin signalname="XLXN_33" name="I0" />
            <blockpin signalname="ENS" name="I1" />
            <blockpin signalname="XLXN_6" name="O" />
        </block>
        <block symbolname="fd" name="XLXI_13">
            <blockpin signalname="ck" name="C" />
            <blockpin signalname="XLXN_16" name="D" />
            <blockpin signalname="XLXN_27" name="Q" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="2688" height="1900">
        <attr value="CM" name="LengthUnitName" />
        <attr value="4" name="GridsPerUnit" />
        <branch name="ck">
            <wire x2="256" y1="832" y2="832" x1="160" />
        </branch>
        <instance x="256" y="960" name="XLXI_1" orien="R0" />
        <branch name="EN">
            <wire x2="256" y1="704" y2="704" x1="160" />
        </branch>
        <iomarker fontsize="28" x="160" y="704" name="EN" orien="R180" />
        <iomarker fontsize="28" x="160" y="832" name="ck" orien="R180" />
        <instance x="1872" y="1456" name="XLXI_2" orien="R0" />
        <branch name="ck">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1776" y="1328" type="branch" />
            <wire x2="1872" y1="1328" y2="1328" x1="1776" />
        </branch>
        <instance x="1568" y="1296" name="XLXI_4" orien="R0" />
        <branch name="XLXN_6">
            <wire x2="1872" y1="1200" y2="1200" x1="1824" />
        </branch>
        <branch name="ENS">
            <wire x2="2336" y1="1040" y2="1040" x1="1552" />
            <wire x2="2336" y1="1040" y2="1200" x1="2336" />
            <wire x2="2448" y1="1200" y2="1200" x1="2336" />
            <wire x2="1552" y1="1040" y2="1168" x1="1552" />
            <wire x2="1568" y1="1168" y2="1168" x1="1552" />
            <wire x2="2336" y1="1200" y2="1200" x1="2256" />
        </branch>
        <instance x="864" y="512" name="XLXI_12" orien="R0" />
        <branch name="XLXN_16">
            <wire x2="704" y1="704" y2="704" x1="640" />
            <wire x2="784" y1="704" y2="704" x1="704" />
            <wire x2="864" y1="480" y2="480" x1="704" />
            <wire x2="704" y1="480" y2="704" x1="704" />
        </branch>
        <branch name="XLXN_26">
            <wire x2="1184" y1="480" y2="480" x1="1088" />
            <wire x2="1184" y1="480" y2="640" x1="1184" />
            <wire x2="1232" y1="640" y2="640" x1="1184" />
        </branch>
        <branch name="XLXN_27">
            <wire x2="1232" y1="704" y2="704" x1="1168" />
        </branch>
        <iomarker fontsize="28" x="2448" y="1200" name="ENS" orien="R0" />
        <instance x="1232" y="768" name="XLXI_7" orien="R0" />
        <branch name="ck">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="672" y="832" type="branch" />
            <wire x2="688" y1="832" y2="832" x1="672" />
            <wire x2="784" y1="832" y2="832" x1="688" />
        </branch>
        <instance x="784" y="960" name="XLXI_13" orien="R0" />
        <branch name="XLXN_33">
            <wire x2="1520" y1="672" y2="672" x1="1488" />
            <wire x2="1520" y1="672" y2="1232" x1="1520" />
            <wire x2="1568" y1="1232" y2="1232" x1="1520" />
        </branch>
    </sheet>
</drawing>