<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="bf" />
        <signal name="af" />
        <signal name="S" />
        <signal name="A" />
        <signal name="ck" />
        <port polarity="Output" name="S" />
        <port polarity="Input" name="A" />
        <port polarity="Input" name="ck" />
        <blockdef name="fd">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <rect width="256" x="64" y="-320" height="256" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="320" y1="-256" y2="-256" x1="384" />
            <line x2="64" y1="-128" y2="-144" x1="80" />
            <line x2="80" y1="-112" y2="-128" x1="64" />
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
        <block symbolname="fd" name="XLXI_1">
            <blockpin signalname="ck" name="C" />
            <blockpin signalname="A" name="D" />
            <blockpin signalname="af" name="Q" />
        </block>
        <block symbolname="fd" name="XLXI_19">
            <blockpin signalname="ck" name="C" />
            <blockpin signalname="af" name="D" />
            <blockpin signalname="bf" name="Q" />
        </block>
        <block symbolname="and2b1" name="XLXI_17">
            <blockpin signalname="af" name="I0" />
            <blockpin signalname="bf" name="I1" />
            <blockpin signalname="S" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="1900" height="1344">
        <attr value="CM" name="LengthUnitName" />
        <attr value="4" name="GridsPerUnit" />
        <instance x="336" y="896" name="XLXI_1" orien="R0" />
        <branch name="bf">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="1232" y="416" type="branch" />
            <wire x2="1232" y1="640" y2="640" x1="1168" />
            <wire x2="1232" y1="416" y2="640" x1="1232" />
            <wire x2="1312" y1="416" y2="416" x1="1232" />
        </branch>
        <branch name="af">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="800" y="352" type="branch" />
            <wire x2="752" y1="640" y2="640" x1="720" />
            <wire x2="784" y1="640" y2="640" x1="752" />
            <wire x2="752" y1="352" y2="640" x1="752" />
            <wire x2="800" y1="352" y2="352" x1="752" />
            <wire x2="1312" y1="352" y2="352" x1="800" />
        </branch>
        <branch name="S">
            <wire x2="1584" y1="384" y2="384" x1="1568" />
            <wire x2="1680" y1="384" y2="384" x1="1584" />
        </branch>
        <branch name="A">
            <wire x2="336" y1="640" y2="640" x1="224" />
        </branch>
        <instance x="784" y="896" name="XLXI_19" orien="R0" />
        <branch name="ck">
            <wire x2="336" y1="992" y2="992" x1="240" />
            <wire x2="784" y1="992" y2="992" x1="336" />
            <wire x2="336" y1="768" y2="992" x1="336" />
            <wire x2="784" y1="768" y2="992" x1="784" />
        </branch>
        <instance x="1312" y="288" name="XLXI_17" orien="M180" />
        <iomarker fontsize="28" x="240" y="992" name="ck" orien="R180" />
        <iomarker fontsize="28" x="224" y="640" name="A" orien="R180" />
        <iomarker fontsize="28" x="1680" y="384" name="S" orien="R0" />
    </sheet>
</drawing>