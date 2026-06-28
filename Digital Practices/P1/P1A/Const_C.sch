<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="C(3)" />
        <signal name="C(2)" />
        <signal name="C(1)" />
        <signal name="C(0)" />
        <signal name="C(3:0)" />
        <port polarity="Output" name="C(3:0)" />
        <blockdef name="vcc">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-64" x1="64" />
            <line x2="64" y1="0" y2="-32" x1="64" />
            <line x2="32" y1="-64" y2="-64" x1="96" />
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
        <block symbolname="vcc" name="XLXI_7">
            <blockpin signalname="C(2)" name="P" />
        </block>
        <block symbolname="vcc" name="XLXI_10">
            <blockpin signalname="C(3)" name="P" />
        </block>
        <block symbolname="gnd" name="XLXI_11">
            <blockpin signalname="C(1)" name="G" />
        </block>
        <block symbolname="gnd" name="XLXI_12">
            <blockpin signalname="C(0)" name="G" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="1760" height="1360">
        <instance x="480" y="608" name="XLXI_7" orien="R0" />
        <instance x="320" y="608" name="XLXI_10" orien="R0" />
        <branch name="C(3)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="384" y="752" type="branch" />
            <wire x2="384" y1="608" y2="752" x1="384" />
        </branch>
        <branch name="C(2)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="544" y="752" type="branch" />
            <wire x2="544" y1="608" y2="752" x1="544" />
        </branch>
        <branch name="C(1)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="704" y="752" type="branch" />
            <wire x2="704" y1="608" y2="752" x1="704" />
        </branch>
        <branch name="C(0)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="864" y="752" type="branch" />
            <wire x2="864" y1="608" y2="752" x1="864" />
        </branch>
        <branch name="C(3:0)">
            <wire x2="960" y1="928" y2="928" x1="384" />
        </branch>
        <iomarker fontsize="28" x="960" y="928" name="C(3:0)" orien="R0" />
        <instance x="768" y="480" name="XLXI_11" orien="R180" />
        <instance x="928" y="480" name="XLXI_12" orien="R180" />
    </sheet>
</drawing>