<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="F(2)" />
        <signal name="F(3)" />
        <signal name="F(1)" />
        <signal name="F(0)" />
        <signal name="F(3:0)" />
        <port polarity="Output" name="F(3:0)" />
        <blockdef name="vcc">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-64" x1="64" />
            <line x2="64" y1="0" y2="-32" x1="64" />
            <line x2="32" y1="-64" y2="-64" x1="96" />
        </blockdef>
        <block symbolname="vcc" name="XLXI_7">
            <blockpin signalname="F(2)" name="P" />
        </block>
        <block symbolname="vcc" name="XLXI_8">
            <blockpin signalname="F(1)" name="P" />
        </block>
        <block symbolname="vcc" name="XLXI_9">
            <blockpin signalname="F(0)" name="P" />
        </block>
        <block symbolname="vcc" name="XLXI_10">
            <blockpin signalname="F(3)" name="P" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="1760" height="1360">
        <attr value="Inch" name="LengthUnitName" />
        <attr value="10" name="GridsPerUnit" />
        <instance x="416" y="704" name="XLXI_7" orien="R0" />
        <instance x="576" y="704" name="XLXI_8" orien="R0" />
        <instance x="736" y="704" name="XLXI_9" orien="R0" />
        <instance x="256" y="704" name="XLXI_10" orien="R0" />
        <branch name="F(3)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="320" y="848" type="branch" />
            <wire x2="320" y1="704" y2="848" x1="320" />
        </branch>
        <branch name="F(2)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="480" y="848" type="branch" />
            <wire x2="480" y1="704" y2="848" x1="480" />
        </branch>
        <branch name="F(1)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="640" y="848" type="branch" />
            <wire x2="640" y1="704" y2="848" x1="640" />
        </branch>
        <branch name="F(0)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="800" y="848" type="branch" />
            <wire x2="800" y1="704" y2="848" x1="800" />
        </branch>
        <branch name="F(3:0)">
            <wire x2="896" y1="1024" y2="1024" x1="320" />
        </branch>
        <iomarker fontsize="28" x="896" y="1024" name="F(3:0)" orien="R0" />
    </sheet>
</drawing>