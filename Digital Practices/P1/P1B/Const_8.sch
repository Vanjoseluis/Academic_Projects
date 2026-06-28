<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="Const_8(3)" />
        <signal name="Const_8(2)" />
        <signal name="Const_8(1)" />
        <signal name="Const_8(0)" />
        <signal name="Const_8(3:0)" />
        <port polarity="Output" name="Const_8(3:0)" />
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
        <block symbolname="vcc" name="XLXI_10">
            <blockpin signalname="Const_8(3)" name="P" />
        </block>
        <block symbolname="gnd" name="XLXI_11">
            <blockpin signalname="Const_8(2)" name="G" />
        </block>
        <block symbolname="gnd" name="XLXI_12">
            <blockpin signalname="Const_8(1)" name="G" />
        </block>
        <block symbolname="gnd" name="XLXI_13">
            <blockpin signalname="Const_8(0)" name="G" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="1760" height="1360">
        <instance x="432" y="576" name="XLXI_10" orien="R0" />
        <branch name="Const_8(3)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="720" type="branch" />
            <wire x2="496" y1="576" y2="720" x1="496" />
        </branch>
        <branch name="Const_8(2)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="656" y="720" type="branch" />
            <wire x2="656" y1="576" y2="720" x1="656" />
        </branch>
        <branch name="Const_8(1)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="816" y="720" type="branch" />
            <wire x2="816" y1="576" y2="720" x1="816" />
        </branch>
        <branch name="Const_8(0)">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="976" y="720" type="branch" />
            <wire x2="976" y1="576" y2="720" x1="976" />
        </branch>
        <instance x="720" y="448" name="XLXI_11" orien="R180" />
        <instance x="880" y="448" name="XLXI_12" orien="R180" />
        <instance x="1040" y="448" name="XLXI_13" orien="R180" />
        <iomarker fontsize="28" x="1072" y="912" name="Const_8(3:0)" orien="R0" />
        <branch name="Const_8(3:0)">
            <wire x2="1072" y1="912" y2="912" x1="496" />
        </branch>
    </sheet>
</drawing>