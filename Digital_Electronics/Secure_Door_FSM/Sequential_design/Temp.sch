<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="CLR" />
        <signal name="Ck" />
        <signal name="FC" />
        <signal name="EST(2:0)" />
        <signal name="Q(7:0)" />
        <signal name="XLXN_18" />
        <signal name="XLXN_7" />
        <signal name="EST(1)" />
        <signal name="EST(0)" />
        <signal name="XLXN_25" />
        <signal name="EST(2)" />
        <port polarity="Input" name="CLR" />
        <port polarity="Input" name="Ck" />
        <port polarity="Output" name="FC" />
        <port polarity="Input" name="EST(2:0)" />
        <port polarity="Output" name="Q(7:0)" />
        <blockdef name="cc8re">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <rect width="256" x="64" y="-320" height="256" />
            <line x2="320" y1="-192" y2="-192" x1="384" />
            <rect width="64" x="320" y="-268" height="24" />
            <line x2="320" y1="-256" y2="-256" x1="384" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-128" y2="-144" x1="80" />
            <line x2="80" y1="-112" y2="-128" x1="64" />
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="64" y1="-32" y2="-32" x1="192" />
            <line x2="192" y1="-64" y2="-32" x1="192" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="320" y1="-128" y2="-128" x1="384" />
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
        <blockdef name="m2_1">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="96" y1="-64" y2="-192" x1="96" />
            <line x2="96" y1="-96" y2="-64" x1="256" />
            <line x2="256" y1="-160" y2="-96" x1="256" />
            <line x2="256" y1="-192" y2="-160" x1="96" />
            <line x2="96" y1="-32" y2="-32" x1="176" />
            <line x2="176" y1="-80" y2="-32" x1="176" />
            <line x2="96" y1="-32" y2="-32" x1="0" />
            <line x2="256" y1="-128" y2="-128" x1="320" />
            <line x2="96" y1="-96" y2="-96" x1="0" />
            <line x2="96" y1="-160" y2="-160" x1="0" />
        </blockdef>
        <block symbolname="cc8re" name="XLXI_1">
            <blockpin signalname="Ck" name="C" />
            <blockpin signalname="XLXN_7" name="CE" />
            <blockpin signalname="XLXN_18" name="R" />
            <blockpin name="CEO" />
            <blockpin signalname="Q(7:0)" name="Q(7:0)" />
            <blockpin signalname="FC" name="TC" />
        </block>
        <block symbolname="inv" name="XLXI_7">
            <blockpin signalname="CLR" name="I" />
            <blockpin signalname="XLXN_18" name="O" />
        </block>
        <block symbolname="m2_1" name="XLXI_9">
            <blockpin signalname="EST(1)" name="D0" />
            <blockpin signalname="XLXN_25" name="D1" />
            <blockpin signalname="EST(0)" name="S0" />
            <blockpin signalname="XLXN_7" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_10">
            <blockpin signalname="EST(2)" name="I" />
            <blockpin signalname="XLXN_25" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="1760" height="1360">
        <branch name="EST(2:0)">
            <wire x2="416" y1="80" y2="80" x1="240" />
        </branch>
        <branch name="Ck">
            <wire x2="416" y1="128" y2="128" x1="240" />
        </branch>
        <branch name="CLR">
            <wire x2="416" y1="192" y2="192" x1="240" />
        </branch>
        <branch name="FC">
            <wire x2="720" y1="128" y2="128" x1="560" />
        </branch>
        <iomarker fontsize="28" x="720" y="128" name="FC" orien="R0" />
        <iomarker fontsize="28" x="240" y="80" name="EST(2:0)" orien="R180" />
        <iomarker fontsize="28" x="240" y="128" name="Ck" orien="R180" />
        <iomarker fontsize="28" x="240" y="192" name="CLR" orien="R180" />
        <branch name="Q(7:0)">
            <wire x2="720" y1="192" y2="192" x1="560" />
        </branch>
        <iomarker fontsize="28" x="720" y="192" name="Q(7:0)" orien="R0" />
        <branch name="Ck">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="816" y="672" type="branch" />
            <wire x2="976" y1="672" y2="672" x1="816" />
        </branch>
        <branch name="FC">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1504" y="672" type="branch" />
            <wire x2="1504" y1="672" y2="672" x1="1360" />
        </branch>
        <instance x="976" y="800" name="XLXI_1" orien="R0" />
        <instance x="624" y="800" name="XLXI_7" orien="R0" />
        <branch name="XLXN_18">
            <wire x2="976" y1="768" y2="768" x1="848" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="768" type="branch" />
            <wire x2="624" y1="768" y2="768" x1="496" />
        </branch>
        <branch name="Q(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1472" y="544" type="branch" />
            <wire x2="1472" y1="544" y2="544" x1="1360" />
        </branch>
        <branch name="XLXN_7">
            <wire x2="928" y1="480" y2="480" x1="880" />
            <wire x2="928" y1="480" y2="608" x1="928" />
            <wire x2="976" y1="608" y2="608" x1="928" />
        </branch>
        <instance x="560" y="608" name="XLXI_9" orien="R0" />
        <branch name="EST(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="480" y="448" type="branch" />
            <wire x2="560" y1="448" y2="448" x1="480" />
        </branch>
        <branch name="EST(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="480" y="576" type="branch" />
            <wire x2="560" y1="576" y2="576" x1="480" />
        </branch>
        <instance x="288" y="544" name="XLXI_10" orien="R0" />
        <branch name="XLXN_25">
            <wire x2="560" y1="512" y2="512" x1="512" />
        </branch>
        <branch name="EST(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="208" y="512" type="branch" />
            <wire x2="288" y1="512" y2="512" x1="208" />
        </branch>
    </sheet>
</drawing>