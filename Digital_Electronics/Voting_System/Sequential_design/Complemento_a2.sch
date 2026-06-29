<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="A0" />
        <signal name="A1" />
        <signal name="A2" />
        <signal name="A3" />
        <signal name="XLXN_21" />
        <signal name="XLXN_22" />
        <signal name="S0" />
        <signal name="S1" />
        <signal name="S2" />
        <signal name="S3" />
        <signal name="S4" />
        <signal name="D0" />
        <signal name="D1" />
        <signal name="D3" />
        <signal name="D2" />
        <port polarity="Output" name="S0" />
        <port polarity="Output" name="S1" />
        <port polarity="Output" name="S2" />
        <port polarity="Output" name="S3" />
        <port polarity="Output" name="S4" />
        <port polarity="Input" name="D0" />
        <port polarity="Input" name="D1" />
        <port polarity="Input" name="D3" />
        <port polarity="Input" name="D2" />
        <blockdef name="inv4">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="160" y1="-32" y2="-32" x1="224" />
            <line x2="160" y1="-96" y2="-96" x1="224" />
            <line x2="160" y1="-160" y2="-160" x1="224" />
            <line x2="160" y1="-224" y2="-224" x1="224" />
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="64" y1="-96" y2="-96" x1="0" />
            <line x2="64" y1="-160" y2="-160" x1="0" />
            <line x2="64" y1="-224" y2="-224" x1="0" />
            <line x2="128" y1="-256" y2="-224" x1="64" />
            <line x2="64" y1="-224" y2="-192" x1="128" />
            <line x2="64" y1="-192" y2="-256" x1="64" />
            <circle r="16" cx="144" cy="-32" />
            <line x2="128" y1="-64" y2="-32" x1="64" />
            <line x2="64" y1="-32" y2="0" x1="128" />
            <line x2="64" y1="0" y2="-64" x1="64" />
            <line x2="128" y1="-128" y2="-96" x1="64" />
            <line x2="64" y1="-96" y2="-64" x1="128" />
            <line x2="64" y1="-64" y2="-128" x1="64" />
            <circle r="16" cx="144" cy="-96" />
            <line x2="128" y1="-192" y2="-160" x1="64" />
            <line x2="64" y1="-160" y2="-128" x1="128" />
            <line x2="64" y1="-128" y2="-192" x1="64" />
            <circle r="16" cx="144" cy="-160" />
            <circle r="16" cx="144" cy="-224" />
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
        <blockdef name="Sumador_Completo">
            <timestamp>2026-3-29T14:15:31</timestamp>
            <rect width="256" x="64" y="-576" height="576" />
            <line x2="0" y1="-544" y2="-544" x1="64" />
            <line x2="0" y1="-480" y2="-480" x1="64" />
            <line x2="0" y1="-416" y2="-416" x1="64" />
            <line x2="0" y1="-352" y2="-352" x1="64" />
            <line x2="0" y1="-288" y2="-288" x1="64" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="384" y1="-288" y2="-288" x1="320" />
            <line x2="384" y1="-544" y2="-544" x1="320" />
            <line x2="384" y1="-480" y2="-480" x1="320" />
            <line x2="384" y1="-416" y2="-416" x1="320" />
            <line x2="384" y1="-352" y2="-352" x1="320" />
        </blockdef>
        <block symbolname="gnd" name="XLXI_14">
            <blockpin signalname="XLXN_22" name="G" />
        </block>
        <block symbolname="inv4" name="XLXI_6">
            <blockpin signalname="D3" name="I0" />
            <blockpin signalname="D2" name="I1" />
            <blockpin signalname="D1" name="I2" />
            <blockpin signalname="D0" name="I3" />
            <blockpin signalname="A3" name="O0" />
            <blockpin signalname="A2" name="O1" />
            <blockpin signalname="A1" name="O2" />
            <blockpin signalname="A0" name="O3" />
        </block>
        <block symbolname="vcc" name="XLXI_18">
            <blockpin signalname="XLXN_21" name="P" />
        </block>
        <block symbolname="Sumador_Completo" name="XLXI_19">
            <blockpin signalname="A0" name="A0" />
            <blockpin signalname="A1" name="A1" />
            <blockpin signalname="A2" name="A2" />
            <blockpin signalname="A3" name="A3" />
            <blockpin signalname="XLXN_21" name="B0" />
            <blockpin signalname="XLXN_22" name="B1" />
            <blockpin signalname="XLXN_22" name="B2" />
            <blockpin signalname="XLXN_22" name="B3" />
            <blockpin signalname="XLXN_22" name="C0" />
            <blockpin signalname="S4" name="C4" />
            <blockpin signalname="S0" name="S0" />
            <blockpin signalname="S1" name="S1" />
            <blockpin signalname="S2" name="S2" />
            <blockpin signalname="S3" name="S3" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <branch name="A0">
            <wire x2="1808" y1="1072" y2="1072" x1="1792" />
            <wire x2="2000" y1="1072" y2="1072" x1="1808" />
        </branch>
        <branch name="A1">
            <wire x2="1808" y1="1136" y2="1136" x1="1792" />
            <wire x2="2000" y1="1136" y2="1136" x1="1808" />
        </branch>
        <branch name="A2">
            <wire x2="1808" y1="1200" y2="1200" x1="1792" />
            <wire x2="2000" y1="1200" y2="1200" x1="1808" />
        </branch>
        <branch name="S1">
            <wire x2="2528" y1="1136" y2="1136" x1="2384" />
        </branch>
        <branch name="S4">
            <wire x2="2528" y1="1328" y2="1328" x1="2384" />
        </branch>
        <branch name="S3">
            <wire x2="2528" y1="1264" y2="1264" x1="2384" />
        </branch>
        <branch name="S2">
            <wire x2="2528" y1="1200" y2="1200" x1="2384" />
        </branch>
        <branch name="S0">
            <wire x2="2528" y1="1072" y2="1072" x1="2384" />
        </branch>
        <branch name="A3">
            <wire x2="1808" y1="1264" y2="1264" x1="1792" />
            <wire x2="2000" y1="1264" y2="1264" x1="1808" />
        </branch>
        <branch name="XLXN_22">
            <wire x2="1616" y1="1392" y2="1456" x1="1616" />
            <wire x2="1616" y1="1456" y2="1520" x1="1616" />
            <wire x2="1616" y1="1520" y2="1584" x1="1616" />
            <wire x2="2000" y1="1584" y2="1584" x1="1616" />
            <wire x2="1616" y1="1584" y2="1600" x1="1616" />
            <wire x2="2000" y1="1520" y2="1520" x1="1616" />
            <wire x2="2000" y1="1456" y2="1456" x1="1616" />
            <wire x2="2000" y1="1392" y2="1392" x1="1616" />
        </branch>
        <branch name="D0">
            <wire x2="1568" y1="1072" y2="1072" x1="1456" />
        </branch>
        <branch name="D1">
            <wire x2="1568" y1="1136" y2="1136" x1="1456" />
        </branch>
        <branch name="D3">
            <wire x2="1568" y1="1264" y2="1264" x1="1456" />
        </branch>
        <instance x="1568" y="1296" name="XLXI_6" orien="R0" />
        <branch name="D2">
            <wire x2="1552" y1="1200" y2="1200" x1="1456" />
            <wire x2="1568" y1="1200" y2="1200" x1="1552" />
        </branch>
        <branch name="XLXN_21">
            <wire x2="1456" y1="1328" y2="1616" x1="1456" />
            <wire x2="2000" y1="1328" y2="1328" x1="1456" />
        </branch>
        <instance x="1520" y="1616" name="XLXI_18" orien="R180" />
        <instance x="2000" y="1616" name="XLXI_19" orien="R0">
        </instance>
        <iomarker fontsize="28" x="2528" y="1136" name="S1" orien="R0" />
        <iomarker fontsize="28" x="2528" y="1328" name="S4" orien="R0" />
        <iomarker fontsize="28" x="2528" y="1264" name="S3" orien="R0" />
        <iomarker fontsize="28" x="2528" y="1200" name="S2" orien="R0" />
        <iomarker fontsize="28" x="2528" y="1072" name="S0" orien="R0" />
        <iomarker fontsize="28" x="1456" y="1072" name="D0" orien="R180" />
        <iomarker fontsize="28" x="1456" y="1136" name="D1" orien="R180" />
        <iomarker fontsize="28" x="1456" y="1264" name="D3" orien="R180" />
        <iomarker fontsize="28" x="1456" y="1200" name="D2" orien="R180" />
        <instance x="1552" y="1728" name="XLXI_14" orien="R0" />
    </sheet>
</drawing>