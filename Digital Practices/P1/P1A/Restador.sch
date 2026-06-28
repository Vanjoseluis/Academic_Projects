<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="B0" />
        <signal name="B1" />
        <signal name="B2" />
        <signal name="B3" />
        <signal name="C0" />
        <signal name="A0" />
        <signal name="A1" />
        <signal name="A2" />
        <signal name="A3" />
        <signal name="S0" />
        <signal name="S1" />
        <signal name="S2" />
        <signal name="S3" />
        <signal name="S4" />
        <signal name="XLXN_114" />
        <signal name="XLXN_115" />
        <signal name="XLXN_116" />
        <signal name="XLXN_117" />
        <port polarity="Input" name="B0" />
        <port polarity="Input" name="B1" />
        <port polarity="Input" name="B2" />
        <port polarity="Input" name="B3" />
        <port polarity="Input" name="C0" />
        <port polarity="Input" name="A0" />
        <port polarity="Input" name="A1" />
        <port polarity="Input" name="A2" />
        <port polarity="Input" name="A3" />
        <port polarity="Output" name="S0" />
        <port polarity="Output" name="S1" />
        <port polarity="Output" name="S2" />
        <port polarity="Output" name="S3" />
        <port polarity="Output" name="S4" />
        <blockdef name="Complemento_a2">
            <timestamp>2026-3-25T12:28:52</timestamp>
            <rect width="256" x="64" y="-256" height="256" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="384" y1="-224" y2="-224" x1="320" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
            <line x2="384" y1="-32" y2="-32" x1="320" />
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
        <block symbolname="Complemento_a2" name="XLXI_17">
            <blockpin signalname="B0" name="D0" />
            <blockpin signalname="B1" name="D1" />
            <blockpin signalname="B2" name="D2" />
            <blockpin signalname="B3" name="D3" />
            <blockpin signalname="XLXN_114" name="S0" />
            <blockpin signalname="XLXN_115" name="S1" />
            <blockpin signalname="XLXN_116" name="S2" />
            <blockpin signalname="XLXN_117" name="S3" />
        </block>
        <block symbolname="Sumador_Completo" name="XLXI_18">
            <blockpin signalname="A0" name="A0" />
            <blockpin signalname="A1" name="A1" />
            <blockpin signalname="A2" name="A2" />
            <blockpin signalname="A3" name="A3" />
            <blockpin signalname="XLXN_114" name="B0" />
            <blockpin signalname="XLXN_115" name="B1" />
            <blockpin signalname="XLXN_116" name="B2" />
            <blockpin signalname="XLXN_117" name="B3" />
            <blockpin signalname="C0" name="C0" />
            <blockpin signalname="S4" name="C4" />
            <blockpin signalname="S0" name="S0" />
            <blockpin signalname="S1" name="S1" />
            <blockpin signalname="S2" name="S2" />
            <blockpin signalname="S3" name="S3" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <branch name="A0">
            <wire x2="1504" y1="448" y2="448" x1="1280" />
        </branch>
        <branch name="A1">
            <wire x2="1504" y1="512" y2="512" x1="1280" />
        </branch>
        <branch name="A2">
            <wire x2="1504" y1="576" y2="576" x1="1280" />
        </branch>
        <branch name="A3">
            <wire x2="1504" y1="640" y2="640" x1="1280" />
        </branch>
        <branch name="S0">
            <wire x2="2016" y1="448" y2="448" x1="1888" />
        </branch>
        <branch name="S1">
            <wire x2="2016" y1="512" y2="512" x1="1888" />
        </branch>
        <branch name="S2">
            <wire x2="2016" y1="576" y2="576" x1="1888" />
        </branch>
        <branch name="S3">
            <wire x2="2016" y1="640" y2="640" x1="1888" />
        </branch>
        <branch name="S4">
            <wire x2="2016" y1="704" y2="704" x1="1888" />
        </branch>
        <text x="1884" y="740">C4 not used</text>
        <branch name="C0">
            <wire x2="1504" y1="960" y2="960" x1="1232" />
        </branch>
        <branch name="B3">
            <wire x2="720" y1="896" y2="896" x1="704" />
            <wire x2="768" y1="896" y2="896" x1="720" />
        </branch>
        <branch name="B2">
            <wire x2="720" y1="832" y2="832" x1="704" />
            <wire x2="768" y1="832" y2="832" x1="720" />
        </branch>
        <branch name="B1">
            <wire x2="720" y1="768" y2="768" x1="704" />
            <wire x2="768" y1="768" y2="768" x1="720" />
        </branch>
        <branch name="B0">
            <wire x2="720" y1="704" y2="704" x1="704" />
            <wire x2="768" y1="704" y2="704" x1="720" />
        </branch>
        <branch name="XLXN_114">
            <wire x2="1488" y1="704" y2="704" x1="1152" />
            <wire x2="1504" y1="704" y2="704" x1="1488" />
        </branch>
        <branch name="XLXN_115">
            <wire x2="1488" y1="768" y2="768" x1="1152" />
            <wire x2="1504" y1="768" y2="768" x1="1488" />
        </branch>
        <branch name="XLXN_116">
            <wire x2="1168" y1="832" y2="832" x1="1152" />
            <wire x2="1488" y1="832" y2="832" x1="1168" />
            <wire x2="1504" y1="832" y2="832" x1="1488" />
        </branch>
        <branch name="XLXN_117">
            <wire x2="1488" y1="896" y2="896" x1="1152" />
            <wire x2="1504" y1="896" y2="896" x1="1488" />
        </branch>
        <instance x="768" y="928" name="XLXI_17" orien="R0">
        </instance>
        <instance x="1504" y="992" name="XLXI_18" orien="R0">
        </instance>
        <iomarker fontsize="28" x="1280" y="448" name="A0" orien="R180" />
        <iomarker fontsize="28" x="1280" y="512" name="A1" orien="R180" />
        <iomarker fontsize="28" x="1280" y="576" name="A2" orien="R180" />
        <iomarker fontsize="28" x="1280" y="640" name="A3" orien="R180" />
        <iomarker fontsize="28" x="704" y="896" name="B3" orien="R180" />
        <iomarker fontsize="28" x="704" y="832" name="B2" orien="R180" />
        <iomarker fontsize="28" x="704" y="768" name="B1" orien="R180" />
        <iomarker fontsize="28" x="704" y="704" name="B0" orien="R180" />
        <iomarker fontsize="28" x="1232" y="960" name="C0" orien="R180" />
        <iomarker fontsize="28" x="2016" y="448" name="S0" orien="R0" />
        <iomarker fontsize="28" x="2016" y="512" name="S1" orien="R0" />
        <iomarker fontsize="28" x="2016" y="576" name="S2" orien="R0" />
        <iomarker fontsize="28" x="2016" y="640" name="S3" orien="R0" />
        <iomarker fontsize="28" x="2016" y="704" name="S4" orien="R0" />
    </sheet>
</drawing>