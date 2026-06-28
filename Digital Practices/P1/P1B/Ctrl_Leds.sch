<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="VLed(7:0)" />
        <signal name="VRes(7:0)" />
        <signal name="XLXN_33" />
        <signal name="V_R(7:0)" />
        <signal name="V_RR" />
        <signal name="On_Off" />
        <signal name="XLXN_41(7:0)" />
        <signal name="Leds(7:0)" />
        <signal name="VerVot" />
        <port polarity="Input" name="VLed(7:0)" />
        <port polarity="Input" name="VRes(7:0)" />
        <port polarity="Input" name="V_R(7:0)" />
        <port polarity="Input" name="V_RR" />
        <port polarity="Input" name="On_Off" />
        <port polarity="Output" name="Leds(7:0)" />
        <port polarity="Input" name="VerVot" />
        <blockdef name="vcc">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-64" x1="64" />
            <line x2="64" y1="0" y2="-32" x1="64" />
            <line x2="32" y1="-64" y2="-64" x1="96" />
        </blockdef>
        <blockdef name="m4_1e">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="96" y1="-416" y2="-416" x1="0" />
            <line x2="96" y1="-352" y2="-352" x1="0" />
            <line x2="96" y1="-288" y2="-288" x1="0" />
            <line x2="96" y1="-224" y2="-224" x1="0" />
            <line x2="96" y1="-32" y2="-32" x1="0" />
            <line x2="256" y1="-320" y2="-320" x1="320" />
            <line x2="96" y1="-160" y2="-160" x1="0" />
            <line x2="96" y1="-96" y2="-96" x1="0" />
            <line x2="96" y1="-96" y2="-96" x1="176" />
            <line x2="176" y1="-208" y2="-96" x1="176" />
            <line x2="96" y1="-32" y2="-32" x1="224" />
            <line x2="224" y1="-216" y2="-32" x1="224" />
            <line x2="96" y1="-224" y2="-192" x1="256" />
            <line x2="256" y1="-416" y2="-224" x1="256" />
            <line x2="256" y1="-448" y2="-416" x1="96" />
            <line x2="96" y1="-192" y2="-448" x1="96" />
            <line x2="96" y1="-160" y2="-160" x1="128" />
            <line x2="128" y1="-200" y2="-160" x1="128" />
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
        <block symbolname="vcc" name="XLXI_252">
            <blockpin signalname="XLXN_33" name="P" />
        </block>
        <block symbolname="m4_1e" name="XLXI_251(7:0)">
            <blockpin signalname="VLed(7:0)" name="D0" />
            <blockpin signalname="VRes(7:0)" name="D1" />
            <blockpin signalname="V_R(7:0)" name="D2" />
            <blockpin signalname="V_RR" name="D3" />
            <blockpin signalname="XLXN_33" name="E" />
            <blockpin signalname="V_RR" name="S0" />
            <blockpin signalname="VerVot" name="S1" />
            <blockpin signalname="XLXN_41(7:0)" name="O" />
        </block>
        <block symbolname="or2" name="XLXI_253(7:0)">
            <blockpin signalname="On_Off" name="I0" />
            <blockpin signalname="XLXN_41(7:0)" name="I1" />
            <blockpin signalname="Leds(7:0)" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="1760" height="1360">
        <branch name="VLed(7:0)">
            <wire x2="672" y1="416" y2="416" x1="608" />
        </branch>
        <branch name="VRes(7:0)">
            <wire x2="672" y1="480" y2="480" x1="608" />
        </branch>
        <instance x="192" y="768" name="XLXI_252" orien="R0" />
        <branch name="XLXN_33">
            <wire x2="256" y1="768" y2="800" x1="256" />
            <wire x2="672" y1="800" y2="800" x1="256" />
        </branch>
        <branch name="V_R(7:0)">
            <wire x2="672" y1="544" y2="544" x1="608" />
        </branch>
        <instance x="672" y="832" name="XLXI_251(7:0)" orien="R0" />
        <branch name="V_RR">
            <wire x2="592" y1="672" y2="672" x1="560" />
            <wire x2="672" y1="672" y2="672" x1="592" />
            <wire x2="672" y1="608" y2="608" x1="592" />
            <wire x2="592" y1="608" y2="672" x1="592" />
        </branch>
        <instance x="1104" y="640" name="XLXI_253(7:0)" orien="R0" />
        <branch name="On_Off">
            <wire x2="1104" y1="864" y2="864" x1="592" />
            <wire x2="1104" y1="576" y2="576" x1="1040" />
            <wire x2="1040" y1="576" y2="656" x1="1040" />
            <wire x2="1104" y1="656" y2="656" x1="1040" />
            <wire x2="1104" y1="656" y2="864" x1="1104" />
        </branch>
        <branch name="XLXN_41(7:0)">
            <wire x2="1104" y1="512" y2="512" x1="992" />
        </branch>
        <branch name="Leds(7:0)">
            <wire x2="1424" y1="544" y2="544" x1="1360" />
        </branch>
        <iomarker fontsize="28" x="608" y="416" name="VLed(7:0)" orien="R180" />
        <iomarker fontsize="28" x="608" y="480" name="VRes(7:0)" orien="R180" />
        <iomarker fontsize="28" x="608" y="544" name="V_R(7:0)" orien="R180" />
        <iomarker fontsize="28" x="560" y="672" name="V_RR" orien="R180" />
        <iomarker fontsize="28" x="592" y="864" name="On_Off" orien="R180" />
        <iomarker fontsize="28" x="1424" y="544" name="Leds(7:0)" orien="R0" />
        <branch name="VerVot">
            <wire x2="672" y1="736" y2="736" x1="592" />
        </branch>
        <iomarker fontsize="28" x="592" y="736" name="VerVot" orien="R180" />
    </sheet>
</drawing>