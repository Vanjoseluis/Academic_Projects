<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="SVF(3:0)" />
        <signal name="SVC(3:0)" />
        <signal name="VVal" />
        <signal name="XLXN_13(3:0)" />
        <signal name="XLXN_14(3:0)" />
        <signal name="SevenSeg0(7:0)" />
        <signal name="SevenSeg1(7:0)" />
        <signal name="SevenSeg2(7:0)" />
        <signal name="SevenSeg3(7:0)" />
        <port polarity="Input" name="SVF(3:0)" />
        <port polarity="Input" name="SVC(3:0)" />
        <port polarity="Input" name="VVal" />
        <port polarity="Output" name="SevenSeg0(7:0)" />
        <port polarity="Output" name="SevenSeg1(7:0)" />
        <port polarity="Output" name="SevenSeg2(7:0)" />
        <port polarity="Output" name="SevenSeg3(7:0)" />
        <blockdef name="Hexadecimal_7Segmentos">
            <timestamp>2026-4-1T15:50:23</timestamp>
            <rect width="256" x="64" y="-128" height="128" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
            <rect width="64" x="320" y="-108" height="24" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <rect width="64" x="0" y="-108" height="24" />
        </blockdef>
        <blockdef name="Const_F">
            <timestamp>2026-4-1T17:45:9</timestamp>
            <rect width="256" x="0" y="-64" height="64" />
            <line x2="320" y1="-32" y2="-32" x1="256" />
            <rect width="64" x="256" y="-44" height="24" />
        </blockdef>
        <blockdef name="Const_C">
            <timestamp>2026-4-1T18:1:56</timestamp>
            <rect width="256" x="0" y="-64" height="64" />
            <line x2="320" y1="-32" y2="-32" x1="256" />
            <rect width="64" x="256" y="-44" height="24" />
        </blockdef>
        <block symbolname="Hexadecimal_7Segmentos" name="XLXI_1">
            <blockpin signalname="VVal" name="enable" />
            <blockpin signalname="SevenSeg3(7:0)" name="segmentos(7:0)" />
            <blockpin signalname="XLXN_14(3:0)" name="x(3:0)" />
        </block>
        <block symbolname="Hexadecimal_7Segmentos" name="XLXI_2">
            <blockpin signalname="VVal" name="enable" />
            <blockpin signalname="SevenSeg2(7:0)" name="segmentos(7:0)" />
            <blockpin signalname="SVC(3:0)" name="x(3:0)" />
        </block>
        <block symbolname="Hexadecimal_7Segmentos" name="XLXI_3">
            <blockpin signalname="VVal" name="enable" />
            <blockpin signalname="SevenSeg1(7:0)" name="segmentos(7:0)" />
            <blockpin signalname="XLXN_13(3:0)" name="x(3:0)" />
        </block>
        <block symbolname="Hexadecimal_7Segmentos" name="XLXI_4">
            <blockpin signalname="VVal" name="enable" />
            <blockpin signalname="SevenSeg0(7:0)" name="segmentos(7:0)" />
            <blockpin signalname="SVF(3:0)" name="x(3:0)" />
        </block>
        <block symbolname="Const_F" name="XLXI_5">
            <blockpin signalname="XLXN_13(3:0)" name="F(3:0)" />
        </block>
        <block symbolname="Const_C" name="XLXI_6">
            <blockpin signalname="XLXN_14(3:0)" name="C(3:0)" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <instance x="1568" y="1744" name="XLXI_1" orien="R0">
        </instance>
        <instance x="1568" y="1216" name="XLXI_3" orien="R0">
        </instance>
        <instance x="1568" y="976" name="XLXI_4" orien="R0">
        </instance>
        <instance x="1584" y="1456" name="XLXI_2" orien="R0">
        </instance>
        <branch name="SVC(3:0)">
            <wire x2="1568" y1="1360" y2="1360" x1="1056" />
            <wire x2="1584" y1="1360" y2="1360" x1="1568" />
        </branch>
        <branch name="SVF(3:0)">
            <wire x2="1552" y1="880" y2="880" x1="1040" />
            <wire x2="1568" y1="880" y2="880" x1="1552" />
        </branch>
        <branch name="VVal">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1376" y="944" type="branch" />
            <wire x2="1568" y1="944" y2="944" x1="1376" />
        </branch>
        <branch name="VVal">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1376" y="1184" type="branch" />
            <wire x2="1568" y1="1184" y2="1184" x1="1376" />
        </branch>
        <branch name="VVal">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1376" y="1424" type="branch" />
            <wire x2="1584" y1="1424" y2="1424" x1="1376" />
        </branch>
        <branch name="VVal">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1376" y="1712" type="branch" />
            <wire x2="1568" y1="1712" y2="1712" x1="1376" />
        </branch>
        <instance x="896" y="1152" name="XLXI_5" orien="R0">
        </instance>
        <iomarker fontsize="28" x="1040" y="880" name="SVF(3:0)" orien="R180" />
        <branch name="XLXN_13(3:0)">
            <wire x2="1568" y1="1120" y2="1120" x1="1216" />
        </branch>
        <iomarker fontsize="28" x="1056" y="1360" name="SVC(3:0)" orien="R180" />
        <instance x="896" y="1680" name="XLXI_6" orien="R0">
        </instance>
        <branch name="XLXN_14(3:0)">
            <wire x2="1568" y1="1648" y2="1648" x1="1216" />
        </branch>
        <branch name="SevenSeg0(7:0)">
            <wire x2="2112" y1="880" y2="880" x1="1952" />
            <wire x2="2128" y1="880" y2="880" x1="2112" />
        </branch>
        <branch name="SevenSeg1(7:0)">
            <wire x2="2128" y1="1120" y2="1120" x1="1952" />
        </branch>
        <branch name="SevenSeg2(7:0)">
            <wire x2="2128" y1="1360" y2="1360" x1="1968" />
        </branch>
        <branch name="SevenSeg3(7:0)">
            <wire x2="2128" y1="1648" y2="1648" x1="1952" />
        </branch>
        <branch name="VVal">
            <wire x2="688" y1="960" y2="1104" x1="688" />
        </branch>
        <iomarker fontsize="28" x="688" y="960" name="VVal" orien="R270" />
        <iomarker fontsize="28" x="2128" y="880" name="SevenSeg0(7:0)" orien="R0" />
        <iomarker fontsize="28" x="2128" y="1120" name="SevenSeg1(7:0)" orien="R0" />
        <iomarker fontsize="28" x="2128" y="1360" name="SevenSeg2(7:0)" orien="R0" />
        <iomarker fontsize="28" x="2128" y="1648" name="SevenSeg3(7:0)" orien="R0" />
    </sheet>
</drawing>