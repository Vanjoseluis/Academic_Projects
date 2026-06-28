<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="SVC(3:0)" />
        <signal name="VVal" />
        <signal name="SVF(3:0)" />
        <signal name="M(7:0)" />
        <signal name="VF(2:0)" />
        <signal name="VC(2:0)" />
        <signal name="NA(1:0)" />
        <signal name="SevenSeg_VF(7:0)" />
        <signal name="SevenSeg_F(7:0)" />
        <signal name="SevenSeg_VC(7:0)" />
        <signal name="SevenSeg_C(7:0)" />
        <port polarity="Output" name="SVC(3:0)" />
        <port polarity="Input" name="VVal" />
        <port polarity="Output" name="SVF(3:0)" />
        <port polarity="Input" name="M(7:0)" />
        <port polarity="Output" name="VF(2:0)" />
        <port polarity="Output" name="VC(2:0)" />
        <port polarity="Output" name="NA(1:0)" />
        <port polarity="Output" name="SevenSeg_VF(7:0)" />
        <port polarity="Output" name="SevenSeg_F(7:0)" />
        <port polarity="Output" name="SevenSeg_VC(7:0)" />
        <port polarity="Output" name="SevenSeg_C(7:0)" />
        <blockdef name="Cuenta_Votos_COMPLETO">
            <timestamp>2026-3-29T14:19:47</timestamp>
            <rect width="256" x="64" y="-128" height="128" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <rect width="64" x="0" y="-108" height="24" />
            <line x2="384" y1="-32" y2="-32" x1="320" />
            <rect width="64" x="320" y="-44" height="24" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
            <rect width="64" x="320" y="-108" height="24" />
        </blockdef>
        <blockdef name="Visualiza_Cuenta">
            <timestamp>2026-4-4T11:57:46</timestamp>
            <rect width="320" x="64" y="-256" height="256" />
            <line x2="448" y1="-224" y2="-224" x1="384" />
            <rect width="64" x="384" y="-236" height="24" />
            <line x2="448" y1="-160" y2="-160" x1="384" />
            <rect width="64" x="384" y="-172" height="24" />
            <line x2="448" y1="-96" y2="-96" x1="384" />
            <rect width="64" x="384" y="-108" height="24" />
            <line x2="448" y1="-32" y2="-32" x1="384" />
            <rect width="64" x="384" y="-44" height="24" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <rect width="64" x="0" y="-108" height="24" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <rect width="64" x="0" y="-236" height="24" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
        </blockdef>
        <blockdef name="Resultado_de_votacion">
            <timestamp>2026-4-5T17:24:47</timestamp>
            <rect width="256" x="64" y="-192" height="192" />
            <line x2="384" y1="-32" y2="-32" x1="320" />
            <rect width="64" x="320" y="-44" height="24" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <rect width="64" x="0" y="-108" height="24" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
            <rect width="64" x="320" y="-108" height="24" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
            <rect width="64" x="320" y="-172" height="24" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
        </blockdef>
        <block symbolname="Cuenta_Votos_COMPLETO" name="XLXI_2">
            <blockpin signalname="M(7:0)" name="M(7:0)" />
            <blockpin signalname="SVC(3:0)" name="SVC(3:0)" />
            <blockpin signalname="SVF(3:0)" name="SVF(3:0)" />
        </block>
        <block symbolname="Visualiza_Cuenta" name="XLXI_9">
            <blockpin signalname="SevenSeg_VF(7:0)" name="SevenSeg0(7:0)" />
            <blockpin signalname="SevenSeg_F(7:0)" name="SevenSeg1(7:0)" />
            <blockpin signalname="SevenSeg_VC(7:0)" name="SevenSeg2(7:0)" />
            <blockpin signalname="SevenSeg_C(7:0)" name="SevenSeg3(7:0)" />
            <blockpin signalname="SVC(3:0)" name="SVC(3:0)" />
            <blockpin signalname="SVF(3:0)" name="SVF(3:0)" />
            <blockpin signalname="VVal" name="VVal" />
        </block>
        <block symbolname="Resultado_de_votacion" name="XLXI_12">
            <blockpin signalname="NA(1:0)" name="NA(1:0)" />
            <blockpin signalname="SVC(3:0)" name="SVC(3:0)" />
            <blockpin signalname="VC(2:0)" name="VC(2:0)" />
            <blockpin signalname="VF(2:0)" name="VF(2:0)" />
            <blockpin signalname="VVal" name="VVal" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="2720" height="1760">
        <instance x="544" y="1040" name="XLXI_2" orien="R0">
        </instance>
        <branch name="SVF(3:0)">
            <wire x2="1024" y1="944" y2="944" x1="928" />
        </branch>
        <branch name="SVC(3:0)">
            <wire x2="1024" y1="1008" y2="1008" x1="928" />
        </branch>
        <branch name="M(7:0)">
            <wire x2="544" y1="944" y2="944" x1="304" />
        </branch>
        <iomarker fontsize="28" x="304" y="944" name="M(7:0)" orien="R180" />
        <iomarker fontsize="28" x="288" y="1216" name="VVal" orien="R180" />
        <branch name="VVal">
            <wire x2="480" y1="1216" y2="1216" x1="288" />
        </branch>
        <iomarker fontsize="28" x="1024" y="1008" name="SVC(3:0)" orien="R0" />
        <iomarker fontsize="28" x="1024" y="944" name="SVF(3:0)" orien="R0" />
        <instance x="1504" y="1104" name="XLXI_9" orien="R0">
        </instance>
        <branch name="VVal">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1472" y="1296" type="branch" />
            <wire x2="1488" y1="1296" y2="1296" x1="1472" />
            <wire x2="1568" y1="1296" y2="1296" x1="1488" />
        </branch>
        <branch name="SVC(3:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1488" y="1360" type="branch" />
            <wire x2="1504" y1="1360" y2="1360" x1="1488" />
            <wire x2="1568" y1="1360" y2="1360" x1="1504" />
        </branch>
        <branch name="VF(2:0)">
            <wire x2="1968" y1="1296" y2="1296" x1="1952" />
            <wire x2="1984" y1="1296" y2="1296" x1="1968" />
        </branch>
        <branch name="VC(2:0)">
            <wire x2="1968" y1="1360" y2="1360" x1="1952" />
            <wire x2="1984" y1="1360" y2="1360" x1="1968" />
        </branch>
        <branch name="NA(1:0)">
            <wire x2="1968" y1="1424" y2="1424" x1="1952" />
            <wire x2="1984" y1="1424" y2="1424" x1="1968" />
        </branch>
        <iomarker fontsize="28" x="1984" y="1296" name="VF(2:0)" orien="R0" />
        <iomarker fontsize="28" x="1984" y="1360" name="VC(2:0)" orien="R0" />
        <iomarker fontsize="28" x="1984" y="1424" name="NA(1:0)" orien="R0" />
        <branch name="VVal">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1472" y="944" type="branch" />
            <wire x2="1504" y1="944" y2="944" x1="1472" />
        </branch>
        <branch name="SVF(3:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1472" y="880" type="branch" />
            <wire x2="1504" y1="880" y2="880" x1="1472" />
        </branch>
        <branch name="SVC(3:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1472" y="1008" type="branch" />
            <wire x2="1504" y1="1008" y2="1008" x1="1472" />
        </branch>
        <branch name="SevenSeg_VF(7:0)">
            <wire x2="2032" y1="880" y2="880" x1="1952" />
            <wire x2="2080" y1="880" y2="880" x1="2032" />
        </branch>
        <branch name="SevenSeg_F(7:0)">
            <wire x2="2032" y1="944" y2="944" x1="1952" />
            <wire x2="2080" y1="944" y2="944" x1="2032" />
        </branch>
        <branch name="SevenSeg_VC(7:0)">
            <wire x2="2032" y1="1008" y2="1008" x1="1952" />
            <wire x2="2080" y1="1008" y2="1008" x1="2032" />
        </branch>
        <branch name="SevenSeg_C(7:0)">
            <wire x2="2032" y1="1072" y2="1072" x1="1952" />
            <wire x2="2080" y1="1072" y2="1072" x1="2032" />
        </branch>
        <iomarker fontsize="28" x="2080" y="880" name="SevenSeg_VF(7:0)" orien="R0" />
        <iomarker fontsize="28" x="2080" y="944" name="SevenSeg_F(7:0)" orien="R0" />
        <iomarker fontsize="28" x="2080" y="1008" name="SevenSeg_VC(7:0)" orien="R0" />
        <iomarker fontsize="28" x="2080" y="1072" name="SevenSeg_C(7:0)" orien="R0" />
        <instance x="1568" y="1456" name="XLXI_12" orien="R0">
        </instance>
    </sheet>
</drawing>