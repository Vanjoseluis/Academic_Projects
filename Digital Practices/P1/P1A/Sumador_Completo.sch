<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="A0" />
        <signal name="B0" />
        <signal name="C1" />
        <signal name="B1" />
        <signal name="A1" />
        <signal name="B2" />
        <signal name="A2" />
        <signal name="C3" />
        <signal name="S2" />
        <signal name="C2" />
        <signal name="S1" />
        <signal name="S0" />
        <signal name="C0" />
        <signal name="B3" />
        <signal name="A3" />
        <signal name="C4" />
        <signal name="S3" />
        <port polarity="Input" name="A0" />
        <port polarity="Input" name="B0" />
        <port polarity="Input" name="B1" />
        <port polarity="Input" name="A1" />
        <port polarity="Input" name="B2" />
        <port polarity="Input" name="A2" />
        <port polarity="Output" name="C3" />
        <port polarity="Output" name="S2" />
        <port polarity="Output" name="S1" />
        <port polarity="Output" name="S0" />
        <port polarity="Input" name="C0" />
        <port polarity="Input" name="B3" />
        <port polarity="Input" name="A3" />
        <port polarity="Output" name="C4" />
        <port polarity="Output" name="S3" />
        <blockdef name="Sumador">
            <timestamp>2026-3-19T8:55:49</timestamp>
            <rect width="256" x="64" y="-192" height="192" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
        </blockdef>
        <block symbolname="Sumador" name="XLXI_1">
            <blockpin signalname="A0" name="A" />
            <blockpin signalname="B0" name="B" />
            <blockpin signalname="C0" name="C0" />
            <blockpin signalname="S0" name="S" />
            <blockpin signalname="C1" name="C1" />
        </block>
        <block symbolname="Sumador" name="XLXI_2">
            <blockpin signalname="A1" name="A" />
            <blockpin signalname="B1" name="B" />
            <blockpin signalname="C1" name="C0" />
            <blockpin signalname="S1" name="S" />
            <blockpin signalname="C2" name="C1" />
        </block>
        <block symbolname="Sumador" name="XLXI_3">
            <blockpin signalname="A2" name="A" />
            <blockpin signalname="B2" name="B" />
            <blockpin signalname="C2" name="C0" />
            <blockpin signalname="S2" name="S" />
            <blockpin signalname="C3" name="C1" />
        </block>
        <block symbolname="Sumador" name="XLXI_4">
            <blockpin signalname="A3" name="A" />
            <blockpin signalname="B3" name="B" />
            <blockpin signalname="C3" name="C0" />
            <blockpin signalname="S3" name="S" />
            <blockpin signalname="C4" name="C1" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <instance x="1696" y="1184" name="XLXI_3" orien="R90">
        </instance>
        <instance x="1280" y="1184" name="XLXI_2" orien="R90">
        </instance>
        <branch name="B1">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="1376" y="1072" type="branch" />
            <wire x2="1376" y1="1072" y2="1184" x1="1376" />
        </branch>
        <branch name="A1">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="1440" y="1088" type="branch" />
            <wire x2="1440" y1="1088" y2="1184" x1="1440" />
        </branch>
        <branch name="C2">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="1728" y="1056" type="branch" />
            <wire x2="1728" y1="1056" y2="1184" x1="1728" />
        </branch>
        <branch name="B2">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="1792" y="1072" type="branch" />
            <wire x2="1792" y1="1072" y2="1184" x1="1792" />
        </branch>
        <branch name="A2">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="1856" y="1072" type="branch" />
            <wire x2="1856" y1="1072" y2="1184" x1="1856" />
        </branch>
        <branch name="C3">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1792" y="1664" type="branch" />
            <wire x2="1792" y1="1568" y2="1664" x1="1792" />
        </branch>
        <branch name="S2">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1856" y="1664" type="branch" />
            <wire x2="1856" y1="1568" y2="1664" x1="1856" />
        </branch>
        <branch name="C2">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1376" y="1696" type="branch" />
            <wire x2="1376" y1="1568" y2="1696" x1="1376" />
        </branch>
        <branch name="S1">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1456" y="1680" type="branch" />
            <wire x2="1440" y1="1568" y2="1680" x1="1440" />
            <wire x2="1456" y1="1680" y2="1680" x1="1440" />
        </branch>
        <instance x="864" y="1168" name="XLXI_1" orien="R90">
        </instance>
        <branch name="B0">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="960" y="1056" type="branch" />
            <wire x2="960" y1="1056" y2="1168" x1="960" />
        </branch>
        <branch name="C1">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="960" y="1680" type="branch" />
            <wire x2="960" y1="1552" y2="1680" x1="960" />
        </branch>
        <branch name="A0">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="1056" type="branch" />
            <wire x2="1024" y1="1056" y2="1168" x1="1024" />
        </branch>
        <branch name="S0">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="1680" type="branch" />
            <wire x2="1024" y1="1552" y2="1680" x1="1024" />
        </branch>
        <branch name="C1">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="1312" y="1072" type="branch" />
            <wire x2="1312" y1="1072" y2="1184" x1="1312" />
        </branch>
        <branch name="C0">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="896" y="1072" type="branch" />
            <wire x2="896" y1="1072" y2="1168" x1="896" />
        </branch>
        <instance x="2192" y="1184" name="XLXI_4" orien="R90">
        </instance>
        <branch name="C3">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="2224" y="1056" type="branch" />
            <wire x2="2224" y1="1056" y2="1184" x1="2224" />
        </branch>
        <branch name="B3">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="2288" y="1072" type="branch" />
            <wire x2="2288" y1="1072" y2="1184" x1="2288" />
        </branch>
        <branch name="A3">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="2352" y="1072" type="branch" />
            <wire x2="2352" y1="1072" y2="1184" x1="2352" />
        </branch>
        <branch name="C4">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2288" y="1664" type="branch" />
            <wire x2="2288" y1="1568" y2="1664" x1="2288" />
        </branch>
        <branch name="S3">
            <attrtext style="alignment:SOFT-VRIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2352" y="1664" type="branch" />
            <wire x2="2352" y1="1568" y2="1664" x1="2352" />
        </branch>
        <branch name="A0">
            <wire x2="864" y1="256" y2="400" x1="864" />
        </branch>
        <branch name="A1">
            <wire x2="896" y1="256" y2="400" x1="896" />
        </branch>
        <branch name="A2">
            <wire x2="928" y1="256" y2="400" x1="928" />
        </branch>
        <branch name="A3">
            <wire x2="960" y1="256" y2="400" x1="960" />
        </branch>
        <branch name="B0">
            <wire x2="992" y1="256" y2="400" x1="992" />
        </branch>
        <branch name="B1">
            <wire x2="1024" y1="256" y2="400" x1="1024" />
        </branch>
        <branch name="B2">
            <wire x2="1056" y1="256" y2="400" x1="1056" />
        </branch>
        <branch name="B3">
            <wire x2="1088" y1="256" y2="400" x1="1088" />
        </branch>
        <branch name="S0">
            <wire x2="1120" y1="256" y2="400" x1="1120" />
        </branch>
        <branch name="S1">
            <wire x2="1152" y1="256" y2="400" x1="1152" />
        </branch>
        <branch name="S2">
            <wire x2="1184" y1="256" y2="400" x1="1184" />
        </branch>
        <branch name="S3">
            <wire x2="1216" y1="256" y2="400" x1="1216" />
        </branch>
        <branch name="C0">
            <wire x2="1248" y1="256" y2="400" x1="1248" />
        </branch>
        <branch name="C4">
            <wire x2="1280" y1="256" y2="400" x1="1280" />
        </branch>
        <iomarker fontsize="28" x="864" y="256" name="A0" orien="R270" />
        <iomarker fontsize="28" x="896" y="256" name="A1" orien="R270" />
        <iomarker fontsize="28" x="928" y="256" name="A2" orien="R270" />
        <iomarker fontsize="28" x="960" y="256" name="A3" orien="R270" />
        <iomarker fontsize="28" x="992" y="256" name="B0" orien="R270" />
        <iomarker fontsize="28" x="1024" y="256" name="B1" orien="R270" />
        <iomarker fontsize="28" x="1056" y="256" name="B2" orien="R270" />
        <iomarker fontsize="28" x="1088" y="256" name="B3" orien="R270" />
        <iomarker fontsize="28" x="1248" y="256" name="C0" orien="R270" />
        <iomarker fontsize="28" x="1120" y="256" name="S0" orien="R270" />
        <iomarker fontsize="28" x="1152" y="256" name="S1" orien="R270" />
        <iomarker fontsize="28" x="1184" y="256" name="S2" orien="R270" />
        <iomarker fontsize="28" x="1216" y="256" name="S3" orien="R270" />
        <iomarker fontsize="28" x="1280" y="256" name="C4" orien="R270" />
    </sheet>
</drawing>