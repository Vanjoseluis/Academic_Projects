<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="clk_out" />
        <signal name="EppWAIT" />
        <signal name="DB(7:0)" />
        <signal name="swInternal(7:0)" />
        <signal name="btnInternal(4:0)" />
        <signal name="RxInternal" />
        <signal name="Led(7:0)" />
        <signal name="seg(7:0)" />
        <signal name="an(3:0)" />
        <signal name="RsTx" />
        <signal name="EppASTB" />
        <signal name="EppDSTB" />
        <signal name="sw(7:0)" />
        <signal name="btn(4:0)" />
        <signal name="EppWRITE" />
        <signal name="LedInternal(7:0)" />
        <signal name="seven_seg3(7:0)" />
        <signal name="seven_seg2(7:0)" />
        <signal name="seven_seg1(7:0)" />
        <signal name="seven_seg0(7:0)" />
        <signal name="RsRx" />
        <signal name="clk" />
        <signal name="XLXN_289" />
        <port polarity="Output" name="EppWAIT" />
        <port polarity="BiDirectional" name="DB(7:0)" />
        <port polarity="Output" name="Led(7:0)" />
        <port polarity="Output" name="seg(7:0)" />
        <port polarity="Output" name="an(3:0)" />
        <port polarity="Output" name="RsTx" />
        <port polarity="Input" name="EppASTB" />
        <port polarity="Input" name="EppDSTB" />
        <port polarity="Input" name="sw(7:0)" />
        <port polarity="Input" name="btn(4:0)" />
        <port polarity="Input" name="EppWRITE" />
        <port polarity="Input" name="RsRx" />
        <port polarity="Input" name="clk" />
        <blockdef name="MyDesign">
            <timestamp>2020-12-16T9:57:5</timestamp>
            <rect width="400" x="64" y="-320" height="320" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <rect width="64" x="0" y="-236" height="24" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <line x2="528" y1="-288" y2="-288" x1="464" />
            <rect width="64" x="464" y="-300" height="24" />
            <line x2="528" y1="-32" y2="-32" x1="464" />
            <rect width="64" x="464" y="-44" height="24" />
            <line x2="528" y1="-96" y2="-96" x1="464" />
            <rect width="64" x="464" y="-108" height="24" />
            <line x2="528" y1="-160" y2="-160" x1="464" />
            <rect width="64" x="464" y="-172" height="24" />
            <line x2="528" y1="-224" y2="-224" x1="464" />
            <rect width="64" x="464" y="-236" height="24" />
            <line x2="0" y1="-288" y2="-288" x1="64" />
            <rect width="64" x="0" y="-300" height="24" />
        </blockdef>
        <blockdef name="Remote_Lab">
            <timestamp>2022-10-7T8:44:14</timestamp>
            <line x2="464" y1="96" y2="96" x1="400" />
            <line x2="0" y1="-672" y2="-672" x1="64" />
            <line x2="0" y1="-608" y2="-608" x1="64" />
            <line x2="0" y1="-544" y2="-544" x1="64" />
            <rect width="336" x="64" y="-704" height="832" />
            <line x2="0" y1="-480" y2="-480" x1="64" />
            <rect width="64" x="0" y="-428" height="24" />
            <line x2="0" y1="-416" y2="-416" x1="64" />
            <rect width="64" x="0" y="-364" height="24" />
            <line x2="0" y1="-352" y2="-352" x1="64" />
            <line x2="0" y1="-288" y2="-288" x1="64" />
            <rect width="64" x="0" y="-236" height="24" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <rect width="64" x="0" y="-172" height="24" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <rect width="64" x="0" y="-108" height="24" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <rect width="64" x="0" y="-44" height="24" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <rect width="64" x="0" y="20" height="24" />
            <line x2="0" y1="32" y2="32" x1="64" />
            <line x2="0" y1="96" y2="96" x1="64" />
            <rect width="64" x="400" y="-172" height="24" />
            <line x2="464" y1="-160" y2="-160" x1="400" />
            <rect width="64" x="400" y="-108" height="24" />
            <line x2="464" y1="-96" y2="-96" x1="400" />
            <line x2="464" y1="-288" y2="-288" x1="400" />
            <rect width="64" x="400" y="-428" height="24" />
            <line x2="464" y1="-416" y2="-416" x1="400" />
            <rect width="64" x="400" y="-236" height="24" />
            <line x2="464" y1="-224" y2="-224" x1="400" />
            <line x2="464" y1="-608" y2="-608" x1="400" />
            <rect width="64" x="400" y="-556" height="24" />
            <line x2="464" y1="-544" y2="-544" x1="400" />
            <rect width="64" x="400" y="-364" height="24" />
            <line x2="464" y1="-352" y2="-352" x1="400" />
        </blockdef>
        <blockdef name="bufg">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="0" x1="64" />
            <line x2="64" y1="-32" y2="-64" x1="128" />
            <line x2="128" y1="0" y2="-32" x1="64" />
            <line x2="128" y1="-32" y2="-32" x1="224" />
            <line x2="64" y1="-32" y2="-32" x1="0" />
        </blockdef>
        <blockdef name="vcc">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-64" x1="64" />
            <line x2="64" y1="0" y2="-32" x1="64" />
            <line x2="32" y1="-64" y2="-64" x1="96" />
        </blockdef>
        <block symbolname="MyDesign" name="XLXI_241">
            <blockpin signalname="btnInternal(4:0)" name="btn(4:0)" />
            <blockpin signalname="clk_out" name="Ck" />
            <blockpin signalname="LedInternal(7:0)" name="Leds(7:0)" />
            <blockpin signalname="seven_seg0(7:0)" name="SevenSeg0(7:0)" />
            <blockpin signalname="seven_seg1(7:0)" name="SevenSeg1(7:0)" />
            <blockpin signalname="seven_seg2(7:0)" name="SevenSeg2(7:0)" />
            <blockpin signalname="seven_seg3(7:0)" name="SevenSeg3(7:0)" />
            <blockpin signalname="swInternal(7:0)" name="sw(7:0)" />
        </block>
        <block symbolname="Remote_Lab" name="XLXI_255">
            <blockpin signalname="clk_out" name="Clk" />
            <blockpin signalname="EppASTB" name="EppASTB" />
            <blockpin signalname="EppDSTB" name="EppDSTB" />
            <blockpin signalname="RsTx" name="RsTx" />
            <blockpin signalname="EppWRITE" name="EppWRITE" />
            <blockpin signalname="sw(7:0)" name="sw(7:0)" />
            <blockpin signalname="btn(4:0)" name="btn(4:0)" />
            <blockpin signalname="RsRx" name="RsRx" />
            <blockpin signalname="LedInternal(7:0)" name="LedInternal(7:0)" />
            <blockpin signalname="seven_seg3(7:0)" name="SevenSeg3(7:0)" />
            <blockpin signalname="seven_seg2(7:0)" name="SevenSeg2(7:0)" />
            <blockpin signalname="seven_seg1(7:0)" name="SevenSeg1(7:0)" />
            <blockpin signalname="seven_seg0(7:0)" name="SevenSeg0(7:0)" />
            <blockpin signalname="XLXN_289" name="TxInternal" />
            <blockpin signalname="seg(7:0)" name="seg(7:0)" />
            <blockpin signalname="an(3:0)" name="an(3:0)" />
            <blockpin signalname="RxInternal" name="RxInternal" />
            <blockpin signalname="swInternal(7:0)" name="swInternal(7:0)" />
            <blockpin signalname="Led(7:0)" name="Led(7:0)" />
            <blockpin signalname="EppWAIT" name="EppWAIT" />
            <blockpin signalname="DB(7:0)" name="EppDB(7:0)" />
            <blockpin signalname="btnInternal(4:0)" name="btnInternal(4:0)" />
        </block>
        <block symbolname="bufg" name="XLXI_29">
            <blockpin signalname="clk" name="I" />
            <blockpin signalname="clk_out" name="O" />
        </block>
        <block symbolname="vcc" name="XLXI_262">
            <blockpin signalname="XLXN_289" name="P" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="2688" height="1900">
        <attr value="CM" name="LengthUnitName" />
        <attr value="4" name="GridsPerUnit" />
        <branch name="swInternal(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="880" y="1424" type="branch" />
            <wire x2="896" y1="1424" y2="1424" x1="880" />
            <wire x2="992" y1="1424" y2="1424" x1="896" />
        </branch>
        <branch name="btnInternal(4:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="880" y="1488" type="branch" />
            <wire x2="896" y1="1488" y2="1488" x1="880" />
            <wire x2="992" y1="1488" y2="1488" x1="896" />
        </branch>
        <branch name="seven_seg2(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1712" y="1552" type="branch" />
            <wire x2="1696" y1="1552" y2="1552" x1="1520" />
            <wire x2="1712" y1="1552" y2="1552" x1="1696" />
        </branch>
        <branch name="seven_seg0(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1728" y="1680" type="branch" />
            <wire x2="1712" y1="1680" y2="1680" x1="1520" />
            <wire x2="1728" y1="1680" y2="1680" x1="1712" />
        </branch>
        <branch name="seven_seg1(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1728" y="1616" type="branch" />
            <wire x2="1712" y1="1616" y2="1616" x1="1520" />
            <wire x2="1728" y1="1616" y2="1616" x1="1712" />
        </branch>
        <branch name="seven_seg3(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1712" y="1488" type="branch" />
            <wire x2="1696" y1="1488" y2="1488" x1="1520" />
            <wire x2="1712" y1="1488" y2="1488" x1="1696" />
        </branch>
        <branch name="clk_out">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="880" y="1552" type="branch" />
            <wire x2="896" y1="1552" y2="1552" x1="880" />
            <wire x2="992" y1="1552" y2="1552" x1="896" />
        </branch>
        <instance x="992" y="1712" name="XLXI_241" orien="R0">
        </instance>
        <branch name="EppWAIT">
            <wire x2="1840" y1="464" y2="464" x1="1712" />
        </branch>
        <branch name="DB(7:0)">
            <wire x2="1840" y1="528" y2="528" x1="1712" />
        </branch>
        <instance x="1248" y="1072" name="XLXI_255" orien="R0">
        </instance>
        <branch name="swInternal(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1856" y="656" type="branch" />
            <wire x2="1856" y1="656" y2="656" x1="1712" />
        </branch>
        <rect style="linewidth:W;linecolor:rgb(255,0,0)" width="631" x="1452" y="620" height="192" />
        <branch name="btnInternal(4:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1856" y="720" type="branch" />
            <wire x2="1856" y1="720" y2="720" x1="1712" />
        </branch>
        <branch name="RxInternal">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1856" y="784" type="branch" />
            <wire x2="1856" y1="784" y2="784" x1="1712" />
        </branch>
        <branch name="Led(7:0)">
            <wire x2="1856" y1="848" y2="848" x1="1712" />
        </branch>
        <branch name="seg(7:0)">
            <wire x2="1856" y1="912" y2="912" x1="1712" />
        </branch>
        <branch name="an(3:0)">
            <wire x2="1856" y1="976" y2="976" x1="1712" />
        </branch>
        <branch name="RsTx">
            <wire x2="1840" y1="1168" y2="1168" x1="1712" />
        </branch>
        <branch name="EppASTB">
            <wire x2="1248" y1="464" y2="464" x1="1056" />
        </branch>
        <branch name="EppDSTB">
            <wire x2="1248" y1="528" y2="528" x1="1056" />
        </branch>
        <branch name="sw(7:0)">
            <wire x2="1248" y1="656" y2="656" x1="1024" />
        </branch>
        <branch name="btn(4:0)">
            <wire x2="1248" y1="720" y2="720" x1="1024" />
        </branch>
        <branch name="EppWRITE">
            <wire x2="1248" y1="592" y2="592" x1="1072" />
        </branch>
        <branch name="clk_out">
            <attrtext style="alignment:SOFT-RIGHT" attrname="Name" x="1088" y="400" type="branch" />
            <wire x2="1248" y1="400" y2="400" x1="1088" />
        </branch>
        <branch name="LedInternal(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1088" y="848" type="branch" />
            <wire x2="1248" y1="848" y2="848" x1="1088" />
        </branch>
        <branch name="seven_seg3(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1088" y="912" type="branch" />
            <wire x2="1248" y1="912" y2="912" x1="1088" />
        </branch>
        <branch name="seven_seg2(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1088" y="976" type="branch" />
            <wire x2="1248" y1="976" y2="976" x1="1088" />
        </branch>
        <branch name="seven_seg1(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1088" y="1040" type="branch" />
            <wire x2="1248" y1="1040" y2="1040" x1="1088" />
        </branch>
        <branch name="seven_seg0(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1088" y="1104" type="branch" />
            <wire x2="1248" y1="1104" y2="1104" x1="1088" />
        </branch>
        <branch name="RsRx">
            <wire x2="1248" y1="784" y2="784" x1="992" />
        </branch>
        <rect style="linewidth:W;linecolor:rgb(255,0,0)" width="680" x="836" y="816" height="376" />
        <instance x="1376" y="256" name="XLXI_29" orien="R0" />
        <branch name="clk_out">
            <attrtext style="alignment:SOFT-LEFT" attrname="Name" x="1776" y="224" type="branch" />
            <wire x2="1776" y1="224" y2="224" x1="1600" />
        </branch>
        <branch name="clk">
            <wire x2="1376" y1="224" y2="224" x1="1152" />
        </branch>
        <text style="fontsize:48;fontname:Arial;textcolor:rgb(255,0,0)" x="615" y="1008">Outputs</text>
        <branch name="XLXN_289">
            <wire x2="736" y1="1152" y2="1168" x1="736" />
            <wire x2="1248" y1="1168" y2="1168" x1="736" />
        </branch>
        <instance x="672" y="1152" name="XLXI_262" orien="R0" />
        <text style="fontsize:48;fontname:Arial;textcolor:rgb(255,0,0)" x="2119" y="712">Inputs</text>
        <iomarker fontsize="28" x="1840" y="464" name="EppWAIT" orien="R0" />
        <iomarker fontsize="28" x="1840" y="528" name="DB(7:0)" orien="R0" />
        <iomarker fontsize="28" x="1856" y="848" name="Led(7:0)" orien="R0" />
        <iomarker fontsize="28" x="1856" y="912" name="seg(7:0)" orien="R0" />
        <iomarker fontsize="28" x="1856" y="976" name="an(3:0)" orien="R0" />
        <iomarker fontsize="28" x="1840" y="1168" name="RsTx" orien="R0" />
        <iomarker fontsize="28" x="1056" y="464" name="EppASTB" orien="R180" />
        <iomarker fontsize="28" x="1056" y="528" name="EppDSTB" orien="R180" />
        <iomarker fontsize="28" x="1072" y="592" name="EppWRITE" orien="R180" />
        <iomarker fontsize="28" x="1024" y="656" name="sw(7:0)" orien="R180" />
        <iomarker fontsize="28" x="1024" y="720" name="btn(4:0)" orien="R180" />
        <iomarker fontsize="28" x="992" y="784" name="RsRx" orien="R180" />
        <iomarker fontsize="28" x="1152" y="224" name="clk" orien="R180" />
        <branch name="LedInternal(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1760" y="1424" type="branch" />
            <wire x2="1568" y1="1424" y2="1424" x1="1520" />
            <wire x2="1584" y1="1424" y2="1424" x1="1568" />
            <wire x2="1744" y1="1424" y2="1424" x1="1584" />
            <wire x2="1760" y1="1424" y2="1424" x1="1744" />
        </branch>
    </sheet>
</drawing>