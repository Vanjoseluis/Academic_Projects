<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="swInternal(7:0)" />
        <signal name="btnInternal(4:0)" />
        <signal name="seven_seg2(7:0)" />
        <signal name="seven_seg0(7:0)" />
        <signal name="seven_seg1(7:0)" />
        <signal name="seven_seg3(7:0)" />
        <signal name="clk_out" />
        <signal name="EppWAIT" />
        <signal name="DB(7:0)" />
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
        <blockdef name="bufg">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="0" x1="64" />
            <line x2="64" y1="-32" y2="-64" x1="128" />
            <line x2="128" y1="0" y2="-32" x1="64" />
            <line x2="128" y1="-32" y2="-32" x1="224" />
            <line x2="64" y1="-32" y2="-32" x1="0" />
        </blockdef>
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
        <blockdef name="vcc">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-64" x1="64" />
            <line x2="64" y1="0" y2="-32" x1="64" />
            <line x2="32" y1="-64" y2="-64" x1="96" />
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
        <block symbolname="bufg" name="XLXI_29">
            <blockpin signalname="clk" name="I" />
            <blockpin signalname="clk_out" name="O" />
        </block>
        <block symbolname="vcc" name="XLXI_262">
            <blockpin signalname="XLXN_289" name="P" />
        </block>
        <block symbolname="Remote_Lab" name="XLXI_264">
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
    </netlist>
    <sheet sheetnum="1" width="2688" height="1900">
        <attr value="CM" name="LengthUnitName" />
        <attr value="4" name="GridsPerUnit" />
        <branch name="swInternal(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="704" y="1344" type="branch" />
            <wire x2="720" y1="1344" y2="1344" x1="704" />
            <wire x2="816" y1="1344" y2="1344" x1="720" />
        </branch>
        <branch name="btnInternal(4:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="704" y="1408" type="branch" />
            <wire x2="720" y1="1408" y2="1408" x1="704" />
            <wire x2="816" y1="1408" y2="1408" x1="720" />
        </branch>
        <branch name="seven_seg2(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1536" y="1472" type="branch" />
            <wire x2="1520" y1="1472" y2="1472" x1="1344" />
            <wire x2="1536" y1="1472" y2="1472" x1="1520" />
        </branch>
        <branch name="seven_seg0(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1552" y="1600" type="branch" />
            <wire x2="1536" y1="1600" y2="1600" x1="1344" />
            <wire x2="1552" y1="1600" y2="1600" x1="1536" />
        </branch>
        <branch name="seven_seg1(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1552" y="1536" type="branch" />
            <wire x2="1536" y1="1536" y2="1536" x1="1344" />
            <wire x2="1552" y1="1536" y2="1536" x1="1536" />
        </branch>
        <branch name="seven_seg3(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1536" y="1408" type="branch" />
            <wire x2="1520" y1="1408" y2="1408" x1="1344" />
            <wire x2="1536" y1="1408" y2="1408" x1="1520" />
        </branch>
        <branch name="clk_out">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="704" y="1472" type="branch" />
            <wire x2="720" y1="1472" y2="1472" x1="704" />
            <wire x2="816" y1="1472" y2="1472" x1="720" />
        </branch>
        <instance x="816" y="1632" name="XLXI_241" orien="R0">
        </instance>
        <branch name="EppWAIT">
            <wire x2="1664" y1="384" y2="384" x1="1536" />
        </branch>
        <branch name="DB(7:0)">
            <wire x2="1664" y1="448" y2="448" x1="1536" />
        </branch>
        <branch name="swInternal(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1680" y="576" type="branch" />
            <wire x2="1680" y1="576" y2="576" x1="1536" />
        </branch>
        <rect style="linewidth:W;linecolor:rgb(255,0,0)" width="631" x="1276" y="540" height="192" />
        <branch name="btnInternal(4:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1680" y="640" type="branch" />
            <wire x2="1680" y1="640" y2="640" x1="1536" />
        </branch>
        <branch name="RxInternal">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1680" y="704" type="branch" />
            <wire x2="1680" y1="704" y2="704" x1="1536" />
        </branch>
        <branch name="Led(7:0)">
            <wire x2="1680" y1="768" y2="768" x1="1536" />
        </branch>
        <branch name="seg(7:0)">
            <wire x2="1680" y1="832" y2="832" x1="1536" />
        </branch>
        <branch name="an(3:0)">
            <wire x2="1680" y1="896" y2="896" x1="1536" />
        </branch>
        <branch name="RsTx">
            <wire x2="1664" y1="1088" y2="1088" x1="1536" />
        </branch>
        <branch name="EppASTB">
            <wire x2="1072" y1="384" y2="384" x1="880" />
        </branch>
        <branch name="EppDSTB">
            <wire x2="1072" y1="448" y2="448" x1="880" />
        </branch>
        <branch name="sw(7:0)">
            <wire x2="1072" y1="576" y2="576" x1="848" />
        </branch>
        <branch name="btn(4:0)">
            <wire x2="1072" y1="640" y2="640" x1="848" />
        </branch>
        <branch name="EppWRITE">
            <wire x2="1072" y1="512" y2="512" x1="896" />
        </branch>
        <branch name="clk_out">
            <attrtext style="alignment:SOFT-RIGHT" attrname="Name" x="912" y="320" type="branch" />
            <wire x2="1072" y1="320" y2="320" x1="912" />
        </branch>
        <branch name="LedInternal(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="912" y="768" type="branch" />
            <wire x2="1072" y1="768" y2="768" x1="912" />
        </branch>
        <branch name="seven_seg3(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="912" y="832" type="branch" />
            <wire x2="1072" y1="832" y2="832" x1="912" />
        </branch>
        <branch name="seven_seg2(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="912" y="896" type="branch" />
            <wire x2="1072" y1="896" y2="896" x1="912" />
        </branch>
        <branch name="seven_seg1(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="912" y="960" type="branch" />
            <wire x2="1072" y1="960" y2="960" x1="912" />
        </branch>
        <branch name="seven_seg0(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="912" y="1024" type="branch" />
            <wire x2="1072" y1="1024" y2="1024" x1="912" />
        </branch>
        <branch name="RsRx">
            <wire x2="1072" y1="704" y2="704" x1="816" />
        </branch>
        <rect style="linewidth:W;linecolor:rgb(255,0,0)" width="680" x="660" y="736" height="376" />
        <instance x="1200" y="176" name="XLXI_29" orien="R0" />
        <branch name="clk_out">
            <attrtext style="alignment:SOFT-LEFT" attrname="Name" x="1600" y="144" type="branch" />
            <wire x2="1600" y1="144" y2="144" x1="1424" />
        </branch>
        <branch name="clk">
            <wire x2="1200" y1="144" y2="144" x1="976" />
        </branch>
        <text style="fontsize:48;fontname:Arial;textcolor:rgb(255,0,0)" x="439" y="928">Outputs</text>
        <branch name="XLXN_289">
            <wire x2="560" y1="1072" y2="1088" x1="560" />
            <wire x2="1072" y1="1088" y2="1088" x1="560" />
        </branch>
        <instance x="496" y="1072" name="XLXI_262" orien="R0" />
        <text style="fontsize:48;fontname:Arial;textcolor:rgb(255,0,0)" x="1943" y="632">Inputs</text>
        <branch name="LedInternal(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1584" y="1344" type="branch" />
            <wire x2="1392" y1="1344" y2="1344" x1="1344" />
            <wire x2="1408" y1="1344" y2="1344" x1="1392" />
            <wire x2="1568" y1="1344" y2="1344" x1="1408" />
            <wire x2="1584" y1="1344" y2="1344" x1="1568" />
        </branch>
        <iomarker fontsize="28" x="1664" y="384" name="EppWAIT" orien="R0" />
        <iomarker fontsize="28" x="1664" y="448" name="DB(7:0)" orien="R0" />
        <iomarker fontsize="28" x="1680" y="768" name="Led(7:0)" orien="R0" />
        <iomarker fontsize="28" x="1680" y="832" name="seg(7:0)" orien="R0" />
        <iomarker fontsize="28" x="1680" y="896" name="an(3:0)" orien="R0" />
        <iomarker fontsize="28" x="1664" y="1088" name="RsTx" orien="R0" />
        <iomarker fontsize="28" x="880" y="384" name="EppASTB" orien="R180" />
        <iomarker fontsize="28" x="880" y="448" name="EppDSTB" orien="R180" />
        <iomarker fontsize="28" x="896" y="512" name="EppWRITE" orien="R180" />
        <iomarker fontsize="28" x="848" y="576" name="sw(7:0)" orien="R180" />
        <iomarker fontsize="28" x="848" y="640" name="btn(4:0)" orien="R180" />
        <iomarker fontsize="28" x="816" y="704" name="RsRx" orien="R180" />
        <iomarker fontsize="28" x="976" y="144" name="clk" orien="R180" />
        <instance x="1072" y="992" name="XLXI_264" orien="R0">
        </instance>
    </sheet>
</drawing>