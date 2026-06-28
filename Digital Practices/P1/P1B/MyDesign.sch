<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="sw(7:0)" />
        <signal name="btn(4:0)" />
        <signal name="Leds(7:0)" />
        <signal name="SevenSeg3(7:0)" />
        <signal name="SevenSeg2(7:0)" />
        <signal name="SevenSeg1(7:0)" />
        <signal name="SevenSeg0(7:0)" />
        <signal name="Ck" />
        <signal name="VLed(7:0)" />
        <signal name="V_R(7:0)" />
        <signal name="btn(0)" />
        <signal name="V_RR" />
        <signal name="On_Off" />
        <signal name="VRes(7:0)" />
        <signal name="btn(3)" />
        <signal name="XLXN_134" />
        <signal name="btn(2)" />
        <signal name="btn(4)" />
        <signal name="XLXN_7" />
        <signal name="VF_C(7:0)" />
        <signal name="VRes(7:5)" />
        <signal name="VRes(4:3)" />
        <signal name="VRes(2:0)" />
        <signal name="XLXN_171(2:0)" />
        <signal name="XLXN_172(1:0)" />
        <signal name="XLXN_173(2:0)" />
        <port polarity="Input" name="sw(7:0)" />
        <port polarity="Input" name="btn(4:0)" />
        <port polarity="Output" name="Leds(7:0)" />
        <port polarity="Output" name="SevenSeg3(7:0)" />
        <port polarity="Output" name="SevenSeg2(7:0)" />
        <port polarity="Output" name="SevenSeg1(7:0)" />
        <port polarity="Output" name="SevenSeg0(7:0)" />
        <port polarity="Input" name="Ck" />
        <blockdef name="clk_div">
            <timestamp>2014-3-10T11:8:13</timestamp>
            <rect width="256" x="64" y="-128" height="128" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
            <line x2="384" y1="-32" y2="-32" x1="320" />
        </blockdef>
        <blockdef name="buf">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="128" y1="-32" y2="-32" x1="224" />
            <line x2="128" y1="0" y2="-32" x1="64" />
            <line x2="64" y1="-32" y2="-64" x1="128" />
            <line x2="64" y1="-64" y2="0" x1="64" />
        </blockdef>
        <blockdef name="Puls_On_Off">
            <timestamp>2016-4-1T8:6:46</timestamp>
            <rect width="256" x="64" y="-128" height="128" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
        </blockdef>
        <blockdef name="Regs_Vot">
            <timestamp>2023-3-12T19:40:13</timestamp>
            <rect width="256" x="64" y="-320" height="320" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="384" y1="-224" y2="-224" x1="320" />
            <rect width="64" x="320" y="-236" height="24" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <rect width="64" x="0" y="-172" height="24" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <line x2="0" y1="-288" y2="-288" x1="64" />
            <line x2="384" y1="-288" y2="-288" x1="320" />
            <rect width="64" x="320" y="-300" height="24" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
            <rect width="64" x="320" y="-172" height="24" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
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
        <blockdef name="Ctrl_Leds">
            <timestamp>2023-3-13T12:4:39</timestamp>
            <rect width="256" x="64" y="-384" height="384" />
            <line x2="384" y1="-352" y2="-352" x1="320" />
            <rect width="64" x="320" y="-364" height="24" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="0" y1="-352" y2="-352" x1="64" />
            <rect width="64" x="0" y="-364" height="24" />
            <line x2="0" y1="-288" y2="-288" x1="64" />
            <rect width="64" x="0" y="-300" height="24" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <rect width="64" x="0" y="-236" height="24" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
        </blockdef>
        <blockdef name="Modulo_general">
            <timestamp>2026-4-16T0:36:58</timestamp>
            <rect width="256" x="64" y="-448" height="448" />
            <line x2="0" y1="-416" y2="-416" x1="64" />
            <rect width="64" x="0" y="-428" height="24" />
            <line x2="384" y1="-352" y2="-352" x1="320" />
            <rect width="64" x="320" y="-364" height="24" />
            <line x2="384" y1="-32" y2="-32" x1="320" />
            <rect width="64" x="320" y="-44" height="24" />
            <line x2="384" y1="-96" y2="-96" x1="320" />
            <rect width="64" x="320" y="-108" height="24" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
            <rect width="64" x="320" y="-172" height="24" />
            <line x2="384" y1="-224" y2="-224" x1="320" />
            <rect width="64" x="320" y="-236" height="24" />
            <line x2="384" y1="-288" y2="-288" x1="320" />
            <rect width="64" x="320" y="-300" height="24" />
            <line x2="384" y1="-416" y2="-416" x1="320" />
            <rect width="64" x="320" y="-428" height="24" />
            <line x2="0" y1="-352" y2="-352" x1="64" />
        </blockdef>
        <block symbolname="clk_div" name="XLXI_16">
            <blockpin signalname="Ck" name="clk_in" />
            <blockpin name="div_1hz" />
            <blockpin signalname="XLXN_7" name="div_4hz" />
        </block>
        <block symbolname="Puls_On_Off" name="XLXI_255">
            <blockpin signalname="Ck" name="ck" />
            <blockpin signalname="btn(0)" name="EN" />
            <blockpin signalname="XLXN_134" name="ENS" />
        </block>
        <block symbolname="inv" name="XLXI_278">
            <blockpin signalname="XLXN_134" name="I" />
            <blockpin signalname="On_Off" name="O" />
        </block>
        <block symbolname="Ctrl_Leds" name="XLXI_290">
            <blockpin signalname="Leds(7:0)" name="Leds(7:0)" />
            <blockpin signalname="On_Off" name="On_Off" />
            <blockpin signalname="btn(3)" name="VerVot" />
            <blockpin signalname="VLed(7:0)" name="VLed(7:0)" />
            <blockpin signalname="VRes(7:0)" name="VRes(7:0)" />
            <blockpin signalname="V_R(7:0)" name="V_R(7:0)" />
            <blockpin signalname="V_RR" name="V_RR" />
        </block>
        <block symbolname="Regs_Vot" name="XLXI_269">
            <blockpin signalname="XLXN_7" name="CK" />
            <blockpin signalname="On_Off" name="CLR" />
            <blockpin signalname="VLed(7:0)" name="Led_V(7:0)" />
            <blockpin signalname="sw(7:0)" name="Sel(7:0)" />
            <blockpin signalname="btn(4)" name="VC" />
            <blockpin signalname="btn(2)" name="VF" />
            <blockpin signalname="VF_C(7:0)" name="VF_C(7:0)" />
            <blockpin signalname="V_R(7:0)" name="V_R(7:0)" />
            <blockpin signalname="V_RR" name="V_RR" />
        </block>
        <block symbolname="Modulo_general" name="XLXI_291">
            <blockpin signalname="VF_C(7:0)" name="M(7:0)" />
            <blockpin signalname="XLXN_172(1:0)" name="NA(1:0)" />
            <blockpin signalname="SevenSeg1(7:0)" name="SevenSeg_C(7:0)" />
            <blockpin signalname="SevenSeg3(7:0)" name="SevenSeg_F(7:0)" />
            <blockpin signalname="SevenSeg0(7:0)" name="SevenSeg_VC(7:0)" />
            <blockpin signalname="SevenSeg2(7:0)" name="SevenSeg_VF(7:0)" />
            <blockpin signalname="XLXN_173(2:0)" name="VC(2:0)" />
            <blockpin signalname="XLXN_171(2:0)" name="VF(2:0)" />
            <blockpin signalname="V_RR" name="VVal" />
        </block>
        <block symbolname="buf" name="XLXI_313(2:0)">
            <blockpin signalname="XLXN_171(2:0)" name="I" />
            <blockpin signalname="VRes(7:5)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_314(1:0)">
            <blockpin signalname="XLXN_172(1:0)" name="I" />
            <blockpin signalname="VRes(4:3)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_315(2:0)">
            <blockpin signalname="XLXN_173(2:0)" name="I" />
            <blockpin signalname="VRes(2:0)" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="2720" height="1760">
        <attr value="Inch" name="LengthUnitName" />
        <attr value="10" name="GridsPerUnit" />
        <branch name="sw(7:0)">
            <wire x2="368" y1="64" y2="64" x1="224" />
        </branch>
        <branch name="btn(4:0)">
            <wire x2="384" y1="144" y2="144" x1="224" />
        </branch>
        <iomarker fontsize="28" x="224" y="64" name="sw(7:0)" orien="R180" />
        <iomarker fontsize="28" x="224" y="144" name="btn(4:0)" orien="R180" />
        <branch name="Leds(7:0)">
            <wire x2="624" y1="64" y2="64" x1="464" />
        </branch>
        <branch name="SevenSeg3(7:0)">
            <wire x2="624" y1="144" y2="144" x1="464" />
        </branch>
        <branch name="SevenSeg2(7:0)">
            <wire x2="624" y1="224" y2="224" x1="464" />
        </branch>
        <branch name="SevenSeg1(7:0)">
            <wire x2="624" y1="304" y2="304" x1="464" />
        </branch>
        <branch name="SevenSeg0(7:0)">
            <wire x2="624" y1="384" y2="384" x1="464" />
        </branch>
        <iomarker fontsize="28" x="624" y="64" name="Leds(7:0)" orien="R0" />
        <iomarker fontsize="28" x="624" y="144" name="SevenSeg3(7:0)" orien="R0" />
        <iomarker fontsize="28" x="624" y="224" name="SevenSeg2(7:0)" orien="R0" />
        <iomarker fontsize="28" x="624" y="304" name="SevenSeg1(7:0)" orien="R0" />
        <iomarker fontsize="28" x="624" y="384" name="SevenSeg0(7:0)" orien="R0" />
        <branch name="Ck">
            <wire x2="384" y1="208" y2="208" x1="224" />
        </branch>
        <iomarker fontsize="28" x="224" y="208" name="Ck" orien="R180" />
        <instance x="224" y="896" name="XLXI_16" orien="R0">
        </instance>
        <branch name="Ck">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="128" y="800" type="branch" />
            <wire x2="192" y1="800" y2="800" x1="128" />
            <wire x2="224" y1="800" y2="800" x1="192" />
            <wire x2="192" y1="800" y2="1104" x1="192" />
            <wire x2="224" y1="1104" y2="1104" x1="192" />
        </branch>
        <instance x="224" y="1136" name="XLXI_255" orien="R0">
        </instance>
        <branch name="btn(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="144" y="1040" type="branch" />
            <wire x2="224" y1="1040" y2="1040" x1="144" />
        </branch>
        <branch name="V_RR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1536" y="1360" type="branch" />
            <wire x2="1664" y1="1360" y2="1360" x1="1536" />
        </branch>
        <branch name="V_R(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1536" y="1296" type="branch" />
            <wire x2="1664" y1="1296" y2="1296" x1="1536" />
        </branch>
        <branch name="VRes(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1520" y="1232" type="branch" />
            <wire x2="1664" y1="1232" y2="1232" x1="1520" />
        </branch>
        <branch name="VLed(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1520" y="1168" type="branch" />
            <wire x2="1664" y1="1168" y2="1168" x1="1520" />
        </branch>
        <branch name="Leds(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2160" y="1168" type="branch" />
            <wire x2="2160" y1="1168" y2="1168" x1="2048" />
        </branch>
        <instance x="1664" y="1520" name="XLXI_290" orien="R0">
        </instance>
        <branch name="btn(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1536" y="1424" type="branch" />
            <wire x2="1664" y1="1424" y2="1424" x1="1536" />
        </branch>
        <branch name="On_Off">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1536" y="1488" type="branch" />
            <wire x2="1664" y1="1488" y2="1488" x1="1536" />
        </branch>
        <branch name="XLXN_134">
            <wire x2="640" y1="1040" y2="1040" x1="608" />
        </branch>
        <branch name="btn(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="784" y="608" type="branch" />
            <wire x2="800" y1="608" y2="608" x1="784" />
            <wire x2="896" y1="608" y2="608" x1="800" />
        </branch>
        <branch name="btn(4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="784" y="672" type="branch" />
            <wire x2="800" y1="672" y2="672" x1="784" />
            <wire x2="896" y1="672" y2="672" x1="800" />
        </branch>
        <branch name="sw(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="784" y="736" type="branch" />
            <wire x2="800" y1="736" y2="736" x1="784" />
            <wire x2="896" y1="736" y2="736" x1="800" />
        </branch>
        <branch name="XLXN_7">
            <wire x2="624" y1="864" y2="864" x1="608" />
            <wire x2="896" y1="864" y2="864" x1="624" />
        </branch>
        <branch name="V_RR">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="1344" y="800" type="branch" />
            <wire x2="1344" y1="800" y2="800" x1="1280" />
            <wire x2="1520" y1="800" y2="800" x1="1344" />
            <wire x2="1520" y1="672" y2="800" x1="1520" />
            <wire x2="1664" y1="672" y2="672" x1="1520" />
        </branch>
        <branch name="On_Off">
            <attrtext style="alignment:SOFT-TVCENTER;fontsize:28;fontname:Arial" attrname="Name" x="880" y="992" type="branch" />
            <wire x2="880" y1="1040" y2="1040" x1="864" />
            <wire x2="880" y1="800" y2="992" x1="880" />
            <wire x2="880" y1="992" y2="1040" x1="880" />
            <wire x2="896" y1="800" y2="800" x1="880" />
        </branch>
        <instance x="640" y="1072" name="XLXI_278" orien="R0" />
        <branch name="VF_C(7:0)">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="1376" y="608" type="branch" />
            <wire x2="1376" y1="608" y2="608" x1="1280" />
            <wire x2="1664" y1="608" y2="608" x1="1376" />
        </branch>
        <branch name="VLed(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1360" y="672" type="branch" />
            <wire x2="1344" y1="672" y2="672" x1="1280" />
            <wire x2="1360" y1="672" y2="672" x1="1344" />
        </branch>
        <instance x="1664" y="1024" name="XLXI_291" orien="R0">
        </instance>
        <branch name="V_R(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1360" y="736" type="branch" />
            <wire x2="1296" y1="736" y2="736" x1="1280" />
            <wire x2="1344" y1="736" y2="736" x1="1296" />
            <wire x2="1360" y1="736" y2="736" x1="1344" />
        </branch>
        <branch name="SevenSeg2(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2240" y="800" type="branch" />
            <wire x2="2240" y1="800" y2="800" x1="2048" />
        </branch>
        <branch name="SevenSeg0(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2240" y="864" type="branch" />
            <wire x2="2240" y1="864" y2="864" x1="2048" />
        </branch>
        <branch name="SevenSeg3(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2240" y="928" type="branch" />
            <wire x2="2240" y1="928" y2="928" x1="2048" />
        </branch>
        <branch name="SevenSeg1(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2240" y="992" type="branch" />
            <wire x2="2240" y1="992" y2="992" x1="2048" />
        </branch>
        <instance x="896" y="896" name="XLXI_269" orien="R0">
        </instance>
        <instance x="2080" y="640" name="XLXI_313(2:0)" orien="R0" />
        <instance x="2080" y="704" name="XLXI_314(1:0)" orien="R0" />
        <branch name="VRes(7:5)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2368" y="608" type="branch" />
            <wire x2="2368" y1="608" y2="608" x1="2304" />
        </branch>
        <branch name="VRes(4:3)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2368" y="672" type="branch" />
            <wire x2="2368" y1="672" y2="672" x1="2304" />
        </branch>
        <branch name="VRes(2:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2368" y="736" type="branch" />
            <wire x2="2368" y1="736" y2="736" x1="2304" />
        </branch>
        <instance x="2080" y="768" name="XLXI_315(2:0)" orien="R0" />
        <branch name="XLXN_171(2:0)">
            <wire x2="2080" y1="608" y2="608" x1="2048" />
        </branch>
        <branch name="XLXN_172(1:0)">
            <wire x2="2080" y1="672" y2="672" x1="2048" />
        </branch>
        <branch name="XLXN_173(2:0)">
            <wire x2="2080" y1="736" y2="736" x1="2048" />
        </branch>
    </sheet>
</drawing>