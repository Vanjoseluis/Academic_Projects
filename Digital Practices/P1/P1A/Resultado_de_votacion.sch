<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="SVC(3:0)" />
        <signal name="XLXN_2" />
        <signal name="XLXN_3" />
        <signal name="XLXN_4" />
        <signal name="SVC(0)" />
        <signal name="SVC(1)" />
        <signal name="SVC(2)" />
        <signal name="VF(0)" />
        <signal name="XLXN_15" />
        <signal name="XLXN_17" />
        <signal name="XLXN_25" />
        <signal name="XLXN_26" />
        <signal name="XLXN_27" />
        <signal name="XLXN_28" />
        <signal name="XLXN_31" />
        <signal name="VC(0)" />
        <signal name="SVC(3)" />
        <signal name="XLXN_39" />
        <signal name="XLXN_42" />
        <signal name="XLXN_44" />
        <signal name="XLXN_45" />
        <signal name="XLXN_46" />
        <signal name="XLXN_49" />
        <signal name="NA(0)" />
        <signal name="VVal" />
        <signal name="XLXN_50" />
        <signal name="XLXN_51" />
        <signal name="XLXN_52" />
        <signal name="VF(1)" />
        <signal name="VF(2)" />
        <signal name="VF(2:0)" />
        <signal name="VC(1)" />
        <signal name="VC(2)" />
        <signal name="VC(2:0)" />
        <signal name="NA(1)" />
        <signal name="NA(1:0)" />
        <port polarity="Input" name="SVC(3:0)" />
        <port polarity="Input" name="VVal" />
        <port polarity="Output" name="VF(2:0)" />
        <port polarity="Output" name="VC(2:0)" />
        <port polarity="Output" name="NA(1:0)" />
        <blockdef name="and3">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-64" x1="0" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="192" y1="-128" y2="-128" x1="256" />
            <line x2="144" y1="-176" y2="-176" x1="64" />
            <line x2="64" y1="-80" y2="-80" x1="144" />
            <arc ex="144" ey="-176" sx="144" sy="-80" r="48" cx="144" cy="-128" />
            <line x2="64" y1="-64" y2="-192" x1="64" />
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
        <blockdef name="and4">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-112" y2="-112" x1="144" />
            <arc ex="144" ey="-208" sx="144" sy="-112" r="48" cx="144" cy="-160" />
            <line x2="144" y1="-208" y2="-208" x1="64" />
            <line x2="64" y1="-64" y2="-256" x1="64" />
            <line x2="192" y1="-160" y2="-160" x1="256" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-64" y2="-64" x1="0" />
        </blockdef>
        <blockdef name="and2">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-64" x1="0" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="192" y1="-96" y2="-96" x1="256" />
            <arc ex="144" ey="-144" sx="144" sy="-48" r="48" cx="144" cy="-96" />
            <line x2="64" y1="-48" y2="-48" x1="144" />
            <line x2="144" y1="-144" y2="-144" x1="64" />
            <line x2="64" y1="-48" y2="-144" x1="64" />
        </blockdef>
        <blockdef name="buf">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="128" y1="-32" y2="-32" x1="224" />
            <line x2="128" y1="0" y2="-32" x1="64" />
            <line x2="64" y1="-32" y2="-64" x1="128" />
            <line x2="64" y1="-64" y2="0" x1="64" />
        </blockdef>
        <block symbolname="and3" name="XLXI_1">
            <blockpin signalname="XLXN_4" name="I0" />
            <blockpin signalname="XLXN_3" name="I1" />
            <blockpin signalname="XLXN_2" name="I2" />
            <blockpin signalname="XLXN_50" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_2">
            <blockpin signalname="SVC(3)" name="I" />
            <blockpin signalname="XLXN_2" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_3">
            <blockpin signalname="SVC(1)" name="I" />
            <blockpin signalname="XLXN_3" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_4">
            <blockpin signalname="SVC(2)" name="I" />
            <blockpin signalname="XLXN_4" name="O" />
        </block>
        <block symbolname="or2" name="XLXI_5">
            <blockpin signalname="XLXN_31" name="I0" />
            <blockpin signalname="XLXN_28" name="I1" />
            <blockpin signalname="XLXN_51" name="O" />
        </block>
        <block symbolname="and4" name="XLXI_6">
            <blockpin signalname="XLXN_27" name="I0" />
            <blockpin signalname="XLXN_26" name="I1" />
            <blockpin signalname="XLXN_25" name="I2" />
            <blockpin signalname="SVC(3)" name="I3" />
            <blockpin signalname="XLXN_31" name="O" />
        </block>
        <block symbolname="or2" name="XLXI_8">
            <blockpin signalname="SVC(1)" name="I0" />
            <blockpin signalname="SVC(0)" name="I1" />
            <blockpin signalname="XLXN_15" name="O" />
        </block>
        <block symbolname="and3" name="XLXI_15">
            <blockpin signalname="XLXN_15" name="I0" />
            <blockpin signalname="SVC(2)" name="I1" />
            <blockpin signalname="XLXN_17" name="I2" />
            <blockpin signalname="XLXN_28" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_16">
            <blockpin signalname="SVC(3)" name="I" />
            <blockpin signalname="XLXN_17" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_18">
            <blockpin signalname="SVC(1)" name="I" />
            <blockpin signalname="XLXN_27" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_19">
            <blockpin signalname="SVC(2)" name="I" />
            <blockpin signalname="XLXN_25" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_20">
            <blockpin signalname="SVC(0)" name="I" />
            <blockpin signalname="XLXN_26" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_21">
            <blockpin signalname="SVC(0)" name="I" />
            <blockpin signalname="XLXN_45" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_23">
            <blockpin signalname="SVC(2)" name="I" />
            <blockpin signalname="XLXN_39" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_24">
            <blockpin signalname="SVC(3)" name="I" />
            <blockpin signalname="XLXN_44" name="O" />
        </block>
        <block symbolname="and3" name="XLXI_25">
            <blockpin signalname="XLXN_44" name="I0" />
            <blockpin signalname="SVC(1)" name="I1" />
            <blockpin signalname="XLXN_39" name="I2" />
            <blockpin signalname="XLXN_49" name="O" />
        </block>
        <block symbolname="and4" name="XLXI_26">
            <blockpin signalname="XLXN_42" name="I0" />
            <blockpin signalname="XLXN_44" name="I1" />
            <blockpin signalname="XLXN_45" name="I2" />
            <blockpin signalname="SVC(2)" name="I3" />
            <blockpin signalname="XLXN_46" name="O" />
        </block>
        <block symbolname="or2" name="XLXI_27">
            <blockpin signalname="XLXN_49" name="I0" />
            <blockpin signalname="XLXN_46" name="I1" />
            <blockpin signalname="XLXN_52" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_28">
            <blockpin signalname="SVC(1)" name="I" />
            <blockpin signalname="XLXN_42" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_29">
            <blockpin signalname="XLXN_50" name="I0" />
            <blockpin signalname="VVal" name="I1" />
            <blockpin signalname="VF(0)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_32">
            <blockpin signalname="VF(0)" name="I" />
            <blockpin signalname="VF(1)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_33">
            <blockpin signalname="VF(0)" name="I" />
            <blockpin signalname="VF(2)" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_30">
            <blockpin signalname="XLXN_51" name="I0" />
            <blockpin signalname="VVal" name="I1" />
            <blockpin signalname="VC(0)" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_31">
            <blockpin signalname="XLXN_52" name="I0" />
            <blockpin signalname="VVal" name="I1" />
            <blockpin signalname="NA(0)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_37">
            <blockpin signalname="VC(0)" name="I" />
            <blockpin signalname="VC(1)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_38">
            <blockpin signalname="VC(0)" name="I" />
            <blockpin signalname="VC(2)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_45">
            <blockpin signalname="NA(0)" name="I" />
            <blockpin signalname="NA(1)" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <branch name="SVC(3:0)">
            <wire x2="448" y1="544" y2="912" x1="448" />
        </branch>
        <iomarker fontsize="28" x="448" y="544" name="SVC(3:0)" orien="R270" />
        <instance x="1184" y="720" name="XLXI_1" orien="R0" />
        <branch name="XLXN_2">
            <wire x2="1184" y1="528" y2="528" x1="1088" />
        </branch>
        <branch name="XLXN_3">
            <wire x2="1184" y1="592" y2="592" x1="1088" />
        </branch>
        <branch name="XLXN_4">
            <wire x2="1184" y1="656" y2="656" x1="1104" />
        </branch>
        <instance x="864" y="560" name="XLXI_2" orien="R0" />
        <instance x="864" y="624" name="XLXI_3" orien="R0" />
        <instance x="880" y="688" name="XLXI_4" orien="R0" />
        <branch name="SVC(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="816" y="528" type="branch" />
            <wire x2="864" y1="528" y2="528" x1="816" />
        </branch>
        <branch name="SVC(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="816" y="592" type="branch" />
            <wire x2="864" y1="592" y2="592" x1="816" />
        </branch>
        <branch name="SVC(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="816" y="656" type="branch" />
            <wire x2="880" y1="656" y2="656" x1="816" />
        </branch>
        <branch name="SVC(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="768" y="1168" type="branch" />
            <wire x2="928" y1="1168" y2="1168" x1="768" />
            <wire x2="1136" y1="1168" y2="1168" x1="928" />
            <wire x2="928" y1="1168" y2="1488" x1="928" />
            <wire x2="1104" y1="1488" y2="1488" x1="928" />
        </branch>
        <branch name="SVC(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="768" y="1232" type="branch" />
            <wire x2="864" y1="1232" y2="1232" x1="768" />
            <wire x2="1136" y1="1232" y2="1232" x1="864" />
            <wire x2="864" y1="1232" y2="1552" x1="864" />
            <wire x2="1104" y1="1552" y2="1552" x1="864" />
        </branch>
        <instance x="864" y="1056" name="XLXI_16" orien="R0" />
        <branch name="XLXN_15">
            <wire x2="1408" y1="1200" y2="1200" x1="1392" />
        </branch>
        <branch name="XLXN_17">
            <wire x2="1408" y1="1024" y2="1024" x1="1088" />
            <wire x2="1408" y1="1024" y2="1072" x1="1408" />
        </branch>
        <instance x="1376" y="1616" name="XLXI_6" orien="R0" />
        <instance x="1104" y="1456" name="XLXI_19" orien="R0" />
        <instance x="1104" y="1520" name="XLXI_20" orien="R0" />
        <branch name="SVC(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="736" y="1024" type="branch" />
            <wire x2="800" y1="1024" y2="1024" x1="736" />
            <wire x2="864" y1="1024" y2="1024" x1="800" />
            <wire x2="800" y1="1024" y2="1264" x1="800" />
            <wire x2="1376" y1="1264" y2="1264" x1="800" />
            <wire x2="1376" y1="1264" y2="1360" x1="1376" />
        </branch>
        <instance x="1104" y="1584" name="XLXI_18" orien="R0" />
        <instance x="1408" y="1264" name="XLXI_15" orien="R0" />
        <instance x="1136" y="1296" name="XLXI_8" orien="R0" />
        <branch name="SVC(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="768" y="1104" type="branch" />
            <wire x2="928" y1="1104" y2="1104" x1="768" />
            <wire x2="928" y1="1104" y2="1136" x1="928" />
            <wire x2="1008" y1="1136" y2="1136" x1="928" />
            <wire x2="1408" y1="1136" y2="1136" x1="1008" />
            <wire x2="1008" y1="1136" y2="1424" x1="1008" />
            <wire x2="1104" y1="1424" y2="1424" x1="1008" />
        </branch>
        <branch name="XLXN_25">
            <wire x2="1376" y1="1424" y2="1424" x1="1328" />
        </branch>
        <branch name="XLXN_26">
            <wire x2="1376" y1="1488" y2="1488" x1="1328" />
        </branch>
        <branch name="XLXN_27">
            <wire x2="1376" y1="1552" y2="1552" x1="1328" />
        </branch>
        <instance x="1728" y="1392" name="XLXI_5" orien="R0" />
        <branch name="XLXN_28">
            <wire x2="1696" y1="1136" y2="1136" x1="1664" />
            <wire x2="1696" y1="1136" y2="1264" x1="1696" />
            <wire x2="1728" y1="1264" y2="1264" x1="1696" />
        </branch>
        <branch name="XLXN_31">
            <wire x2="1680" y1="1456" y2="1456" x1="1632" />
            <wire x2="1680" y1="1328" y2="1456" x1="1680" />
            <wire x2="1728" y1="1328" y2="1328" x1="1680" />
        </branch>
        <instance x="912" y="2064" name="XLXI_21" orien="R0" />
        <instance x="912" y="2240" name="XLXI_23" orien="R0" />
        <instance x="912" y="2384" name="XLXI_24" orien="R0" />
        <branch name="SVC(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="736" y="2032" type="branch" />
            <wire x2="912" y1="2032" y2="2032" x1="736" />
        </branch>
        <branch name="SVC(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="736" y="2208" type="branch" />
            <wire x2="800" y1="2208" y2="2208" x1="736" />
            <wire x2="912" y1="2208" y2="2208" x1="800" />
            <wire x2="1328" y1="1936" y2="1936" x1="800" />
            <wire x2="800" y1="1936" y2="2208" x1="800" />
        </branch>
        <branch name="SVC(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="736" y="2352" type="branch" />
            <wire x2="912" y1="2352" y2="2352" x1="736" />
        </branch>
        <branch name="SVC(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="720" y="2128" type="branch" />
            <wire x2="864" y1="2128" y2="2128" x1="720" />
            <wire x2="912" y1="2128" y2="2128" x1="864" />
            <wire x2="864" y1="2128" y2="2272" x1="864" />
            <wire x2="1280" y1="2272" y2="2272" x1="864" />
        </branch>
        <instance x="912" y="2160" name="XLXI_28" orien="R0" />
        <instance x="1280" y="2400" name="XLXI_25" orien="R0" />
        <branch name="XLXN_39">
            <wire x2="1280" y1="2208" y2="2208" x1="1136" />
        </branch>
        <instance x="1328" y="2192" name="XLXI_26" orien="R0" />
        <branch name="XLXN_42">
            <wire x2="1328" y1="2128" y2="2128" x1="1136" />
        </branch>
        <branch name="XLXN_44">
            <wire x2="1248" y1="2352" y2="2352" x1="1136" />
            <wire x2="1280" y1="2352" y2="2352" x1="1248" />
            <wire x2="1328" y1="2064" y2="2064" x1="1248" />
            <wire x2="1248" y1="2064" y2="2352" x1="1248" />
            <wire x2="1280" y1="2336" y2="2352" x1="1280" />
        </branch>
        <branch name="XLXN_45">
            <wire x2="1232" y1="2032" y2="2032" x1="1136" />
            <wire x2="1232" y1="2000" y2="2032" x1="1232" />
            <wire x2="1328" y1="2000" y2="2000" x1="1232" />
        </branch>
        <instance x="1616" y="2256" name="XLXI_27" orien="R0" />
        <branch name="XLXN_46">
            <wire x2="1600" y1="2032" y2="2032" x1="1584" />
            <wire x2="1600" y1="2032" y2="2128" x1="1600" />
            <wire x2="1616" y1="2128" y2="2128" x1="1600" />
        </branch>
        <branch name="XLXN_49">
            <wire x2="1568" y1="2272" y2="2272" x1="1536" />
            <wire x2="1568" y1="2192" y2="2272" x1="1568" />
            <wire x2="1616" y1="2192" y2="2192" x1="1568" />
        </branch>
        <instance x="1888" y="656" name="XLXI_29" orien="R0" />
        <branch name="VVal">
            <wire x2="240" y1="432" y2="640" x1="240" />
        </branch>
        <iomarker fontsize="28" x="240" y="432" name="VVal" orien="R270" />
        <branch name="VVal">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1760" y="528" type="branch" />
            <wire x2="1888" y1="528" y2="528" x1="1760" />
        </branch>
        <branch name="VC(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2816" y="1264" type="branch" />
            <wire x2="2560" y1="1264" y2="1264" x1="2432" />
            <wire x2="2816" y1="1264" y2="1264" x1="2560" />
            <wire x2="2560" y1="1264" y2="1376" x1="2560" />
            <wire x2="2560" y1="1376" y2="1392" x1="2560" />
            <wire x2="2560" y1="1392" y2="1488" x1="2560" />
            <wire x2="2576" y1="1488" y2="1488" x1="2560" />
            <wire x2="2576" y1="1376" y2="1376" x1="2560" />
        </branch>
        <branch name="VF(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2544" y="560" type="branch" />
            <wire x2="2256" y1="560" y2="560" x1="2144" />
            <wire x2="2256" y1="560" y2="672" x1="2256" />
            <wire x2="2256" y1="672" y2="784" x1="2256" />
            <wire x2="2304" y1="784" y2="784" x1="2256" />
            <wire x2="2304" y1="672" y2="672" x1="2256" />
            <wire x2="2544" y1="560" y2="560" x1="2256" />
        </branch>
        <branch name="NA(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2672" y="2128" type="branch" />
            <wire x2="2400" y1="2128" y2="2128" x1="2320" />
            <wire x2="2608" y1="2128" y2="2128" x1="2400" />
            <wire x2="2672" y1="2128" y2="2128" x1="2608" />
            <wire x2="2400" y1="2128" y2="2256" x1="2400" />
            <wire x2="2432" y1="2256" y2="2256" x1="2400" />
        </branch>
        <branch name="XLXN_50">
            <wire x2="1888" y1="592" y2="592" x1="1440" />
        </branch>
        <branch name="XLXN_51">
            <wire x2="2000" y1="1296" y2="1296" x1="1984" />
            <wire x2="2176" y1="1296" y2="1296" x1="2000" />
        </branch>
        <branch name="XLXN_52">
            <wire x2="1888" y1="2160" y2="2160" x1="1872" />
            <wire x2="2064" y1="2160" y2="2160" x1="1888" />
        </branch>
        <instance x="2304" y="704" name="XLXI_32" orien="R0" />
        <instance x="2304" y="816" name="XLXI_33" orien="R0" />
        <branch name="VF(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2544" y="672" type="branch" />
            <wire x2="2544" y1="672" y2="672" x1="2528" />
        </branch>
        <branch name="VF(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2544" y="784" type="branch" />
            <wire x2="2544" y1="784" y2="784" x1="2528" />
        </branch>
        <branch name="VF(2:0)">
            <wire x2="2816" y1="672" y2="672" x1="2720" />
        </branch>
        <iomarker fontsize="28" x="2816" y="672" name="VF(2:0)" orien="R0" />
        <instance x="2176" y="1360" name="XLXI_30" orien="R0" />
        <branch name="VVal">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2080" y="1232" type="branch" />
            <wire x2="2176" y1="1232" y2="1232" x1="2080" />
        </branch>
        <branch name="VVal">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1920" y="2096" type="branch" />
            <wire x2="2064" y1="2096" y2="2096" x1="1920" />
        </branch>
        <instance x="2064" y="2224" name="XLXI_31" orien="R0" />
        <instance x="2576" y="1408" name="XLXI_37" orien="R0" />
        <instance x="2576" y="1520" name="XLXI_38" orien="R0" />
        <branch name="VC(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2816" y="1376" type="branch" />
            <wire x2="2816" y1="1376" y2="1376" x1="2800" />
        </branch>
        <branch name="VC(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2816" y="1488" type="branch" />
            <wire x2="2816" y1="1488" y2="1488" x1="2800" />
        </branch>
        <branch name="VC(2:0)">
            <wire x2="3072" y1="1376" y2="1376" x1="2976" />
        </branch>
        <instance x="2432" y="2288" name="XLXI_45" orien="R0" />
        <branch name="NA(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2672" y="2256" type="branch" />
            <wire x2="2672" y1="2256" y2="2256" x1="2656" />
        </branch>
        <branch name="NA(1:0)">
            <wire x2="2896" y1="2192" y2="2192" x1="2800" />
        </branch>
        <iomarker fontsize="28" x="2896" y="2192" name="NA(1:0)" orien="R0" />
        <iomarker fontsize="28" x="3072" y="1376" name="VC(2:0)" orien="R0" />
    </sheet>
</drawing>