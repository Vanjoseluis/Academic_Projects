<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="Ck" />
        <signal name="CLR" />
        <signal name="R" />
        <signal name="U" />
        <signal name="S" />
        <signal name="EST(2:0)" />
        <signal name="D(2:0)" />
        <signal name="D(0)" />
        <signal name="D(1)" />
        <signal name="D(2)" />
        <signal name="EST(1)" />
        <signal name="EST(0)" />
        <signal name="XLXN_17" />
        <signal name="EST(2)" />
        <signal name="XLXN_19" />
        <signal name="XLXN_20" />
        <signal name="XLXN_21" />
        <signal name="XLXN_22" />
        <signal name="XLXN_31" />
        <signal name="XLXN_32" />
        <signal name="XLXN_42" />
        <signal name="XLXN_44" />
        <port polarity="Input" name="Ck" />
        <port polarity="Input" name="CLR" />
        <port polarity="Input" name="R" />
        <port polarity="Input" name="U" />
        <port polarity="Output" name="S" />
        <port polarity="Output" name="EST(2:0)" />
        <blockdef name="Control_Combinacional">
            <timestamp>2026-5-21T11:48:56</timestamp>
            <rect width="256" x="64" y="-192" height="192" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
            <rect width="64" x="320" y="-172" height="24" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <rect width="64" x="0" y="-44" height="24" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
        </blockdef>
        <blockdef name="fdc">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="320" y1="-256" y2="-256" x1="384" />
            <rect width="256" x="64" y="-320" height="256" />
            <line x2="80" y1="-112" y2="-128" x1="64" />
            <line x2="64" y1="-128" y2="-144" x1="80" />
            <line x2="192" y1="-64" y2="-32" x1="192" />
            <line x2="64" y1="-32" y2="-32" x1="192" />
        </blockdef>
        <blockdef name="and3b1">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="192" y1="-128" y2="-128" x1="256" />
            <line x2="64" y1="-64" y2="-192" x1="64" />
            <arc ex="144" ey="-176" sx="144" sy="-80" r="48" cx="144" cy="-128" />
            <line x2="64" y1="-80" y2="-80" x1="144" />
            <line x2="144" y1="-176" y2="-176" x1="64" />
        </blockdef>
        <blockdef name="and2b2">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="40" y1="-128" y2="-128" x1="0" />
            <circle r="12" cx="52" cy="-128" />
            <line x2="192" y1="-96" y2="-96" x1="256" />
            <arc ex="144" ey="-144" sx="144" sy="-48" r="48" cx="144" cy="-96" />
            <line x2="64" y1="-48" y2="-144" x1="64" />
            <line x2="64" y1="-48" y2="-48" x1="144" />
            <line x2="144" y1="-144" y2="-144" x1="64" />
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
        <block symbolname="Control_Combinacional" name="XLXI_8">
            <blockpin signalname="D(2:0)" name="D(2:0)" />
            <blockpin signalname="EST(2:0)" name="q(2:0)" />
            <blockpin signalname="R" name="R" />
            <blockpin signalname="U" name="U" />
        </block>
        <block symbolname="fdc" name="XLXI_9">
            <blockpin signalname="Ck" name="C" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="D(0)" name="D" />
            <blockpin signalname="EST(0)" name="Q" />
        </block>
        <block symbolname="fdc" name="XLXI_10">
            <blockpin signalname="Ck" name="C" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="D(1)" name="D" />
            <blockpin signalname="EST(1)" name="Q" />
        </block>
        <block symbolname="fdc" name="XLXI_11">
            <blockpin signalname="Ck" name="C" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="D(2)" name="D" />
            <blockpin signalname="EST(2)" name="Q" />
        </block>
        <block symbolname="and3b1" name="XLXI_12">
            <blockpin signalname="EST(2)" name="I0" />
            <blockpin signalname="EST(1)" name="I1" />
            <blockpin signalname="EST(0)" name="I2" />
            <blockpin signalname="XLXN_42" name="O" />
        </block>
        <block symbolname="and2b2" name="XLXI_15">
            <blockpin signalname="EST(2)" name="I0" />
            <blockpin signalname="EST(1)" name="I1" />
            <blockpin signalname="XLXN_44" name="O" />
        </block>
        <block symbolname="or2" name="XLXI_16">
            <blockpin signalname="XLXN_44" name="I0" />
            <blockpin signalname="XLXN_42" name="I1" />
            <blockpin signalname="S" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <branch name="U">
            <wire x2="576" y1="64" y2="64" x1="432" />
        </branch>
        <branch name="CLR">
            <wire x2="592" y1="192" y2="192" x1="432" />
        </branch>
        <branch name="Ck">
            <wire x2="592" y1="240" y2="240" x1="432" />
        </branch>
        <iomarker fontsize="28" x="432" y="64" name="U" orien="R180" />
        <iomarker fontsize="28" x="432" y="192" name="CLR" orien="R180" />
        <iomarker fontsize="28" x="432" y="240" name="Ck" orien="R180" />
        <branch name="S">
            <wire x2="896" y1="64" y2="64" x1="720" />
        </branch>
        <branch name="EST(2:0)">
            <wire x2="896" y1="128" y2="128" x1="720" />
        </branch>
        <iomarker fontsize="28" x="896" y="64" name="S" orien="R0" />
        <iomarker fontsize="28" x="896" y="128" name="EST(2:0)" orien="R0" />
        <branch name="R">
            <wire x2="592" y1="128" y2="128" x1="464" />
        </branch>
        <iomarker fontsize="28" x="464" y="128" name="R" orien="R180" />
        <instance x="608" y="912" name="XLXI_8" orien="R0">
        </instance>
        <branch name="U">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="544" y="752" type="branch" />
            <wire x2="608" y1="752" y2="752" x1="544" />
        </branch>
        <branch name="R">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="544" y="816" type="branch" />
            <wire x2="608" y1="816" y2="816" x1="544" />
        </branch>
        <branch name="EST(2:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="544" y="880" type="branch" />
            <wire x2="608" y1="880" y2="880" x1="544" />
        </branch>
        <branch name="D(2:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1056" y="752" type="branch" />
            <wire x2="1056" y1="752" y2="752" x1="992" />
        </branch>
        <branch name="D(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1424" y="624" type="branch" />
            <wire x2="1520" y1="624" y2="624" x1="1424" />
        </branch>
        <branch name="Ck">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1424" y="752" type="branch" />
            <wire x2="1520" y1="752" y2="752" x1="1424" />
        </branch>
        <branch name="D(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1440" y="1024" type="branch" />
            <wire x2="1520" y1="1024" y2="1024" x1="1440" />
        </branch>
        <branch name="Ck">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1440" y="1152" type="branch" />
            <wire x2="1520" y1="1152" y2="1152" x1="1440" />
        </branch>
        <branch name="D(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1488" y="1440" type="branch" />
            <wire x2="1536" y1="1440" y2="1440" x1="1488" />
        </branch>
        <branch name="EST(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="1024" type="branch" />
            <wire x2="1936" y1="1024" y2="1024" x1="1904" />
        </branch>
        <branch name="EST(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1968" y="624" type="branch" />
            <wire x2="1968" y1="624" y2="624" x1="1904" />
        </branch>
        <instance x="1520" y="880" name="XLXI_9" orien="R0" />
        <instance x="1536" y="1696" name="XLXI_11" orien="R0" />
        <branch name="Ck">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1456" y="1568" type="branch" />
            <wire x2="1536" y1="1568" y2="1568" x1="1456" />
        </branch>
        <branch name="EST(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="1440" type="branch" />
            <wire x2="1936" y1="1440" y2="1440" x1="1920" />
        </branch>
        <instance x="1520" y="1280" name="XLXI_10" orien="R0" />
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1424" y="848" type="branch" />
            <wire x2="1520" y1="848" y2="848" x1="1424" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1440" y="1248" type="branch" />
            <wire x2="1520" y1="1248" y2="1248" x1="1440" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1456" y="1664" type="branch" />
            <wire x2="1536" y1="1664" y2="1664" x1="1456" />
        </branch>
        <instance x="2432" y="832" name="XLXI_12" orien="R0" />
        <branch name="EST(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2384" y="640" type="branch" />
            <wire x2="2432" y1="640" y2="640" x1="2384" />
        </branch>
        <branch name="EST(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2384" y="704" type="branch" />
            <wire x2="2432" y1="704" y2="704" x1="2384" />
        </branch>
        <branch name="EST(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2384" y="768" type="branch" />
            <wire x2="2432" y1="768" y2="768" x1="2384" />
        </branch>
        <instance x="2800" y="928" name="XLXI_16" orien="R0" />
        <instance x="2432" y="1088" name="XLXI_15" orien="R0" />
        <branch name="EST(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2400" y="960" type="branch" />
            <wire x2="2432" y1="960" y2="960" x1="2400" />
        </branch>
        <branch name="EST(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2400" y="1024" type="branch" />
            <wire x2="2432" y1="1024" y2="1024" x1="2400" />
        </branch>
        <branch name="XLXN_42">
            <wire x2="2720" y1="704" y2="704" x1="2688" />
            <wire x2="2720" y1="704" y2="800" x1="2720" />
            <wire x2="2800" y1="800" y2="800" x1="2720" />
        </branch>
        <branch name="XLXN_44">
            <wire x2="2720" y1="992" y2="992" x1="2688" />
            <wire x2="2800" y1="864" y2="864" x1="2720" />
            <wire x2="2720" y1="864" y2="992" x1="2720" />
        </branch>
        <branch name="S">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3088" y="832" type="branch" />
            <wire x2="3088" y1="832" y2="832" x1="3056" />
        </branch>
    </sheet>
</drawing>