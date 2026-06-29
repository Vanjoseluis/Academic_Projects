<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="VA" />
        <signal name="XLXN_5" />
        <signal name="F" />
        <signal name="P(0)" />
        <signal name="P(1)" />
        <signal name="P(2)" />
        <signal name="P(3)" />
        <signal name="USU(1)" />
        <signal name="USU(2)" />
        <signal name="En" />
        <signal name="C" />
        <signal name="EST(0)" />
        <signal name="EST(1)" />
        <signal name="EST(2)" />
        <signal name="V" />
        <signal name="P(3:0)" />
        <signal name="USU(2:1)" />
        <signal name="EST(2:0)" />
        <signal name="XLXN_78" />
        <port polarity="Output" name="F" />
        <port polarity="Output" name="V" />
        <port polarity="Input" name="P(3:0)" />
        <port polarity="Input" name="USU(2:1)" />
        <port polarity="Input" name="EST(2:0)" />
        <blockdef name="m16_1e">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="96" y1="-288" y2="-288" x1="0" />
            <line x2="96" y1="-96" y2="-96" x1="0" />
            <line x2="96" y1="-160" y2="-160" x1="0" />
            <line x2="96" y1="-224" y2="-224" x1="0" />
            <line x2="96" y1="-1312" y2="-1312" x1="0" />
            <line x2="96" y1="-352" y2="-352" x1="0" />
            <line x2="96" y1="-1248" y2="-1248" x1="0" />
            <line x2="96" y1="-416" y2="-416" x1="0" />
            <line x2="96" y1="-480" y2="-480" x1="0" />
            <line x2="96" y1="-1120" y2="-1120" x1="0" />
            <line x2="96" y1="-544" y2="-544" x1="0" />
            <line x2="96" y1="-608" y2="-608" x1="0" />
            <line x2="96" y1="-992" y2="-992" x1="0" />
            <line x2="96" y1="-672" y2="-672" x1="0" />
            <line x2="96" y1="-864" y2="-864" x1="0" />
            <line x2="96" y1="-800" y2="-800" x1="0" />
            <line x2="96" y1="-32" y2="-32" x1="0" />
            <line x2="96" y1="-32" y2="-32" x1="232" />
            <line x2="232" y1="-344" y2="-32" x1="232" />
            <line x2="92" y1="-96" y2="-96" x1="200" />
            <line x2="200" y1="-340" y2="-96" x1="200" />
            <line x2="96" y1="-160" y2="-160" x1="172" />
            <line x2="172" y1="-336" y2="-160" x1="172" />
            <line x2="96" y1="-224" y2="-224" x1="148" />
            <line x2="148" y1="-328" y2="-224" x1="148" />
            <line x2="96" y1="-288" y2="-288" x1="120" />
            <line x2="120" y1="-324" y2="-288" x1="120" />
            <line x2="256" y1="-832" y2="-832" x1="320" />
            <line x2="96" y1="-1344" y2="-320" x1="96" />
            <line x2="96" y1="-1312" y2="-1344" x1="256" />
            <line x2="256" y1="-352" y2="-1312" x1="256" />
            <line x2="256" y1="-320" y2="-352" x1="96" />
            <line x2="96" y1="-1184" y2="-1184" x1="0" />
            <line x2="96" y1="-1056" y2="-1056" x1="0" />
            <line x2="96" y1="-928" y2="-928" x1="0" />
            <line x2="96" y1="-736" y2="-736" x1="0" />
        </blockdef>
        <blockdef name="or4">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="48" y1="-64" y2="-64" x1="0" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="48" y1="-256" y2="-256" x1="0" />
            <line x2="192" y1="-160" y2="-160" x1="256" />
            <arc ex="112" ey="-208" sx="192" sy="-160" r="88" cx="116" cy="-120" />
            <line x2="48" y1="-208" y2="-208" x1="112" />
            <line x2="48" y1="-112" y2="-112" x1="112" />
            <line x2="48" y1="-256" y2="-208" x1="48" />
            <line x2="48" y1="-64" y2="-112" x1="48" />
            <arc ex="48" ey="-208" sx="48" sy="-112" r="56" cx="16" cy="-160" />
            <arc ex="192" ey="-160" sx="112" sy="-112" r="88" cx="116" cy="-200" />
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
        <blockdef name="and2b1">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-48" y2="-144" x1="64" />
            <line x2="144" y1="-144" y2="-144" x1="64" />
            <line x2="64" y1="-48" y2="-48" x1="144" />
            <arc ex="144" ey="-144" sx="144" sy="-48" r="48" cx="144" cy="-96" />
            <line x2="192" y1="-96" y2="-96" x1="256" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
        </blockdef>
        <blockdef name="xor2">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-64" x1="0" />
            <line x2="60" y1="-128" y2="-128" x1="0" />
            <line x2="208" y1="-96" y2="-96" x1="256" />
            <arc ex="44" ey="-144" sx="48" sy="-48" r="56" cx="16" cy="-96" />
            <arc ex="64" ey="-144" sx="64" sy="-48" r="56" cx="32" cy="-96" />
            <line x2="64" y1="-144" y2="-144" x1="128" />
            <line x2="64" y1="-48" y2="-48" x1="128" />
            <arc ex="128" ey="-144" sx="208" sy="-96" r="88" cx="132" cy="-56" />
            <arc ex="208" ey="-96" sx="128" sy="-48" r="88" cx="132" cy="-136" />
        </blockdef>
        <blockdef name="gnd">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-96" x1="64" />
            <line x2="52" y1="-48" y2="-48" x1="76" />
            <line x2="60" y1="-32" y2="-32" x1="68" />
            <line x2="40" y1="-64" y2="-64" x1="88" />
            <line x2="64" y1="-64" y2="-80" x1="64" />
            <line x2="64" y1="-128" y2="-96" x1="64" />
        </blockdef>
        <block symbolname="m16_1e" name="XLXI_3">
            <blockpin signalname="P(0)" name="D0" />
            <blockpin signalname="P(2)" name="D1" />
            <blockpin signalname="XLXN_78" name="D10" />
            <blockpin signalname="P(2)" name="D11" />
            <blockpin signalname="XLXN_78" name="D12" />
            <blockpin signalname="XLXN_78" name="D13" />
            <blockpin signalname="P(0)" name="D14" />
            <blockpin signalname="P(2)" name="D15" />
            <blockpin signalname="XLXN_78" name="D2" />
            <blockpin signalname="P(1)" name="D3" />
            <blockpin signalname="XLXN_78" name="D4" />
            <blockpin signalname="XLXN_78" name="D5" />
            <blockpin signalname="P(1)" name="D6" />
            <blockpin signalname="P(2)" name="D7" />
            <blockpin signalname="P(1)" name="D8" />
            <blockpin signalname="P(3)" name="D9" />
            <blockpin signalname="En" name="E" />
            <blockpin signalname="EST(0)" name="S0" />
            <blockpin signalname="EST(1)" name="S1" />
            <blockpin signalname="EST(2)" name="S2" />
            <blockpin signalname="C" name="S3" />
            <blockpin signalname="VA" name="O" />
        </block>
        <block symbolname="or4" name="XLXI_4">
            <blockpin signalname="P(3)" name="I0" />
            <blockpin signalname="P(2)" name="I1" />
            <blockpin signalname="P(1)" name="I2" />
            <blockpin signalname="P(0)" name="I3" />
            <blockpin signalname="XLXN_5" name="O" />
        </block>
        <block symbolname="and2b1" name="XLXI_6">
            <blockpin signalname="VA" name="I0" />
            <blockpin signalname="XLXN_5" name="I1" />
            <blockpin signalname="F" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_5">
            <blockpin signalname="VA" name="I0" />
            <blockpin signalname="XLXN_5" name="I1" />
            <blockpin signalname="V" name="O" />
        </block>
        <block symbolname="xor2" name="XLXI_10">
            <blockpin signalname="USU(2)" name="I0" />
            <blockpin signalname="USU(1)" name="I1" />
            <blockpin signalname="En" name="O" />
        </block>
        <block symbolname="and2b1" name="XLXI_11">
            <blockpin signalname="USU(1)" name="I0" />
            <blockpin signalname="USU(2)" name="I1" />
            <blockpin signalname="C" name="O" />
        </block>
        <block symbolname="gnd" name="XLXI_12">
            <blockpin signalname="XLXN_78" name="G" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="2720" height="1760">
        <instance x="1088" y="1600" name="XLXI_3" orien="R0" />
        <instance x="1648" y="1344" name="XLXI_4" orien="R0" />
        <instance x="1984" y="1248" name="XLXI_6" orien="R0" />
        <instance x="1984" y="1040" name="XLXI_5" orien="R0" />
        <branch name="F">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2336" y="1152" type="branch" />
            <wire x2="2336" y1="1152" y2="1152" x1="2240" />
        </branch>
        <branch name="P(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1568" y="1088" type="branch" />
            <wire x2="1648" y1="1088" y2="1088" x1="1568" />
        </branch>
        <branch name="P(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1568" y="1152" type="branch" />
            <wire x2="1648" y1="1152" y2="1152" x1="1568" />
        </branch>
        <branch name="P(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1568" y="1216" type="branch" />
            <wire x2="1648" y1="1216" y2="1216" x1="1568" />
        </branch>
        <branch name="P(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1568" y="1280" type="branch" />
            <wire x2="1648" y1="1280" y2="1280" x1="1568" />
        </branch>
        <instance x="384" y="1536" name="XLXI_11" orien="R0" />
        <branch name="USU(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="256" y="1472" type="branch" />
            <wire x2="368" y1="1472" y2="1472" x1="256" />
            <wire x2="384" y1="1472" y2="1472" x1="368" />
            <wire x2="368" y1="1472" y2="1536" x1="368" />
            <wire x2="560" y1="1536" y2="1536" x1="368" />
        </branch>
        <branch name="USU(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="256" y="1600" type="branch" />
            <wire x2="320" y1="1600" y2="1600" x1="256" />
            <wire x2="560" y1="1600" y2="1600" x1="320" />
            <wire x2="384" y1="1408" y2="1408" x1="320" />
            <wire x2="320" y1="1408" y2="1600" x1="320" />
        </branch>
        <branch name="En">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="992" y="1568" type="branch" />
            <wire x2="992" y1="1568" y2="1568" x1="816" />
            <wire x2="1088" y1="1568" y2="1568" x1="992" />
        </branch>
        <branch name="C">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="976" y="1504" type="branch" />
            <wire x2="784" y1="1440" y2="1440" x1="640" />
            <wire x2="784" y1="1440" y2="1504" x1="784" />
            <wire x2="976" y1="1504" y2="1504" x1="784" />
            <wire x2="1088" y1="1504" y2="1504" x1="976" />
        </branch>
        <branch name="EST(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="880" y="1312" type="branch" />
            <wire x2="1088" y1="1312" y2="1312" x1="880" />
        </branch>
        <branch name="EST(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="880" y="1376" type="branch" />
            <wire x2="1088" y1="1376" y2="1376" x1="880" />
        </branch>
        <branch name="EST(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="896" y="1440" type="branch" />
            <wire x2="1088" y1="1440" y2="1440" x1="896" />
        </branch>
        <instance x="512" y="800" name="XLXI_12" orien="R0" />
        <branch name="P(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="976" y="288" type="branch" />
            <wire x2="1088" y1="288" y2="288" x1="976" />
        </branch>
        <branch name="P(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="992" y="352" type="branch" />
            <wire x2="1088" y1="352" y2="352" x1="992" />
        </branch>
        <branch name="P(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1008" y="800" type="branch" />
            <wire x2="1088" y1="800" y2="800" x1="1008" />
        </branch>
        <branch name="P(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1008" y="864" type="branch" />
            <wire x2="1088" y1="864" y2="864" x1="1008" />
        </branch>
        <branch name="V">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2336" y="944" type="branch" />
            <wire x2="2336" y1="944" y2="944" x1="2240" />
        </branch>
        <branch name="P(3:0)">
            <wire x2="480" y1="128" y2="128" x1="288" />
        </branch>
        <branch name="USU(2:1)">
            <wire x2="480" y1="192" y2="192" x1="288" />
        </branch>
        <branch name="EST(2:0)">
            <wire x2="496" y1="240" y2="240" x1="288" />
        </branch>
        <instance x="560" y="1664" name="XLXI_10" orien="R0" />
        <iomarker fontsize="28" x="288" y="128" name="P(3:0)" orien="R180" />
        <iomarker fontsize="28" x="288" y="192" name="USU(2:1)" orien="R180" />
        <iomarker fontsize="28" x="288" y="240" name="EST(2:0)" orien="R180" />
        <branch name="V">
            <wire x2="784" y1="128" y2="128" x1="640" />
        </branch>
        <branch name="F">
            <wire x2="800" y1="192" y2="192" x1="656" />
        </branch>
        <iomarker fontsize="28" x="784" y="128" name="V" orien="R0" />
        <iomarker fontsize="28" x="800" y="192" name="F" orien="R0" />
        <branch name="VA">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="1536" y="768" type="branch" />
            <wire x2="1536" y1="768" y2="768" x1="1408" />
            <wire x2="1808" y1="768" y2="768" x1="1536" />
            <wire x2="1808" y1="768" y2="1056" x1="1808" />
            <wire x2="1968" y1="1056" y2="1056" x1="1808" />
            <wire x2="1968" y1="1056" y2="1184" x1="1968" />
            <wire x2="1984" y1="1184" y2="1184" x1="1968" />
            <wire x2="1984" y1="976" y2="976" x1="1968" />
            <wire x2="1968" y1="976" y2="1056" x1="1968" />
        </branch>
        <branch name="XLXN_5">
            <wire x2="1936" y1="1184" y2="1184" x1="1904" />
            <wire x2="1984" y1="912" y2="912" x1="1936" />
            <wire x2="1936" y1="912" y2="1120" x1="1936" />
            <wire x2="1936" y1="1120" y2="1184" x1="1936" />
            <wire x2="1984" y1="1120" y2="1120" x1="1936" />
        </branch>
        <branch name="P(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="992" y="480" type="branch" />
            <wire x2="1088" y1="480" y2="480" x1="992" />
        </branch>
        <branch name="P(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="672" type="branch" />
            <wire x2="1088" y1="672" y2="672" x1="1024" />
        </branch>
        <branch name="P(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="736" type="branch" />
            <wire x2="1088" y1="736" y2="736" x1="1024" />
        </branch>
        <branch name="P(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1008" y="992" type="branch" />
            <wire x2="1088" y1="992" y2="992" x1="1008" />
        </branch>
        <branch name="P(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="1248" type="branch" />
            <wire x2="1088" y1="1248" y2="1248" x1="1024" />
        </branch>
        <branch name="P(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="1184" type="branch" />
            <wire x2="1088" y1="1184" y2="1184" x1="1024" />
        </branch>
        <branch name="XLXN_78">
            <wire x2="576" y1="608" y2="672" x1="576" />
            <wire x2="704" y1="608" y2="608" x1="576" />
            <wire x2="1088" y1="608" y2="608" x1="704" />
            <wire x2="704" y1="608" y2="928" x1="704" />
            <wire x2="1088" y1="928" y2="928" x1="704" />
            <wire x2="704" y1="928" y2="1056" x1="704" />
            <wire x2="1088" y1="1056" y2="1056" x1="704" />
            <wire x2="704" y1="1056" y2="1120" x1="704" />
            <wire x2="1088" y1="1120" y2="1120" x1="704" />
            <wire x2="1088" y1="416" y2="416" x1="704" />
            <wire x2="704" y1="416" y2="544" x1="704" />
            <wire x2="1088" y1="544" y2="544" x1="704" />
            <wire x2="704" y1="544" y2="608" x1="704" />
        </branch>
    </sheet>
</drawing>