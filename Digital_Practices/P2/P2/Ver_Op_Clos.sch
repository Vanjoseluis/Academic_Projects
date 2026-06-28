<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="Seven_Seg3(0)" />
        <signal name="Seven_Seg3(1)" />
        <signal name="Seven_Seg3(2)" />
        <signal name="Seven_Seg3(3)" />
        <signal name="Seven_Seg3(4)" />
        <signal name="Seven_Seg3(5)" />
        <signal name="Seven_Seg3(6)" />
        <signal name="Seven_Seg3(7)" />
        <signal name="Op" />
        <signal name="Close" />
        <signal name="XLXN_15" />
        <signal name="XLXN_19" />
        <signal name="Seven_Seg2(0)" />
        <signal name="Seven_Seg2(1)" />
        <signal name="Seven_Seg2(2)" />
        <signal name="Seven_Seg2(3)" />
        <signal name="Seven_Seg2(4)" />
        <signal name="Seven_Seg2(5)" />
        <signal name="Seven_Seg2(6)" />
        <signal name="Seven_Seg2(7)" />
        <signal name="XLXN_32" />
        <signal name="XLXN_34" />
        <signal name="XLXN_39" />
        <signal name="XLXN_42" />
        <signal name="Seven_Seg1(0)" />
        <signal name="Seven_Seg1(1)" />
        <signal name="Seven_Seg1(2)" />
        <signal name="Seven_Seg1(3)" />
        <signal name="Seven_Seg1(4)" />
        <signal name="Seven_Seg1(5)" />
        <signal name="Seven_Seg1(6)" />
        <signal name="Seven_Seg1(7)" />
        <signal name="XLXN_53" />
        <signal name="XLXN_58" />
        <signal name="Seven_Seg0(0)" />
        <signal name="Seven_Seg0(1)" />
        <signal name="Seven_Seg0(2)" />
        <signal name="Seven_Seg0(3)" />
        <signal name="Seven_Seg0(4)" />
        <signal name="Seven_Seg0(5)" />
        <signal name="Seven_Seg0(6)" />
        <signal name="Seven_Seg0(7)" />
        <signal name="Seven_Seg3(7:0)" />
        <signal name="Seven_Seg2(7:0)" />
        <signal name="Seven_Seg1(7:0)" />
        <signal name="Seven_Seg0(7:0)" />
        <port polarity="Input" name="Op" />
        <port polarity="Input" name="Close" />
        <port polarity="Output" name="Seven_Seg3(7:0)" />
        <port polarity="Output" name="Seven_Seg2(7:0)" />
        <port polarity="Output" name="Seven_Seg1(7:0)" />
        <port polarity="Output" name="Seven_Seg0(7:0)" />
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
        <blockdef name="or2b1">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="32" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="44" cy="-64" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="192" y1="-96" y2="-96" x1="256" />
            <line x2="48" y1="-48" y2="-48" x1="112" />
            <arc ex="112" ey="-144" sx="192" sy="-96" r="88" cx="116" cy="-56" />
            <line x2="48" y1="-144" y2="-144" x1="112" />
            <arc ex="48" ey="-144" sx="48" sy="-48" r="56" cx="16" cy="-96" />
            <arc ex="192" ey="-96" sx="112" sy="-48" r="88" cx="116" cy="-136" />
        </blockdef>
        <blockdef name="buf">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-32" x1="0" />
            <line x2="128" y1="-32" y2="-32" x1="224" />
            <line x2="128" y1="0" y2="-32" x1="64" />
            <line x2="64" y1="-32" y2="-64" x1="128" />
            <line x2="64" y1="-64" y2="0" x1="64" />
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
        <blockdef name="gnd">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-96" x1="64" />
            <line x2="52" y1="-48" y2="-48" x1="76" />
            <line x2="60" y1="-32" y2="-32" x1="68" />
            <line x2="40" y1="-64" y2="-64" x1="88" />
            <line x2="64" y1="-64" y2="-80" x1="64" />
            <line x2="64" y1="-128" y2="-96" x1="64" />
        </blockdef>
        <blockdef name="vcc">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-32" y2="-64" x1="64" />
            <line x2="64" y1="0" y2="-32" x1="64" />
            <line x2="32" y1="-64" y2="-64" x1="96" />
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
        <block symbolname="or2" name="XLXI_1">
            <blockpin signalname="Close" name="I0" />
            <blockpin signalname="Op" name="I1" />
            <blockpin signalname="Seven_Seg3(0)" name="O" />
        </block>
        <block symbolname="or2b1" name="XLXI_2">
            <blockpin signalname="Close" name="I0" />
            <blockpin signalname="Op" name="I1" />
            <blockpin signalname="Seven_Seg3(1)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_3">
            <blockpin signalname="XLXN_15" name="I" />
            <blockpin signalname="Seven_Seg3(3)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_4">
            <blockpin signalname="XLXN_15" name="I" />
            <blockpin signalname="Seven_Seg3(4)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_5">
            <blockpin signalname="Seven_Seg3(1)" name="I" />
            <blockpin signalname="Seven_Seg3(2)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_6">
            <blockpin signalname="Seven_Seg3(0)" name="I" />
            <blockpin signalname="Seven_Seg3(5)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_7">
            <blockpin signalname="XLXN_19" name="I" />
            <blockpin signalname="Seven_Seg3(7)" name="O" />
        </block>
        <block symbolname="and2b2" name="XLXI_8">
            <blockpin signalname="Close" name="I0" />
            <blockpin signalname="Op" name="I1" />
            <blockpin signalname="Seven_Seg3(6)" name="O" />
        </block>
        <block symbolname="gnd" name="XLXI_9">
            <blockpin signalname="XLXN_19" name="G" />
        </block>
        <block symbolname="vcc" name="XLXI_10">
            <blockpin signalname="XLXN_15" name="P" />
        </block>
        <block symbolname="buf" name="XLXI_11">
            <blockpin signalname="Op" name="I" />
            <blockpin signalname="Seven_Seg2(0)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_12">
            <blockpin signalname="Op" name="I" />
            <blockpin signalname="Seven_Seg2(1)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_13">
            <blockpin signalname="Seven_Seg3(6)" name="I" />
            <blockpin signalname="Seven_Seg2(2)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_14">
            <blockpin signalname="XLXN_32" name="I" />
            <blockpin signalname="Seven_Seg2(4)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_16">
            <blockpin signalname="XLXN_34" name="I" />
            <blockpin signalname="Seven_Seg2(7)" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_17">
            <blockpin signalname="Op" name="I" />
            <blockpin signalname="Seven_Seg2(3)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_18">
            <blockpin signalname="Seven_Seg3(0)" name="I" />
            <blockpin signalname="Seven_Seg2(5)" name="O" />
        </block>
        <block symbolname="vcc" name="XLXI_19">
            <blockpin signalname="XLXN_32" name="P" />
        </block>
        <block symbolname="gnd" name="XLXI_20">
            <blockpin signalname="XLXN_34" name="G" />
        </block>
        <block symbolname="buf" name="XLXI_21">
            <blockpin signalname="Seven_Seg3(0)" name="I" />
            <blockpin signalname="Seven_Seg1(0)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_22">
            <blockpin signalname="Close" name="I" />
            <blockpin signalname="Seven_Seg1(1)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_24">
            <blockpin signalname="XLXN_39" name="I" />
            <blockpin signalname="Seven_Seg1(3)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_25">
            <blockpin signalname="XLXN_39" name="I" />
            <blockpin signalname="Seven_Seg1(4)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_26">
            <blockpin signalname="Seven_Seg3(0)" name="I" />
            <blockpin signalname="Seven_Seg1(5)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_28">
            <blockpin signalname="XLXN_42" name="I" />
            <blockpin signalname="Seven_Seg1(7)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_29">
            <blockpin signalname="Close" name="I" />
            <blockpin signalname="Seven_Seg0(0)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_30">
            <blockpin signalname="XLXN_53" name="I" />
            <blockpin signalname="Seven_Seg0(1)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_31">
            <blockpin signalname="Seven_Seg3(0)" name="I" />
            <blockpin signalname="Seven_Seg0(2)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_32">
            <blockpin signalname="Close" name="I" />
            <blockpin signalname="Seven_Seg0(3)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_34">
            <blockpin signalname="Close" name="I" />
            <blockpin signalname="Seven_Seg0(5)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_35">
            <blockpin signalname="XLXN_58" name="I" />
            <blockpin signalname="Seven_Seg0(6)" name="O" />
        </block>
        <block symbolname="buf" name="XLXI_36">
            <blockpin signalname="XLXN_53" name="I" />
            <blockpin signalname="Seven_Seg0(7)" name="O" />
        </block>
        <block symbolname="vcc" name="XLXI_37">
            <blockpin signalname="XLXN_39" name="P" />
        </block>
        <block symbolname="gnd" name="XLXI_38">
            <blockpin signalname="XLXN_42" name="G" />
        </block>
        <block symbolname="gnd" name="XLXI_39">
            <blockpin signalname="XLXN_53" name="G" />
        </block>
        <block symbolname="inv" name="XLXI_40">
            <blockpin signalname="Close" name="I" />
            <blockpin signalname="Seven_Seg1(6)" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_41">
            <blockpin signalname="Op" name="I" />
            <blockpin signalname="Seven_Seg1(2)" name="O" />
        </block>
        <block symbolname="vcc" name="XLXI_42">
            <blockpin signalname="XLXN_58" name="P" />
        </block>
        <block symbolname="inv" name="XLXI_43">
            <blockpin signalname="Close" name="I" />
            <blockpin signalname="Seven_Seg2(6)" name="O" />
        </block>
        <block symbolname="inv" name="XLXI_44">
            <blockpin signalname="Close" name="I" />
            <blockpin signalname="Seven_Seg0(4)" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <instance x="624" y="608" name="XLXI_3" orien="R0" />
        <instance x="624" y="688" name="XLXI_4" orien="R0" />
        <instance x="624" y="528" name="XLXI_5" orien="R0" />
        <instance x="624" y="784" name="XLXI_6" orien="R0" />
        <instance x="624" y="1040" name="XLXI_7" orien="R0" />
        <instance x="624" y="976" name="XLXI_8" orien="R0" />
        <instance x="624" y="448" name="XLXI_2" orien="R0" />
        <instance x="624" y="304" name="XLXI_1" orien="R0" />
        <branch name="Seven_Seg3(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="960" y="208" type="branch" />
            <wire x2="464" y1="1264" y2="1440" x1="464" />
            <wire x2="656" y1="1440" y2="1440" x1="464" />
            <wire x2="464" y1="1440" y2="2064" x1="464" />
            <wire x2="656" y1="2064" y2="2064" x1="464" />
            <wire x2="1328" y1="1264" y2="1264" x1="464" />
            <wire x2="1328" y1="1264" y2="1696" x1="1328" />
            <wire x2="1680" y1="1696" y2="1696" x1="1328" />
            <wire x2="928" y1="704" y2="704" x1="512" />
            <wire x2="1328" y1="704" y2="704" x1="928" />
            <wire x2="1328" y1="704" y2="752" x1="1328" />
            <wire x2="1728" y1="752" y2="752" x1="1328" />
            <wire x2="1328" y1="752" y2="1264" x1="1328" />
            <wire x2="512" y1="704" y2="752" x1="512" />
            <wire x2="624" y1="752" y2="752" x1="512" />
            <wire x2="928" y1="208" y2="208" x1="880" />
            <wire x2="960" y1="208" y2="208" x1="928" />
            <wire x2="928" y1="208" y2="704" x1="928" />
        </branch>
        <branch name="Seven_Seg3(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="960" y="352" type="branch" />
            <wire x2="560" y1="448" y2="496" x1="560" />
            <wire x2="624" y1="496" y2="496" x1="560" />
            <wire x2="896" y1="448" y2="448" x1="560" />
            <wire x2="896" y1="352" y2="352" x1="880" />
            <wire x2="896" y1="352" y2="448" x1="896" />
            <wire x2="960" y1="352" y2="352" x1="896" />
        </branch>
        <branch name="Seven_Seg3(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="976" y="496" type="branch" />
            <wire x2="976" y1="496" y2="496" x1="848" />
        </branch>
        <branch name="Seven_Seg3(3)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="960" y="576" type="branch" />
            <wire x2="960" y1="576" y2="576" x1="848" />
        </branch>
        <branch name="Seven_Seg3(4)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="960" y="656" type="branch" />
            <wire x2="960" y1="656" y2="656" x1="848" />
        </branch>
        <branch name="Seven_Seg3(5)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="960" y="752" type="branch" />
            <wire x2="960" y1="752" y2="752" x1="848" />
        </branch>
        <branch name="Seven_Seg3(6)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="976" y="880" type="branch" />
            <wire x2="912" y1="880" y2="880" x1="880" />
            <wire x2="976" y1="880" y2="880" x1="912" />
            <wire x2="912" y1="816" y2="880" x1="912" />
            <wire x2="1424" y1="816" y2="816" x1="912" />
            <wire x2="1728" y1="448" y2="448" x1="1424" />
            <wire x2="1424" y1="448" y2="816" x1="1424" />
        </branch>
        <branch name="Seven_Seg3(7)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="976" y="1008" type="branch" />
            <wire x2="976" y1="1008" y2="1008" x1="848" />
        </branch>
        <branch name="Op">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="448" y="176" type="branch" />
            <wire x2="624" y1="176" y2="176" x1="448" />
        </branch>
        <branch name="Close">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="240" type="branch" />
            <wire x2="624" y1="240" y2="240" x1="528" />
        </branch>
        <branch name="Op">
            <wire x2="608" y1="320" y2="320" x1="448" />
            <wire x2="624" y1="320" y2="320" x1="608" />
        </branch>
        <branch name="Close">
            <wire x2="608" y1="384" y2="384" x1="480" />
            <wire x2="624" y1="384" y2="384" x1="608" />
        </branch>
        <branch name="XLXN_15">
            <wire x2="320" y1="576" y2="592" x1="320" />
            <wire x2="320" y1="592" y2="656" x1="320" />
            <wire x2="624" y1="656" y2="656" x1="320" />
            <wire x2="560" y1="592" y2="592" x1="320" />
            <wire x2="624" y1="576" y2="576" x1="560" />
            <wire x2="560" y1="576" y2="592" x1="560" />
        </branch>
        <branch name="Op">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="560" y="848" type="branch" />
            <wire x2="624" y1="848" y2="848" x1="560" />
        </branch>
        <branch name="Close">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="560" y="912" type="branch" />
            <wire x2="624" y1="912" y2="912" x1="560" />
        </branch>
        <branch name="XLXN_19">
            <wire x2="464" y1="1008" y2="1024" x1="464" />
            <wire x2="624" y1="1008" y2="1008" x1="464" />
        </branch>
        <instance x="400" y="1152" name="XLXI_9" orien="R0" />
        <instance x="256" y="576" name="XLXI_10" orien="R0" />
        <instance x="1728" y="240" name="XLXI_11" orien="R0" />
        <instance x="1728" y="352" name="XLXI_12" orien="R0" />
        <instance x="1728" y="480" name="XLXI_13" orien="R0" />
        <instance x="1728" y="672" name="XLXI_14" orien="R0" />
        <instance x="1728" y="1024" name="XLXI_16" orien="R0" />
        <instance x="1728" y="592" name="XLXI_17" orien="R0" />
        <instance x="1728" y="784" name="XLXI_18" orien="R0" />
        <branch name="Seven_Seg2(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="208" type="branch" />
            <wire x2="2064" y1="208" y2="208" x1="1952" />
        </branch>
        <branch name="Seven_Seg2(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="320" type="branch" />
            <wire x2="2064" y1="320" y2="320" x1="1952" />
        </branch>
        <branch name="Seven_Seg2(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="448" type="branch" />
            <wire x2="2064" y1="448" y2="448" x1="1952" />
        </branch>
        <branch name="Seven_Seg2(3)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="560" type="branch" />
            <wire x2="2064" y1="560" y2="560" x1="1952" />
        </branch>
        <branch name="Seven_Seg2(4)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2080" y="640" type="branch" />
            <wire x2="2080" y1="640" y2="640" x1="1952" />
        </branch>
        <branch name="Seven_Seg2(5)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2080" y="752" type="branch" />
            <wire x2="2080" y1="752" y2="752" x1="1952" />
        </branch>
        <branch name="Seven_Seg2(6)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2080" y="864" type="branch" />
            <wire x2="2080" y1="864" y2="864" x1="1952" />
        </branch>
        <branch name="Seven_Seg2(7)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2080" y="992" type="branch" />
            <wire x2="2080" y1="992" y2="992" x1="1952" />
        </branch>
        <branch name="Op">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1600" y="208" type="branch" />
            <wire x2="1664" y1="208" y2="208" x1="1600" />
            <wire x2="1728" y1="208" y2="208" x1="1664" />
            <wire x2="1664" y1="208" y2="320" x1="1664" />
            <wire x2="1728" y1="320" y2="320" x1="1664" />
            <wire x2="1664" y1="320" y2="560" x1="1664" />
            <wire x2="1728" y1="560" y2="560" x1="1664" />
        </branch>
        <instance x="1488" y="624" name="XLXI_19" orien="R0" />
        <branch name="XLXN_32">
            <wire x2="1552" y1="624" y2="640" x1="1552" />
            <wire x2="1728" y1="640" y2="640" x1="1552" />
        </branch>
        <branch name="Close">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1632" y="864" type="branch" />
            <wire x2="1728" y1="864" y2="864" x1="1632" />
        </branch>
        <branch name="XLXN_34">
            <wire x2="1600" y1="992" y2="1024" x1="1600" />
            <wire x2="1728" y1="992" y2="992" x1="1600" />
        </branch>
        <instance x="656" y="1472" name="XLXI_21" orien="R0" />
        <instance x="656" y="1600" name="XLXI_22" orien="R0" />
        <instance x="656" y="1840" name="XLXI_24" orien="R0" />
        <instance x="656" y="1968" name="XLXI_25" orien="R0" />
        <instance x="656" y="2096" name="XLXI_26" orien="R0" />
        <instance x="656" y="2352" name="XLXI_28" orien="R0" />
        <branch name="Close">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="576" y="1568" type="branch" />
            <wire x2="656" y1="1568" y2="1568" x1="576" />
        </branch>
        <branch name="Op">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="576" y="1696" type="branch" />
            <wire x2="656" y1="1696" y2="1696" x1="576" />
        </branch>
        <branch name="XLXN_39">
            <wire x2="336" y1="1776" y2="1808" x1="336" />
            <wire x2="336" y1="1808" y2="1936" x1="336" />
            <wire x2="656" y1="1936" y2="1936" x1="336" />
            <wire x2="656" y1="1808" y2="1808" x1="336" />
        </branch>
        <branch name="Close">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="576" y="2176" type="branch" />
            <wire x2="656" y1="2176" y2="2176" x1="576" />
        </branch>
        <branch name="XLXN_42">
            <wire x2="480" y1="2320" y2="2336" x1="480" />
            <wire x2="656" y1="2320" y2="2320" x1="480" />
        </branch>
        <branch name="Seven_Seg1(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="976" y="1440" type="branch" />
            <wire x2="976" y1="1440" y2="1440" x1="880" />
        </branch>
        <branch name="Seven_Seg1(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1008" y="1568" type="branch" />
            <wire x2="1008" y1="1568" y2="1568" x1="880" />
        </branch>
        <branch name="Seven_Seg1(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1008" y="1696" type="branch" />
            <wire x2="1008" y1="1696" y2="1696" x1="880" />
        </branch>
        <branch name="Seven_Seg1(3)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1008" y="1808" type="branch" />
            <wire x2="1008" y1="1808" y2="1808" x1="880" />
        </branch>
        <branch name="Seven_Seg1(4)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="1936" type="branch" />
            <wire x2="1024" y1="1936" y2="1936" x1="880" />
        </branch>
        <branch name="Seven_Seg1(5)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="2064" type="branch" />
            <wire x2="1024" y1="2064" y2="2064" x1="880" />
        </branch>
        <branch name="Seven_Seg1(6)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="2176" type="branch" />
            <wire x2="1024" y1="2176" y2="2176" x1="880" />
        </branch>
        <branch name="Seven_Seg1(7)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="2320" type="branch" />
            <wire x2="1024" y1="2320" y2="2320" x1="880" />
        </branch>
        <branch name="Close">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1600" y="1440" type="branch" />
            <wire x2="1680" y1="1440" y2="1440" x1="1600" />
        </branch>
        <branch name="Close">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1600" y="1808" type="branch" />
            <wire x2="1616" y1="1808" y2="1808" x1="1600" />
            <wire x2="1680" y1="1808" y2="1808" x1="1616" />
            <wire x2="1616" y1="1808" y2="1936" x1="1616" />
            <wire x2="1616" y1="1936" y2="2064" x1="1616" />
            <wire x2="1680" y1="2064" y2="2064" x1="1616" />
            <wire x2="1680" y1="1936" y2="1936" x1="1616" />
        </branch>
        <branch name="XLXN_58">
            <wire x2="1408" y1="2160" y2="2176" x1="1408" />
            <wire x2="1680" y1="2176" y2="2176" x1="1408" />
        </branch>
        <branch name="Seven_Seg0(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2000" y="1440" type="branch" />
            <wire x2="2000" y1="1440" y2="1440" x1="1904" />
        </branch>
        <branch name="Seven_Seg0(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1568" type="branch" />
            <wire x2="2032" y1="1568" y2="1568" x1="1904" />
        </branch>
        <branch name="Seven_Seg0(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1696" type="branch" />
            <wire x2="2032" y1="1696" y2="1696" x1="1904" />
        </branch>
        <branch name="Seven_Seg0(3)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1808" type="branch" />
            <wire x2="2032" y1="1808" y2="1808" x1="1904" />
        </branch>
        <branch name="Seven_Seg0(4)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2048" y="1936" type="branch" />
            <wire x2="2048" y1="1936" y2="1936" x1="1904" />
        </branch>
        <branch name="Seven_Seg0(5)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2048" y="2064" type="branch" />
            <wire x2="2048" y1="2064" y2="2064" x1="1904" />
        </branch>
        <branch name="Seven_Seg0(6)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2048" y="2176" type="branch" />
            <wire x2="2048" y1="2176" y2="2176" x1="1904" />
        </branch>
        <branch name="Seven_Seg0(7)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2048" y="2320" type="branch" />
            <wire x2="2048" y1="2320" y2="2320" x1="1904" />
        </branch>
        <instance x="1680" y="1472" name="XLXI_29" orien="R0" />
        <instance x="1680" y="1600" name="XLXI_30" orien="R0" />
        <instance x="1680" y="1728" name="XLXI_31" orien="R0" />
        <instance x="1680" y="1840" name="XLXI_32" orien="R0" />
        <instance x="1680" y="2096" name="XLXI_34" orien="R0" />
        <instance x="1680" y="2208" name="XLXI_35" orien="R0" />
        <instance x="1680" y="2352" name="XLXI_36" orien="R0" />
        <instance x="272" y="1776" name="XLXI_37" orien="R0" />
        <instance x="416" y="2464" name="XLXI_38" orien="R0" />
        <instance x="1472" y="2480" name="XLXI_39" orien="R0" />
        <instance x="656" y="2208" name="XLXI_40" orien="R0" />
        <instance x="656" y="1728" name="XLXI_41" orien="R0" />
        <instance x="1344" y="2160" name="XLXI_42" orien="R0" />
        <branch name="Seven_Seg3(7:0)">
            <wire x2="2848" y1="352" y2="352" x1="2672" />
        </branch>
        <branch name="Seven_Seg2(7:0)">
            <wire x2="2848" y1="432" y2="432" x1="2672" />
        </branch>
        <branch name="Seven_Seg1(7:0)">
            <wire x2="2864" y1="512" y2="512" x1="2688" />
        </branch>
        <branch name="Seven_Seg0(7:0)">
            <wire x2="2864" y1="576" y2="576" x1="2688" />
        </branch>
        <iomarker fontsize="28" x="2848" y="352" name="Seven_Seg3(7:0)" orien="R0" />
        <iomarker fontsize="28" x="2848" y="432" name="Seven_Seg2(7:0)" orien="R0" />
        <iomarker fontsize="28" x="2864" y="512" name="Seven_Seg1(7:0)" orien="R0" />
        <iomarker fontsize="28" x="2864" y="576" name="Seven_Seg0(7:0)" orien="R0" />
        <instance x="1728" y="896" name="XLXI_43" orien="R0" />
        <instance x="1536" y="1152" name="XLXI_20" orien="R0" />
        <branch name="XLXN_53">
            <wire x2="1680" y1="1568" y2="1568" x1="1536" />
            <wire x2="1536" y1="1568" y2="2320" x1="1536" />
            <wire x2="1536" y1="2320" y2="2352" x1="1536" />
            <wire x2="1680" y1="2320" y2="2320" x1="1536" />
        </branch>
        <instance x="1680" y="1968" name="XLXI_44" orien="R0" />
        <iomarker fontsize="28" x="448" y="320" name="Op" orien="R180" />
        <iomarker fontsize="28" x="480" y="384" name="Close" orien="R180" />
    </sheet>
</drawing>