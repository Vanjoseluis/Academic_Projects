<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="Sel(0)" />
        <signal name="Sel(4)" />
        <signal name="Sel(6)" />
        <signal name="Sel(1)" />
        <signal name="Sel(2)" />
        <signal name="Ena(3)" />
        <signal name="Sel(3)" />
        <signal name="XLXN_141" />
        <signal name="Ena(0)" />
        <signal name="Sel(7)" />
        <signal name="Ena(2)" />
        <signal name="XLXN_154" />
        <signal name="XLXN_156" />
        <signal name="Ena(5)" />
        <signal name="XLXN_158" />
        <signal name="Ena(4)" />
        <signal name="XLXN_157" />
        <signal name="Ena(1)" />
        <signal name="XLXN_155" />
        <signal name="Sel(5)" />
        <signal name="Ena(6)" />
        <signal name="XLXN_290" />
        <signal name="Ena(7)" />
        <signal name="XLXN_300" />
        <signal name="Sel(7:0)" />
        <signal name="Ena(7:0)" />
        <port polarity="Input" name="Sel(7:0)" />
        <port polarity="Output" name="Ena(7:0)" />
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
        <blockdef name="nor7">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="48" y1="-304" y2="-304" x1="72" />
            <line x2="48" y1="-208" y2="-208" x1="72" />
            <line x2="48" y1="-64" y2="-208" x1="48" />
            <line x2="48" y1="-448" y2="-304" x1="48" />
            <line x2="48" y1="-384" y2="-384" x1="0" />
            <line x2="48" y1="-448" y2="-448" x1="0" />
            <line x2="48" y1="-320" y2="-320" x1="0" />
            <line x2="48" y1="-192" y2="-192" x1="0" />
            <line x2="48" y1="-128" y2="-128" x1="0" />
            <line x2="48" y1="-64" y2="-64" x1="0" />
            <line x2="228" y1="-256" y2="-256" x1="256" />
            <circle r="10" cx="218" cy="-254" />
            <arc ex="128" ey="-304" sx="208" sy="-256" r="88" cx="132" cy="-216" />
            <line x2="64" y1="-208" y2="-208" x1="128" />
            <line x2="64" y1="-304" y2="-304" x1="128" />
            <line x2="72" y1="-256" y2="-256" x1="0" />
            <arc ex="48" ey="-304" sx="48" sy="-208" r="56" cx="16" cy="-256" />
            <arc ex="208" ey="-256" sx="128" sy="-208" r="88" cx="132" cy="-296" />
        </blockdef>
        <block symbolname="and2" name="XLXI_2">
            <blockpin signalname="XLXN_156" name="I0" />
            <blockpin signalname="Sel(3)" name="I1" />
            <blockpin signalname="Ena(3)" name="O" />
        </block>
        <block symbolname="nor7" name="XLXI_37">
            <blockpin signalname="Sel(7)" name="I0" />
            <blockpin signalname="Sel(6)" name="I1" />
            <blockpin signalname="Sel(5)" name="I2" />
            <blockpin signalname="Sel(4)" name="I3" />
            <blockpin signalname="Sel(3)" name="I4" />
            <blockpin signalname="Sel(2)" name="I5" />
            <blockpin signalname="Sel(1)" name="I6" />
            <blockpin signalname="XLXN_141" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_20">
            <blockpin signalname="XLXN_141" name="I0" />
            <blockpin signalname="Sel(0)" name="I1" />
            <blockpin signalname="Ena(0)" name="O" />
        </block>
        <block symbolname="nor7" name="XLXI_40">
            <blockpin signalname="Sel(7)" name="I0" />
            <blockpin signalname="Sel(1)" name="I1" />
            <blockpin signalname="Sel(6)" name="I2" />
            <blockpin signalname="Sel(5)" name="I3" />
            <blockpin signalname="Sel(4)" name="I4" />
            <blockpin signalname="Sel(2)" name="I5" />
            <blockpin signalname="Sel(0)" name="I6" />
            <blockpin signalname="XLXN_156" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_24">
            <blockpin signalname="XLXN_154" name="I0" />
            <blockpin signalname="Sel(2)" name="I1" />
            <blockpin signalname="Ena(2)" name="O" />
        </block>
        <block symbolname="nor7" name="XLXI_39">
            <blockpin signalname="Sel(7)" name="I0" />
            <blockpin signalname="Sel(6)" name="I1" />
            <blockpin signalname="Sel(5)" name="I2" />
            <blockpin signalname="Sel(4)" name="I3" />
            <blockpin signalname="Sel(3)" name="I4" />
            <blockpin signalname="Sel(1)" name="I5" />
            <blockpin signalname="Sel(0)" name="I6" />
            <blockpin signalname="XLXN_154" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_14">
            <blockpin signalname="XLXN_158" name="I0" />
            <blockpin signalname="Sel(5)" name="I1" />
            <blockpin signalname="Ena(5)" name="O" />
        </block>
        <block symbolname="nor7" name="XLXI_42">
            <blockpin signalname="Sel(7)" name="I0" />
            <blockpin signalname="Sel(0)" name="I1" />
            <blockpin signalname="Sel(1)" name="I2" />
            <blockpin signalname="Sel(2)" name="I3" />
            <blockpin signalname="Sel(3)" name="I4" />
            <blockpin signalname="Sel(6)" name="I5" />
            <blockpin signalname="Sel(4)" name="I6" />
            <blockpin signalname="XLXN_158" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_4">
            <blockpin signalname="XLXN_157" name="I0" />
            <blockpin signalname="Sel(4)" name="I1" />
            <blockpin signalname="Ena(4)" name="O" />
        </block>
        <block symbolname="nor7" name="XLXI_41">
            <blockpin signalname="Sel(7)" name="I0" />
            <blockpin signalname="Sel(5)" name="I1" />
            <blockpin signalname="Sel(3)" name="I2" />
            <blockpin signalname="Sel(2)" name="I3" />
            <blockpin signalname="Sel(1)" name="I4" />
            <blockpin signalname="Sel(0)" name="I5" />
            <blockpin signalname="Sel(6)" name="I6" />
            <blockpin signalname="XLXN_157" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_22">
            <blockpin signalname="XLXN_155" name="I0" />
            <blockpin signalname="Sel(1)" name="I1" />
            <blockpin signalname="Ena(1)" name="O" />
        </block>
        <block symbolname="nor7" name="XLXI_38">
            <blockpin signalname="Sel(7)" name="I0" />
            <blockpin signalname="Sel(5)" name="I1" />
            <blockpin signalname="Sel(4)" name="I2" />
            <blockpin signalname="Sel(3)" name="I3" />
            <blockpin signalname="Sel(2)" name="I4" />
            <blockpin signalname="Sel(6)" name="I5" />
            <blockpin signalname="Sel(0)" name="I6" />
            <blockpin signalname="XLXN_155" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_69">
            <blockpin signalname="XLXN_290" name="I0" />
            <blockpin signalname="Sel(6)" name="I1" />
            <blockpin signalname="Ena(6)" name="O" />
        </block>
        <block symbolname="nor7" name="XLXI_70">
            <blockpin signalname="Sel(7)" name="I0" />
            <blockpin signalname="Sel(5)" name="I1" />
            <blockpin signalname="Sel(4)" name="I2" />
            <blockpin signalname="Sel(3)" name="I3" />
            <blockpin signalname="Sel(2)" name="I4" />
            <blockpin signalname="Sel(1)" name="I5" />
            <blockpin signalname="Sel(0)" name="I6" />
            <blockpin signalname="XLXN_290" name="O" />
        </block>
        <block symbolname="and2" name="XLXI_71">
            <blockpin signalname="XLXN_300" name="I0" />
            <blockpin signalname="Sel(7)" name="I1" />
            <blockpin signalname="Ena(7)" name="O" />
        </block>
        <block symbolname="nor7" name="XLXI_72">
            <blockpin signalname="Sel(6)" name="I0" />
            <blockpin signalname="Sel(5)" name="I1" />
            <blockpin signalname="Sel(4)" name="I2" />
            <blockpin signalname="Sel(3)" name="I3" />
            <blockpin signalname="Sel(2)" name="I4" />
            <blockpin signalname="Sel(1)" name="I5" />
            <blockpin signalname="Sel(0)" name="I6" />
            <blockpin signalname="XLXN_300" name="O" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="2720" height="1760">
        <branch name="Sel(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="208" y="736" type="branch" />
            <wire x2="240" y1="736" y2="736" x1="208" />
        </branch>
        <branch name="Sel(4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="208" y="864" type="branch" />
            <wire x2="240" y1="864" y2="864" x1="208" />
        </branch>
        <branch name="Sel(5)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="208" y="928" type="branch" />
            <wire x2="240" y1="928" y2="928" x1="208" />
        </branch>
        <branch name="Sel(6)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="208" y="992" type="branch" />
            <wire x2="240" y1="992" y2="992" x1="208" />
        </branch>
        <branch name="Sel(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="208" y="1056" type="branch" />
            <wire x2="240" y1="1056" y2="1056" x1="208" />
        </branch>
        <branch name="Sel(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="208" y="800" type="branch" />
            <wire x2="240" y1="800" y2="800" x1="208" />
        </branch>
        <instance x="528" y="960" name="XLXI_2" orien="R0" />
        <branch name="Ena(3)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="832" y="864" type="branch" />
            <wire x2="832" y1="864" y2="864" x1="784" />
        </branch>
        <branch name="Sel(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="208" y="688" type="branch" />
            <wire x2="512" y1="688" y2="688" x1="208" />
            <wire x2="512" y1="688" y2="832" x1="512" />
            <wire x2="528" y1="832" y2="832" x1="512" />
        </branch>
        <branch name="Sel(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="192" y="160" type="branch" />
            <wire x2="224" y1="160" y2="160" x1="192" />
        </branch>
        <branch name="Sel(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="192" y="288" type="branch" />
            <wire x2="224" y1="288" y2="288" x1="192" />
        </branch>
        <branch name="Sel(4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="192" y="352" type="branch" />
            <wire x2="224" y1="352" y2="352" x1="192" />
        </branch>
        <branch name="Sel(5)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="192" y="416" type="branch" />
            <wire x2="224" y1="416" y2="416" x1="192" />
        </branch>
        <branch name="Sel(6)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="192" y="480" type="branch" />
            <wire x2="224" y1="480" y2="480" x1="192" />
        </branch>
        <branch name="Sel(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="192" y="224" type="branch" />
            <wire x2="224" y1="224" y2="224" x1="192" />
        </branch>
        <branch name="Sel(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="192" y="112" type="branch" />
            <wire x2="496" y1="112" y2="112" x1="192" />
            <wire x2="496" y1="112" y2="288" x1="496" />
            <wire x2="512" y1="288" y2="288" x1="496" />
        </branch>
        <instance x="224" y="608" name="XLXI_37" orien="R0" />
        <instance x="512" y="416" name="XLXI_20" orien="R0" />
        <branch name="XLXN_141">
            <wire x2="512" y1="352" y2="352" x1="480" />
        </branch>
        <branch name="Ena(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="800" y="320" type="branch" />
            <wire x2="800" y1="320" y2="320" x1="768" />
        </branch>
        <branch name="Sel(7)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="192" y="544" type="branch" />
            <wire x2="224" y1="544" y2="544" x1="192" />
        </branch>
        <instance x="240" y="1184" name="XLXI_40" orien="R0" />
        <branch name="Sel(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="160" type="branch" />
            <wire x2="1968" y1="160" y2="160" x1="1936" />
        </branch>
        <branch name="Sel(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="288" type="branch" />
            <wire x2="1968" y1="288" y2="288" x1="1936" />
        </branch>
        <branch name="Sel(4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="352" type="branch" />
            <wire x2="1968" y1="352" y2="352" x1="1936" />
        </branch>
        <branch name="Sel(5)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="416" type="branch" />
            <wire x2="1968" y1="416" y2="416" x1="1936" />
        </branch>
        <branch name="Sel(6)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="480" type="branch" />
            <wire x2="1968" y1="480" y2="480" x1="1936" />
        </branch>
        <branch name="Sel(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="224" type="branch" />
            <wire x2="1968" y1="224" y2="224" x1="1936" />
        </branch>
        <branch name="Sel(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="112" type="branch" />
            <wire x2="2240" y1="112" y2="112" x1="1936" />
            <wire x2="2240" y1="112" y2="256" x1="2240" />
            <wire x2="2256" y1="256" y2="256" x1="2240" />
        </branch>
        <instance x="2256" y="384" name="XLXI_24" orien="R0" />
        <branch name="Ena(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2576" y="288" type="branch" />
            <wire x2="2576" y1="288" y2="288" x1="2512" />
        </branch>
        <instance x="1968" y="608" name="XLXI_39" orien="R0" />
        <branch name="XLXN_154">
            <wire x2="2240" y1="352" y2="352" x1="2224" />
            <wire x2="2240" y1="320" y2="352" x1="2240" />
            <wire x2="2256" y1="320" y2="320" x1="2240" />
        </branch>
        <branch name="XLXN_156">
            <wire x2="512" y1="928" y2="928" x1="496" />
            <wire x2="512" y1="896" y2="928" x1="512" />
            <wire x2="528" y1="896" y2="896" x1="512" />
        </branch>
        <branch name="Sel(4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="720" type="branch" />
            <wire x2="1968" y1="720" y2="720" x1="1936" />
        </branch>
        <branch name="Sel(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="848" type="branch" />
            <wire x2="1968" y1="848" y2="848" x1="1936" />
        </branch>
        <branch name="Sel(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="912" type="branch" />
            <wire x2="1968" y1="912" y2="912" x1="1936" />
        </branch>
        <branch name="Sel(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="976" type="branch" />
            <wire x2="1968" y1="976" y2="976" x1="1936" />
        </branch>
        <branch name="Sel(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="1040" type="branch" />
            <wire x2="1968" y1="1040" y2="1040" x1="1936" />
        </branch>
        <branch name="Sel(6)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="784" type="branch" />
            <wire x2="1968" y1="784" y2="784" x1="1936" />
        </branch>
        <branch name="Ena(5)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2560" y="848" type="branch" />
            <wire x2="2560" y1="848" y2="848" x1="2512" />
        </branch>
        <branch name="Sel(5)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1936" y="672" type="branch" />
            <wire x2="2240" y1="672" y2="672" x1="1936" />
            <wire x2="2240" y1="672" y2="816" x1="2240" />
            <wire x2="2256" y1="816" y2="816" x1="2240" />
        </branch>
        <instance x="2256" y="944" name="XLXI_14" orien="R0" />
        <instance x="1968" y="1168" name="XLXI_42" orien="R0" />
        <branch name="XLXN_158">
            <wire x2="2240" y1="912" y2="912" x1="2224" />
            <wire x2="2240" y1="880" y2="912" x1="2240" />
            <wire x2="2256" y1="880" y2="880" x1="2240" />
        </branch>
        <branch name="Sel(6)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1056" y="736" type="branch" />
            <wire x2="1088" y1="736" y2="736" x1="1056" />
        </branch>
        <branch name="Sel(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1056" y="864" type="branch" />
            <wire x2="1088" y1="864" y2="864" x1="1056" />
        </branch>
        <branch name="Sel(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1056" y="928" type="branch" />
            <wire x2="1088" y1="928" y2="928" x1="1056" />
        </branch>
        <branch name="Sel(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1056" y="992" type="branch" />
            <wire x2="1088" y1="992" y2="992" x1="1056" />
        </branch>
        <branch name="Sel(5)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1056" y="1056" type="branch" />
            <wire x2="1088" y1="1056" y2="1056" x1="1056" />
        </branch>
        <branch name="Sel(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1056" y="800" type="branch" />
            <wire x2="1088" y1="800" y2="800" x1="1056" />
        </branch>
        <branch name="Ena(4)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1680" y="864" type="branch" />
            <wire x2="1680" y1="864" y2="864" x1="1632" />
        </branch>
        <branch name="Sel(4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1056" y="688" type="branch" />
            <wire x2="1360" y1="688" y2="688" x1="1056" />
            <wire x2="1360" y1="688" y2="832" x1="1360" />
            <wire x2="1376" y1="832" y2="832" x1="1360" />
        </branch>
        <instance x="1376" y="960" name="XLXI_4" orien="R0" />
        <instance x="1088" y="1184" name="XLXI_41" orien="R0" />
        <branch name="XLXN_157">
            <wire x2="1360" y1="928" y2="928" x1="1344" />
            <wire x2="1360" y1="896" y2="928" x1="1360" />
            <wire x2="1376" y1="896" y2="896" x1="1360" />
        </branch>
        <branch name="Sel(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1040" y="144" type="branch" />
            <wire x2="1072" y1="144" y2="144" x1="1040" />
        </branch>
        <branch name="Sel(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1040" y="272" type="branch" />
            <wire x2="1072" y1="272" y2="272" x1="1040" />
        </branch>
        <branch name="Sel(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1040" y="336" type="branch" />
            <wire x2="1072" y1="336" y2="336" x1="1040" />
        </branch>
        <branch name="Sel(4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1040" y="400" type="branch" />
            <wire x2="1072" y1="400" y2="400" x1="1040" />
        </branch>
        <branch name="Sel(5)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1040" y="464" type="branch" />
            <wire x2="1072" y1="464" y2="464" x1="1040" />
        </branch>
        <branch name="Ena(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1664" y="272" type="branch" />
            <wire x2="1664" y1="272" y2="272" x1="1616" />
        </branch>
        <branch name="Sel(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1040" y="96" type="branch" />
            <wire x2="1344" y1="96" y2="96" x1="1040" />
            <wire x2="1344" y1="96" y2="240" x1="1344" />
            <wire x2="1360" y1="240" y2="240" x1="1344" />
        </branch>
        <instance x="1360" y="368" name="XLXI_22" orien="R0" />
        <branch name="Sel(6)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="208" type="branch" />
            <wire x2="1072" y1="208" y2="208" x1="1024" />
        </branch>
        <instance x="1072" y="592" name="XLXI_38" orien="R0" />
        <branch name="XLXN_155">
            <wire x2="1344" y1="336" y2="336" x1="1328" />
            <wire x2="1344" y1="304" y2="336" x1="1344" />
            <wire x2="1360" y1="304" y2="304" x1="1344" />
        </branch>
        <branch name="Sel(7)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1024" y="528" type="branch" />
            <wire x2="1072" y1="528" y2="528" x1="1024" />
        </branch>
        <branch name="Sel(7)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1920" y="544" type="branch" />
            <wire x2="1968" y1="544" y2="544" x1="1920" />
        </branch>
        <branch name="Sel(7)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="176" y="1120" type="branch" />
            <wire x2="240" y1="1120" y2="1120" x1="176" />
        </branch>
        <branch name="Sel(7)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1008" y="1120" type="branch" />
            <wire x2="1088" y1="1120" y2="1120" x1="1008" />
        </branch>
        <branch name="Sel(7)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1920" y="1104" type="branch" />
            <wire x2="1968" y1="1104" y2="1104" x1="1920" />
        </branch>
        <branch name="Sel(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="608" y="1280" type="branch" />
            <wire x2="640" y1="1280" y2="1280" x1="608" />
        </branch>
        <branch name="Sel(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="608" y="1408" type="branch" />
            <wire x2="640" y1="1408" y2="1408" x1="608" />
        </branch>
        <branch name="Sel(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="608" y="1472" type="branch" />
            <wire x2="640" y1="1472" y2="1472" x1="608" />
        </branch>
        <branch name="Sel(4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="608" y="1536" type="branch" />
            <wire x2="640" y1="1536" y2="1536" x1="608" />
        </branch>
        <branch name="Sel(5)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="608" y="1600" type="branch" />
            <wire x2="640" y1="1600" y2="1600" x1="608" />
        </branch>
        <branch name="Sel(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="608" y="1344" type="branch" />
            <wire x2="640" y1="1344" y2="1344" x1="608" />
        </branch>
        <branch name="Ena(6)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1232" y="1408" type="branch" />
            <wire x2="1232" y1="1408" y2="1408" x1="1184" />
        </branch>
        <branch name="Sel(6)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="608" y="1232" type="branch" />
            <wire x2="912" y1="1232" y2="1232" x1="608" />
            <wire x2="912" y1="1232" y2="1376" x1="912" />
            <wire x2="928" y1="1376" y2="1376" x1="912" />
        </branch>
        <instance x="928" y="1504" name="XLXI_69" orien="R0" />
        <instance x="640" y="1728" name="XLXI_70" orien="R0" />
        <branch name="XLXN_290">
            <wire x2="912" y1="1472" y2="1472" x1="896" />
            <wire x2="912" y1="1440" y2="1472" x1="912" />
            <wire x2="928" y1="1440" y2="1440" x1="912" />
        </branch>
        <branch name="Sel(7)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="592" y="1664" type="branch" />
            <wire x2="640" y1="1664" y2="1664" x1="592" />
        </branch>
        <branch name="Sel(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1664" y="1280" type="branch" />
            <wire x2="1696" y1="1280" y2="1280" x1="1664" />
        </branch>
        <branch name="Sel(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1664" y="1408" type="branch" />
            <wire x2="1696" y1="1408" y2="1408" x1="1664" />
        </branch>
        <branch name="Sel(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1664" y="1472" type="branch" />
            <wire x2="1696" y1="1472" y2="1472" x1="1664" />
        </branch>
        <branch name="Sel(4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1664" y="1536" type="branch" />
            <wire x2="1696" y1="1536" y2="1536" x1="1664" />
        </branch>
        <branch name="Sel(5)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1664" y="1600" type="branch" />
            <wire x2="1696" y1="1600" y2="1600" x1="1664" />
        </branch>
        <branch name="Sel(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1664" y="1344" type="branch" />
            <wire x2="1696" y1="1344" y2="1344" x1="1664" />
        </branch>
        <branch name="Ena(7)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2288" y="1408" type="branch" />
            <wire x2="2288" y1="1408" y2="1408" x1="2240" />
        </branch>
        <branch name="Sel(7)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1664" y="1232" type="branch" />
            <wire x2="1968" y1="1232" y2="1232" x1="1664" />
            <wire x2="1968" y1="1232" y2="1376" x1="1968" />
            <wire x2="1984" y1="1376" y2="1376" x1="1968" />
        </branch>
        <instance x="1984" y="1504" name="XLXI_71" orien="R0" />
        <instance x="1696" y="1728" name="XLXI_72" orien="R0" />
        <branch name="XLXN_300">
            <wire x2="1968" y1="1472" y2="1472" x1="1952" />
            <wire x2="1968" y1="1440" y2="1472" x1="1968" />
            <wire x2="1984" y1="1440" y2="1440" x1="1968" />
        </branch>
        <branch name="Sel(6)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1648" y="1664" type="branch" />
            <wire x2="1696" y1="1664" y2="1664" x1="1648" />
        </branch>
        <branch name="Ena(7:0)">
            <wire x2="304" y1="1392" y2="1392" x1="144" />
        </branch>
        <iomarker fontsize="28" x="304" y="1392" name="Ena(7:0)" orien="R0" />
        <iomarker fontsize="28" x="192" y="1296" name="Sel(7:0)" orien="R180" />
        <branch name="Sel(7:0)">
            <wire x2="288" y1="1296" y2="1296" x1="192" />
        </branch>
    </sheet>
</drawing>