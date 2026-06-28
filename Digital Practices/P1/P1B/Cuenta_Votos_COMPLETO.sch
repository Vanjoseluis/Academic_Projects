<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="SA(0)" />
        <signal name="SA(1)" />
        <signal name="SA(2)" />
        <signal name="SB(0)" />
        <signal name="SB(1)" />
        <signal name="SB(2)" />
        <signal name="SVF(2)" />
        <signal name="SVF(0)" />
        <signal name="SVF(1)" />
        <signal name="SVF(3)" />
        <signal name="XLXN_80" />
        <signal name="XLXN_81" />
        <signal name="XLXN_88" />
        <signal name="Const_8(3)" />
        <signal name="XLXN_27" />
        <signal name="SVF(3:0)" />
        <signal name="SVC(3)" />
        <signal name="SVC(2)" />
        <signal name="SVC(1)" />
        <signal name="SVC(0)" />
        <signal name="SVC(3:0)" />
        <signal name="M(7:0)" />
        <signal name="Const_8(3:0)" />
        <signal name="Const_8(2)" />
        <signal name="Const_8(1)" />
        <signal name="Const_8(0)" />
        <signal name="SA(2:0)" />
        <signal name="M(3:0)" />
        <signal name="SB(2:0)" />
        <signal name="M(7:4)" />
        <port polarity="Output" name="SVF(3:0)" />
        <port polarity="Output" name="SVC(3:0)" />
        <port polarity="Input" name="M(7:0)" />
        <blockdef name="gnd">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-96" x1="64" />
            <line x2="52" y1="-48" y2="-48" x1="76" />
            <line x2="60" y1="-32" y2="-32" x1="68" />
            <line x2="40" y1="-64" y2="-64" x1="88" />
            <line x2="64" y1="-64" y2="-80" x1="64" />
            <line x2="64" y1="-128" y2="-96" x1="64" />
        </blockdef>
        <blockdef name="Sumador_Completo">
            <timestamp>2026-3-29T14:15:31</timestamp>
            <rect width="256" x="64" y="-576" height="576" />
            <line x2="0" y1="-544" y2="-544" x1="64" />
            <line x2="0" y1="-480" y2="-480" x1="64" />
            <line x2="0" y1="-416" y2="-416" x1="64" />
            <line x2="0" y1="-352" y2="-352" x1="64" />
            <line x2="0" y1="-288" y2="-288" x1="64" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="384" y1="-288" y2="-288" x1="320" />
            <line x2="384" y1="-544" y2="-544" x1="320" />
            <line x2="384" y1="-480" y2="-480" x1="320" />
            <line x2="384" y1="-416" y2="-416" x1="320" />
            <line x2="384" y1="-352" y2="-352" x1="320" />
        </blockdef>
        <blockdef name="Restador">
            <timestamp>2026-3-29T14:18:58</timestamp>
            <rect width="256" x="64" y="-576" height="576" />
            <line x2="0" y1="-544" y2="-544" x1="64" />
            <line x2="0" y1="-480" y2="-480" x1="64" />
            <line x2="0" y1="-416" y2="-416" x1="64" />
            <line x2="0" y1="-352" y2="-352" x1="64" />
            <line x2="0" y1="-288" y2="-288" x1="64" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="384" y1="-544" y2="-544" x1="320" />
            <line x2="384" y1="-480" y2="-480" x1="320" />
            <line x2="384" y1="-416" y2="-416" x1="320" />
            <line x2="384" y1="-352" y2="-352" x1="320" />
            <line x2="384" y1="-288" y2="-288" x1="320" />
        </blockdef>
        <blockdef name="Const_8">
            <timestamp>2026-4-1T17:54:36</timestamp>
            <rect width="256" x="0" y="-64" height="64" />
            <line x2="320" y1="-32" y2="-32" x1="256" />
            <rect width="64" x="256" y="-44" height="24" />
        </blockdef>
        <blockdef name="Cuenta_4votosVHDL">
            <timestamp>2026-4-16T8:40:2</timestamp>
            <rect width="256" x="64" y="-64" height="64" />
            <rect width="64" x="0" y="-44" height="24" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <rect width="64" x="320" y="-44" height="24" />
            <line x2="384" y1="-32" y2="-32" x1="320" />
        </blockdef>
        <block symbolname="gnd" name="XLXI_39">
            <blockpin signalname="XLXN_80" name="G" />
        </block>
        <block symbolname="gnd" name="XLXI_40">
            <blockpin signalname="XLXN_81" name="G" />
        </block>
        <block symbolname="gnd" name="XLXI_41">
            <blockpin signalname="XLXN_88" name="G" />
        </block>
        <block symbolname="gnd" name="XLXI_12">
            <blockpin signalname="XLXN_27" name="G" />
        </block>
        <block symbolname="Sumador_Completo" name="XLXI_43">
            <blockpin signalname="SA(0)" name="A0" />
            <blockpin signalname="SA(1)" name="A1" />
            <blockpin signalname="SA(2)" name="A2" />
            <blockpin signalname="XLXN_88" name="A3" />
            <blockpin signalname="SB(0)" name="B0" />
            <blockpin signalname="SB(1)" name="B1" />
            <blockpin signalname="SB(2)" name="B2" />
            <blockpin signalname="XLXN_80" name="B3" />
            <blockpin signalname="XLXN_81" name="C0" />
            <blockpin name="C4" />
            <blockpin signalname="SVF(0)" name="S0" />
            <blockpin signalname="SVF(1)" name="S1" />
            <blockpin signalname="SVF(2)" name="S2" />
            <blockpin signalname="SVF(3)" name="S3" />
        </block>
        <block symbolname="Restador" name="XLXI_44">
            <blockpin signalname="Const_8(0)" name="A0" />
            <blockpin signalname="Const_8(1)" name="A1" />
            <blockpin signalname="Const_8(2)" name="A2" />
            <blockpin signalname="Const_8(3)" name="A3" />
            <blockpin signalname="SVF(0)" name="B0" />
            <blockpin signalname="SVF(1)" name="B1" />
            <blockpin signalname="SVF(2)" name="B2" />
            <blockpin signalname="SVF(3)" name="B3" />
            <blockpin signalname="XLXN_27" name="C0" />
            <blockpin signalname="SVC(0)" name="S0" />
            <blockpin signalname="SVC(1)" name="S1" />
            <blockpin signalname="SVC(2)" name="S2" />
            <blockpin signalname="SVC(3)" name="S3" />
            <blockpin name="S4" />
        </block>
        <block symbolname="Const_8" name="XLXI_45">
            <blockpin signalname="Const_8(3:0)" name="Const_8(3:0)" />
        </block>
        <block symbolname="Cuenta_4votosVHDL" name="XLXI_47">
            <blockpin signalname="M(3:0)" name="M(3:0)" />
            <blockpin signalname="SA(2:0)" name="S(2:0)" />
        </block>
        <block symbolname="Cuenta_4votosVHDL" name="XLXI_46">
            <blockpin signalname="M(7:4)" name="M(3:0)" />
            <blockpin signalname="SB(2:0)" name="S(2:0)" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <branch name="SA(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1312" y="752" type="branch" />
            <wire x2="1392" y1="752" y2="752" x1="1312" />
        </branch>
        <branch name="SA(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1312" y="816" type="branch" />
            <wire x2="1392" y1="816" y2="816" x1="1312" />
        </branch>
        <branch name="SA(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1312" y="880" type="branch" />
            <wire x2="1392" y1="880" y2="880" x1="1312" />
        </branch>
        <branch name="SVF(2)">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2816" y="880" type="branch" />
            <wire x2="2224" y1="880" y2="880" x1="1776" />
            <wire x2="2224" y1="880" y2="1248" x1="2224" />
            <wire x2="2816" y1="880" y2="880" x1="2224" />
            <wire x2="2896" y1="880" y2="880" x1="2816" />
        </branch>
        <branch name="SVF(0)">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2816" y="752" type="branch" />
            <wire x2="2352" y1="752" y2="752" x1="1776" />
            <wire x2="2352" y1="752" y2="1248" x1="2352" />
            <wire x2="2816" y1="752" y2="752" x1="2352" />
            <wire x2="2896" y1="752" y2="752" x1="2816" />
        </branch>
        <branch name="SVF(1)">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2816" y="816" type="branch" />
            <wire x2="2288" y1="816" y2="816" x1="1776" />
            <wire x2="2288" y1="816" y2="1248" x1="2288" />
            <wire x2="2816" y1="816" y2="816" x1="2288" />
            <wire x2="2896" y1="816" y2="816" x1="2816" />
        </branch>
        <branch name="SVF(3)">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2816" y="944" type="branch" />
            <wire x2="2160" y1="944" y2="944" x1="1776" />
            <wire x2="2160" y1="944" y2="1248" x1="2160" />
            <wire x2="2816" y1="944" y2="944" x1="2160" />
            <wire x2="2896" y1="944" y2="944" x1="2816" />
        </branch>
        <branch name="XLXN_80">
            <wire x2="1392" y1="1200" y2="1200" x1="1360" />
        </branch>
        <instance x="1232" y="1200" name="XLXI_40" orien="R90" />
        <instance x="1232" y="1136" name="XLXI_39" orien="R90" />
        <branch name="XLXN_81">
            <wire x2="1392" y1="1264" y2="1264" x1="1360" />
        </branch>
        <branch name="SB(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1360" y="1008" type="branch" />
            <wire x2="1392" y1="1008" y2="1008" x1="1360" />
        </branch>
        <branch name="SB(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1360" y="1072" type="branch" />
            <wire x2="1392" y1="1072" y2="1072" x1="1360" />
        </branch>
        <branch name="SB(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1360" y="1136" type="branch" />
            <wire x2="1376" y1="1136" y2="1136" x1="1360" />
            <wire x2="1392" y1="1136" y2="1136" x1="1376" />
        </branch>
        <instance x="1248" y="880" name="XLXI_41" orien="R90" />
        <branch name="XLXN_88">
            <wire x2="1392" y1="944" y2="944" x1="1376" />
        </branch>
        <branch name="Const_8(3)">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="2416" y="1200" type="branch" />
            <wire x2="2416" y1="1200" y2="1248" x1="2416" />
        </branch>
        <branch name="XLXN_27">
            <wire x2="2096" y1="1216" y2="1248" x1="2096" />
        </branch>
        <instance x="2160" y="1088" name="XLXI_12" orien="R180" />
        <branch name="SVF(3:0)">
            <wire x2="2992" y1="624" y2="752" x1="2992" />
            <wire x2="2992" y1="752" y2="816" x1="2992" />
            <wire x2="2992" y1="816" y2="880" x1="2992" />
            <wire x2="2992" y1="880" y2="944" x1="2992" />
            <wire x2="2992" y1="944" y2="1024" x1="2992" />
        </branch>
        <bustap x2="2896" y1="752" y2="752" x1="2992" />
        <bustap x2="2896" y1="816" y2="816" x1="2992" />
        <bustap x2="2896" y1="880" y2="880" x1="2992" />
        <bustap x2="2896" y1="944" y2="944" x1="2992" />
        <branch name="SVC(3)">
            <attrtext style="alignment:SOFT-TVCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2416" y="1648" type="branch" />
            <wire x2="2416" y1="1632" y2="1648" x1="2416" />
            <wire x2="2416" y1="1648" y2="1712" x1="2416" />
        </branch>
        <branch name="SVC(2)">
            <attrtext style="alignment:SOFT-TVCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2480" y="1648" type="branch" />
            <wire x2="2480" y1="1632" y2="1648" x1="2480" />
            <wire x2="2480" y1="1648" y2="1712" x1="2480" />
        </branch>
        <branch name="SVC(1)">
            <attrtext style="alignment:SOFT-TVCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2544" y="1648" type="branch" />
            <wire x2="2544" y1="1632" y2="1648" x1="2544" />
            <wire x2="2544" y1="1648" y2="1712" x1="2544" />
        </branch>
        <branch name="SVC(0)">
            <attrtext style="alignment:SOFT-TVCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2608" y="1648" type="branch" />
            <wire x2="2608" y1="1632" y2="1648" x1="2608" />
            <wire x2="2608" y1="1648" y2="1712" x1="2608" />
        </branch>
        <branch name="SVC(3:0)">
            <wire x2="2416" y1="1808" y2="1808" x1="2224" />
            <wire x2="2480" y1="1808" y2="1808" x1="2416" />
            <wire x2="2544" y1="1808" y2="1808" x1="2480" />
            <wire x2="2608" y1="1808" y2="1808" x1="2544" />
            <wire x2="2784" y1="1808" y2="1808" x1="2608" />
        </branch>
        <bustap x2="2416" y1="1808" y2="1712" x1="2416" />
        <bustap x2="2480" y1="1808" y2="1712" x1="2480" />
        <bustap x2="2544" y1="1808" y2="1712" x1="2544" />
        <bustap x2="2608" y1="1808" y2="1712" x1="2608" />
        <iomarker fontsize="28" x="288" y="704" name="M(7:0)" orien="R180" />
        <branch name="M(7:0)">
            <wire x2="288" y1="704" y2="848" x1="288" />
        </branch>
        <iomarker fontsize="28" x="2992" y="624" name="SVF(3:0)" orien="R270" />
        <iomarker fontsize="28" x="2224" y="1808" name="SVC(3:0)" orien="R180" />
        <instance x="1392" y="1296" name="XLXI_43" orien="R0">
        </instance>
        <instance x="2064" y="1248" name="XLXI_44" orien="R90">
        </instance>
        <branch name="Const_8(2)">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="2480" y="1200" type="branch" />
            <wire x2="2480" y1="1200" y2="1248" x1="2480" />
        </branch>
        <branch name="Const_8(1)">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="2544" y="1200" type="branch" />
            <wire x2="2544" y1="1200" y2="1248" x1="2544" />
        </branch>
        <branch name="Const_8(0)">
            <attrtext style="alignment:SOFT-VLEFT;fontsize:28;fontname:Arial" attrname="Name" x="2608" y="1200" type="branch" />
            <wire x2="2608" y1="1200" y2="1248" x1="2608" />
        </branch>
        <branch name="Const_8(3:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3120" y="1232" type="branch" />
            <wire x2="3104" y1="1232" y2="1232" x1="3024" />
            <wire x2="3120" y1="1232" y2="1232" x1="3104" />
        </branch>
        <instance x="2704" y="1264" name="XLXI_45" orien="R0">
        </instance>
        <instance x="592" y="880" name="XLXI_47" orien="R0">
        </instance>
        <branch name="SA(2:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1056" y="848" type="branch" />
            <wire x2="1056" y1="848" y2="848" x1="976" />
        </branch>
        <branch name="M(3:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="512" y="848" type="branch" />
            <wire x2="592" y1="848" y2="848" x1="512" />
        </branch>
        <branch name="SB(2:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1056" y="1104" type="branch" />
            <wire x2="1040" y1="1104" y2="1104" x1="976" />
            <wire x2="1056" y1="1104" y2="1104" x1="1040" />
        </branch>
        <branch name="M(7:4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="512" y="1104" type="branch" />
            <wire x2="528" y1="1104" y2="1104" x1="512" />
            <wire x2="592" y1="1104" y2="1104" x1="528" />
        </branch>
        <instance x="592" y="1136" name="XLXI_46" orien="R0">
        </instance>
    </sheet>
</drawing>