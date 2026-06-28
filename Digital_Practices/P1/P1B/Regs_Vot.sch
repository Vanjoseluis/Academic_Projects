<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="VF" />
        <signal name="VC" />
        <signal name="CK" />
        <signal name="CLR" />
        <signal name="V_RR" />
        <signal name="Sel(7:0)" />
        <signal name="VF_C(7:0)" />
        <signal name="Led_V(7:0)" />
        <signal name="V_R(7:0)" />
        <signal name="Led_V(0)" />
        <signal name="V_R(0)" />
        <signal name="VF_C(0)" />
        <signal name="Led_V(1)" />
        <signal name="V_R(1)" />
        <signal name="VF_C(1)" />
        <signal name="Led_V(2)" />
        <signal name="V_R(2)" />
        <signal name="VF_C(2)" />
        <signal name="Led_V(3)" />
        <signal name="V_R(3)" />
        <signal name="VF_C(3)" />
        <signal name="Led_V(4)" />
        <signal name="V_R(4)" />
        <signal name="VF_C(4)" />
        <signal name="Led_V(5)" />
        <signal name="V_R(5)" />
        <signal name="VF_C(5)" />
        <signal name="V_R(6)" />
        <signal name="Ena(0)" />
        <signal name="Ena(3)" />
        <signal name="Ena(1)" />
        <signal name="Ena(4)" />
        <signal name="Ena(2)" />
        <signal name="Ena(5)" />
        <signal name="VF_C(6)" />
        <signal name="Ena(6)" />
        <signal name="V_R(7)" />
        <signal name="VF_C(7)" />
        <signal name="Ena(7:0)" />
        <signal name="Ena(7)" />
        <signal name="Led_V(6)" />
        <signal name="Led_V(7)" />
        <port polarity="Input" name="VF" />
        <port polarity="Input" name="VC" />
        <port polarity="Input" name="CK" />
        <port polarity="Input" name="CLR" />
        <port polarity="Output" name="V_RR" />
        <port polarity="Input" name="Sel(7:0)" />
        <port polarity="Output" name="VF_C(7:0)" />
        <port polarity="Output" name="Led_V(7:0)" />
        <port polarity="Output" name="V_R(7:0)" />
        <blockdef name="ENA_GEN">
            <timestamp>2023-3-12T20:1:41</timestamp>
            <rect width="256" x="64" y="-64" height="64" />
            <line x2="384" y1="-32" y2="-32" x1="320" />
            <rect width="64" x="320" y="-44" height="24" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <rect width="64" x="0" y="-44" height="24" />
        </blockdef>
        <blockdef name="and8">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-512" x1="64" />
            <line x2="64" y1="-64" y2="-64" x1="0" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="64" y1="-320" y2="-320" x1="0" />
            <line x2="64" y1="-384" y2="-384" x1="0" />
            <line x2="64" y1="-448" y2="-448" x1="0" />
            <line x2="64" y1="-512" y2="-512" x1="0" />
            <line x2="144" y1="-336" y2="-336" x1="64" />
            <line x2="64" y1="-240" y2="-240" x1="144" />
            <arc ex="144" ey="-336" sx="144" sy="-240" r="48" cx="144" cy="-288" />
            <line x2="192" y1="-288" y2="-288" x1="256" />
        </blockdef>
        <blockdef name="Reg_Vot_VHDL_Exc">
            <timestamp>2026-5-7T8:20:22</timestamp>
            <rect width="256" x="64" y="-320" height="320" />
            <line x2="0" y1="-288" y2="-288" x1="64" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <line x2="384" y1="-288" y2="-288" x1="320" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="384" y1="-160" y2="-160" x1="320" />
            <line x2="384" y1="-224" y2="-224" x1="320" />
        </blockdef>
        <blockdef name="Reg_Vot_VHDL_Case">
            <timestamp>2026-5-7T8:45:27</timestamp>
            <rect width="256" x="64" y="-320" height="320" />
            <line x2="0" y1="-288" y2="-288" x1="64" />
            <line x2="0" y1="-224" y2="-224" x1="64" />
            <line x2="0" y1="-160" y2="-160" x1="64" />
            <line x2="0" y1="-96" y2="-96" x1="64" />
            <line x2="0" y1="-32" y2="-32" x1="64" />
            <line x2="384" y1="-288" y2="-288" x1="320" />
            <line x2="384" y1="-208" y2="-208" x1="320" />
            <line x2="384" y1="-128" y2="-128" x1="320" />
        </blockdef>
        <block symbolname="and8" name="XLXI_33">
            <blockpin signalname="V_R(7)" name="I0" />
            <blockpin signalname="V_R(6)" name="I1" />
            <blockpin signalname="V_R(5)" name="I2" />
            <blockpin signalname="V_R(4)" name="I3" />
            <blockpin signalname="V_R(3)" name="I4" />
            <blockpin signalname="V_R(2)" name="I5" />
            <blockpin signalname="V_R(1)" name="I6" />
            <blockpin signalname="V_R(0)" name="I7" />
            <blockpin signalname="V_RR" name="O" />
        </block>
        <block symbolname="ENA_GEN" name="XLXI_34">
            <blockpin signalname="Ena(7:0)" name="Ena(7:0)" />
            <blockpin signalname="Sel(7:0)" name="Sel(7:0)" />
        </block>
        <block symbolname="Reg_Vot_VHDL_Exc" name="XLXI_40">
            <blockpin signalname="VF" name="VF" />
            <blockpin signalname="VC" name="VC" />
            <blockpin signalname="Ena(0)" name="ENA" />
            <blockpin signalname="VF_C(0)" name="VF_C" />
            <blockpin signalname="CK" name="CK" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="Led_V(0)" name="Led_V" />
            <blockpin signalname="V_R(0)" name="V_R" />
        </block>
        <block symbolname="Reg_Vot_VHDL_Exc" name="XLXI_41">
            <blockpin signalname="VF" name="VF" />
            <blockpin signalname="VC" name="VC" />
            <blockpin signalname="Ena(1)" name="ENA" />
            <blockpin signalname="VF_C(1)" name="VF_C" />
            <blockpin signalname="CK" name="CK" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="Led_V(1)" name="Led_V" />
            <blockpin signalname="V_R(1)" name="V_R" />
        </block>
        <block symbolname="Reg_Vot_VHDL_Exc" name="XLXI_42">
            <blockpin signalname="VF" name="VF" />
            <blockpin signalname="VC" name="VC" />
            <blockpin signalname="Ena(2)" name="ENA" />
            <blockpin signalname="VF_C(2)" name="VF_C" />
            <blockpin signalname="CK" name="CK" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="Led_V(2)" name="Led_V" />
            <blockpin signalname="V_R(2)" name="V_R" />
        </block>
        <block symbolname="Reg_Vot_VHDL_Exc" name="XLXI_45">
            <blockpin signalname="VF" name="VF" />
            <blockpin signalname="VC" name="VC" />
            <blockpin signalname="Ena(5)" name="ENA" />
            <blockpin signalname="VF_C(5)" name="VF_C" />
            <blockpin signalname="CK" name="CK" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="Led_V(5)" name="Led_V" />
            <blockpin signalname="V_R(5)" name="V_R" />
        </block>
        <block symbolname="Reg_Vot_VHDL_Exc" name="XLXI_46">
            <blockpin signalname="VF" name="VF" />
            <blockpin signalname="VC" name="VC" />
            <blockpin signalname="Ena(4)" name="ENA" />
            <blockpin signalname="VF_C(4)" name="VF_C" />
            <blockpin signalname="CK" name="CK" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="Led_V(4)" name="Led_V" />
            <blockpin signalname="V_R(4)" name="V_R" />
        </block>
        <block symbolname="Reg_Vot_VHDL_Exc" name="XLXI_47">
            <blockpin signalname="VF" name="VF" />
            <blockpin signalname="VC" name="VC" />
            <blockpin signalname="Ena(3)" name="ENA" />
            <blockpin signalname="VF_C(3)" name="VF_C" />
            <blockpin signalname="CK" name="CK" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="Led_V(3)" name="Led_V" />
            <blockpin signalname="V_R(3)" name="V_R" />
        </block>
        <block symbolname="Reg_Vot_VHDL_Case" name="XLXI_52">
            <blockpin signalname="VF" name="VF" />
            <blockpin signalname="VC" name="VC" />
            <blockpin signalname="Ena(6)" name="ENA" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="CK" name="CK" />
            <blockpin signalname="VF_C(6)" name="VF_C" />
            <blockpin signalname="V_R(6)" name="V_R" />
            <blockpin signalname="Led_V(6)" name="led_V" />
        </block>
        <block symbolname="Reg_Vot_VHDL_Case" name="XLXI_54">
            <blockpin signalname="VF" name="VF" />
            <blockpin signalname="VC" name="VC" />
            <blockpin signalname="Ena(7)" name="ENA" />
            <blockpin signalname="CLR" name="CLR" />
            <blockpin signalname="CK" name="CK" />
            <blockpin signalname="VF_C(7)" name="VF_C" />
            <blockpin signalname="V_R(7)" name="V_R" />
            <blockpin signalname="Led_V(7)" name="led_V" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="3520" height="2720">
        <branch name="VF">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="480" y="128" type="branch" />
            <wire x2="480" y1="128" y2="128" x1="464" />
            <wire x2="624" y1="128" y2="128" x1="480" />
        </branch>
        <branch name="VC">
            <wire x2="624" y1="192" y2="192" x1="464" />
        </branch>
        <branch name="Sel(7:0)">
            <wire x2="640" y1="240" y2="240" x1="432" />
        </branch>
        <branch name="CLR">
            <wire x2="656" y1="304" y2="304" x1="448" />
        </branch>
        <branch name="CK">
            <wire x2="672" y1="384" y2="384" x1="464" />
        </branch>
        <iomarker fontsize="28" x="464" y="128" name="VF" orien="R180" />
        <iomarker fontsize="28" x="464" y="192" name="VC" orien="R180" />
        <iomarker fontsize="28" x="432" y="240" name="Sel(7:0)" orien="R180" />
        <iomarker fontsize="28" x="448" y="304" name="CLR" orien="R180" />
        <iomarker fontsize="28" x="464" y="384" name="CK" orien="R180" />
        <branch name="VF_C(7:0)">
            <wire x2="1104" y1="128" y2="128" x1="880" />
        </branch>
        <branch name="Led_V(7:0)">
            <wire x2="1104" y1="192" y2="192" x1="880" />
        </branch>
        <iomarker fontsize="28" x="1104" y="128" name="VF_C(7:0)" orien="R0" />
        <iomarker fontsize="28" x="1104" y="192" name="Led_V(7:0)" orien="R0" />
        <branch name="V_RR">
            <wire x2="1120" y1="304" y2="304" x1="880" />
        </branch>
        <iomarker fontsize="28" x="1120" y="304" name="V_RR" orien="R0" />
        <branch name="V_R(7:0)">
            <wire x2="1088" y1="256" y2="256" x1="880" />
        </branch>
        <iomarker fontsize="28" x="1088" y="256" name="V_R(7:0)" orien="R0" />
        <branch name="VF">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="560" y="640" type="branch" />
            <wire x2="672" y1="640" y2="640" x1="560" />
        </branch>
        <branch name="VC">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="560" y="704" type="branch" />
            <wire x2="672" y1="704" y2="704" x1="560" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="608" y="832" type="branch" />
            <wire x2="672" y1="832" y2="832" x1="608" />
        </branch>
        <branch name="CK">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="608" y="896" type="branch" />
            <wire x2="672" y1="896" y2="896" x1="608" />
        </branch>
        <branch name="Led_V(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1152" y="768" type="branch" />
            <wire x2="1152" y1="768" y2="768" x1="1056" />
        </branch>
        <branch name="V_R(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1152" y="704" type="branch" />
            <wire x2="1152" y1="704" y2="704" x1="1056" />
        </branch>
        <branch name="VF_C(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1152" y="640" type="branch" />
            <wire x2="1152" y1="640" y2="640" x1="1056" />
        </branch>
        <branch name="VF">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1440" y="640" type="branch" />
            <wire x2="1552" y1="640" y2="640" x1="1440" />
        </branch>
        <branch name="VC">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1440" y="704" type="branch" />
            <wire x2="1552" y1="704" y2="704" x1="1440" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1488" y="832" type="branch" />
            <wire x2="1552" y1="832" y2="832" x1="1488" />
        </branch>
        <branch name="CK">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1488" y="896" type="branch" />
            <wire x2="1552" y1="896" y2="896" x1="1488" />
        </branch>
        <branch name="Led_V(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="768" type="branch" />
            <wire x2="2032" y1="768" y2="768" x1="1936" />
        </branch>
        <branch name="V_R(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="704" type="branch" />
            <wire x2="2032" y1="704" y2="704" x1="1936" />
        </branch>
        <branch name="VF_C(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="640" type="branch" />
            <wire x2="2032" y1="640" y2="640" x1="1936" />
        </branch>
        <branch name="VF">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2304" y="640" type="branch" />
            <wire x2="2416" y1="640" y2="640" x1="2304" />
        </branch>
        <branch name="VC">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2304" y="704" type="branch" />
            <wire x2="2416" y1="704" y2="704" x1="2304" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2352" y="832" type="branch" />
            <wire x2="2416" y1="832" y2="832" x1="2352" />
        </branch>
        <branch name="CK">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2352" y="896" type="branch" />
            <wire x2="2416" y1="896" y2="896" x1="2352" />
        </branch>
        <branch name="Led_V(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2896" y="768" type="branch" />
            <wire x2="2896" y1="768" y2="768" x1="2800" />
        </branch>
        <branch name="V_R(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2896" y="704" type="branch" />
            <wire x2="2896" y1="704" y2="704" x1="2800" />
        </branch>
        <branch name="VF_C(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2896" y="640" type="branch" />
            <wire x2="2896" y1="640" y2="640" x1="2800" />
        </branch>
        <branch name="VF">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="560" y="1184" type="branch" />
            <wire x2="672" y1="1184" y2="1184" x1="560" />
        </branch>
        <branch name="VC">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="560" y="1248" type="branch" />
            <wire x2="672" y1="1248" y2="1248" x1="560" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="608" y="1376" type="branch" />
            <wire x2="672" y1="1376" y2="1376" x1="608" />
        </branch>
        <branch name="CK">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="608" y="1440" type="branch" />
            <wire x2="672" y1="1440" y2="1440" x1="608" />
        </branch>
        <branch name="Led_V(3)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1152" y="1312" type="branch" />
            <wire x2="1152" y1="1312" y2="1312" x1="1056" />
        </branch>
        <branch name="V_R(3)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1152" y="1248" type="branch" />
            <wire x2="1152" y1="1248" y2="1248" x1="1056" />
        </branch>
        <branch name="VF_C(3)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1152" y="1184" type="branch" />
            <wire x2="1152" y1="1184" y2="1184" x1="1056" />
        </branch>
        <branch name="VF">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1440" y="1184" type="branch" />
            <wire x2="1552" y1="1184" y2="1184" x1="1440" />
        </branch>
        <branch name="VC">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1440" y="1248" type="branch" />
            <wire x2="1552" y1="1248" y2="1248" x1="1440" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1488" y="1376" type="branch" />
            <wire x2="1552" y1="1376" y2="1376" x1="1488" />
        </branch>
        <branch name="CK">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1488" y="1440" type="branch" />
            <wire x2="1552" y1="1440" y2="1440" x1="1488" />
        </branch>
        <branch name="Led_V(4)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1312" type="branch" />
            <wire x2="2032" y1="1312" y2="1312" x1="1936" />
        </branch>
        <branch name="V_R(4)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1248" type="branch" />
            <wire x2="2032" y1="1248" y2="1248" x1="1936" />
        </branch>
        <branch name="VF_C(4)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1184" type="branch" />
            <wire x2="2032" y1="1184" y2="1184" x1="1936" />
        </branch>
        <branch name="VF">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2304" y="1184" type="branch" />
            <wire x2="2416" y1="1184" y2="1184" x1="2304" />
        </branch>
        <branch name="VC">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2304" y="1248" type="branch" />
            <wire x2="2416" y1="1248" y2="1248" x1="2304" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2352" y="1376" type="branch" />
            <wire x2="2416" y1="1376" y2="1376" x1="2352" />
        </branch>
        <branch name="CK">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2352" y="1440" type="branch" />
            <wire x2="2416" y1="1440" y2="1440" x1="2352" />
        </branch>
        <branch name="Led_V(5)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2896" y="1312" type="branch" />
            <wire x2="2816" y1="1312" y2="1312" x1="2800" />
            <wire x2="2832" y1="1312" y2="1312" x1="2816" />
            <wire x2="2896" y1="1312" y2="1312" x1="2832" />
        </branch>
        <branch name="V_R(5)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2896" y="1248" type="branch" />
            <wire x2="2816" y1="1248" y2="1248" x1="2800" />
            <wire x2="2832" y1="1248" y2="1248" x1="2816" />
            <wire x2="2896" y1="1248" y2="1248" x1="2832" />
        </branch>
        <branch name="VF_C(5)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2896" y="1184" type="branch" />
            <wire x2="2816" y1="1184" y2="1184" x1="2800" />
            <wire x2="2832" y1="1184" y2="1184" x1="2816" />
            <wire x2="2896" y1="1184" y2="1184" x1="2832" />
        </branch>
        <branch name="V_R(6)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2768" y="2096" type="branch" />
            <wire x2="2912" y1="2096" y2="2096" x1="2768" />
        </branch>
        <branch name="V_R(5)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2784" y="2032" type="branch" />
            <wire x2="2912" y1="2032" y2="2032" x1="2784" />
        </branch>
        <branch name="V_R(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2784" y="1904" type="branch" />
            <wire x2="2912" y1="1904" y2="1904" x1="2784" />
        </branch>
        <branch name="V_R(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2784" y="1776" type="branch" />
            <wire x2="2912" y1="1776" y2="1776" x1="2784" />
        </branch>
        <branch name="V_R(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2800" y="1712" type="branch" />
            <wire x2="2912" y1="1712" y2="1712" x1="2800" />
        </branch>
        <branch name="V_R(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2784" y="1840" type="branch" />
            <wire x2="2912" y1="1840" y2="1840" x1="2784" />
        </branch>
        <branch name="V_R(4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2784" y="1968" type="branch" />
            <wire x2="2912" y1="1968" y2="1968" x1="2784" />
        </branch>
        <branch name="Ena(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="560" y="768" type="branch" />
            <wire x2="672" y1="768" y2="768" x1="560" />
        </branch>
        <branch name="Ena(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="576" y="1312" type="branch" />
            <wire x2="672" y1="1312" y2="1312" x1="576" />
        </branch>
        <branch name="Ena(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1472" y="768" type="branch" />
            <wire x2="1552" y1="768" y2="768" x1="1472" />
        </branch>
        <branch name="Ena(4)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1456" y="1312" type="branch" />
            <wire x2="1552" y1="1312" y2="1312" x1="1456" />
        </branch>
        <branch name="Ena(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2336" y="768" type="branch" />
            <wire x2="2416" y1="768" y2="768" x1="2336" />
        </branch>
        <branch name="Ena(5)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2320" y="1312" type="branch" />
            <wire x2="2416" y1="1312" y2="1312" x1="2320" />
        </branch>
        <branch name="VF_C(6)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1136" y="1664" type="branch" />
            <wire x2="1120" y1="1664" y2="1664" x1="1040" />
            <wire x2="1136" y1="1664" y2="1664" x1="1120" />
        </branch>
        <branch name="VF">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1440" y="1664" type="branch" />
            <wire x2="1536" y1="1664" y2="1664" x1="1440" />
            <wire x2="1552" y1="1664" y2="1664" x1="1536" />
        </branch>
        <branch name="VC">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1440" y="1728" type="branch" />
            <wire x2="1536" y1="1728" y2="1728" x1="1440" />
            <wire x2="1552" y1="1728" y2="1728" x1="1536" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1488" y="1856" type="branch" />
            <wire x2="1536" y1="1856" y2="1856" x1="1488" />
            <wire x2="1552" y1="1856" y2="1856" x1="1536" />
        </branch>
        <branch name="CK">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1488" y="1920" type="branch" />
            <wire x2="1536" y1="1920" y2="1920" x1="1488" />
            <wire x2="1552" y1="1920" y2="1920" x1="1536" />
        </branch>
        <branch name="VF_C(7)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1664" type="branch" />
            <wire x2="2032" y1="1664" y2="1664" x1="1936" />
        </branch>
        <instance x="2912" y="2224" name="XLXI_33" orien="R0" />
        <branch name="V_RR">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3264" y="1936" type="branch" />
            <wire x2="3264" y1="1936" y2="1936" x1="3168" />
        </branch>
        <branch name="V_R(7)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2768" y="2160" type="branch" />
            <wire x2="2912" y1="2160" y2="2160" x1="2768" />
        </branch>
        <branch name="Sel(7:0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1424" y="448" type="branch" />
            <wire x2="1536" y1="448" y2="448" x1="1424" />
        </branch>
        <branch name="Ena(7:0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="448" type="branch" />
            <wire x2="2064" y1="448" y2="448" x1="1920" />
        </branch>
        <branch name="Ena(7)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1440" y="1792" type="branch" />
            <wire x2="1536" y1="1792" y2="1792" x1="1440" />
            <wire x2="1552" y1="1792" y2="1792" x1="1536" />
        </branch>
        <instance x="1536" y="480" name="XLXI_34" orien="R0">
        </instance>
        <instance x="672" y="928" name="XLXI_40" orien="R0">
        </instance>
        <instance x="1552" y="928" name="XLXI_41" orien="R0">
        </instance>
        <instance x="2416" y="928" name="XLXI_42" orien="R0">
        </instance>
        <instance x="2416" y="1472" name="XLXI_45" orien="R0">
        </instance>
        <instance x="1552" y="1472" name="XLXI_46" orien="R0">
        </instance>
        <instance x="672" y="1472" name="XLXI_47" orien="R0">
        </instance>
        <branch name="CK">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="592" y="1920" type="branch" />
            <wire x2="608" y1="1920" y2="1920" x1="592" />
            <wire x2="656" y1="1920" y2="1920" x1="608" />
        </branch>
        <branch name="CLR">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="592" y="1856" type="branch" />
            <wire x2="608" y1="1856" y2="1856" x1="592" />
            <wire x2="656" y1="1856" y2="1856" x1="608" />
        </branch>
        <branch name="Ena(6)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="560" y="1792" type="branch" />
            <wire x2="576" y1="1792" y2="1792" x1="560" />
            <wire x2="656" y1="1792" y2="1792" x1="576" />
        </branch>
        <branch name="VC">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="544" y="1728" type="branch" />
            <wire x2="560" y1="1728" y2="1728" x1="544" />
            <wire x2="656" y1="1728" y2="1728" x1="560" />
        </branch>
        <branch name="VF">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="544" y="1664" type="branch" />
            <wire x2="560" y1="1664" y2="1664" x1="544" />
            <wire x2="656" y1="1664" y2="1664" x1="560" />
        </branch>
        <instance x="656" y="1952" name="XLXI_52" orien="R0">
        </instance>
        <branch name="Led_V(6)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1136" y="1824" type="branch" />
            <wire x2="1136" y1="1824" y2="1824" x1="1040" />
        </branch>
        <branch name="V_R(6)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1136" y="1744" type="branch" />
            <wire x2="1136" y1="1744" y2="1744" x1="1040" />
        </branch>
        <instance x="1552" y="1952" name="XLXI_54" orien="R0">
        </instance>
        <branch name="Led_V(7)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2048" y="1824" type="branch" />
            <wire x2="1952" y1="1824" y2="1824" x1="1936" />
            <wire x2="2048" y1="1824" y2="1824" x1="1952" />
        </branch>
        <branch name="V_R(7)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1744" type="branch" />
            <wire x2="2032" y1="1744" y2="1744" x1="1936" />
        </branch>
    </sheet>
</drawing>