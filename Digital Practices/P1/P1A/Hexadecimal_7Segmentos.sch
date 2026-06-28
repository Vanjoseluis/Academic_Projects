<?xml version="1.0" encoding="UTF-8"?>
<drawing version="7">
    <attr value="spartan6" name="DeviceFamilyName">
        <trait delete="all:0" />
        <trait editname="all:0" />
        <trait edittrait="all:0" />
    </attr>
    <netlist>
        <signal name="x(0)" />
        <signal name="x(3)" />
        <signal name="x(2)" />
        <signal name="x(1)" />
        <signal name="enable" />
        <signal name="XLXN_524" />
        <signal name="XLXN_525" />
        <signal name="XLXN_523" />
        <signal name="XLXN_520" />
        <signal name="XLXN_725" />
        <signal name="XLXN_571" />
        <signal name="XLXN_572" />
        <signal name="XLXN_569" />
        <signal name="XLXN_570" />
        <signal name="XLXN_746" />
        <signal name="XLXN_630" />
        <signal name="XLXN_632" />
        <signal name="XLXN_631" />
        <signal name="x(3:0)" />
        <signal name="XLXN_1399" />
        <signal name="segmentos(6)" />
        <signal name="segmentos(0)" />
        <signal name="XLXN_736" />
        <signal name="XLXN_538" />
        <signal name="XLXN_537" />
        <signal name="XLXN_540" />
        <signal name="XLXN_539" />
        <signal name="XLXN_764" />
        <signal name="XLXN_331" />
        <signal name="XLXN_332" />
        <signal name="XLXN_333" />
        <signal name="XLXN_775" />
        <signal name="XLXN_584" />
        <signal name="XLXN_583" />
        <signal name="XLXN_586" />
        <signal name="XLXN_585" />
        <signal name="XLXN_739" />
        <signal name="XLXN_557" />
        <signal name="XLXN_556" />
        <signal name="XLXN_555" />
        <signal name="segmentos(4)" />
        <signal name="segmentos(3)" />
        <signal name="segmentos(1)" />
        <signal name="segmentos(2)" />
        <signal name="segmentos(5)" />
        <signal name="segmentos(7:0)" />
        <signal name="segmentos(7)" />
        <port polarity="Input" name="enable" />
        <port polarity="Input" name="x(3:0)" />
        <port polarity="Output" name="segmentos(7:0)" />
        <blockdef name="and4b3">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="40" y1="-128" y2="-128" x1="0" />
            <circle r="12" cx="52" cy="-128" />
            <line x2="40" y1="-192" y2="-192" x1="0" />
            <circle r="12" cx="52" cy="-192" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="192" y1="-160" y2="-160" x1="256" />
            <line x2="64" y1="-64" y2="-256" x1="64" />
            <line x2="64" y1="-112" y2="-112" x1="144" />
            <arc ex="144" ey="-208" sx="144" sy="-112" r="48" cx="144" cy="-160" />
            <line x2="144" y1="-208" y2="-208" x1="64" />
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
        <blockdef name="and3b2">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="40" y1="-128" y2="-128" x1="0" />
            <circle r="12" cx="52" cy="-128" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="192" y1="-128" y2="-128" x1="256" />
            <line x2="64" y1="-64" y2="-192" x1="64" />
            <arc ex="144" ey="-176" sx="144" sy="-80" r="48" cx="144" cy="-128" />
            <line x2="64" y1="-80" y2="-80" x1="144" />
            <line x2="144" y1="-176" y2="-176" x1="64" />
        </blockdef>
        <blockdef name="and4b1">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="64" y1="-128" y2="-128" x1="0" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="192" y1="-160" y2="-160" x1="256" />
            <line x2="64" y1="-64" y2="-256" x1="64" />
            <line x2="64" y1="-112" y2="-112" x1="144" />
            <arc ex="144" ey="-208" sx="144" sy="-112" r="48" cx="144" cy="-160" />
            <line x2="144" y1="-208" y2="-208" x1="64" />
        </blockdef>
        <blockdef name="or3">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="48" y1="-64" y2="-64" x1="0" />
            <line x2="72" y1="-128" y2="-128" x1="0" />
            <line x2="48" y1="-192" y2="-192" x1="0" />
            <line x2="192" y1="-128" y2="-128" x1="256" />
            <arc ex="192" ey="-128" sx="112" sy="-80" r="88" cx="116" cy="-168" />
            <arc ex="48" ey="-176" sx="48" sy="-80" r="56" cx="16" cy="-128" />
            <line x2="48" y1="-64" y2="-80" x1="48" />
            <line x2="48" y1="-192" y2="-176" x1="48" />
            <line x2="48" y1="-80" y2="-80" x1="112" />
            <arc ex="112" ey="-176" sx="192" sy="-128" r="88" cx="116" cy="-88" />
            <line x2="48" y1="-176" y2="-176" x1="112" />
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
        <blockdef name="and3b3">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="40" y1="-128" y2="-128" x1="0" />
            <circle r="12" cx="52" cy="-128" />
            <line x2="40" y1="-192" y2="-192" x1="0" />
            <circle r="12" cx="52" cy="-192" />
            <line x2="192" y1="-128" y2="-128" x1="256" />
            <line x2="144" y1="-176" y2="-176" x1="64" />
            <line x2="64" y1="-64" y2="-192" x1="64" />
            <arc ex="144" ey="-176" sx="144" sy="-80" r="48" cx="144" cy="-128" />
            <line x2="64" y1="-80" y2="-80" x1="144" />
        </blockdef>
        <blockdef name="and4b2">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="40" y1="-64" y2="-64" x1="0" />
            <circle r="12" cx="52" cy="-64" />
            <line x2="40" y1="-128" y2="-128" x1="0" />
            <circle r="12" cx="52" cy="-128" />
            <line x2="64" y1="-192" y2="-192" x1="0" />
            <line x2="64" y1="-256" y2="-256" x1="0" />
            <line x2="192" y1="-160" y2="-160" x1="256" />
            <line x2="144" y1="-208" y2="-208" x1="64" />
            <arc ex="144" ey="-208" sx="144" sy="-112" r="48" cx="144" cy="-160" />
            <line x2="64" y1="-64" y2="-256" x1="64" />
            <line x2="64" y1="-112" y2="-112" x1="144" />
        </blockdef>
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
        <blockdef name="gnd">
            <timestamp>2000-1-1T10:10:10</timestamp>
            <line x2="64" y1="-64" y2="-96" x1="64" />
            <line x2="52" y1="-48" y2="-48" x1="76" />
            <line x2="60" y1="-32" y2="-32" x1="68" />
            <line x2="40" y1="-64" y2="-64" x1="88" />
            <line x2="64" y1="-64" y2="-80" x1="64" />
            <line x2="64" y1="-128" y2="-96" x1="64" />
        </blockdef>
        <block symbolname="and4b3" name="XLXI_1">
            <blockpin signalname="x(1)" name="I0" />
            <blockpin signalname="x(2)" name="I1" />
            <blockpin signalname="x(3)" name="I2" />
            <blockpin signalname="x(0)" name="I3" />
            <blockpin signalname="XLXN_524" name="O" />
        </block>
        <block symbolname="and4b3" name="XLXI_2">
            <blockpin signalname="x(0)" name="I0" />
            <blockpin signalname="x(1)" name="I1" />
            <blockpin signalname="x(3)" name="I2" />
            <blockpin signalname="x(2)" name="I3" />
            <blockpin signalname="XLXN_523" name="O" />
        </block>
        <block symbolname="and4b1" name="XLXI_68">
            <blockpin signalname="x(1)" name="I0" />
            <blockpin signalname="x(0)" name="I1" />
            <blockpin signalname="x(2)" name="I2" />
            <blockpin signalname="x(3)" name="I3" />
            <blockpin signalname="XLXN_520" name="O" />
        </block>
        <block symbolname="and4b1" name="XLXI_103">
            <blockpin signalname="x(2)" name="I0" />
            <blockpin signalname="x(0)" name="I1" />
            <blockpin signalname="x(1)" name="I2" />
            <blockpin signalname="x(3)" name="I3" />
            <blockpin signalname="XLXN_525" name="O" />
        </block>
        <block symbolname="or4" name="XLXI_104">
            <blockpin signalname="XLXN_525" name="I0" />
            <blockpin signalname="XLXN_520" name="I1" />
            <blockpin signalname="XLXN_523" name="I2" />
            <blockpin signalname="XLXN_524" name="I3" />
            <blockpin signalname="XLXN_725" name="O" />
        </block>
        <block symbolname="and4b3" name="XLXI_36">
            <blockpin signalname="x(0)" name="I0" />
            <blockpin signalname="x(1)" name="I1" />
            <blockpin signalname="x(3)" name="I2" />
            <blockpin signalname="x(2)" name="I3" />
            <blockpin signalname="XLXN_569" name="O" />
        </block>
        <block symbolname="and4b3" name="XLXI_35">
            <blockpin signalname="x(1)" name="I0" />
            <blockpin signalname="x(2)" name="I1" />
            <blockpin signalname="x(3)" name="I2" />
            <blockpin signalname="x(0)" name="I3" />
            <blockpin signalname="XLXN_572" name="O" />
        </block>
        <block symbolname="and4b2" name="XLXI_132">
            <blockpin signalname="x(0)" name="I0" />
            <blockpin signalname="x(2)" name="I1" />
            <blockpin signalname="x(1)" name="I2" />
            <blockpin signalname="x(3)" name="I3" />
            <blockpin signalname="XLXN_570" name="O" />
        </block>
        <block symbolname="or4" name="XLXI_134">
            <blockpin signalname="XLXN_571" name="I0" />
            <blockpin signalname="XLXN_570" name="I1" />
            <blockpin signalname="XLXN_569" name="I2" />
            <blockpin signalname="XLXN_572" name="I3" />
            <blockpin signalname="XLXN_746" name="O" />
        </block>
        <block symbolname="and3" name="XLXI_133">
            <blockpin signalname="x(0)" name="I0" />
            <blockpin signalname="x(1)" name="I1" />
            <blockpin signalname="x(2)" name="I2" />
            <blockpin signalname="XLXN_571" name="O" />
        </block>
        <block symbolname="and3b3" name="XLXI_47">
            <blockpin signalname="x(1)" name="I0" />
            <blockpin signalname="x(2)" name="I1" />
            <blockpin signalname="x(3)" name="I2" />
            <blockpin signalname="XLXN_631" name="O" />
        </block>
        <block symbolname="and4b1" name="XLXI_46">
            <blockpin signalname="x(3)" name="I0" />
            <blockpin signalname="x(0)" name="I1" />
            <blockpin signalname="x(1)" name="I2" />
            <blockpin signalname="x(2)" name="I3" />
            <blockpin signalname="XLXN_630" name="O" />
        </block>
        <block symbolname="and4b2" name="XLXI_141">
            <blockpin signalname="x(0)" name="I0" />
            <blockpin signalname="x(1)" name="I1" />
            <blockpin signalname="x(2)" name="I2" />
            <blockpin signalname="x(3)" name="I3" />
            <blockpin signalname="XLXN_632" name="O" />
        </block>
        <block symbolname="or3" name="XLXI_155">
            <blockpin signalname="XLXN_632" name="I0" />
            <blockpin signalname="XLXN_630" name="I1" />
            <blockpin signalname="XLXN_631" name="I2" />
            <blockpin signalname="XLXN_1399" name="O" />
        </block>
        <block symbolname="or4" name="XLXI_11">
            <blockpin signalname="XLXN_539" name="I0" />
            <blockpin signalname="XLXN_538" name="I1" />
            <blockpin signalname="XLXN_537" name="I2" />
            <blockpin signalname="XLXN_540" name="I3" />
            <blockpin signalname="XLXN_736" name="O" />
        </block>
        <block symbolname="and3" name="XLXI_110">
            <blockpin signalname="x(0)" name="I0" />
            <blockpin signalname="x(1)" name="I1" />
            <blockpin signalname="x(3)" name="I2" />
            <blockpin signalname="XLXN_539" name="O" />
        </block>
        <block symbolname="and3b1" name="XLXI_109">
            <blockpin signalname="x(0)" name="I0" />
            <blockpin signalname="x(2)" name="I1" />
            <blockpin signalname="x(3)" name="I2" />
            <blockpin signalname="XLXN_538" name="O" />
        </block>
        <block symbolname="and3b1" name="XLXI_108">
            <blockpin signalname="x(0)" name="I0" />
            <blockpin signalname="x(1)" name="I1" />
            <blockpin signalname="x(2)" name="I2" />
            <blockpin signalname="XLXN_537" name="O" />
        </block>
        <block symbolname="and4b2" name="XLXI_107">
            <blockpin signalname="x(1)" name="I0" />
            <blockpin signalname="x(3)" name="I1" />
            <blockpin signalname="x(0)" name="I2" />
            <blockpin signalname="x(2)" name="I3" />
            <blockpin signalname="XLXN_540" name="O" />
        </block>
        <block symbolname="and3b2" name="XLXI_38">
            <blockpin signalname="x(1)" name="I0" />
            <blockpin signalname="x(2)" name="I1" />
            <blockpin signalname="x(0)" name="I2" />
            <blockpin signalname="XLXN_333" name="O" />
        </block>
        <block symbolname="or3" name="XLXI_34">
            <blockpin signalname="XLXN_333" name="I0" />
            <blockpin signalname="XLXN_331" name="I1" />
            <blockpin signalname="XLXN_332" name="I2" />
            <blockpin signalname="XLXN_764" name="O" />
        </block>
        <block symbolname="and3b2" name="XLXI_37">
            <blockpin signalname="x(1)" name="I0" />
            <blockpin signalname="x(3)" name="I1" />
            <blockpin signalname="x(2)" name="I2" />
            <blockpin signalname="XLXN_331" name="O" />
        </block>
        <block symbolname="and2b1" name="XLXI_39">
            <blockpin signalname="x(3)" name="I0" />
            <blockpin signalname="x(0)" name="I1" />
            <blockpin signalname="XLXN_332" name="O" />
        </block>
        <block symbolname="and4b1" name="XLXI_136">
            <blockpin signalname="x(1)" name="I0" />
            <blockpin signalname="x(0)" name="I1" />
            <blockpin signalname="x(2)" name="I2" />
            <blockpin signalname="x(3)" name="I3" />
            <blockpin signalname="XLXN_586" name="O" />
        </block>
        <block symbolname="or4" name="XLXI_137">
            <blockpin signalname="XLXN_586" name="I0" />
            <blockpin signalname="XLXN_584" name="I1" />
            <blockpin signalname="XLXN_583" name="I2" />
            <blockpin signalname="XLXN_585" name="I3" />
            <blockpin signalname="XLXN_775" name="O" />
        </block>
        <block symbolname="and3b2" name="XLXI_40">
            <blockpin signalname="x(2)" name="I0" />
            <blockpin signalname="x(3)" name="I1" />
            <blockpin signalname="x(0)" name="I2" />
            <blockpin signalname="XLXN_585" name="O" />
        </block>
        <block symbolname="and3b2" name="XLXI_41">
            <blockpin signalname="x(2)" name="I0" />
            <blockpin signalname="x(3)" name="I1" />
            <blockpin signalname="x(1)" name="I2" />
            <blockpin signalname="XLXN_583" name="O" />
        </block>
        <block symbolname="and3b1" name="XLXI_44">
            <blockpin signalname="x(3)" name="I0" />
            <blockpin signalname="x(0)" name="I1" />
            <blockpin signalname="x(1)" name="I2" />
            <blockpin signalname="XLXN_584" name="O" />
        </block>
        <block symbolname="or3" name="XLXI_32">
            <blockpin signalname="XLXN_557" name="I0" />
            <blockpin signalname="XLXN_555" name="I1" />
            <blockpin signalname="XLXN_556" name="I2" />
            <blockpin signalname="XLXN_739" name="O" />
        </block>
        <block symbolname="and3" name="XLXI_114">
            <blockpin signalname="x(1)" name="I0" />
            <blockpin signalname="x(2)" name="I1" />
            <blockpin signalname="x(3)" name="I2" />
            <blockpin signalname="XLXN_557" name="O" />
        </block>
        <block symbolname="and4b3" name="XLXI_115">
            <blockpin signalname="x(0)" name="I0" />
            <blockpin signalname="x(2)" name="I1" />
            <blockpin signalname="x(3)" name="I2" />
            <blockpin signalname="x(1)" name="I3" />
            <blockpin signalname="XLXN_556" name="O" />
        </block>
        <block symbolname="and3b1" name="XLXI_113">
            <blockpin signalname="x(0)" name="I0" />
            <blockpin signalname="x(2)" name="I1" />
            <blockpin signalname="x(3)" name="I2" />
            <blockpin signalname="XLXN_555" name="O" />
        </block>
        <block symbolname="and2b1" name="XLXI_245">
            <blockpin signalname="XLXN_725" name="I0" />
            <blockpin signalname="enable" name="I1" />
            <blockpin signalname="segmentos(0)" name="O" />
        </block>
        <block symbolname="and2b1" name="XLXI_481">
            <blockpin signalname="XLXN_736" name="I0" />
            <blockpin signalname="enable" name="I1" />
            <blockpin signalname="segmentos(1)" name="O" />
        </block>
        <block symbolname="and2b1" name="XLXI_482">
            <blockpin signalname="XLXN_739" name="I0" />
            <blockpin signalname="enable" name="I1" />
            <blockpin signalname="segmentos(2)" name="O" />
        </block>
        <block symbolname="and2b1" name="XLXI_483">
            <blockpin signalname="XLXN_775" name="I0" />
            <blockpin signalname="enable" name="I1" />
            <blockpin signalname="segmentos(5)" name="O" />
        </block>
        <block symbolname="and2b1" name="XLXI_484">
            <blockpin signalname="XLXN_764" name="I0" />
            <blockpin signalname="enable" name="I1" />
            <blockpin signalname="segmentos(4)" name="O" />
        </block>
        <block symbolname="and2b1" name="XLXI_485">
            <blockpin signalname="XLXN_746" name="I0" />
            <blockpin signalname="enable" name="I1" />
            <blockpin signalname="segmentos(3)" name="O" />
        </block>
        <block symbolname="and2b1" name="XLXI_486">
            <blockpin signalname="XLXN_1399" name="I0" />
            <blockpin signalname="enable" name="I1" />
            <blockpin signalname="segmentos(6)" name="O" />
        </block>
        <block symbolname="gnd" name="XLXI_487">
            <blockpin signalname="segmentos(7)" name="G" />
        </block>
    </netlist>
    <sheet sheetnum="1" width="5440" height="3520">
        <attr value="Inch" name="LengthUnitName" />
        <attr value="10" name="GridsPerUnit" />
        <instance x="592" y="448" name="XLXI_1" orien="R0" />
        <instance x="592" y="720" name="XLXI_2" orien="R0" />
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="464" type="branch" />
            <wire x2="592" y1="464" y2="464" x1="528" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="528" type="branch" />
            <wire x2="592" y1="528" y2="528" x1="528" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="592" type="branch" />
            <wire x2="592" y1="592" y2="592" x1="528" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="656" type="branch" />
            <wire x2="592" y1="656" y2="656" x1="528" />
        </branch>
        <instance x="592" y="976" name="XLXI_68" orien="R0" />
        <instance x="608" y="1264" name="XLXI_103" orien="R0" />
        <branch name="XLXN_524">
            <wire x2="896" y1="288" y2="288" x1="848" />
            <wire x2="896" y1="288" y2="592" x1="896" />
        </branch>
        <branch name="XLXN_525">
            <wire x2="896" y1="1104" y2="1104" x1="864" />
            <wire x2="896" y1="784" y2="1104" x1="896" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="720" type="branch" />
            <wire x2="592" y1="720" y2="720" x1="528" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="784" type="branch" />
            <wire x2="592" y1="784" y2="784" x1="528" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="848" type="branch" />
            <wire x2="592" y1="848" y2="848" x1="528" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="912" type="branch" />
            <wire x2="592" y1="912" y2="912" x1="528" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1008" type="branch" />
            <wire x2="608" y1="1008" y2="1008" x1="528" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1072" type="branch" />
            <wire x2="608" y1="1072" y2="1072" x1="528" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1136" type="branch" />
            <wire x2="608" y1="1136" y2="1136" x1="528" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1200" type="branch" />
            <wire x2="608" y1="1200" y2="1200" x1="528" />
        </branch>
        <instance x="896" y="848" name="XLXI_104" orien="R0" />
        <branch name="XLXN_523">
            <wire x2="864" y1="560" y2="560" x1="848" />
            <wire x2="864" y1="560" y2="656" x1="864" />
            <wire x2="896" y1="656" y2="656" x1="864" />
        </branch>
        <branch name="XLXN_520">
            <wire x2="864" y1="816" y2="816" x1="848" />
            <wire x2="896" y1="720" y2="720" x1="864" />
            <wire x2="864" y1="720" y2="816" x1="864" />
        </branch>
        <branch name="XLXN_725">
            <wire x2="1232" y1="688" y2="688" x1="1152" />
        </branch>
        <instance x="592" y="1952" name="XLXI_36" orien="R0" />
        <instance x="592" y="1680" name="XLXI_35" orien="R0" />
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1888" type="branch" />
            <wire x2="592" y1="1888" y2="1888" x1="528" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1824" type="branch" />
            <wire x2="592" y1="1824" y2="1824" x1="528" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1760" type="branch" />
            <wire x2="592" y1="1760" y2="1760" x1="528" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1696" type="branch" />
            <wire x2="592" y1="1696" y2="1696" x1="528" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1616" type="branch" />
            <wire x2="592" y1="1616" y2="1616" x1="528" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1552" type="branch" />
            <wire x2="592" y1="1552" y2="1552" x1="528" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1488" type="branch" />
            <wire x2="592" y1="1488" y2="1488" x1="528" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1424" type="branch" />
            <wire x2="592" y1="1424" y2="1424" x1="528" />
        </branch>
        <instance x="592" y="2208" name="XLXI_132" orien="R0" />
        <branch name="XLXN_571">
            <wire x2="896" y1="2272" y2="2272" x1="848" />
            <wire x2="896" y1="2016" y2="2272" x1="896" />
        </branch>
        <branch name="XLXN_572">
            <wire x2="896" y1="1520" y2="1520" x1="848" />
            <wire x2="896" y1="1520" y2="1824" x1="896" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="1952" type="branch" />
            <wire x2="592" y1="1952" y2="1952" x1="528" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="2016" type="branch" />
            <wire x2="592" y1="2016" y2="2016" x1="528" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="2080" type="branch" />
            <wire x2="592" y1="2080" y2="2080" x1="528" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="2144" type="branch" />
            <wire x2="592" y1="2144" y2="2144" x1="528" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="2272" type="branch" />
            <wire x2="592" y1="2272" y2="2272" x1="528" />
        </branch>
        <branch name="XLXN_569">
            <wire x2="864" y1="1792" y2="1792" x1="848" />
            <wire x2="864" y1="1792" y2="1888" x1="864" />
            <wire x2="896" y1="1888" y2="1888" x1="864" />
        </branch>
        <branch name="XLXN_570">
            <wire x2="864" y1="2048" y2="2048" x1="848" />
            <wire x2="864" y1="1952" y2="2048" x1="864" />
            <wire x2="896" y1="1952" y2="1952" x1="864" />
        </branch>
        <branch name="XLXN_746">
            <wire x2="1200" y1="1920" y2="1920" x1="1152" />
        </branch>
        <instance x="896" y="2080" name="XLXI_134" orien="R0" />
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="2336" type="branch" />
            <wire x2="592" y1="2336" y2="2336" x1="528" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="528" y="2208" type="branch" />
            <wire x2="592" y1="2208" y2="2208" x1="528" />
        </branch>
        <instance x="592" y="2400" name="XLXI_133" orien="R0" />
        <instance x="592" y="2800" name="XLXI_47" orien="R0" />
        <instance x="592" y="3088" name="XLXI_46" orien="R0" />
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="2672" type="branch" />
            <wire x2="592" y1="2672" y2="2672" x1="496" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="2736" type="branch" />
            <wire x2="592" y1="2736" y2="2736" x1="496" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="2608" type="branch" />
            <wire x2="592" y1="2608" y2="2608" x1="496" />
        </branch>
        <instance x="592" y="3360" name="XLXI_141" orien="R0" />
        <branch name="XLXN_630">
            <wire x2="896" y1="2928" y2="2928" x1="848" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="2832" type="branch" />
            <wire x2="592" y1="2832" y2="2832" x1="496" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="2960" type="branch" />
            <wire x2="592" y1="2960" y2="2960" x1="496" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="3024" type="branch" />
            <wire x2="592" y1="3024" y2="3024" x1="496" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="2896" type="branch" />
            <wire x2="592" y1="2896" y2="2896" x1="496" />
        </branch>
        <branch name="XLXN_632">
            <wire x2="896" y1="3200" y2="3200" x1="848" />
            <wire x2="896" y1="2992" y2="3200" x1="896" />
        </branch>
        <branch name="XLXN_631">
            <wire x2="896" y1="2672" y2="2672" x1="848" />
            <wire x2="896" y1="2672" y2="2864" x1="896" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="3104" type="branch" />
            <wire x2="592" y1="3104" y2="3104" x1="496" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="3168" type="branch" />
            <wire x2="592" y1="3168" y2="3168" x1="496" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="3232" type="branch" />
            <wire x2="592" y1="3232" y2="3232" x1="496" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="496" y="3296" type="branch" />
            <wire x2="592" y1="3296" y2="3296" x1="496" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="496" y="256" type="branch" />
            <wire x2="496" y1="256" y2="256" x1="400" />
            <wire x2="592" y1="256" y2="256" x1="496" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="496" y="320" type="branch" />
            <wire x2="496" y1="320" y2="320" x1="400" />
            <wire x2="592" y1="320" y2="320" x1="496" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="496" y="384" type="branch" />
            <wire x2="496" y1="384" y2="384" x1="400" />
            <wire x2="592" y1="384" y2="384" x1="496" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-BCENTER;fontsize:28;fontname:Arial" attrname="Name" x="496" y="192" type="branch" />
            <wire x2="496" y1="192" y2="192" x1="400" />
            <wire x2="592" y1="192" y2="192" x1="496" />
        </branch>
        <branch name="x(3:0)">
            <wire x2="304" y1="128" y2="128" x1="208" />
            <wire x2="304" y1="128" y2="176" x1="304" />
            <wire x2="304" y1="176" y2="192" x1="304" />
            <wire x2="304" y1="192" y2="256" x1="304" />
            <wire x2="304" y1="256" y2="320" x1="304" />
            <wire x2="304" y1="320" y2="384" x1="304" />
            <wire x2="304" y1="384" y2="416" x1="304" />
        </branch>
        <bustap x2="400" y1="192" y2="192" x1="304" />
        <bustap x2="400" y1="256" y2="256" x1="304" />
        <bustap x2="400" y1="320" y2="320" x1="304" />
        <bustap x2="400" y1="384" y2="384" x1="304" />
        <instance x="896" y="3056" name="XLXI_155" orien="R0" />
        <branch name="enable">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1152" y="3120" type="branch" />
            <wire x2="1168" y1="3120" y2="3120" x1="1152" />
            <wire x2="1248" y1="2992" y2="2992" x1="1168" />
            <wire x2="1168" y1="2992" y2="3120" x1="1168" />
        </branch>
        <branch name="XLXN_1399">
            <wire x2="1248" y1="2928" y2="2928" x1="1152" />
        </branch>
        <branch name="enable">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="1152" y="2080" type="branch" />
            <wire x2="1168" y1="2080" y2="2080" x1="1152" />
            <wire x2="1168" y1="1984" y2="2080" x1="1168" />
            <wire x2="1200" y1="1984" y2="1984" x1="1168" />
        </branch>
        <branch name="enable">
            <wire x2="1184" y1="864" y2="864" x1="1152" />
            <wire x2="1184" y1="752" y2="864" x1="1184" />
            <wire x2="1232" y1="752" y2="752" x1="1184" />
        </branch>
        <iomarker fontsize="28" x="1152" y="864" name="enable" orien="R180" />
        <branch name="segmentos(6)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1552" y="2960" type="branch" />
            <wire x2="1552" y1="2960" y2="2960" x1="1504" />
        </branch>
        <branch name="segmentos(0)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1520" y="720" type="branch" />
            <wire x2="1520" y1="720" y2="720" x1="1488" />
        </branch>
        <instance x="2416" y="864" name="XLXI_11" orien="R0" />
        <instance x="2112" y="1152" name="XLXI_110" orien="R0" />
        <instance x="2112" y="944" name="XLXI_109" orien="R0" />
        <instance x="2112" y="736" name="XLXI_108" orien="R0" />
        <instance x="2112" y="528" name="XLXI_107" orien="R0" />
        <branch name="enable">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2624" y="880" type="branch" />
            <wire x2="2688" y1="880" y2="880" x1="2624" />
            <wire x2="2736" y1="768" y2="768" x1="2688" />
            <wire x2="2688" y1="768" y2="880" x1="2688" />
        </branch>
        <branch name="XLXN_736">
            <wire x2="2736" y1="704" y2="704" x1="2672" />
        </branch>
        <branch name="XLXN_538">
            <wire x2="2384" y1="816" y2="816" x1="2368" />
            <wire x2="2416" y1="736" y2="736" x1="2384" />
            <wire x2="2384" y1="736" y2="816" x1="2384" />
        </branch>
        <branch name="XLXN_537">
            <wire x2="2384" y1="608" y2="608" x1="2368" />
            <wire x2="2384" y1="608" y2="672" x1="2384" />
            <wire x2="2416" y1="672" y2="672" x1="2384" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="1088" type="branch" />
            <wire x2="2112" y1="1088" y2="1088" x1="2064" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="1024" type="branch" />
            <wire x2="2112" y1="1024" y2="1024" x1="2064" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="960" type="branch" />
            <wire x2="2112" y1="960" y2="960" x1="2064" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="880" type="branch" />
            <wire x2="2112" y1="880" y2="880" x1="2064" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="816" type="branch" />
            <wire x2="2112" y1="816" y2="816" x1="2064" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="752" type="branch" />
            <wire x2="2112" y1="752" y2="752" x1="2064" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="672" type="branch" />
            <wire x2="2112" y1="672" y2="672" x1="2064" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="608" type="branch" />
            <wire x2="2112" y1="608" y2="608" x1="2064" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="544" type="branch" />
            <wire x2="2112" y1="544" y2="544" x1="2064" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="464" type="branch" />
            <wire x2="2112" y1="464" y2="464" x1="2064" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="400" type="branch" />
            <wire x2="2112" y1="400" y2="400" x1="2064" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="336" type="branch" />
            <wire x2="2112" y1="336" y2="336" x1="2064" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2064" y="272" type="branch" />
            <wire x2="2112" y1="272" y2="272" x1="2064" />
        </branch>
        <branch name="XLXN_540">
            <wire x2="2416" y1="368" y2="368" x1="2368" />
            <wire x2="2416" y1="368" y2="608" x1="2416" />
        </branch>
        <branch name="XLXN_539">
            <wire x2="2416" y1="1024" y2="1024" x1="2368" />
            <wire x2="2416" y1="800" y2="1024" x1="2416" />
        </branch>
        <instance x="2112" y="2256" name="XLXI_38" orien="R0" />
        <instance x="2400" y="2032" name="XLXI_34" orien="R0" />
        <instance x="2112" y="2032" name="XLXI_37" orien="R0" />
        <instance x="2112" y="1824" name="XLXI_39" orien="R0" />
        <branch name="enable">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2656" y="2080" type="branch" />
            <wire x2="2688" y1="2080" y2="2080" x1="2656" />
            <wire x2="2688" y1="1968" y2="2080" x1="2688" />
            <wire x2="2720" y1="1968" y2="1968" x1="2688" />
        </branch>
        <branch name="XLXN_764">
            <wire x2="2720" y1="1904" y2="1904" x1="2656" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="2192" type="branch" />
            <wire x2="2112" y1="2192" y2="2192" x1="2032" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="2128" type="branch" />
            <wire x2="2112" y1="2128" y2="2128" x1="2032" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="2064" type="branch" />
            <wire x2="2112" y1="2064" y2="2064" x1="2032" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1968" type="branch" />
            <wire x2="2112" y1="1968" y2="1968" x1="2032" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1904" type="branch" />
            <wire x2="2112" y1="1904" y2="1904" x1="2032" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1840" type="branch" />
            <wire x2="2112" y1="1840" y2="1840" x1="2032" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1696" type="branch" />
            <wire x2="2112" y1="1696" y2="1696" x1="2032" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="2032" y="1760" type="branch" />
            <wire x2="2112" y1="1760" y2="1760" x1="2032" />
        </branch>
        <branch name="XLXN_331">
            <wire x2="2400" y1="1904" y2="1904" x1="2368" />
        </branch>
        <branch name="XLXN_332">
            <wire x2="2400" y1="1728" y2="1728" x1="2368" />
            <wire x2="2400" y1="1728" y2="1840" x1="2400" />
        </branch>
        <branch name="XLXN_333">
            <wire x2="2400" y1="2128" y2="2128" x1="2368" />
            <wire x2="2400" y1="1968" y2="2128" x1="2400" />
        </branch>
        <instance x="3520" y="2416" name="XLXI_136" orien="R0" />
        <instance x="3824" y="2064" name="XLXI_137" orien="R0" />
        <instance x="3520" y="1696" name="XLXI_40" orien="R0" />
        <instance x="3520" y="1920" name="XLXI_41" orien="R0" />
        <instance x="3520" y="2144" name="XLXI_44" orien="R0" />
        <branch name="enable">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="4080" y="2096" type="branch" />
            <wire x2="4096" y1="2096" y2="2096" x1="4080" />
            <wire x2="4160" y1="1968" y2="1968" x1="4096" />
            <wire x2="4096" y1="1968" y2="2096" x1="4096" />
        </branch>
        <branch name="XLXN_775">
            <wire x2="4160" y1="1904" y2="1904" x1="4080" />
        </branch>
        <branch name="XLXN_584">
            <wire x2="3792" y1="2016" y2="2016" x1="3776" />
            <wire x2="3824" y1="1936" y2="1936" x1="3792" />
            <wire x2="3792" y1="1936" y2="2016" x1="3792" />
        </branch>
        <branch name="XLXN_583">
            <wire x2="3792" y1="1792" y2="1792" x1="3776" />
            <wire x2="3792" y1="1792" y2="1872" x1="3792" />
            <wire x2="3824" y1="1872" y2="1872" x1="3792" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="2352" type="branch" />
            <wire x2="3520" y1="2352" y2="2352" x1="3456" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="2288" type="branch" />
            <wire x2="3520" y1="2288" y2="2288" x1="3456" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="2224" type="branch" />
            <wire x2="3520" y1="2224" y2="2224" x1="3456" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="2160" type="branch" />
            <wire x2="3520" y1="2160" y2="2160" x1="3456" />
        </branch>
        <branch name="XLXN_586">
            <wire x2="3824" y1="2256" y2="2256" x1="3776" />
            <wire x2="3824" y1="2000" y2="2256" x1="3824" />
        </branch>
        <branch name="XLXN_585">
            <wire x2="3824" y1="1568" y2="1568" x1="3776" />
            <wire x2="3824" y1="1568" y2="1808" x1="3824" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="1504" type="branch" />
            <wire x2="3520" y1="1504" y2="1504" x1="3456" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="1568" type="branch" />
            <wire x2="3520" y1="1568" y2="1568" x1="3456" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="1632" type="branch" />
            <wire x2="3520" y1="1632" y2="1632" x1="3456" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="1728" type="branch" />
            <wire x2="3520" y1="1728" y2="1728" x1="3456" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="1792" type="branch" />
            <wire x2="3520" y1="1792" y2="1792" x1="3456" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="1856" type="branch" />
            <wire x2="3520" y1="1856" y2="1856" x1="3456" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="1952" type="branch" />
            <wire x2="3520" y1="1952" y2="1952" x1="3456" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="2016" type="branch" />
            <wire x2="3520" y1="2016" y2="2016" x1="3456" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3456" y="2080" type="branch" />
            <wire x2="3520" y1="2080" y2="2080" x1="3456" />
        </branch>
        <instance x="3792" y="832" name="XLXI_32" orien="R0" />
        <instance x="3520" y="1056" name="XLXI_114" orien="R0" />
        <instance x="3520" y="608" name="XLXI_115" orien="R0" />
        <instance x="3520" y="832" name="XLXI_113" orien="R0" />
        <branch name="enable">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="4000" y="912" type="branch" />
            <wire x2="4048" y1="912" y2="912" x1="4000" />
            <wire x2="4096" y1="768" y2="768" x1="4048" />
            <wire x2="4048" y1="768" y2="912" x1="4048" />
        </branch>
        <branch name="XLXN_739">
            <wire x2="4096" y1="704" y2="704" x1="4048" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3472" y="992" type="branch" />
            <wire x2="3520" y1="992" y2="992" x1="3472" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3472" y="928" type="branch" />
            <wire x2="3520" y1="928" y2="928" x1="3472" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3472" y="864" type="branch" />
            <wire x2="3520" y1="864" y2="864" x1="3472" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3472" y="768" type="branch" />
            <wire x2="3520" y1="768" y2="768" x1="3472" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3472" y="704" type="branch" />
            <wire x2="3520" y1="704" y2="704" x1="3472" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3472" y="640" type="branch" />
            <wire x2="3520" y1="640" y2="640" x1="3472" />
        </branch>
        <branch name="x(0)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3472" y="544" type="branch" />
            <wire x2="3520" y1="544" y2="544" x1="3472" />
        </branch>
        <branch name="x(2)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3472" y="480" type="branch" />
            <wire x2="3520" y1="480" y2="480" x1="3472" />
        </branch>
        <branch name="x(3)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3472" y="416" type="branch" />
            <wire x2="3520" y1="416" y2="416" x1="3472" />
        </branch>
        <branch name="x(1)">
            <attrtext style="alignment:SOFT-RIGHT;fontsize:28;fontname:Arial" attrname="Name" x="3472" y="352" type="branch" />
            <wire x2="3520" y1="352" y2="352" x1="3472" />
        </branch>
        <branch name="XLXN_557">
            <wire x2="3792" y1="928" y2="928" x1="3776" />
            <wire x2="3792" y1="768" y2="928" x1="3792" />
        </branch>
        <branch name="XLXN_556">
            <wire x2="3792" y1="448" y2="448" x1="3776" />
            <wire x2="3792" y1="448" y2="640" x1="3792" />
        </branch>
        <branch name="XLXN_555">
            <wire x2="3792" y1="704" y2="704" x1="3776" />
        </branch>
        <branch name="segmentos(4)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3024" y="1936" type="branch" />
            <wire x2="3024" y1="1936" y2="1936" x1="2976" />
        </branch>
        <iomarker fontsize="28" x="208" y="128" name="x(3:0)" orien="R180" />
        <branch name="segmentos(3)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="1504" y="1952" type="branch" />
            <wire x2="1504" y1="1952" y2="1952" x1="1456" />
        </branch>
        <branch name="segmentos(1)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="3024" y="736" type="branch" />
            <wire x2="3024" y1="736" y2="736" x1="2992" />
        </branch>
        <branch name="segmentos(2)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="4400" y="736" type="branch" />
            <wire x2="4400" y1="736" y2="736" x1="4352" />
        </branch>
        <branch name="segmentos(5)">
            <attrtext style="alignment:SOFT-LEFT;fontsize:28;fontname:Arial" attrname="Name" x="4448" y="1936" type="branch" />
            <wire x2="4448" y1="1936" y2="1936" x1="4416" />
        </branch>
        <iomarker fontsize="28" x="2576" y="2960" name="segmentos(7:0)" orien="R0" />
        <branch name="segmentos(7:0)">
            <wire x2="2400" y1="2960" y2="2960" x1="2224" />
            <wire x2="2576" y1="2960" y2="2960" x1="2400" />
        </branch>
        <instance x="1232" y="624" name="XLXI_245" orien="M180" />
        <instance x="2736" y="640" name="XLXI_481" orien="M180" />
        <instance x="4096" y="640" name="XLXI_482" orien="M180" />
        <instance x="4160" y="1840" name="XLXI_483" orien="M180" />
        <instance x="2720" y="1840" name="XLXI_484" orien="M180" />
        <instance x="1200" y="1856" name="XLXI_485" orien="M180" />
        <instance x="1248" y="2864" name="XLXI_486" orien="M180" />
        <instance x="2336" y="3392" name="XLXI_487" orien="R0" />
        <bustap x2="2400" y1="2960" y2="3056" x1="2400" />
        <branch name="segmentos(7)">
            <attrtext style="alignment:SOFT-TVCENTER;fontsize:28;fontname:Arial" attrname="Name" x="2400" y="3160" type="branch" />
            <wire x2="2400" y1="3056" y2="3160" x1="2400" />
            <wire x2="2400" y1="3160" y2="3264" x1="2400" />
        </branch>
    </sheet>
</drawing>