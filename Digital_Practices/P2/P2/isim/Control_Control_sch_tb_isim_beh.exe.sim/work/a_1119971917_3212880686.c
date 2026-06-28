/**********************************************************************/
/*   ____  ____                                                       */
/*  /   /\/   /                                                       */
/* /___/  \  /                                                        */
/* \   \   \/                                                       */
/*  \   \        Copyright (c) 2003-2009 Xilinx, Inc.                */
/*  /   /          All Right Reserved.                                 */
/* /---/   /\                                                         */
/* \   \  /  \                                                      */
/*  \___\/\___\                                                    */
/***********************************************************************/

/* This file is designed for use with ISim build 0x7708f090 */

#define XSI_HIDE_SYMBOL_SPEC true
#include "xsi.h"
#include <memory.h>
#ifdef __GNUC__
#include <stdlib.h>
#else
#include <malloc.h>
#define alloca _alloca
#endif
static const char *ng0 = "C:/Users/edie/Desktop/Practica2_JoseLuis_Raul_26_05_26/Control_TEST2.vhd";



static void work_a_1119971917_3212880686_p_0(char *t0)
{
    char *t1;
    char *t2;
    char *t3;
    char *t4;
    char *t5;
    char *t6;
    int t7;
    int t8;
    char *t9;
    char *t10;
    int64 t11;
    int t12;

LAB0:    t1 = (t0 + 3112U);
    t2 = *((char **)t1);
    if (t2 == 0)
        goto LAB2;

LAB3:    goto *t2;

LAB2:    xsi_set_current_line(55, ng0);
    t2 = (t0 + 3496);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(56, ng0);
    t2 = (t0 + 3560);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(57, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(59, ng0);
    t2 = (t0 + 6052);
    *((int *)t2) = 0;
    t3 = (t0 + 6056);
    *((int *)t3) = 18;
    t7 = 0;
    t8 = 18;

LAB4:    if (t7 <= t8)
        goto LAB5;

LAB7:    xsi_set_current_line(115, ng0);
    t2 = (t0 + 3688);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(116, ng0);
    t2 = (t0 + 2128U);
    t3 = *((char **)t2);
    t11 = *((int64 *)t3);
    t2 = (t0 + 2920);
    xsi_process_wait(t2, t11);

LAB58:    *((char **)t1) = &&LAB59;

LAB1:    return;
LAB5:    xsi_set_current_line(60, ng0);
    t4 = (t0 + 3688);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    t9 = (t6 + 56U);
    t10 = *((char **)t9);
    *((unsigned char *)t10) = (unsigned char)3;
    xsi_driver_first_trans_fast(t4);
    xsi_set_current_line(61, ng0);
    t2 = (t0 + 2128U);
    t3 = *((char **)t2);
    t11 = *((int64 *)t3);
    t2 = (t0 + 2920);
    xsi_process_wait(t2, t11);

LAB10:    *((char **)t1) = &&LAB11;
    goto LAB1;

LAB6:    t2 = (t0 + 6052);
    t7 = *((int *)t2);
    t3 = (t0 + 6056);
    t8 = *((int *)t3);
    if (t7 == t8)
        goto LAB7;

LAB55:    t12 = (t7 + 1);
    t7 = t12;
    t4 = (t0 + 6052);
    *((int *)t4) = t7;
    goto LAB4;

LAB8:    xsi_set_current_line(63, ng0);
    t2 = (t0 + 6052);
    if (*((int *)t2) == 1)
        goto LAB13;

LAB32:    if (*((int *)t2) == 2)
        goto LAB14;

LAB33:    if (*((int *)t2) == 3)
        goto LAB15;

LAB34:    if (*((int *)t2) == 4)
        goto LAB16;

LAB35:    if (*((int *)t2) == 5)
        goto LAB17;

LAB36:    if (*((int *)t2) == 6)
        goto LAB18;

LAB37:    if (*((int *)t2) == 7)
        goto LAB19;

LAB38:    if (*((int *)t2) == 8)
        goto LAB20;

LAB39:    if (*((int *)t2) == 9)
        goto LAB21;

LAB40:    if (*((int *)t2) == 10)
        goto LAB22;

LAB41:    if (*((int *)t2) == 11)
        goto LAB23;

LAB42:    if (*((int *)t2) == 12)
        goto LAB24;

LAB43:    if (*((int *)t2) == 13)
        goto LAB25;

LAB44:    if (*((int *)t2) == 14)
        goto LAB26;

LAB45:    if (*((int *)t2) == 15)
        goto LAB27;

LAB46:    if (*((int *)t2) == 16)
        goto LAB28;

LAB47:    if (*((int *)t2) == 17)
        goto LAB29;

LAB48:    if (*((int *)t2) == 18)
        goto LAB30;

LAB49:
LAB31:    xsi_set_current_line(108, ng0);

LAB12:    xsi_set_current_line(111, ng0);
    t2 = (t0 + 3688);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(112, ng0);
    t2 = (t0 + 2128U);
    t3 = *((char **)t2);
    t11 = *((int64 *)t3);
    t2 = (t0 + 2920);
    xsi_process_wait(t2, t11);

LAB53:    *((char **)t1) = &&LAB54;
    goto LAB1;

LAB9:    goto LAB8;

LAB11:    goto LAB9;

LAB13:    xsi_set_current_line(66, ng0);
    t3 = (t0 + 3496);
    t4 = (t3 + 56U);
    t5 = *((char **)t4);
    t6 = (t5 + 56U);
    t9 = *((char **)t6);
    *((unsigned char *)t9) = (unsigned char)3;
    xsi_driver_first_trans_fast(t3);
    goto LAB12;

LAB14:    xsi_set_current_line(68, ng0);
    t2 = (t0 + 3496);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB15:    xsi_set_current_line(72, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB16:    xsi_set_current_line(74, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB17:    xsi_set_current_line(76, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB18:    xsi_set_current_line(78, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB19:    xsi_set_current_line(80, ng0);
    t2 = (t0 + 3560);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB20:    xsi_set_current_line(82, ng0);
    t2 = (t0 + 3560);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB21:    xsi_set_current_line(86, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB22:    xsi_set_current_line(88, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB23:    xsi_set_current_line(90, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB24:    xsi_set_current_line(92, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB25:    xsi_set_current_line(94, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB26:    xsi_set_current_line(96, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB27:    xsi_set_current_line(100, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB28:    xsi_set_current_line(102, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB29:    xsi_set_current_line(104, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB30:    xsi_set_current_line(106, ng0);
    t2 = (t0 + 3624);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB50:;
LAB51:    goto LAB6;

LAB52:    goto LAB51;

LAB54:    goto LAB52;

LAB56:    xsi_set_current_line(117, ng0);

LAB62:    *((char **)t1) = &&LAB63;
    goto LAB1;

LAB57:    goto LAB56;

LAB59:    goto LAB57;

LAB60:    goto LAB2;

LAB61:    goto LAB60;

LAB63:    goto LAB61;

}


extern void work_a_1119971917_3212880686_init()
{
	static char *pe[] = {(void *)work_a_1119971917_3212880686_p_0};
	xsi_register_didat("work_a_1119971917_3212880686", "isim/Control_Control_sch_tb_isim_beh.exe.sim/work/a_1119971917_3212880686.didat");
	xsi_register_executes(pe);
}
