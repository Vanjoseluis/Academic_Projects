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
static const char *ng0 = "C:/Users/Usuario_UMA/Downloads/RLab_Plant_Jose_Raul_14-05-26_1B/Reg_Vot_VHDL_CASE_TEST.vhd";



static void work_a_1080852945_2372691052_p_0(char *t0)
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

LAB0:    t1 = (t0 + 3432U);
    t2 = *((char **)t1);
    if (t2 == 0)
        goto LAB2;

LAB3:    goto *t2;

LAB2:    xsi_set_current_line(76, ng0);
    t2 = (t0 + 3816);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(77, ng0);
    t2 = (t0 + 3880);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(78, ng0);
    t2 = (t0 + 3944);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(79, ng0);
    t2 = (t0 + 4008);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(81, ng0);
    t2 = (t0 + 6776);
    *((int *)t2) = 0;
    t3 = (t0 + 6780);
    *((int *)t3) = 13;
    t7 = 0;
    t8 = 13;

LAB4:    if (t7 <= t8)
        goto LAB5;

LAB7:    xsi_set_current_line(120, ng0);
    t2 = (t0 + 4072);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(121, ng0);
    t2 = (t0 + 2448U);
    t3 = *((char **)t2);
    t11 = *((int64 *)t3);
    t2 = (t0 + 3240);
    xsi_process_wait(t2, t11);

LAB48:    *((char **)t1) = &&LAB49;

LAB1:    return;
LAB5:    xsi_set_current_line(82, ng0);
    t4 = (t0 + 4072);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    t9 = (t6 + 56U);
    t10 = *((char **)t9);
    *((unsigned char *)t10) = (unsigned char)3;
    xsi_driver_first_trans_fast(t4);
    xsi_set_current_line(83, ng0);
    t2 = (t0 + 2448U);
    t3 = *((char **)t2);
    t11 = *((int64 *)t3);
    t2 = (t0 + 3240);
    xsi_process_wait(t2, t11);

LAB10:    *((char **)t1) = &&LAB11;
    goto LAB1;

LAB6:    t2 = (t0 + 6776);
    t7 = *((int *)t2);
    t3 = (t0 + 6780);
    t8 = *((int *)t3);
    if (t7 == t8)
        goto LAB7;

LAB45:    t12 = (t7 + 1);
    t7 = t12;
    t4 = (t0 + 6776);
    *((int *)t4) = t7;
    goto LAB4;

LAB8:    xsi_set_current_line(85, ng0);
    t2 = (t0 + 6776);
    if (*((int *)t2) == 1)
        goto LAB13;

LAB27:    if (*((int *)t2) == 2)
        goto LAB14;

LAB28:    if (*((int *)t2) == 3)
        goto LAB15;

LAB29:    if (*((int *)t2) == 4)
        goto LAB16;

LAB30:    if (*((int *)t2) == 5)
        goto LAB17;

LAB31:    if (*((int *)t2) == 6)
        goto LAB18;

LAB32:    if (*((int *)t2) == 7)
        goto LAB19;

LAB33:    if (*((int *)t2) == 8)
        goto LAB20;

LAB34:    if (*((int *)t2) == 9)
        goto LAB21;

LAB35:    if (*((int *)t2) == 10)
        goto LAB22;

LAB36:    if (*((int *)t2) == 11)
        goto LAB23;

LAB37:    if (*((int *)t2) == 12)
        goto LAB24;

LAB38:    if (*((int *)t2) == 13)
        goto LAB25;

LAB39:
LAB26:    xsi_set_current_line(113, ng0);

LAB12:    xsi_set_current_line(116, ng0);
    t2 = (t0 + 4072);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(117, ng0);
    t2 = (t0 + 2448U);
    t3 = *((char **)t2);
    t11 = *((int64 *)t3);
    t2 = (t0 + 3240);
    xsi_process_wait(t2, t11);

LAB43:    *((char **)t1) = &&LAB44;
    goto LAB1;

LAB9:    goto LAB8;

LAB11:    goto LAB9;

LAB13:    xsi_set_current_line(87, ng0);
    t3 = (t0 + 3816);
    t4 = (t3 + 56U);
    t5 = *((char **)t4);
    t6 = (t5 + 56U);
    t9 = *((char **)t6);
    *((unsigned char *)t9) = (unsigned char)3;
    xsi_driver_first_trans_fast(t3);
    goto LAB12;

LAB14:    xsi_set_current_line(89, ng0);
    t2 = (t0 + 3816);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB15:    xsi_set_current_line(91, ng0);
    t2 = (t0 + 3880);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB16:    xsi_set_current_line(93, ng0);
    t2 = (t0 + 3944);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(93, ng0);
    t2 = (t0 + 4008);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB17:    xsi_set_current_line(95, ng0);
    t2 = (t0 + 4008);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB18:    xsi_set_current_line(97, ng0);
    t2 = (t0 + 3944);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(97, ng0);
    t2 = (t0 + 4008);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB19:    xsi_set_current_line(99, ng0);
    t2 = (t0 + 3880);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB20:    xsi_set_current_line(101, ng0);
    t2 = (t0 + 3816);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB21:    xsi_set_current_line(103, ng0);
    t2 = (t0 + 3816);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(103, ng0);
    t2 = (t0 + 3880);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(103, ng0);
    t2 = (t0 + 3944);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(103, ng0);
    t2 = (t0 + 4008);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB22:    xsi_set_current_line(105, ng0);
    t2 = (t0 + 3880);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB23:    xsi_set_current_line(107, ng0);
    t2 = (t0 + 3880);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB24:    xsi_set_current_line(109, ng0);
    t2 = (t0 + 4008);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB25:    xsi_set_current_line(111, ng0);
    t2 = (t0 + 3880);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)2;
    xsi_driver_first_trans_fast(t2);
    xsi_set_current_line(111, ng0);
    t2 = (t0 + 3944);
    t3 = (t2 + 56U);
    t4 = *((char **)t3);
    t5 = (t4 + 56U);
    t6 = *((char **)t5);
    *((unsigned char *)t6) = (unsigned char)3;
    xsi_driver_first_trans_fast(t2);
    goto LAB12;

LAB40:;
LAB41:    goto LAB6;

LAB42:    goto LAB41;

LAB44:    goto LAB42;

LAB46:    xsi_set_current_line(122, ng0);

LAB52:    *((char **)t1) = &&LAB53;
    goto LAB1;

LAB47:    goto LAB46;

LAB49:    goto LAB47;

LAB50:    goto LAB2;

LAB51:    goto LAB50;

LAB53:    goto LAB51;

}


extern void work_a_1080852945_2372691052_init()
{
	static char *pe[] = {(void *)work_a_1080852945_2372691052_p_0};
	xsi_register_didat("work_a_1080852945_2372691052", "isim/Reg_Vot_VHDL_CASE_TEST_isim_beh.exe.sim/work/a_1080852945_2372691052.didat");
	xsi_register_executes(pe);
}
