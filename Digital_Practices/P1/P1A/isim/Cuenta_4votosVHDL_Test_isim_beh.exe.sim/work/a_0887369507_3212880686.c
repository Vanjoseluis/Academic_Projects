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
static const char *ng0 = "C:/Users/edie/Desktop/RLab_Plant_Jose_Raul2_16-04-26/Cuenta_4votos.vhd";



static void work_a_0887369507_3212880686_p_0(char *t0)
{
    char *t1;
    char *t2;
    unsigned int t3;
    unsigned int t4;
    unsigned int t5;
    char *t6;
    char *t7;
    int t8;
    char *t9;
    char *t10;
    int t11;
    char *t12;
    int t14;
    char *t15;
    int t17;
    char *t18;
    int t20;
    char *t21;
    int t23;
    char *t24;
    int t26;
    char *t27;
    int t29;
    char *t30;
    int t32;
    char *t33;
    int t35;
    char *t36;
    int t38;
    char *t39;
    int t41;
    char *t42;
    int t44;
    char *t45;
    int t47;
    char *t48;
    int t50;
    char *t51;
    int t53;
    char *t54;
    char *t56;
    char *t57;
    char *t58;
    char *t59;
    char *t60;

LAB0:    xsi_set_current_line(85, ng0);
    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t3 = (3 - 3);
    t4 = (t3 * 1U);
    t5 = (0 + t4);
    t1 = (t2 + t5);
    t6 = (t0 + 4488);
    t8 = xsi_mem_cmp(t6, t1, 4U);
    if (t8 == 1)
        goto LAB3;

LAB20:    t9 = (t0 + 4492);
    t11 = xsi_mem_cmp(t9, t1, 4U);
    if (t11 == 1)
        goto LAB4;

LAB21:    t12 = (t0 + 4496);
    t14 = xsi_mem_cmp(t12, t1, 4U);
    if (t14 == 1)
        goto LAB5;

LAB22:    t15 = (t0 + 4500);
    t17 = xsi_mem_cmp(t15, t1, 4U);
    if (t17 == 1)
        goto LAB6;

LAB23:    t18 = (t0 + 4504);
    t20 = xsi_mem_cmp(t18, t1, 4U);
    if (t20 == 1)
        goto LAB7;

LAB24:    t21 = (t0 + 4508);
    t23 = xsi_mem_cmp(t21, t1, 4U);
    if (t23 == 1)
        goto LAB8;

LAB25:    t24 = (t0 + 4512);
    t26 = xsi_mem_cmp(t24, t1, 4U);
    if (t26 == 1)
        goto LAB9;

LAB26:    t27 = (t0 + 4516);
    t29 = xsi_mem_cmp(t27, t1, 4U);
    if (t29 == 1)
        goto LAB10;

LAB27:    t30 = (t0 + 4520);
    t32 = xsi_mem_cmp(t30, t1, 4U);
    if (t32 == 1)
        goto LAB11;

LAB28:    t33 = (t0 + 4524);
    t35 = xsi_mem_cmp(t33, t1, 4U);
    if (t35 == 1)
        goto LAB12;

LAB29:    t36 = (t0 + 4528);
    t38 = xsi_mem_cmp(t36, t1, 4U);
    if (t38 == 1)
        goto LAB13;

LAB30:    t39 = (t0 + 4532);
    t41 = xsi_mem_cmp(t39, t1, 4U);
    if (t41 == 1)
        goto LAB14;

LAB31:    t42 = (t0 + 4536);
    t44 = xsi_mem_cmp(t42, t1, 4U);
    if (t44 == 1)
        goto LAB15;

LAB32:    t45 = (t0 + 4540);
    t47 = xsi_mem_cmp(t45, t1, 4U);
    if (t47 == 1)
        goto LAB16;

LAB33:    t48 = (t0 + 4544);
    t50 = xsi_mem_cmp(t48, t1, 4U);
    if (t50 == 1)
        goto LAB17;

LAB34:    t51 = (t0 + 4548);
    t53 = xsi_mem_cmp(t51, t1, 4U);
    if (t53 == 1)
        goto LAB18;

LAB35:
LAB19:    xsi_set_current_line(102, ng0);
    t1 = (t0 + 4600);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);

LAB2:    t1 = (t0 + 2672);
    *((int *)t1) = 1;

LAB1:    return;
LAB3:    xsi_set_current_line(86, ng0);
    t54 = (t0 + 4552);
    t56 = (t0 + 2752);
    t57 = (t56 + 56U);
    t58 = *((char **)t57);
    t59 = (t58 + 56U);
    t60 = *((char **)t59);
    memcpy(t60, t54, 3U);
    xsi_driver_first_trans_fast_port(t56);
    goto LAB2;

LAB4:    xsi_set_current_line(87, ng0);
    t1 = (t0 + 4555);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB5:    xsi_set_current_line(88, ng0);
    t1 = (t0 + 4558);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB6:    xsi_set_current_line(89, ng0);
    t1 = (t0 + 4561);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB7:    xsi_set_current_line(90, ng0);
    t1 = (t0 + 4564);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB8:    xsi_set_current_line(91, ng0);
    t1 = (t0 + 4567);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB9:    xsi_set_current_line(92, ng0);
    t1 = (t0 + 4570);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB10:    xsi_set_current_line(93, ng0);
    t1 = (t0 + 4573);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB11:    xsi_set_current_line(94, ng0);
    t1 = (t0 + 4576);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB12:    xsi_set_current_line(95, ng0);
    t1 = (t0 + 4579);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB13:    xsi_set_current_line(96, ng0);
    t1 = (t0 + 4582);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB14:    xsi_set_current_line(97, ng0);
    t1 = (t0 + 4585);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB15:    xsi_set_current_line(98, ng0);
    t1 = (t0 + 4588);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB16:    xsi_set_current_line(99, ng0);
    t1 = (t0 + 4591);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB17:    xsi_set_current_line(100, ng0);
    t1 = (t0 + 4594);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB18:    xsi_set_current_line(101, ng0);
    t1 = (t0 + 4597);
    t6 = (t0 + 2752);
    t7 = (t6 + 56U);
    t9 = *((char **)t7);
    t10 = (t9 + 56U);
    t12 = *((char **)t10);
    memcpy(t12, t1, 3U);
    xsi_driver_first_trans_fast_port(t6);
    goto LAB2;

LAB36:;
}


extern void work_a_0887369507_3212880686_init()
{
	static char *pe[] = {(void *)work_a_0887369507_3212880686_p_0};
	xsi_register_didat("work_a_0887369507_3212880686", "isim/Cuenta_4votosVHDL_Test_isim_beh.exe.sim/work/a_0887369507_3212880686.didat");
	xsi_register_executes(pe);
}
