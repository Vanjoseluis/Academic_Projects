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
static const char *ng0 = "C:/Users/Usuario_UMA/Downloads/RLab_Plant_Jose_Raul_14-05-26_1B/Reg_Vot_VHDL_Case.vhd";
extern char *IEEE_P_2592010699;

unsigned char ieee_p_2592010699_sub_1605435078_503743352(char *, unsigned char , unsigned char );
unsigned char ieee_p_2592010699_sub_1690584930_503743352(char *, unsigned char );
unsigned char ieee_p_2592010699_sub_1744673427_503743352(char *, char *, unsigned int , unsigned int );


static void work_a_1821355141_3212880686_p_0(char *t0)
{
    char *t1;
    char *t2;
    unsigned char t3;
    unsigned char t4;
    char *t5;
    char *t6;
    char *t7;
    char *t8;
    char *t9;
    char *t10;

LAB0:    xsi_set_current_line(51, ng0);
    t1 = (t0 + 1512U);
    t2 = *((char **)t1);
    t3 = *((unsigned char *)t2);
    t4 = (t3 == (unsigned char)3);
    if (t4 != 0)
        goto LAB2;

LAB4:    xsi_set_current_line(54, ng0);
    t1 = (t0 + 1632U);
    t3 = ieee_p_2592010699_sub_1744673427_503743352(IEEE_P_2592010699, t1, 0U, 0U);
    if (t3 != 0)
        goto LAB5;

LAB7:
LAB6:
LAB3:    t1 = (t0 + 4944);
    *((int *)t1) = 1;

LAB1:    return;
LAB2:    xsi_set_current_line(52, ng0);
    t1 = xsi_get_transient_memory(2U);
    memset(t1, 0, 2U);
    t5 = t1;
    memset(t5, (unsigned char)2, 2U);
    t6 = (t0 + 5088);
    t7 = (t6 + 56U);
    t8 = *((char **)t7);
    t9 = (t8 + 56U);
    t10 = *((char **)t9);
    memcpy(t10, t1, 2U);
    xsi_driver_first_trans_fast(t6);
    goto LAB3;

LAB5:    xsi_set_current_line(55, ng0);
    t2 = (t0 + 2472U);
    t5 = *((char **)t2);
    t2 = (t0 + 5088);
    t6 = (t2 + 56U);
    t7 = *((char **)t6);
    t8 = (t7 + 56U);
    t9 = *((char **)t8);
    memcpy(t9, t5, 2U);
    xsi_driver_first_trans_fast(t2);
    goto LAB6;

}

static void work_a_1821355141_3212880686_p_1(char *t0)
{
    char *t1;
    char *t2;
    unsigned char t3;
    unsigned char t4;
    char *t5;
    char *t6;
    int t7;
    char *t8;
    char *t9;
    int t10;
    char *t11;
    char *t12;
    char *t13;
    char *t14;
    char *t15;
    char *t16;
    char *t17;
    unsigned char t18;
    unsigned char t19;
    unsigned char t20;
    unsigned int t21;
    unsigned int t22;
    char *t23;
    char *t24;
    char *t25;

LAB0:    xsi_set_current_line(69, ng0);
    t1 = (t0 + 1352U);
    t2 = *((char **)t1);
    t3 = *((unsigned char *)t2);
    t4 = (t3 == (unsigned char)3);
    if (t4 != 0)
        goto LAB2;

LAB4:    xsi_set_current_line(79, ng0);
    t1 = (t0 + 2312U);
    t2 = *((char **)t1);
    t1 = (t0 + 8275);
    t4 = 1;
    if (2U == 2U)
        goto LAB42;

LAB43:    t4 = 0;

LAB44:    if (t4 == 1)
        goto LAB39;

LAB40:    t9 = (t0 + 2312U);
    t11 = *((char **)t9);
    t9 = (t0 + 8277);
    t18 = 1;
    if (2U == 2U)
        goto LAB48;

LAB49:    t18 = 0;

LAB50:    t3 = t18;

LAB41:    if (t3 != 0)
        goto LAB36;

LAB38:    xsi_set_current_line(82, ng0);
    t1 = (t0 + 8279);
    t5 = (t0 + 5152);
    t6 = (t5 + 56U);
    t8 = *((char **)t6);
    t9 = (t8 + 56U);
    t11 = *((char **)t9);
    memcpy(t11, t1, 2U);
    xsi_driver_first_trans_fast(t5);

LAB37:
LAB3:    t1 = (t0 + 4960);
    *((int *)t1) = 1;

LAB1:    return;
LAB2:    xsi_set_current_line(70, ng0);
    t1 = (t0 + 2312U);
    t5 = *((char **)t1);
    t1 = (t0 + 8261);
    t7 = xsi_mem_cmp(t1, t5, 2U);
    if (t7 == 1)
        goto LAB6;

LAB9:    t8 = (t0 + 8263);
    t10 = xsi_mem_cmp(t8, t5, 2U);
    if (t10 == 1)
        goto LAB7;

LAB10:
LAB8:    xsi_set_current_line(76, ng0);
    t1 = (t0 + 2312U);
    t2 = *((char **)t1);
    t1 = (t0 + 5152);
    t5 = (t1 + 56U);
    t6 = *((char **)t5);
    t8 = (t6 + 56U);
    t9 = *((char **)t8);
    memcpy(t9, t2, 2U);
    xsi_driver_first_trans_fast(t1);

LAB5:    goto LAB3;

LAB6:    xsi_set_current_line(71, ng0);
    t11 = (t0 + 8265);
    t13 = (t0 + 5152);
    t14 = (t13 + 56U);
    t15 = *((char **)t14);
    t16 = (t15 + 56U);
    t17 = *((char **)t16);
    memcpy(t17, t11, 2U);
    xsi_driver_first_trans_fast(t13);
    goto LAB5;

LAB7:    xsi_set_current_line(72, ng0);
    t1 = (t0 + 1192U);
    t2 = *((char **)t1);
    t4 = *((unsigned char *)t2);
    t18 = (t4 == (unsigned char)3);
    if (t18 == 1)
        goto LAB15;

LAB16:    t3 = (unsigned char)0;

LAB17:    if (t3 != 0)
        goto LAB12;

LAB14:
LAB13:    xsi_set_current_line(73, ng0);
    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t4 = *((unsigned char *)t2);
    t18 = (t4 == (unsigned char)3);
    if (t18 == 1)
        goto LAB21;

LAB22:    t3 = (unsigned char)0;

LAB23:    if (t3 != 0)
        goto LAB18;

LAB20:
LAB19:    xsi_set_current_line(74, ng0);
    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t4 = *((unsigned char *)t2);
    t18 = (t4 == (unsigned char)3);
    if (t18 == 1)
        goto LAB27;

LAB28:    t3 = (unsigned char)0;

LAB29:    if (t3 != 0)
        goto LAB24;

LAB26:
LAB25:    xsi_set_current_line(75, ng0);
    t1 = (t0 + 1032U);
    t2 = *((char **)t1);
    t4 = *((unsigned char *)t2);
    t18 = (t4 == (unsigned char)2);
    if (t18 == 1)
        goto LAB33;

LAB34:    t3 = (unsigned char)0;

LAB35:    if (t3 != 0)
        goto LAB30;

LAB32:
LAB31:    goto LAB5;

LAB11:;
LAB12:    xsi_set_current_line(72, ng0);
    t1 = (t0 + 8267);
    t8 = (t0 + 5152);
    t9 = (t8 + 56U);
    t11 = *((char **)t9);
    t12 = (t11 + 56U);
    t13 = *((char **)t12);
    memcpy(t13, t1, 2U);
    xsi_driver_first_trans_fast(t8);
    goto LAB13;

LAB15:    t1 = (t0 + 1032U);
    t5 = *((char **)t1);
    t19 = *((unsigned char *)t5);
    t20 = (t19 == (unsigned char)2);
    t3 = t20;
    goto LAB17;

LAB18:    xsi_set_current_line(73, ng0);
    t1 = (t0 + 8269);
    t8 = (t0 + 5152);
    t9 = (t8 + 56U);
    t11 = *((char **)t9);
    t12 = (t11 + 56U);
    t13 = *((char **)t12);
    memcpy(t13, t1, 2U);
    xsi_driver_first_trans_fast(t8);
    goto LAB19;

LAB21:    t1 = (t0 + 1192U);
    t5 = *((char **)t1);
    t19 = *((unsigned char *)t5);
    t20 = (t19 == (unsigned char)2);
    t3 = t20;
    goto LAB23;

LAB24:    xsi_set_current_line(74, ng0);
    t1 = (t0 + 8271);
    t8 = (t0 + 5152);
    t9 = (t8 + 56U);
    t11 = *((char **)t9);
    t12 = (t11 + 56U);
    t13 = *((char **)t12);
    memcpy(t13, t1, 2U);
    xsi_driver_first_trans_fast(t8);
    goto LAB25;

LAB27:    t1 = (t0 + 1192U);
    t5 = *((char **)t1);
    t19 = *((unsigned char *)t5);
    t20 = (t19 == (unsigned char)3);
    t3 = t20;
    goto LAB29;

LAB30:    xsi_set_current_line(75, ng0);
    t1 = (t0 + 8273);
    t8 = (t0 + 5152);
    t9 = (t8 + 56U);
    t11 = *((char **)t9);
    t12 = (t11 + 56U);
    t13 = *((char **)t12);
    memcpy(t13, t1, 2U);
    xsi_driver_first_trans_fast(t8);
    goto LAB31;

LAB33:    t1 = (t0 + 1192U);
    t5 = *((char **)t1);
    t19 = *((unsigned char *)t5);
    t20 = (t19 == (unsigned char)2);
    t3 = t20;
    goto LAB35;

LAB36:    xsi_set_current_line(80, ng0);
    t15 = (t0 + 2312U);
    t16 = *((char **)t15);
    t15 = (t0 + 5152);
    t17 = (t15 + 56U);
    t23 = *((char **)t17);
    t24 = (t23 + 56U);
    t25 = *((char **)t24);
    memcpy(t25, t16, 2U);
    xsi_driver_first_trans_fast(t15);
    goto LAB37;

LAB39:    t3 = (unsigned char)1;
    goto LAB41;

LAB42:    t21 = 0;

LAB45:    if (t21 < 2U)
        goto LAB46;
    else
        goto LAB44;

LAB46:    t6 = (t2 + t21);
    t8 = (t1 + t21);
    if (*((unsigned char *)t6) != *((unsigned char *)t8))
        goto LAB43;

LAB47:    t21 = (t21 + 1);
    goto LAB45;

LAB48:    t22 = 0;

LAB51:    if (t22 < 2U)
        goto LAB52;
    else
        goto LAB50;

LAB52:    t13 = (t11 + t22);
    t14 = (t9 + t22);
    if (*((unsigned char *)t13) != *((unsigned char *)t14))
        goto LAB49;

LAB53:    t22 = (t22 + 1);
    goto LAB51;

}

static void work_a_1821355141_3212880686_p_2(char *t0)
{
    char *t1;
    char *t2;
    int t3;
    unsigned int t4;
    unsigned int t5;
    unsigned int t6;
    unsigned char t7;
    char *t8;
    char *t9;
    int t10;
    unsigned int t11;
    unsigned int t12;
    unsigned int t13;
    unsigned char t14;
    unsigned char t15;
    char *t16;
    char *t17;
    char *t18;
    char *t19;
    char *t20;
    char *t21;

LAB0:    xsi_set_current_line(88, ng0);

LAB3:    t1 = (t0 + 2312U);
    t2 = *((char **)t1);
    t3 = (0 - 1);
    t4 = (t3 * -1);
    t5 = (1U * t4);
    t6 = (0 + t5);
    t1 = (t2 + t6);
    t7 = *((unsigned char *)t1);
    t8 = (t0 + 2312U);
    t9 = *((char **)t8);
    t10 = (1 - 1);
    t11 = (t10 * -1);
    t12 = (1U * t11);
    t13 = (0 + t12);
    t8 = (t9 + t13);
    t14 = *((unsigned char *)t8);
    t15 = ieee_p_2592010699_sub_1605435078_503743352(IEEE_P_2592010699, t7, t14);
    t16 = (t0 + 5216);
    t17 = (t16 + 56U);
    t18 = *((char **)t17);
    t19 = (t18 + 56U);
    t20 = *((char **)t19);
    *((unsigned char *)t20) = t15;
    xsi_driver_first_trans_fast_port(t16);

LAB2:    t21 = (t0 + 4976);
    *((int *)t21) = 1;

LAB1:    return;
LAB4:    goto LAB2;

}

static void work_a_1821355141_3212880686_p_3(char *t0)
{
    char *t1;
    char *t2;
    int t3;
    unsigned int t4;
    unsigned int t5;
    unsigned int t6;
    unsigned char t7;
    char *t8;
    char *t9;
    char *t10;
    char *t11;
    char *t12;
    char *t13;

LAB0:    xsi_set_current_line(89, ng0);

LAB3:    t1 = (t0 + 2312U);
    t2 = *((char **)t1);
    t3 = (1 - 1);
    t4 = (t3 * -1);
    t5 = (1U * t4);
    t6 = (0 + t5);
    t1 = (t2 + t6);
    t7 = *((unsigned char *)t1);
    t8 = (t0 + 5280);
    t9 = (t8 + 56U);
    t10 = *((char **)t9);
    t11 = (t10 + 56U);
    t12 = *((char **)t11);
    *((unsigned char *)t12) = t7;
    xsi_driver_first_trans_fast_port(t8);

LAB2:    t13 = (t0 + 4992);
    *((int *)t13) = 1;

LAB1:    return;
LAB4:    goto LAB2;

}

static void work_a_1821355141_3212880686_p_4(char *t0)
{
    char *t1;
    char *t2;
    int t3;
    unsigned int t4;
    unsigned int t5;
    unsigned int t6;
    unsigned char t7;
    unsigned char t8;
    char *t9;
    char *t10;
    int t11;
    unsigned int t12;
    unsigned int t13;
    unsigned int t14;
    unsigned char t15;
    unsigned char t16;
    char *t17;
    char *t18;
    char *t19;
    char *t20;
    char *t21;
    char *t22;

LAB0:    xsi_set_current_line(90, ng0);

LAB3:    t1 = (t0 + 2312U);
    t2 = *((char **)t1);
    t3 = (1 - 1);
    t4 = (t3 * -1);
    t5 = (1U * t4);
    t6 = (0 + t5);
    t1 = (t2 + t6);
    t7 = *((unsigned char *)t1);
    t8 = ieee_p_2592010699_sub_1690584930_503743352(IEEE_P_2592010699, t7);
    t9 = (t0 + 2312U);
    t10 = *((char **)t9);
    t11 = (0 - 1);
    t12 = (t11 * -1);
    t13 = (1U * t12);
    t14 = (0 + t13);
    t9 = (t10 + t14);
    t15 = *((unsigned char *)t9);
    t16 = ieee_p_2592010699_sub_1605435078_503743352(IEEE_P_2592010699, t8, t15);
    t17 = (t0 + 5344);
    t18 = (t17 + 56U);
    t19 = *((char **)t18);
    t20 = (t19 + 56U);
    t21 = *((char **)t20);
    *((unsigned char *)t21) = t16;
    xsi_driver_first_trans_fast_port(t17);

LAB2:    t22 = (t0 + 5008);
    *((int *)t22) = 1;

LAB1:    return;
LAB4:    goto LAB2;

}


extern void work_a_1821355141_3212880686_init()
{
	static char *pe[] = {(void *)work_a_1821355141_3212880686_p_0,(void *)work_a_1821355141_3212880686_p_1,(void *)work_a_1821355141_3212880686_p_2,(void *)work_a_1821355141_3212880686_p_3,(void *)work_a_1821355141_3212880686_p_4};
	xsi_register_didat("work_a_1821355141_3212880686", "isim/Reg_Vot_VHDL_CASE_TEST_isim_beh.exe.sim/work/a_1821355141_3212880686.didat");
	xsi_register_executes(pe);
}
