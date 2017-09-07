/*
 * drivers/amlogic/amports/arch/regs/hcodec_regs.h
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
*/

#ifndef HCODEC_REG_HEADERS___
#define HCODEC_REG_HEADERS___

#define HCODEC_ASSIST_MMC_CTRL0 0x1001
#define HCODEC_ASSIST_MMC_CTRL1 0x1002
/*add from M8M2*/
#define HCODEC_ASSIST_MMC_CTRL2 0x1003
#define HCODEC_ASSIST_MMC_CTRL3 0x1004
/**/
#define HCODEC_MFDIN_REG0_CRST 0x1010
#define HCODEC_MFDIN_REG1_CTRL 0x1011
#define HCODEC_MFDIN_REG2_STAT 0x1012
#define HCODEC_MFDIN_REG3_CANV 0x1013
#define HCODEC_MFDIN_REG4_LNR0 0x1014
#define HCODEC_MFDIN_REG5_LNR1 0x1015
#define HCODEC_MFDIN_REG6_DCFG 0x1016
#define HCODEC_MFDIN_REG7_SCMD 0x1017
#define HCODEC_MFDIN_REG8_DMBL 0x1018
#define HCODEC_MFDIN_REG9_ENDN 0x1019
#define HCODEC_MFDIN_REGA_CAV1 0x101a
#define HCODEC_MFDIN_REGB_AMPC 0x101b
#define HCODEC_ASSIST_AMR1_INT0 0x1025
#define HCODEC_ASSIST_AMR1_INT1 0x1026
#define HCODEC_ASSIST_AMR1_INT2 0x1027
#define HCODEC_ASSIST_AMR1_INT3 0x1028
#define HCODEC_ASSIST_AMR1_INT4 0x1029
#define HCODEC_ASSIST_AMR1_INT5 0x102a
#define HCODEC_ASSIST_AMR1_INT6 0x102b
#define HCODEC_ASSIST_AMR1_INT7 0x102c
#define HCODEC_ASSIST_AMR1_INT8 0x102d
#define HCODEC_ASSIST_AMR1_INT9 0x102e
#define HCODEC_ASSIST_AMR1_INTA 0x102f
#define HCODEC_ASSIST_AMR1_INTB 0x1030
#define HCODEC_ASSIST_AMR1_INTC 0x1031
#define HCODEC_ASSIST_AMR1_INTD 0x1032
#define HCODEC_ASSIST_AMR1_INTE 0x1033
#define HCODEC_ASSIST_AMR1_INTF 0x1034
#define HCODEC_ASSIST_AMR2_INT0 0x1035
#define HCODEC_ASSIST_AMR2_INT1 0x1036
#define HCODEC_ASSIST_AMR2_INT2 0x1037
#define HCODEC_ASSIST_AMR2_INT3 0x1038
#define HCODEC_ASSIST_AMR2_INT4 0x1039
#define HCODEC_ASSIST_AMR2_INT5 0x103a
#define HCODEC_ASSIST_AMR2_INT6 0x103b
#define HCODEC_ASSIST_AMR2_INT7 0x103c
#define HCODEC_ASSIST_AMR2_INT8 0x103d
#define HCODEC_ASSIST_AMR2_INT9 0x103e
#define HCODEC_ASSIST_AMR2_INTA 0x103f
#define HCODEC_ASSIST_AMR2_INTB 0x1040
#define HCODEC_ASSIST_AMR2_INTC 0x1041
#define HCODEC_ASSIST_AMR2_INTD 0x1042
#define HCODEC_ASSIST_AMR2_INTE 0x1043
#define HCODEC_ASSIST_AMR2_INTF 0x1044
#define HCODEC_ASSIST_MBX_SSEL 0x1045
#define HCODEC_ASSIST_TIMER0_LO 0x1060
#define HCODEC_ASSIST_TIMER0_HI 0x1061
#define HCODEC_ASSIST_TIMER1_LO 0x1062
#define HCODEC_ASSIST_TIMER1_HI 0x1063
#define HCODEC_ASSIST_DMA_INT 0x1064
#define HCODEC_ASSIST_DMA_INT_MSK 0x1065
#define HCODEC_ASSIST_DMA_INT2 0x1066
#define HCODEC_ASSIST_DMA_INT_MSK2 0x1067
#define HCODEC_ASSIST_MBOX0_IRQ_REG 0x1070
#define HCODEC_ASSIST_MBOX0_CLR_REG 0x1071
#define HCODEC_ASSIST_MBOX0_MASK 0x1072
#define HCODEC_ASSIST_MBOX0_FIQ_SEL 0x1073
#define HCODEC_ASSIST_MBOX1_IRQ_REG 0x1074
#define HCODEC_ASSIST_MBOX1_CLR_REG 0x1075
#define HCODEC_ASSIST_MBOX1_MASK 0x1076
#define HCODEC_ASSIST_MBOX1_FIQ_SEL 0x1077
#define HCODEC_ASSIST_MBOX2_IRQ_REG 0x1078
#define HCODEC_ASSIST_MBOX2_CLR_REG 0x1079
#define HCODEC_ASSIST_MBOX2_MASK 0x107a
#define HCODEC_ASSIST_MBOX2_FIQ_SEL 0x107b

#define HCODEC_MC_CTRL_REG 0x1900
#define HCODEC_MC_MB_INFO 0x1901
#define HCODEC_MC_PIC_INFO 0x1902
#define HCODEC_MC_HALF_PEL_ONE 0x1903
#define HCODEC_MC_HALF_PEL_TWO 0x1904
#define HCODEC_POWER_CTL_MC 0x1905
#define HCODEC_MC_CMD 0x1906
#define HCODEC_MC_CTRL0 0x1907
#define HCODEC_MC_PIC_W_H 0x1908
#define HCODEC_MC_STATUS0 0x1909
#define HCODEC_MC_STATUS1 0x190a
#define HCODEC_MC_CTRL1 0x190b
#define HCODEC_MC_MIX_RATIO0 0x190c
#define HCODEC_MC_MIX_RATIO1 0x190d
#define HCODEC_MC_DP_MB_XY 0x190e
#define HCODEC_MC_OM_MB_XY 0x190f
#define HCODEC_PSCALE_RST 0x1910
#define HCODEC_PSCALE_CTRL 0x1911
#define HCODEC_PSCALE_PICI_W 0x1912
#define HCODEC_PSCALE_PICI_H 0x1913
#define HCODEC_PSCALE_PICO_W 0x1914
#define HCODEC_PSCALE_PICO_H 0x1915
#define HCODEC_PSCALE_PICO_START_X 0x1916
#define HCODEC_PSCALE_PICO_START_Y 0x1917
#define HCODEC_PSCALE_DUMMY 0x1918
#define HCODEC_PSCALE_FILT0_COEF0 0x1919
#define HCODEC_PSCALE_FILT0_COEF1 0x191a
#define HCODEC_PSCALE_CMD_CTRL 0x191b
#define HCODEC_PSCALE_CMD_BLK_X 0x191c
#define HCODEC_PSCALE_CMD_BLK_Y 0x191d
#define HCODEC_PSCALE_STATUS 0x191e
#define HCODEC_PSCALE_BMEM_ADDR 0x191f
#define HCODEC_PSCALE_BMEM_DAT 0x1920
#define HCODEC_PSCALE_DRAM_BUF_CTRL 0x1921
#define HCODEC_PSCALE_MCMD_CTRL 0x1922
#define HCODEC_PSCALE_MCMD_XSIZE 0x1923
#define HCODEC_PSCALE_MCMD_YSIZE 0x1924
#define HCODEC_PSCALE_RBUF_START_BLKX 0x1925
#define HCODEC_PSCALE_RBUF_START_BLKY 0x1926
#define HCODEC_PSCALE_PICO_SHIFT_XY 0x1928
#define HCODEC_PSCALE_CTRL1 0x1929
#define HCODEC_PSCALE_SRCKEY_CTRL0 0x192a
#define HCODEC_PSCALE_SRCKEY_CTRL1 0x192b
#define HCODEC_PSCALE_CANVAS_RD_ADDR 0x192c
#define HCODEC_PSCALE_CANVAS_WR_ADDR 0x192d
#define HCODEC_PSCALE_CTRL2 0x192e
/*add from M8M2*/
#define HCODEC_HDEC_MC_OMEM_AUTO 0x1930
#define HCODEC_HDEC_MC_MBRIGHT_IDX 0x1931
#define HCODEC_HDEC_MC_MBRIGHT_RD 0x1932
/**/
#define HCODEC_MC_MPORT_CTRL 0x1940
#define HCODEC_MC_MPORT_DAT 0x1941
#define HCODEC_MC_WT_PRED_CTRL 0x1942
#define HCODEC_MC_MBBOT_ST_EVEN_ADDR 0x1944
#define HCODEC_MC_MBBOT_ST_ODD_ADDR 0x1945
#define HCODEC_MC_DPDN_MB_XY 0x1946
#define HCODEC_MC_OMDN_MB_XY 0x1947
#define HCODEC_MC_HCMDBUF_H 0x1948
#define HCODEC_MC_HCMDBUF_L 0x1949
#define HCODEC_MC_HCMD_H 0x194a
#define HCODEC_MC_HCMD_L 0x194b
#define HCODEC_MC_IDCT_DAT 0x194c
#define HCODEC_MC_CTRL_GCLK_CTRL 0x194d
#define HCODEC_MC_OTHER_GCLK_CTRL 0x194e
#define HCODEC_MC_CTRL2 0x194f
#define HCODEC_MDEC_PIC_DC_CTRL 0x198e
#define HCODEC_MDEC_PIC_DC_STATUS 0x198f
#define HCODEC_ANC0_CANVAS_ADDR 0x1990
#define HCODEC_ANC1_CANVAS_ADDR 0x1991
#define HCODEC_ANC2_CANVAS_ADDR 0x1992
#define HCODEC_ANC3_CANVAS_ADDR 0x1993
#define HCODEC_ANC4_CANVAS_ADDR 0x1994
#define HCODEC_ANC5_CANVAS_ADDR 0x1995
#define HCODEC_ANC6_CANVAS_ADDR 0x1996
#define HCODEC_ANC7_CANVAS_ADDR 0x1997
#define HCODEC_ANC8_CANVAS_ADDR 0x1998
#define HCODEC_ANC9_CANVAS_ADDR 0x1999
#define HCODEC_ANC10_CANVAS_ADDR 0x199a
#define HCODEC_ANC11_CANVAS_ADDR 0x199b
#define HCODEC_ANC12_CANVAS_ADDR 0x199c
#define HCODEC_ANC13_CANVAS_ADDR 0x199d
#define HCODEC_ANC14_CANVAS_ADDR 0x199e
#define HCODEC_ANC15_CANVAS_ADDR 0x199f
#define HCODEC_ANC16_CANVAS_ADDR 0x19a0
#define HCODEC_ANC17_CANVAS_ADDR 0x19a1
#define HCODEC_ANC18_CANVAS_ADDR 0x19a2
#define HCODEC_ANC19_CANVAS_ADDR 0x19a3
#define HCODEC_ANC20_CANVAS_ADDR 0x19a4
#define HCODEC_ANC21_CANVAS_ADDR 0x19a5
#define HCODEC_ANC22_CANVAS_ADDR 0x19a6
#define HCODEC_ANC23_CANVAS_ADDR 0x19a7
#define HCODEC_ANC24_CANVAS_ADDR 0x19a8
#define HCODEC_ANC25_CANVAS_ADDR 0x19a9
#define HCODEC_ANC26_CANVAS_ADDR 0x19aa
#define HCODEC_ANC27_CANVAS_ADDR 0x19ab
#define HCODEC_ANC28_CANVAS_ADDR 0x19ac
#define HCODEC_ANC29_CANVAS_ADDR 0x19ad
#define HCODEC_ANC30_CANVAS_ADDR 0x19ae
#define HCODEC_ANC31_CANVAS_ADDR 0x19af
#define HCODEC_DBKR_CANVAS_ADDR 0x19b0
#define HCODEC_DBKW_CANVAS_ADDR 0x19b1
#define HCODEC_REC_CANVAS_ADDR 0x19b2
#define HCODEC_CURR_CANVAS_CTRL 0x19b3
#define HCODEC_MDEC_PIC_DC_THRESH 0x19b8
#define HCODEC_MDEC_PICR_BUF_STATUS 0x19b9
#define HCODEC_MDEC_PICW_BUF_STATUS 0x19ba
#define HCODEC_MCW_DBLK_WRRSP_CNT 0x19bb
#define HCODEC_MC_MBBOT_WRRSP_CNT 0x19bc
#define HCODEC_MDEC_PICW_BUF2_STATUS 0x19bd
#define HCODEC_WRRSP_FIFO_PICW_DBK 0x19be
#define HCODEC_WRRSP_FIFO_PICW_MC 0x19bf
#define HCODEC_AV_SCRATCH_0 0x19c0
#define HCODEC_AV_SCRATCH_1 0x19c1
#define HCODEC_AV_SCRATCH_2 0x19c2
#define HCODEC_AV_SCRATCH_3 0x19c3
#define HCODEC_AV_SCRATCH_4 0x19c4
#define HCODEC_AV_SCRATCH_5 0x19c5
#define HCODEC_AV_SCRATCH_6 0x19c6
#define HCODEC_AV_SCRATCH_7 0x19c7
#define HCODEC_AV_SCRATCH_8 0x19c8
#define HCODEC_AV_SCRATCH_9 0x19c9
#define HCODEC_AV_SCRATCH_A 0x19ca
#define HCODEC_AV_SCRATCH_B 0x19cb
#define HCODEC_AV_SCRATCH_C 0x19cc
#define HCODEC_AV_SCRATCH_D 0x19cd
#define HCODEC_AV_SCRATCH_E 0x19ce
#define HCODEC_AV_SCRATCH_F 0x19cf
#define HCODEC_AV_SCRATCH_G 0x19d0
#define HCODEC_AV_SCRATCH_H 0x19d1
#define HCODEC_AV_SCRATCH_I 0x19d2
#define HCODEC_AV_SCRATCH_J 0x19d3
#define HCODEC_AV_SCRATCH_K 0x19d4
#define HCODEC_AV_SCRATCH_L 0x19d5
#define HCODEC_AV_SCRATCH_M 0x19d6
#define HCODEC_AV_SCRATCH_N 0x19d7
#define HCODEC_WRRSP_CO_MB 0x19d8
#define HCODEC_WRRSP_DCAC 0x19d9
/*add from M8M2*/
#define HCODEC_WRRSP_VLD 0x19da
#define HCODEC_MDEC_DOUBLEW_CFG0 0x19db
#define HCODEC_MDEC_DOUBLEW_CFG1 0x19dc
#define HCODEC_MDEC_DOUBLEW_CFG2 0x19dd
#define HCODEC_MDEC_DOUBLEW_CFG3 0x19de
#define HCODEC_MDEC_DOUBLEW_CFG4 0x19df
#define HCODEC_MDEC_DOUBLEW_CFG5 0x19e0
#define HCODEC_MDEC_DOUBLEW_CFG6 0x19e1
#define HCODEC_MDEC_DOUBLEW_CFG7 0x19e2
#define HCODEC_MDEC_DOUBLEW_STATUS 0x19e3
/**/
#define HCODEC_DBLK_RST 0x1950
#define HCODEC_DBLK_CTRL 0x1951
#define HCODEC_DBLK_MB_WID_HEIGHT 0x1952
#define HCODEC_DBLK_STATUS 0x1953
#define HCODEC_DBLK_CMD_CTRL 0x1954
#define HCODEC_DBLK_MB_XY 0x1955
#define HCODEC_DBLK_QP 0x1956
#define HCODEC_DBLK_Y_BHFILT 0x1957
#define HCODEC_DBLK_Y_BHFILT_HIGH 0x1958
#define HCODEC_DBLK_Y_BVFILT 0x1959
#define HCODEC_DBLK_CB_BFILT 0x195a
#define HCODEC_DBLK_CR_BFILT 0x195b
#define HCODEC_DBLK_Y_HFILT 0x195c
#define HCODEC_DBLK_Y_HFILT_HIGH 0x195d
#define HCODEC_DBLK_Y_VFILT 0x195e
#define HCODEC_DBLK_CB_FILT 0x195f
#define HCODEC_DBLK_CR_FILT 0x1960
#define HCODEC_DBLK_BETAX_QP_SEL 0x1961
#define HCODEC_DBLK_CLIP_CTRL0 0x1962
#define HCODEC_DBLK_CLIP_CTRL1 0x1963
#define HCODEC_DBLK_CLIP_CTRL2 0x1964
#define HCODEC_DBLK_CLIP_CTRL3 0x1965
#define HCODEC_DBLK_CLIP_CTRL4 0x1966
#define HCODEC_DBLK_CLIP_CTRL5 0x1967
#define HCODEC_DBLK_CLIP_CTRL6 0x1968
#define HCODEC_DBLK_CLIP_CTRL7 0x1969
#define HCODEC_DBLK_CLIP_CTRL8 0x196a
#define HCODEC_DBLK_STATUS1 0x196b
#define HCODEC_DBLK_GCLK_FREE 0x196c
#define HCODEC_DBLK_GCLK_OFF 0x196d
#define HCODEC_DBLK_AVSFLAGS 0x196e
#define HCODEC_DBLK_CBPY 0x1970
#define HCODEC_DBLK_CBPY_ADJ 0x1971
#define HCODEC_DBLK_CBPC 0x1972
#define HCODEC_DBLK_CBPC_ADJ 0x1973
#define HCODEC_DBLK_VHMVD 0x1974
#define HCODEC_DBLK_STRONG 0x1975
#define HCODEC_DBLK_RV8_QUANT 0x1976
#define HCODEC_DBLK_CBUS_HCMD2 0x1977
#define HCODEC_DBLK_CBUS_HCMD1 0x1978
#define HCODEC_DBLK_CBUS_HCMD0 0x1979
#define HCODEC_DBLK_VLD_HCMD2 0x197a
#define HCODEC_DBLK_VLD_HCMD1 0x197b
#define HCODEC_DBLK_VLD_HCMD0 0x197c
#define HCODEC_DBLK_OST_YBASE 0x197d
#define HCODEC_DBLK_OST_CBCRDIFF 0x197e
#define HCODEC_DBLK_CTRL1 0x197f
#define HCODEC_MCRCC_CTL1 0x1980
#define HCODEC_MCRCC_CTL2 0x1981
#define HCODEC_MCRCC_CTL3 0x1982
#define HCODEC_GCLK_EN 0x1983
#define HCODEC_MDEC_SW_RESET 0x1984

#define HCODEC_VLD_STATUS_CTRL 0x1c00
#define HCODEC_MPEG1_2_REG 0x1c01
#define HCODEC_F_CODE_REG 0x1c02
#define HCODEC_PIC_HEAD_INFO 0x1c03
#define HCODEC_SLICE_VER_POS_PIC_TYPE 0x1c04
#define HCODEC_QP_VALUE_REG 0x1c05
#define HCODEC_MBA_INC 0x1c06
#define HCODEC_MB_MOTION_MODE 0x1c07
#define HCODEC_POWER_CTL_VLD 0x1c08
#define HCODEC_MB_WIDTH 0x1c09
#define HCODEC_SLICE_QP 0x1c0a
#define HCODEC_PRE_START_CODE 0x1c0b
#define HCODEC_SLICE_START_BYTE_01 0x1c0c
#define HCODEC_SLICE_START_BYTE_23 0x1c0d
#define HCODEC_RESYNC_MARKER_LENGTH 0x1c0e
#define HCODEC_DECODER_BUFFER_INFO 0x1c0f
#define HCODEC_FST_FOR_MV_X 0x1c10
#define HCODEC_FST_FOR_MV_Y 0x1c11
#define HCODEC_SCD_FOR_MV_X 0x1c12
#define HCODEC_SCD_FOR_MV_Y 0x1c13
#define HCODEC_FST_BAK_MV_X 0x1c14
#define HCODEC_FST_BAK_MV_Y 0x1c15
#define HCODEC_SCD_BAK_MV_X 0x1c16
#define HCODEC_SCD_BAK_MV_Y 0x1c17
#define HCODEC_VLD_DECODE_CONTROL 0x1c18
#define HCODEC_VLD_REVERVED_19 0x1c19
#define HCODEC_VIFF_BIT_CNT 0x1c1a
#define HCODEC_BYTE_ALIGN_PEAK_HI 0x1c1b
#define HCODEC_BYTE_ALIGN_PEAK_LO 0x1c1c
#define HCODEC_NEXT_ALIGN_PEAK 0x1c1d
#define HCODEC_VC1_CONTROL_REG 0x1c1e
#define HCODEC_PMV1_X 0x1c20
#define HCODEC_PMV1_Y 0x1c21
#define HCODEC_PMV2_X 0x1c22
#define HCODEC_PMV2_Y 0x1c23
#define HCODEC_PMV3_X 0x1c24
#define HCODEC_PMV3_Y 0x1c25
#define HCODEC_PMV4_X 0x1c26
#define HCODEC_PMV4_Y 0x1c27
#define HCODEC_M4_TABLE_SELECT 0x1c28
#define HCODEC_M4_CONTROL_REG 0x1c29
#define HCODEC_BLOCK_NUM 0x1c2a
#define HCODEC_PATTERN_CODE 0x1c2b
#define HCODEC_MB_INFO 0x1c2c
#define HCODEC_VLD_DC_PRED 0x1c2d
#define HCODEC_VLD_ERROR_MASK 0x1c2e
#define HCODEC_VLD_DC_PRED_C 0x1c2f
#define HCODEC_LAST_SLICE_MV_ADDR 0x1c30
#define HCODEC_LAST_MVX 0x1c31
#define HCODEC_LAST_MVY 0x1c32
#define HCODEC_VLD_C38 0x1c38
#define HCODEC_VLD_C39 0x1c39
#define HCODEC_VLD_STATUS 0x1c3a
#define HCODEC_VLD_SHIFT_STATUS 0x1c3b
#define HCODEC_VOFF_STATUS 0x1c3c
#define HCODEC_VLD_C3D 0x1c3d
#define HCODEC_VLD_DBG_INDEX 0x1c3e
#define HCODEC_VLD_DBG_DATA 0x1c3f
#define HCODEC_VLD_MEM_VIFIFO_START_PTR 0x1c40
#define HCODEC_VLD_MEM_VIFIFO_CURR_PTR 0x1c41
#define HCODEC_VLD_MEM_VIFIFO_END_PTR 0x1c42
#define HCODEC_VLD_MEM_VIFIFO_BYTES_AVAIL 0x1c43
#define HCODEC_VLD_MEM_VIFIFO_CONTROL 0x1c44
#define HCODEC_VLD_MEM_VIFIFO_WP 0x1c45
#define HCODEC_VLD_MEM_VIFIFO_RP 0x1c46
#define HCODEC_VLD_MEM_VIFIFO_LEVEL 0x1c47
#define HCODEC_VLD_MEM_VIFIFO_BUF_CNTL 0x1c48
#define HCODEC_VLD_TIME_STAMP_CNTL 0x1c49
#define HCODEC_VLD_TIME_STAMP_SYNC_0 0x1c4a
#define HCODEC_VLD_TIME_STAMP_SYNC_1 0x1c4b
#define HCODEC_VLD_TIME_STAMP_0 0x1c4c
#define HCODEC_VLD_TIME_STAMP_1 0x1c4d
#define HCODEC_VLD_TIME_STAMP_2 0x1c4e
#define HCODEC_VLD_TIME_STAMP_3 0x1c4f
#define HCODEC_VLD_TIME_STAMP_LENGTH 0x1c50
#define HCODEC_VLD_MEM_VIFIFO_WRAP_COUNT 0x1c51
#define HCODEC_VLD_MEM_VIFIFO_MEM_CTL 0x1c52
#define HCODEC_VLD_MEM_VBUF_RD_PTR 0x1c53
#define HCODEC_VLD_MEM_VBUF2_RD_PTR 0x1c54
#define HCODEC_VLD_MEM_SWAP_ADDR 0x1c55
#define HCODEC_VLD_MEM_SWAP_CTL 0x1c56

#define HCODEC_VCOP_CTRL_REG 0x1e00
#define HCODEC_QP_CTRL_REG 0x1e01
#define HCODEC_INTRA_QUANT_MATRIX 0x1e02
#define HCODEC_NON_I_QUANT_MATRIX 0x1e03
#define HCODEC_DC_SCALER 0x1e04
#define HCODEC_DC_AC_CTRL 0x1e05
#define HCODEC_DC_AC_SCALE_MUL 0x1e06
#define HCODEC_DC_AC_SCALE_DIV 0x1e07
#define HCODEC_POWER_CTL_IQIDCT 0x1e08
#define HCODEC_RV_AI_Y_X 0x1e09
#define HCODEC_RV_AI_U_X 0x1e0a
#define HCODEC_RV_AI_V_X 0x1e0b
#define HCODEC_RV_AI_MB_COUNT 0x1e0c
#define HCODEC_NEXT_INTRA_DMA_ADDRESS 0x1e0d
#define HCODEC_IQIDCT_CONTROL 0x1e0e
#define HCODEC_IQIDCT_DEBUG_INFO_0 0x1e0f
#define HCODEC_DEBLK_CMD 0x1e10
#define HCODEC_IQIDCT_DEBUG_IDCT 0x1e11
#define HCODEC_DCAC_DMA_CTRL 0x1e12
#define HCODEC_DCAC_DMA_ADDRESS 0x1e13
#define HCODEC_DCAC_CPU_ADDRESS 0x1e14
#define HCODEC_DCAC_CPU_DATA 0x1e15
#define HCODEC_DCAC_MB_COUNT 0x1e16
#define HCODEC_IQ_QUANT 0x1e17
#define HCODEC_VC1_BITPLANE_CTL 0x1e18

#define HCODEC_MSP 0x1300
#define HCODEC_MPSR 0x1301
#define HCODEC_MINT_VEC_BASE 0x1302
#define HCODEC_MCPU_INTR_GRP 0x1303
#define HCODEC_MCPU_INTR_MSK 0x1304
#define HCODEC_MCPU_INTR_REQ 0x1305
#define HCODEC_MPC_P 0x1306
#define HCODEC_MPC_D 0x1307
#define HCODEC_MPC_E 0x1308
#define HCODEC_MPC_W 0x1309
#define HCODEC_MINDEX0_REG 0x130a
#define HCODEC_MINDEX1_REG 0x130b
#define HCODEC_MINDEX2_REG 0x130c
#define HCODEC_MINDEX3_REG 0x130d
#define HCODEC_MINDEX4_REG 0x130e
#define HCODEC_MINDEX5_REG 0x130f
#define HCODEC_MINDEX6_REG 0x1310
#define HCODEC_MINDEX7_REG 0x1311
#define HCODEC_MMIN_REG 0x1312
#define HCODEC_MMAX_REG 0x1313
#define HCODEC_MBREAK0_REG 0x1314
#define HCODEC_MBREAK1_REG 0x1315
#define HCODEC_MBREAK2_REG 0x1316
#define HCODEC_MBREAK3_REG 0x1317
#define HCODEC_MBREAK_TYPE 0x1318
#define HCODEC_MBREAK_CTRL 0x1319
#define HCODEC_MBREAK_STAUTS 0x131a
#define HCODEC_MDB_ADDR_REG 0x131b
#define HCODEC_MDB_DATA_REG 0x131c
#define HCODEC_MDB_CTRL 0x131d
#define HCODEC_MSFTINT0 0x131e
#define HCODEC_MSFTINT1 0x131f
#define HCODEC_CSP 0x1320
#define HCODEC_CPSR 0x1321
#define HCODEC_CINT_VEC_BASE 0x1322
#define HCODEC_CCPU_INTR_GRP 0x1323
#define HCODEC_CCPU_INTR_MSK 0x1324
#define HCODEC_CCPU_INTR_REQ 0x1325
#define HCODEC_CPC_P 0x1326
#define HCODEC_CPC_D 0x1327
#define HCODEC_CPC_E 0x1328
#define HCODEC_CPC_W 0x1329
#define HCODEC_CINDEX0_REG 0x132a
#define HCODEC_CINDEX1_REG 0x132b
#define HCODEC_CINDEX2_REG 0x132c
#define HCODEC_CINDEX3_REG 0x132d
#define HCODEC_CINDEX4_REG 0x132e
#define HCODEC_CINDEX5_REG 0x132f
#define HCODEC_CINDEX6_REG 0x1330
#define HCODEC_CINDEX7_REG 0x1331
#define HCODEC_CMIN_REG 0x1332
#define HCODEC_CMAX_REG 0x1333
#define HCODEC_CBREAK0_REG 0x1334
#define HCODEC_CBREAK1_REG 0x1335
#define HCODEC_CBREAK2_REG 0x1336
#define HCODEC_CBREAK3_REG 0x1337
#define HCODEC_CBREAK_TYPE 0x1338
#define HCODEC_CBREAK_CTRL 0x1339
#define HCODEC_CBREAK_STAUTS 0x133a
#define HCODEC_CDB_ADDR_REG 0x133b
#define HCODEC_CDB_DATA_REG 0x133c
#define HCODEC_CDB_CTRL 0x133d
#define HCODEC_CSFTINT0 0x133e
#define HCODEC_CSFTINT1 0x133f
#define HCODEC_IMEM_DMA_CTRL 0x1340
#define HCODEC_IMEM_DMA_ADR 0x1341
#define HCODEC_IMEM_DMA_COUNT 0x1342
#define HCODEC_WRRSP_IMEM 0x1343
#define HCODEC_LMEM_DMA_CTRL 0x1350
#define HCODEC_LMEM_DMA_ADR 0x1351
#define HCODEC_LMEM_DMA_COUNT 0x1352
#define HCODEC_WRRSP_LMEM 0x1353
#define HCODEC_MAC_CTRL1 0x1360
#define HCODEC_ACC0REG1 0x1361
#define HCODEC_ACC1REG1 0x1362
#define HCODEC_MAC_CTRL2 0x1370
#define HCODEC_ACC0REG2 0x1371
#define HCODEC_ACC1REG2 0x1372
#define HCODEC_CPU_TRACE 0x1380

#define HCODEC_HENC_SCRATCH_0 0x1ac0
#define HCODEC_HENC_SCRATCH_1 0x1ac1
#define HCODEC_HENC_SCRATCH_2 0x1ac2
#define HCODEC_HENC_SCRATCH_3 0x1ac3
#define HCODEC_HENC_SCRATCH_4 0x1ac4
#define HCODEC_HENC_SCRATCH_5 0x1ac5
#define HCODEC_HENC_SCRATCH_6 0x1ac6
#define HCODEC_HENC_SCRATCH_7 0x1ac7
#define HCODEC_HENC_SCRATCH_8 0x1ac8
#define HCODEC_HENC_SCRATCH_9 0x1ac9
#define HCODEC_HENC_SCRATCH_A 0x1aca
#define HCODEC_HENC_SCRATCH_B 0x1acb
#define HCODEC_HENC_SCRATCH_C 0x1acc
#define HCODEC_HENC_SCRATCH_D 0x1acd
#define HCODEC_HENC_SCRATCH_E 0x1ace
#define HCODEC_HENC_SCRATCH_F 0x1acf
#define HCODEC_HENC_SCRATCH_G 0x1ad0
#define HCODEC_HENC_SCRATCH_H 0x1ad1
#define HCODEC_HENC_SCRATCH_I 0x1ad2
#define HCODEC_HENC_SCRATCH_J 0x1ad3
#define HCODEC_HENC_SCRATCH_K 0x1ad4
#define HCODEC_HENC_SCRATCH_L 0x1ad5
#define HCODEC_HENC_SCRATCH_M 0x1ad6
#define HCODEC_HENC_SCRATCH_N 0x1ad7
#define HCODEC_IE_DATA_FEED_BUFF_INFO 0x1ad8
#define HCODEC_VLC_STATUS_CTRL 0x1d00
#define HCODEC_VLC_CONFIG 0x1d01
#define HCODEC_VLC_VB_START_PTR 0x1d10
#define HCODEC_VLC_VB_END_PTR 0x1d11
#define HCODEC_VLC_VB_WR_PTR 0x1d12
#define HCODEC_VLC_VB_RD_PTR 0x1d13
#define HCODEC_VLC_VB_SW_RD_PTR 0x1d14
#define HCODEC_VLC_VB_LEFT 0x1d15
#define HCODEC_VLC_VB_CONTROL 0x1d16
#define HCODEC_VLC_VB_MEM_CTL 0x1d17
#define HCODEC_VLC_VB_INT_PTR 0x1d18
#define HCODEC_VLC_WRRSP 0x1d19
#define HCODEC_VLC_TOTAL_BYTES 0x1d1a
#define HCODEC_VLC_VB_BUFF 0x1d1b
#define HCODEC_VLC_VB_PRE_BUFF_HI 0x1d1c
#define HCODEC_VLC_VB_PRE_BUFF_LOW 0x1d1d
#define HCODEC_VLC_STREAM_BUFF 0x1d1e
#define HCODEC_VLC_PUSH_STREAM 0x1d1f
#define HCODEC_VLC_PUSH_ELEMENT 0x1d20
#define HCODEC_VLC_ELEMENT_DATA 0x1d21
/*add from M8m2*/
#define HCODEC_VLC_SPECIAL_CTL 0x1d22
#define HCODEC_VLC_HCMD_T_L_INFO 0x1d23
#define HCODEC_VLC_HCMD_CUR_INFO 0x1d24
/* add from GXBB */
#define HCODEC_VLC_ADV_CONFIG 0x1d25
#define HCODEC_VLC_HCMD_MBXY_AUTO 0x1d26
#define HCODEC_VLC_INT_CONTROL_INTER 0x1d2f
/**/
#define HCODEC_VLC_INT_CONTROL 0x1d30
#define HCODEC_VLC_PIC_SIZE 0x1d31
#define HCODEC_VLC_PIC_INFO 0x1d32
#define HCODEC_VLC_PIC_POSITION 0x1d33
#define HCODEC_VLC_INPUT_STATUS 0x1d34
#define HCODEC_VLC_MB_INFO 0x1d35
#define HCODEC_VLC_ENC_PEND_CMD 0x1d36
#define HCODEC_HENC_TOP_INFO_0 0x1d37
#define HCODEC_HENC_LEFT_INFO_0 0x1d38
#define HCODEC_HENC_TOP_INFO_1 0x1d39
#define HCODEC_HENC_LEFT_INFO_1 0x1d3a
#define HCODEC_VLC_IPRED_MODE_HI 0x1d3b
#define HCODEC_VLC_IPRED_MODE_LO 0x1d3c
#define HCODEC_VLC_DELTA_QP 0x1d3d
#define HCODEC_VLC_MB_HEADER_INFO 0x1d3e
#define HCODEC_VLC_P_MB_HEADER_INFO 0x1d3f
#define HCODEC_VLC_COEFF_BUF_STATUS 0x1d40
#define HCODEC_VLC_COEFF_RD_REQ 0x1d41
#define HCODEC_VLC_COEFF 0x1d42
#define HCODEC_VLC_COEFF_INFO 0x1d43
#define HCODEC_VLC_DC_BUF_STATUS 0x1d44
#define HCODEC_VLC_DC_RD_REQ 0x1d45
#define HCODEC_VLC_DC 0x1d46
#define HCODEC_VLC_DC_INFO 0x1d47
#define HCODEC_VLC_MV_INDEX 0x1d48
#define HCODEC_VLC_MV 0x1d49
#define HCODEC_HENC_TOP_MV_0 0x1d4a
#define HCODEC_HENC_TOP_MV_1 0x1d4b
#define HCODEC_HENC_TOP_MV_2 0x1d4c
#define HCODEC_HENC_TOP_MV_3 0x1d4d
#define HCODEC_HENC_LEFT_MV_0 0x1d4e
#define HCODEC_HENC_LEFT_MV_1 0x1d4f
#define HCODEC_HENC_LEFT_MV_2 0x1d50
#define HCODEC_HENC_LEFT_MV_3 0x1d51
#define HCODEC_TOP_LEFT_READY 0x1d52
#define HCODEC_MB_SKIP_RUN 0x1d53
#define HCODEC_VLC_HCMD_CONFIG 0x1d54
#define HCODEC_VLC_HCMD_DBLK_INFO 0x1d55
#define HCODEC_VLC_DBG_IDX 0x1d56
#define HCODEC_VLC_DBG_READ 0x1d57
#define HCODEC_VLC_JPEG_CTRL 0x1d58
#define HCODEC_VLC_JPEG_COEFF_BUF_STAT 0x1d59
#define HCODEC_VLC_HUFFMAN_ADDR 0x1d5a
#define HCODEC_VLC_HUFFMAN_DATA 0x1d5b
#define HCODEC_VLC_ENC_MV_BITS 0x1d5c
#define HCODEC_VLC_ENC_COEFF_BITS 0x1d5d
#define HCODEC_QDCT_STATUS_CTRL 0x1f00
#define HCODEC_QDCT_CONFIG 0x1f01
#define HCODEC_IGNORE_CONFIG 0x1f02
#define HCODEC_IGNORE_CONFIG_2 0x1f03
#define HCODEC_QDCT_MB_START_PTR 0x1f10
#define HCODEC_QDCT_MB_END_PTR 0x1f11
#define HCODEC_QDCT_MB_WR_PTR 0x1f12
#define HCODEC_QDCT_MB_RD_PTR 0x1f13
#define HCODEC_QDCT_MB_LEVEL 0x1f14
#define HCODEC_QDCT_MB_CONTROL 0x1f15
#define HCODEC_QDCT_MB_MEM_CTL 0x1f16
#define HCODEC_QDCT_MB_BUFF 0x1f17
#define HCODEC_QDCT_MB_MAGIC_WORD 0x1f18
#define HCODEC_QDCT_DCT_STATUS 0x1f19
#define HCODEC_QDCT_Q_STATUS 0x1f1a
#define HCODEC_QDCT_PIC_INFO 0x1f1b
#define HCODEC_QDCT_Q_QUANT_I 0x1f1c
#define HCODEC_QDCT_Q_QUANT_P 0x1f1d
#define HCODEC_QDCT_MB_PAUSE_CTL 0x1f1e
#define HCODEC_QDCT_TOP_CONTROL 0x1f1f
#define HCODEC_QDCT_TOP_BASE_MEM 0x1f20
#define HCODEC_QDCT_TOP_MEM_CTL 0x1f21
#define HCODEC_QDCT_TOP_WRRSP 0x1f22
#define HCODEC_QDCT_DBG_IDX 0x1f23
#define HCODEC_QDCT_DBG_READ 0x1f24
#define HCODEC_QDCT_JPEG_CTRL 0x1f25
#define HCODEC_QDCT_JPEG_X_START_END 0x1f26
#define HCODEC_QDCT_JPEG_Y_START_END 0x1f27
#define HCODEC_QDCT_JPEG_QUANT_ADDR 0x1f28
#define HCODEC_QDCT_JPEG_QUANT_DATA 0x1f29
#define HCODEC_QDCT_JPEG_SOF_RESUME 0x1f2a
#define HCODEC_QDCT_JPEG_DCT_STATUS0 0x1f2b
#define HCODEC_QDCT_JPEG_DCT_STATUS1 0x1f2c
#define HCODEC_QDCT_JPEG_DCT_COEFF01 0x1f2d
#define HCODEC_QDCT_JPEG_DCT_COEFF23 0x1f2e
#define HCODEC_QDCT_JPEG_DCT_COEFF45 0x1f2f
#define HCODEC_QDCT_JPEG_DCT_COEFF67 0x1f30
#define HCODEC_QDCT_JPEG_DCT_COEFF89 0x1f31
/*add from M8M2*/
#define HCODEC_QDCT_I_PRED_REF_WR_IDX 0x1f32
#define HCODEC_QDCT_I_PRED_REF_WR_DATA 0x1f33
/* add from GXBB */
#define HCODEC_QDCT_ADV_CONFIG 0x1f34
#define HCODEC_IE_WEIGHT 0x1f35
#define HCODEC_Q_QUANT_CONTROL 0x1f36
#define HCODEC_MBBOT_EVEN_ADDR 0x1f37
#define HCODEC_MBBOT_ODD_ADDR 0x1f38
#define HCODEC_QUANT_TABLE_DATA 0x1f39
#define HCODEC_SAD_CONTROL_0 0x1f3a
#define HCODEC_SAD_CONTROL_1 0x1f3b
#define HCODEC_QDCT_VLC_QUANT_CTL_0 0x1f3c
#define HCODEC_QDCT_VLC_QUANT_CTL_1 0x1f3d
#define HCODEC_QDCT_INT_STATUS 0x1f3e
#define HCODEC_QDCT_MIX_I_PRED_STATUS 0x1f3f
/**/
#define HCODEC_IE_CONTROL 0x1f40
#define HCODEC_IE_MB_POSITION 0x1f41
#define HCODEC_IE_ME_MB_INFO 0x1f42
#define HCODEC_SAD_CONTROL 0x1f43
#define HCODEC_IE_RESULT_BUFFER 0x1f44
#define HCODEC_IE_I4_PRED_MODE_HI 0x1f45
#define HCODEC_IE_I4_PRED_MODE_LO 0x1f46
#define HCODEC_IE_C_PRED_MODE 0x1f47
#define HCODEC_IE_CUR_REF_SEL 0x1f48
#define HCODEC_ME_CONTROL 0x1f49
#define HCODEC_ME_START_POSITION 0x1f4a
#define HCODEC_ME_STATUS 0x1f4b
#define HCODEC_ME_DEBUG 0x1f4c
#define HCODEC_ME_SKIP_LINE 0x1f4d
#define HCODEC_ME_AB_MEM_CTL 0x1f4e
#define HCODEC_ME_PIC_INFO 0x1f4f
#define HCODEC_ME_SAD_ENOUGH_01 0x1f50
#define HCODEC_ME_SAD_ENOUGH_23 0x1f51
#define HCODEC_ME_STEP0_CLOSE_MV 0x1f52
#define HCODEC_ME_F_SKIP_SAD 0x1f53
#define HCODEC_ME_F_SKIP_WEIGHT 0x1f54
#define HCODEC_ME_MV_MERGE_CTL 0x1f55
#define HCODEC_ME_MV_WEIGHT_01 0x1f56
#define HCODEC_ME_MV_WEIGHT_23 0x1f57
#define HCODEC_ME_SAD_RANGE_INC 0x1f58
#define HCODEC_ME_SUB_MERGE_CTL 0x1f59
#define HCODEC_ME_SUB_REF_MV_CTL 0x1f5a
#define HCODEC_ME_SUB_ANY_WEIGHT_SAD 0x1f5b
#define HCODEC_ME_SUB_FIX_SAD 0x1f5c
#define HCODEC_ME_SUB_FIX_MIN_SAD 0x1f5d
#define HCODEC_ME_SUB_SNAP_GLITCH 0x1f5e
#define HCODEC_ME_SUB_ACT_CTL 0x1f5f
/* add from GXBB */
#define HCODEC_ME_WEIGHT 0x1f60
#define HCODEC_ME_SAD_0 0x1f61
#define HCODEC_ME_SAD_1 0x1f62
#define HCODEC_ME_SAD_2 0x1f63
#define HCODEC_ME_SAD_3 0x1f64
#define HCODEC_IE_SAD_0 0x1f65
#define HCODEC_IE_SAD_1 0x1f66
#define HCODEC_IE_SAD_2 0x1f67
#define HCODEC_IE_SAD_3 0x1f68
#define HCODEC_ADV_MV_CTL0 0x1f69
#define HCODEC_ADV_MV_CTL1 0x1f6a
#define HCODEC_ADV_MV_CTL2 0x1f6b

/* add from GXTVBB */
#define HCODEC_V3_SKIP_CONTROL 0x1f6c
#define HCODEC_V3_TOP_LEFT_CTL 0x1f6d
#define HCODEC_V3_TOP_MV 0x1f6e
#define HCODEC_V3_LEFT_MV 0x1f6f
#define HCODEC_V3_SKIP_WEIGHT 0x1f70
#define HCODEC_V3_L1_SKIP_MAX_SAD 0x1f71
#define HCODEC_V3_L2_SKIP_WEIGHT 0x1f72
#define HCODEC_V3_MV_SAD_TABLE 0x1f73
#define HCODEC_V3_F_ZERO_CTL_0 0x1f74
#define HCODEC_V3_F_ZERO_CTL_1 0x1f75
#define HCODEC_V3_TOP_INTRA_INFO 0x1f76
#define HCODEC_V3_LEFT_INTRA_INFO 0x1f77
#define HCODEC_V3_IPRED_TYPE_WEIGHT_0 0x1f78
#define HCODEC_V3_IPRED_TYPE_WEIGHT_1 0x1f79
#define HCODEC_V3_LEFT_SMALL_MAX_SAD 0x1f7a

/* add from GXL */
#define HCODEC_V4_FORCE_SKIP_CFG 0x1f7b

/* add from TXL */
#define HCODEC_V5_MB_DIFF_SUM 0x1f7c
#define HCODEC_V5_SMALL_DIFF_CNT 0x1f7d
#define HCODEC_V5_SIMPLE_MB_CTL 0x1f7e
#define HCODEC_V5_SIMPLE_MB_DQUANT 0x1f7f
#define HCODEC_V5_SIMPLE_MB_ME_WEIGHT 0x1f80
#endif

