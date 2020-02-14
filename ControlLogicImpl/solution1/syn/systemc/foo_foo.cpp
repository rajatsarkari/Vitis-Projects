// ==============================================================
// RTL generated by Vitis HLS - High-Level Synthesis from C, C++ and OpenCL
// Version: 2020.1.0
// Copyright (C) 1986-2020 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

#include "foo_foo.h"
#include "AESL_pkg.h"

using namespace std;

namespace ap_rtl {

const sc_logic foo_foo::ap_const_logic_1 = sc_dt::Log_1;
const sc_logic foo_foo::ap_const_logic_0 = sc_dt::Log_0;
const sc_lv<3> foo_foo::ap_ST_fsm_state1 = "1";
const sc_lv<3> foo_foo::ap_ST_fsm_pp0_stage0 = "10";
const sc_lv<3> foo_foo::ap_ST_fsm_state4 = "100";
const sc_lv<32> foo_foo::ap_const_lv32_0 = "00000000000000000000000000000000";
const bool foo_foo::ap_const_boolean_1 = true;
const sc_lv<32> foo_foo::ap_const_lv32_1 = "1";
const bool foo_foo::ap_const_boolean_0 = false;
const sc_lv<1> foo_foo::ap_const_lv1_0 = "0";
const sc_lv<1> foo_foo::ap_const_lv1_1 = "1";
const sc_lv<2> foo_foo::ap_const_lv2_0 = "00";
const sc_lv<2> foo_foo::ap_const_lv2_3 = "11";
const sc_lv<2> foo_foo::ap_const_lv2_1 = "1";
const sc_lv<32> foo_foo::ap_const_lv32_2 = "10";

foo_foo::foo_foo(sc_module_name name) : sc_module(name), mVcdFile(0) {
    foo_mul_32s_8s_32_1_1_U1 = new foo_foo_mul_32s_8s_32_1_1<1,1,32,8,32>("foo_mul_32s_8s_32_1_1_U1");
    foo_mul_32s_8s_32_1_1_U1->din0(in_r_q0);
    foo_mul_32s_8s_32_1_1_U1->din1(mul_ln5_fu_154_p1);
    foo_mul_32s_8s_32_1_1_U1->dout(mul_ln5_fu_154_p2);

    SC_METHOD(thread_ap_clk_no_reset_);
    dont_initialize();
    sensitive << ( ap_clk.pos() );

    SC_METHOD(thread_add_ln3_fu_143_p2);
    sensitive << ( i_reg_104 );

    SC_METHOD(thread_add_ln5_fu_127_p2);
    sensitive << ( sext_ln3_fu_123_p1 );
    sensitive << ( b_cast_fu_119_p1 );

    SC_METHOD(thread_ap_CS_fsm_pp0_stage0);
    sensitive << ( ap_CS_fsm );

    SC_METHOD(thread_ap_CS_fsm_state1);
    sensitive << ( ap_CS_fsm );

    SC_METHOD(thread_ap_CS_fsm_state4);
    sensitive << ( ap_CS_fsm );

    SC_METHOD(thread_ap_block_pp0_stage0);

    SC_METHOD(thread_ap_block_pp0_stage0_11001);

    SC_METHOD(thread_ap_block_pp0_stage0_subdone);

    SC_METHOD(thread_ap_block_state2_pp0_stage0_iter0);

    SC_METHOD(thread_ap_block_state3_pp0_stage0_iter1);

    SC_METHOD(thread_ap_condition_pp0_exit_iter0_state2);
    sensitive << ( icmp_ln3_fu_137_p2 );

    SC_METHOD(thread_ap_done);
    sensitive << ( ap_CS_fsm_state4 );

    SC_METHOD(thread_ap_enable_pp0);
    sensitive << ( ap_idle_pp0 );

    SC_METHOD(thread_ap_idle);
    sensitive << ( ap_start );
    sensitive << ( ap_CS_fsm_state1 );

    SC_METHOD(thread_ap_idle_pp0);
    sensitive << ( ap_enable_reg_pp0_iter0 );
    sensitive << ( ap_enable_reg_pp0_iter1 );

    SC_METHOD(thread_ap_ready);
    sensitive << ( ap_CS_fsm_state4 );

    SC_METHOD(thread_b_cast_fu_119_p1);
    sensitive << ( b );

    SC_METHOD(thread_conv_fu_115_p1);
    sensitive << ( a );

    SC_METHOD(thread_icmp_ln3_fu_137_p2);
    sensitive << ( i_reg_104 );
    sensitive << ( ap_CS_fsm_pp0_stage0 );
    sensitive << ( ap_block_pp0_stage0_11001 );
    sensitive << ( ap_enable_reg_pp0_iter0 );

    SC_METHOD(thread_in_r_address0);
    sensitive << ( ap_CS_fsm_pp0_stage0 );
    sensitive << ( ap_enable_reg_pp0_iter0 );
    sensitive << ( zext_ln4_fu_149_p1 );
    sensitive << ( ap_block_pp0_stage0 );

    SC_METHOD(thread_in_r_ce0);
    sensitive << ( ap_CS_fsm_pp0_stage0 );
    sensitive << ( ap_block_pp0_stage0_11001 );
    sensitive << ( ap_enable_reg_pp0_iter0 );

    SC_METHOD(thread_mul_ln5_fu_154_p1);
    sensitive << ( conv_reg_165 );
    sensitive << ( ap_CS_fsm_pp0_stage0 );
    sensitive << ( ap_enable_reg_pp0_iter1 );
    sensitive << ( ap_block_pp0_stage0 );

    SC_METHOD(thread_out_r_address0);
    sensitive << ( ap_CS_fsm_pp0_stage0 );
    sensitive << ( zext_ln4_reg_184 );
    sensitive << ( ap_enable_reg_pp0_iter1 );
    sensitive << ( ap_block_pp0_stage0 );

    SC_METHOD(thread_out_r_ce0);
    sensitive << ( ap_CS_fsm_pp0_stage0 );
    sensitive << ( ap_block_pp0_stage0_11001 );
    sensitive << ( ap_enable_reg_pp0_iter1 );

    SC_METHOD(thread_out_r_d0);
    sensitive << ( sext_ln5_reg_170 );
    sensitive << ( ap_CS_fsm_pp0_stage0 );
    sensitive << ( ap_enable_reg_pp0_iter1 );
    sensitive << ( ap_block_pp0_stage0 );
    sensitive << ( mul_ln5_fu_154_p2 );

    SC_METHOD(thread_out_r_we0);
    sensitive << ( icmp_ln3_reg_175 );
    sensitive << ( ap_CS_fsm_pp0_stage0 );
    sensitive << ( ap_block_pp0_stage0_11001 );
    sensitive << ( ap_enable_reg_pp0_iter1 );

    SC_METHOD(thread_sext_ln3_fu_123_p1);
    sensitive << ( c );

    SC_METHOD(thread_sext_ln5_fu_133_p1);
    sensitive << ( add_ln5_fu_127_p2 );

    SC_METHOD(thread_zext_ln4_fu_149_p1);
    sensitive << ( i_reg_104 );

    SC_METHOD(thread_ap_NS_fsm);
    sensitive << ( ap_start );
    sensitive << ( ap_CS_fsm );
    sensitive << ( ap_CS_fsm_state1 );
    sensitive << ( icmp_ln3_fu_137_p2 );
    sensitive << ( ap_enable_reg_pp0_iter0 );
    sensitive << ( ap_block_pp0_stage0_subdone );

    SC_THREAD(thread_hdltv_gen);
    sensitive << ( ap_clk.pos() );

    ap_CS_fsm = "001";
    ap_enable_reg_pp0_iter0 = SC_LOGIC_0;
    ap_enable_reg_pp0_iter1 = SC_LOGIC_0;
    static int apTFileNum = 0;
    stringstream apTFilenSS;
    apTFilenSS << "foo_foo_sc_trace_" << apTFileNum ++;
    string apTFn = apTFilenSS.str();
    mVcdFile = sc_create_vcd_trace_file(apTFn.c_str());
    mVcdFile->set_time_unit(1, SC_PS);
    if (1) {
#ifdef __HLS_TRACE_LEVEL_PORT__
    sc_trace(mVcdFile, ap_clk, "(port)ap_clk");
    sc_trace(mVcdFile, ap_rst, "(port)ap_rst");
    sc_trace(mVcdFile, ap_start, "(port)ap_start");
    sc_trace(mVcdFile, ap_done, "(port)ap_done");
    sc_trace(mVcdFile, ap_idle, "(port)ap_idle");
    sc_trace(mVcdFile, ap_ready, "(port)ap_ready");
    sc_trace(mVcdFile, in_r_address0, "(port)in_r_address0");
    sc_trace(mVcdFile, in_r_ce0, "(port)in_r_ce0");
    sc_trace(mVcdFile, in_r_q0, "(port)in_r_q0");
    sc_trace(mVcdFile, a, "(port)a");
    sc_trace(mVcdFile, b, "(port)b");
    sc_trace(mVcdFile, c, "(port)c");
    sc_trace(mVcdFile, out_r_address0, "(port)out_r_address0");
    sc_trace(mVcdFile, out_r_ce0, "(port)out_r_ce0");
    sc_trace(mVcdFile, out_r_we0, "(port)out_r_we0");
    sc_trace(mVcdFile, out_r_d0, "(port)out_r_d0");
#endif
#ifdef __HLS_TRACE_LEVEL_INT__
    sc_trace(mVcdFile, ap_CS_fsm, "ap_CS_fsm");
    sc_trace(mVcdFile, ap_CS_fsm_state1, "ap_CS_fsm_state1");
    sc_trace(mVcdFile, i_reg_104, "i_reg_104");
    sc_trace(mVcdFile, conv_fu_115_p1, "conv_fu_115_p1");
    sc_trace(mVcdFile, conv_reg_165, "conv_reg_165");
    sc_trace(mVcdFile, sext_ln5_fu_133_p1, "sext_ln5_fu_133_p1");
    sc_trace(mVcdFile, sext_ln5_reg_170, "sext_ln5_reg_170");
    sc_trace(mVcdFile, icmp_ln3_fu_137_p2, "icmp_ln3_fu_137_p2");
    sc_trace(mVcdFile, icmp_ln3_reg_175, "icmp_ln3_reg_175");
    sc_trace(mVcdFile, ap_CS_fsm_pp0_stage0, "ap_CS_fsm_pp0_stage0");
    sc_trace(mVcdFile, ap_block_state2_pp0_stage0_iter0, "ap_block_state2_pp0_stage0_iter0");
    sc_trace(mVcdFile, ap_block_state3_pp0_stage0_iter1, "ap_block_state3_pp0_stage0_iter1");
    sc_trace(mVcdFile, ap_block_pp0_stage0_11001, "ap_block_pp0_stage0_11001");
    sc_trace(mVcdFile, add_ln3_fu_143_p2, "add_ln3_fu_143_p2");
    sc_trace(mVcdFile, ap_enable_reg_pp0_iter0, "ap_enable_reg_pp0_iter0");
    sc_trace(mVcdFile, zext_ln4_fu_149_p1, "zext_ln4_fu_149_p1");
    sc_trace(mVcdFile, zext_ln4_reg_184, "zext_ln4_reg_184");
    sc_trace(mVcdFile, ap_block_pp0_stage0_subdone, "ap_block_pp0_stage0_subdone");
    sc_trace(mVcdFile, ap_condition_pp0_exit_iter0_state2, "ap_condition_pp0_exit_iter0_state2");
    sc_trace(mVcdFile, ap_enable_reg_pp0_iter1, "ap_enable_reg_pp0_iter1");
    sc_trace(mVcdFile, ap_block_pp0_stage0, "ap_block_pp0_stage0");
    sc_trace(mVcdFile, sext_ln3_fu_123_p1, "sext_ln3_fu_123_p1");
    sc_trace(mVcdFile, b_cast_fu_119_p1, "b_cast_fu_119_p1");
    sc_trace(mVcdFile, add_ln5_fu_127_p2, "add_ln5_fu_127_p2");
    sc_trace(mVcdFile, mul_ln5_fu_154_p1, "mul_ln5_fu_154_p1");
    sc_trace(mVcdFile, mul_ln5_fu_154_p2, "mul_ln5_fu_154_p2");
    sc_trace(mVcdFile, ap_CS_fsm_state4, "ap_CS_fsm_state4");
    sc_trace(mVcdFile, ap_NS_fsm, "ap_NS_fsm");
    sc_trace(mVcdFile, ap_idle_pp0, "ap_idle_pp0");
    sc_trace(mVcdFile, ap_enable_pp0, "ap_enable_pp0");
#endif

    }
    mHdltvinHandle.open("foo_foo.hdltvin.dat");
    mHdltvoutHandle.open("foo_foo.hdltvout.dat");
}

foo_foo::~foo_foo() {
    if (mVcdFile) 
        sc_close_vcd_trace_file(mVcdFile);

    mHdltvinHandle << "] " << endl;
    mHdltvoutHandle << "] " << endl;
    mHdltvinHandle.close();
    mHdltvoutHandle.close();
    delete foo_mul_32s_8s_32_1_1_U1;
}

void foo_foo::thread_ap_clk_no_reset_() {
    if ( ap_rst.read() == ap_const_logic_1) {
        ap_CS_fsm = ap_ST_fsm_state1;
    } else {
        ap_CS_fsm = ap_NS_fsm.read();
    }
    if ( ap_rst.read() == ap_const_logic_1) {
        ap_enable_reg_pp0_iter0 = ap_const_logic_0;
    } else {
        if ((esl_seteq<1,1,1>(ap_const_boolean_0, ap_block_pp0_stage0_subdone.read()) && 
             esl_seteq<1,1,1>(ap_const_logic_1, ap_condition_pp0_exit_iter0_state2.read()) && 
             esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_pp0_stage0.read()))) {
            ap_enable_reg_pp0_iter0 = ap_const_logic_0;
        } else if ((esl_seteq<1,1,1>(ap_start.read(), ap_const_logic_1) && 
                    esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_state1.read()))) {
            ap_enable_reg_pp0_iter0 = ap_const_logic_1;
        }
    }
    if ( ap_rst.read() == ap_const_logic_1) {
        ap_enable_reg_pp0_iter1 = ap_const_logic_0;
    } else {
        if ((esl_seteq<1,1,1>(ap_const_boolean_0, ap_block_pp0_stage0_subdone.read()) && 
             esl_seteq<1,1,1>(ap_const_logic_1, ap_condition_pp0_exit_iter0_state2.read()))) {
            ap_enable_reg_pp0_iter1 = (ap_const_logic_1 ^ ap_condition_pp0_exit_iter0_state2.read());
        } else if (esl_seteq<1,1,1>(ap_const_boolean_0, ap_block_pp0_stage0_subdone.read())) {
            ap_enable_reg_pp0_iter1 = ap_enable_reg_pp0_iter0.read();
        } else if ((esl_seteq<1,1,1>(ap_start.read(), ap_const_logic_1) && 
                    esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_state1.read()))) {
            ap_enable_reg_pp0_iter1 = ap_const_logic_0;
        }
    }
    if ((esl_seteq<1,1,1>(ap_start.read(), ap_const_logic_1) && 
         esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_state1.read()))) {
        i_reg_104 = ap_const_lv2_0;
    } else if ((esl_seteq<1,1,1>(ap_enable_reg_pp0_iter0.read(), ap_const_logic_1) && 
                esl_seteq<1,1,1>(ap_const_boolean_0, ap_block_pp0_stage0_11001.read()) && 
                esl_seteq<1,1,1>(icmp_ln3_fu_137_p2.read(), ap_const_lv1_0) && 
                esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_pp0_stage0.read()))) {
        i_reg_104 = add_ln3_fu_143_p2.read();
    }
    if ((esl_seteq<1,1,1>(ap_start.read(), ap_const_logic_1) && esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_state1.read()))) {
        conv_reg_165 = conv_fu_115_p1.read();
        sext_ln5_reg_170 = sext_ln5_fu_133_p1.read();
    }
    if ((esl_seteq<1,1,1>(ap_const_boolean_0, ap_block_pp0_stage0_11001.read()) && esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_pp0_stage0.read()))) {
        icmp_ln3_reg_175 = icmp_ln3_fu_137_p2.read();
    }
    if ((esl_seteq<1,1,1>(ap_const_boolean_0, ap_block_pp0_stage0_11001.read()) && esl_seteq<1,1,1>(icmp_ln3_fu_137_p2.read(), ap_const_lv1_0) && esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_pp0_stage0.read()))) {
        zext_ln4_reg_184 = zext_ln4_fu_149_p1.read();
    }
}

void foo_foo::thread_add_ln3_fu_143_p2() {
    add_ln3_fu_143_p2 = (!i_reg_104.read().is_01() || !ap_const_lv2_1.is_01())? sc_lv<2>(): (sc_biguint<2>(i_reg_104.read()) + sc_biguint<2>(ap_const_lv2_1));
}

void foo_foo::thread_add_ln5_fu_127_p2() {
    add_ln5_fu_127_p2 = (!sext_ln3_fu_123_p1.read().is_01() || !b_cast_fu_119_p1.read().is_01())? sc_lv<9>(): (sc_bigint<9>(sext_ln3_fu_123_p1.read()) + sc_bigint<9>(b_cast_fu_119_p1.read()));
}

void foo_foo::thread_ap_CS_fsm_pp0_stage0() {
    ap_CS_fsm_pp0_stage0 = ap_CS_fsm.read()[1];
}

void foo_foo::thread_ap_CS_fsm_state1() {
    ap_CS_fsm_state1 = ap_CS_fsm.read()[0];
}

void foo_foo::thread_ap_CS_fsm_state4() {
    ap_CS_fsm_state4 = ap_CS_fsm.read()[2];
}

void foo_foo::thread_ap_block_pp0_stage0() {
    ap_block_pp0_stage0 = !esl_seteq<1,1,1>(ap_const_boolean_1, ap_const_boolean_1);
}

void foo_foo::thread_ap_block_pp0_stage0_11001() {
    ap_block_pp0_stage0_11001 = !esl_seteq<1,1,1>(ap_const_boolean_1, ap_const_boolean_1);
}

void foo_foo::thread_ap_block_pp0_stage0_subdone() {
    ap_block_pp0_stage0_subdone = !esl_seteq<1,1,1>(ap_const_boolean_1, ap_const_boolean_1);
}

void foo_foo::thread_ap_block_state2_pp0_stage0_iter0() {
    ap_block_state2_pp0_stage0_iter0 = !esl_seteq<1,1,1>(ap_const_boolean_1, ap_const_boolean_1);
}

void foo_foo::thread_ap_block_state3_pp0_stage0_iter1() {
    ap_block_state3_pp0_stage0_iter1 = !esl_seteq<1,1,1>(ap_const_boolean_1, ap_const_boolean_1);
}

void foo_foo::thread_ap_condition_pp0_exit_iter0_state2() {
    if (esl_seteq<1,1,1>(icmp_ln3_fu_137_p2.read(), ap_const_lv1_1)) {
        ap_condition_pp0_exit_iter0_state2 = ap_const_logic_1;
    } else {
        ap_condition_pp0_exit_iter0_state2 = ap_const_logic_0;
    }
}

void foo_foo::thread_ap_done() {
    if (esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_state4.read())) {
        ap_done = ap_const_logic_1;
    } else {
        ap_done = ap_const_logic_0;
    }
}

void foo_foo::thread_ap_enable_pp0() {
    ap_enable_pp0 = (ap_idle_pp0.read() ^ ap_const_logic_1);
}

void foo_foo::thread_ap_idle() {
    if ((esl_seteq<1,1,1>(ap_start.read(), ap_const_logic_0) && 
         esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_state1.read()))) {
        ap_idle = ap_const_logic_1;
    } else {
        ap_idle = ap_const_logic_0;
    }
}

void foo_foo::thread_ap_idle_pp0() {
    if ((esl_seteq<1,1,1>(ap_enable_reg_pp0_iter1.read(), ap_const_logic_0) && 
         esl_seteq<1,1,1>(ap_enable_reg_pp0_iter0.read(), ap_const_logic_0))) {
        ap_idle_pp0 = ap_const_logic_1;
    } else {
        ap_idle_pp0 = ap_const_logic_0;
    }
}

void foo_foo::thread_ap_ready() {
    if (esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_state4.read())) {
        ap_ready = ap_const_logic_1;
    } else {
        ap_ready = ap_const_logic_0;
    }
}

void foo_foo::thread_b_cast_fu_119_p1() {
    b_cast_fu_119_p1 = esl_sext<9,8>(b.read());
}

void foo_foo::thread_conv_fu_115_p1() {
    conv_fu_115_p1 = esl_sext<32,8>(a.read());
}

void foo_foo::thread_icmp_ln3_fu_137_p2() {
    icmp_ln3_fu_137_p2 = (!i_reg_104.read().is_01() || !ap_const_lv2_3.is_01())? sc_lv<1>(): sc_lv<1>(i_reg_104.read() == ap_const_lv2_3);
}

void foo_foo::thread_in_r_address0() {
    in_r_address0 =  (sc_lv<2>) (zext_ln4_fu_149_p1.read());
}

void foo_foo::thread_in_r_ce0() {
    if ((esl_seteq<1,1,1>(ap_enable_reg_pp0_iter0.read(), ap_const_logic_1) && 
         esl_seteq<1,1,1>(ap_const_boolean_0, ap_block_pp0_stage0_11001.read()) && 
         esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_pp0_stage0.read()))) {
        in_r_ce0 = ap_const_logic_1;
    } else {
        in_r_ce0 = ap_const_logic_0;
    }
}

void foo_foo::thread_mul_ln5_fu_154_p1() {
    mul_ln5_fu_154_p1 =  (sc_lv<8>) (conv_reg_165.read());
}

void foo_foo::thread_out_r_address0() {
    out_r_address0 =  (sc_lv<2>) (zext_ln4_reg_184.read());
}

void foo_foo::thread_out_r_ce0() {
    if ((esl_seteq<1,1,1>(ap_enable_reg_pp0_iter1.read(), ap_const_logic_1) && 
         esl_seteq<1,1,1>(ap_const_boolean_0, ap_block_pp0_stage0_11001.read()) && 
         esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_pp0_stage0.read()))) {
        out_r_ce0 = ap_const_logic_1;
    } else {
        out_r_ce0 = ap_const_logic_0;
    }
}

void foo_foo::thread_out_r_d0() {
    out_r_d0 = (!sext_ln5_reg_170.read().is_01() || !mul_ln5_fu_154_p2.read().is_01())? sc_lv<32>(): (sc_bigint<32>(sext_ln5_reg_170.read()) + sc_biguint<32>(mul_ln5_fu_154_p2.read()));
}

void foo_foo::thread_out_r_we0() {
    if ((esl_seteq<1,1,1>(ap_enable_reg_pp0_iter1.read(), ap_const_logic_1) && 
         esl_seteq<1,1,1>(ap_const_boolean_0, ap_block_pp0_stage0_11001.read()) && 
         esl_seteq<1,1,1>(icmp_ln3_reg_175.read(), ap_const_lv1_0) && 
         esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_pp0_stage0.read()))) {
        out_r_we0 = ap_const_logic_1;
    } else {
        out_r_we0 = ap_const_logic_0;
    }
}

void foo_foo::thread_sext_ln3_fu_123_p1() {
    sext_ln3_fu_123_p1 = esl_sext<9,8>(c.read());
}

void foo_foo::thread_sext_ln5_fu_133_p1() {
    sext_ln5_fu_133_p1 = esl_sext<32,9>(add_ln5_fu_127_p2.read());
}

void foo_foo::thread_zext_ln4_fu_149_p1() {
    zext_ln4_fu_149_p1 = esl_zext<64,2>(i_reg_104.read());
}

void foo_foo::thread_ap_NS_fsm() {
    switch (ap_CS_fsm.read().to_uint64()) {
        case 1 : 
            if ((esl_seteq<1,1,1>(ap_start.read(), ap_const_logic_1) && esl_seteq<1,1,1>(ap_const_logic_1, ap_CS_fsm_state1.read()))) {
                ap_NS_fsm = ap_ST_fsm_pp0_stage0;
            } else {
                ap_NS_fsm = ap_ST_fsm_state1;
            }
            break;
        case 2 : 
            if (!(esl_seteq<1,1,1>(ap_enable_reg_pp0_iter0.read(), ap_const_logic_1) && esl_seteq<1,1,1>(ap_const_boolean_0, ap_block_pp0_stage0_subdone.read()) && esl_seteq<1,1,1>(icmp_ln3_fu_137_p2.read(), ap_const_lv1_1))) {
                ap_NS_fsm = ap_ST_fsm_pp0_stage0;
            } else if ((esl_seteq<1,1,1>(ap_enable_reg_pp0_iter0.read(), ap_const_logic_1) && esl_seteq<1,1,1>(ap_const_boolean_0, ap_block_pp0_stage0_subdone.read()) && esl_seteq<1,1,1>(icmp_ln3_fu_137_p2.read(), ap_const_lv1_1))) {
                ap_NS_fsm = ap_ST_fsm_state4;
            } else {
                ap_NS_fsm = ap_ST_fsm_pp0_stage0;
            }
            break;
        case 4 : 
            ap_NS_fsm = ap_ST_fsm_state1;
            break;
        default : 
            ap_NS_fsm = "XXX";
            break;
    }
}

void foo_foo::thread_hdltv_gen() {
    const char* dump_tv = std::getenv("AP_WRITE_TV");
    if (!(dump_tv && string(dump_tv) == "on")) return;

    wait();

    mHdltvinHandle << "[ " << endl;
    mHdltvoutHandle << "[ " << endl;
    int ap_cycleNo = 0;
    while (1) {
        wait();
        const char* mComma = ap_cycleNo == 0 ? " " : ", " ;
        mHdltvinHandle << mComma << "{"  <<  " \"ap_rst\" :  \"" << ap_rst.read() << "\" ";
        mHdltvinHandle << " , " <<  " \"ap_start\" :  \"" << ap_start.read() << "\" ";
        mHdltvoutHandle << mComma << "{"  <<  " \"ap_done\" :  \"" << ap_done.read() << "\" ";
        mHdltvoutHandle << " , " <<  " \"ap_idle\" :  \"" << ap_idle.read() << "\" ";
        mHdltvoutHandle << " , " <<  " \"ap_ready\" :  \"" << ap_ready.read() << "\" ";
        mHdltvoutHandle << " , " <<  " \"in_r_address0\" :  \"" << in_r_address0.read() << "\" ";
        mHdltvoutHandle << " , " <<  " \"in_r_ce0\" :  \"" << in_r_ce0.read() << "\" ";
        mHdltvinHandle << " , " <<  " \"in_r_q0\" :  \"" << in_r_q0.read() << "\" ";
        mHdltvinHandle << " , " <<  " \"a\" :  \"" << a.read() << "\" ";
        mHdltvinHandle << " , " <<  " \"b\" :  \"" << b.read() << "\" ";
        mHdltvinHandle << " , " <<  " \"c\" :  \"" << c.read() << "\" ";
        mHdltvoutHandle << " , " <<  " \"out_r_address0\" :  \"" << out_r_address0.read() << "\" ";
        mHdltvoutHandle << " , " <<  " \"out_r_ce0\" :  \"" << out_r_ce0.read() << "\" ";
        mHdltvoutHandle << " , " <<  " \"out_r_we0\" :  \"" << out_r_we0.read() << "\" ";
        mHdltvoutHandle << " , " <<  " \"out_r_d0\" :  \"" << out_r_d0.read() << "\" ";
        mHdltvinHandle << "}" << std::endl;
        mHdltvoutHandle << "}" << std::endl;
        ap_cycleNo++;
    }
}

}
