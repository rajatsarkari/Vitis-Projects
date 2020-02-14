#include <systemc>
#include <iostream>
#include <cstdlib>
#include <cstddef>
#include <stdint.h>
#include "SysCFileHandler.h"
#include "ap_int.h"
#include "ap_fixed.h"
#include <complex>
#include <stdbool.h>
#include "autopilot_cbe.h"
#include "hls_stream.h"
#include "hls_half.h"
#include "hls_signal_handler.h"

using namespace std;
using namespace sc_core;
using namespace sc_dt;

// wrapc file define:
#define AUTOTB_TVIN_in_r "../tv/cdatafile/c.foo_foo.autotvin_in_r.dat"
#define AUTOTB_TVOUT_in_r "../tv/cdatafile/c.foo_foo.autotvout_in_r.dat"
// wrapc file define:
#define AUTOTB_TVIN_a "../tv/cdatafile/c.foo_foo.autotvin_a.dat"
#define AUTOTB_TVOUT_a "../tv/cdatafile/c.foo_foo.autotvout_a.dat"
// wrapc file define:
#define AUTOTB_TVIN_b "../tv/cdatafile/c.foo_foo.autotvin_b.dat"
#define AUTOTB_TVOUT_b "../tv/cdatafile/c.foo_foo.autotvout_b.dat"
// wrapc file define:
#define AUTOTB_TVIN_c "../tv/cdatafile/c.foo_foo.autotvin_c.dat"
#define AUTOTB_TVOUT_c "../tv/cdatafile/c.foo_foo.autotvout_c.dat"
// wrapc file define:
#define AUTOTB_TVIN_out_r "../tv/cdatafile/c.foo_foo.autotvin_out_r.dat"
#define AUTOTB_TVOUT_out_r "../tv/cdatafile/c.foo_foo.autotvout_out_r.dat"

#define INTER_TCL "../tv/cdatafile/ref.tcl"

// tvout file define:
#define AUTOTB_TVOUT_PC_in_r "../tv/rtldatafile/rtl.foo_foo.autotvout_in_r.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_a "../tv/rtldatafile/rtl.foo_foo.autotvout_a.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_b "../tv/rtldatafile/rtl.foo_foo.autotvout_b.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_c "../tv/rtldatafile/rtl.foo_foo.autotvout_c.dat"
// tvout file define:
#define AUTOTB_TVOUT_PC_out_r "../tv/rtldatafile/rtl.foo_foo.autotvout_out_r.dat"

class INTER_TCL_FILE {
  public:
INTER_TCL_FILE(const char* name) {
  mName = name; 
  in_r_depth = 0;
  a_depth = 0;
  b_depth = 0;
  c_depth = 0;
  out_r_depth = 0;
  trans_num =0;
}
~INTER_TCL_FILE() {
  mFile.open(mName);
  if (!mFile.good()) {
    cout << "Failed to open file ref.tcl" << endl;
    exit (1); 
  }
  string total_list = get_depth_list();
  mFile << "set depth_list {\n";
  mFile << total_list;
  mFile << "}\n";
  mFile << "set trans_num "<<trans_num<<endl;
  mFile.close();
}
string get_depth_list () {
  stringstream total_list;
  total_list << "{in_r " << in_r_depth << "}\n";
  total_list << "{a " << a_depth << "}\n";
  total_list << "{b " << b_depth << "}\n";
  total_list << "{c " << c_depth << "}\n";
  total_list << "{out_r " << out_r_depth << "}\n";
  return total_list.str();
}
void set_num (int num , int* class_num) {
  (*class_num) = (*class_num) > num ? (*class_num) : num;
}
  public:
    int in_r_depth;
    int a_depth;
    int b_depth;
    int c_depth;
    int out_r_depth;
    int trans_num;
  private:
    ofstream mFile;
    const char* mName;
};

static void RTLOutputCheckAndReplacement(std::string &AESL_token, std::string PortName) {
  bool no_x = false;
  bool err = false;

  no_x = false;
  // search and replace 'X' with '0' from the 3rd char of token
  while (!no_x) {
    size_t x_found = AESL_token.find('X', 0);
    if (x_found != string::npos) {
      if (!err) { 
        cerr << "WARNING: [SIM 212-201] RTL produces unknown value 'X' on port" 
             << PortName << ", possible cause: There are uninitialized variables in the C design."
             << endl; 
        err = true;
      }
      AESL_token.replace(x_found, 1, "0");
    } else
      no_x = true;
  }
  no_x = false;
  // search and replace 'x' with '0' from the 3rd char of token
  while (!no_x) {
    size_t x_found = AESL_token.find('x', 2);
    if (x_found != string::npos) {
      if (!err) { 
        cerr << "WARNING: [SIM 212-201] RTL produces unknown value 'x' on port" 
             << PortName << ", possible cause: There are uninitialized variables in the C design."
             << endl; 
        err = true;
      }
      AESL_token.replace(x_found, 1, "0");
    } else
      no_x = true;
  }
}
extern "C" void foo_hw_stub(volatile void *, char, char, char, volatile void *);

extern "C" void apatb_foo_hw(volatile void * __xlx_apatb_param_in_r, char __xlx_apatb_param_a, char __xlx_apatb_param_b, char __xlx_apatb_param_c, volatile void * __xlx_apatb_param_out_r) {
  refine_signal_handler();
  fstream wrapc_switch_file_token;
  wrapc_switch_file_token.open(".hls_cosim_wrapc_switch.log");
  int AESL_i;
  if (wrapc_switch_file_token.good())
  {

    CodeState = ENTER_WRAPC_PC;
    static unsigned AESL_transaction_pc = 0;
    string AESL_token;
    string AESL_num;{
      static ifstream rtl_tv_out_file;
      if (!rtl_tv_out_file.is_open()) {
        rtl_tv_out_file.open(AUTOTB_TVOUT_PC_out_r);
        if (rtl_tv_out_file.good()) {
          rtl_tv_out_file >> AESL_token;
          if (AESL_token != "[[[runtime]]]")
            exit(1);
        }
      }
  
      if (rtl_tv_out_file.good()) {
        rtl_tv_out_file >> AESL_token; 
        rtl_tv_out_file >> AESL_num;  // transaction number
        if (AESL_token != "[[transaction]]") {
          cerr << "Unexpected token: " << AESL_token << endl;
          exit(1);
        }
        if (atoi(AESL_num.c_str()) == AESL_transaction_pc) {
          std::vector<sc_bv<32> > out_r_pc_buffer(3);
          int i = 0;

          rtl_tv_out_file >> AESL_token; //data
          while (AESL_token != "[[/transaction]]"){

            RTLOutputCheckAndReplacement(AESL_token, "out_r");
  
            // push token into output port buffer
            if (AESL_token != "") {
              out_r_pc_buffer[i] = AESL_token.c_str();;
              i++;
            }
  
            rtl_tv_out_file >> AESL_token; //data or [[/transaction]]
            if (AESL_token == "[[[/runtime]]]" || rtl_tv_out_file.eof())
              exit(1);
          }
          if (i > 0) {{
            int i = 0;
            for (int j = 0, e = 3; j != e; ++j, ++i) {
            ((int*)__xlx_apatb_param_out_r)[j] = out_r_pc_buffer[i].to_int64();
          }}}
        } // end transaction
      } // end file is good
    } // end post check logic bolck
  
    AESL_transaction_pc++;
    return ;
  }
static unsigned AESL_transaction;
static AESL_FILE_HANDLER aesl_fh;
static INTER_TCL_FILE tcl_file(INTER_TCL);
std::vector<char> __xlx_sprintf_buffer(1024);
CodeState = ENTER_WRAPC;
//in_r
aesl_fh.touch(AUTOTB_TVIN_in_r);
aesl_fh.touch(AUTOTB_TVOUT_in_r);
//a
aesl_fh.touch(AUTOTB_TVIN_a);
aesl_fh.touch(AUTOTB_TVOUT_a);
//b
aesl_fh.touch(AUTOTB_TVIN_b);
aesl_fh.touch(AUTOTB_TVOUT_b);
//c
aesl_fh.touch(AUTOTB_TVIN_c);
aesl_fh.touch(AUTOTB_TVOUT_c);
//out_r
aesl_fh.touch(AUTOTB_TVIN_out_r);
aesl_fh.touch(AUTOTB_TVOUT_out_r);
CodeState = DUMP_INPUTS;
// print in_r Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_in_r, __xlx_sprintf_buffer.data());
  {{for (int j = 0, e = 3; j != e; ++j) {
sc_bv<32> __xlx_tmp_lv = ((int*)__xlx_apatb_param_in_r)[j];

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_in_r, __xlx_sprintf_buffer.data()); 
  }
}}
  tcl_file.set_num(3, &tcl_file.in_r_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_in_r, __xlx_sprintf_buffer.data());
}
// print a Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_a, __xlx_sprintf_buffer.data());
  {
    sc_bv<8> __xlx_tmp_lv = *((char*)&__xlx_apatb_param_a);
    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_a, __xlx_sprintf_buffer.data()); 
  }
  tcl_file.set_num(1, &tcl_file.a_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_a, __xlx_sprintf_buffer.data());
}
// print b Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_b, __xlx_sprintf_buffer.data());
  {
    sc_bv<8> __xlx_tmp_lv = *((char*)&__xlx_apatb_param_b);
    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_b, __xlx_sprintf_buffer.data()); 
  }
  tcl_file.set_num(1, &tcl_file.b_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_b, __xlx_sprintf_buffer.data());
}
// print c Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_c, __xlx_sprintf_buffer.data());
  {
    sc_bv<8> __xlx_tmp_lv = *((char*)&__xlx_apatb_param_c);
    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_c, __xlx_sprintf_buffer.data()); 
  }
  tcl_file.set_num(1, &tcl_file.c_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_c, __xlx_sprintf_buffer.data());
}
// print out_r Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVIN_out_r, __xlx_sprintf_buffer.data());
  {{for (int j = 0, e = 3; j != e; ++j) {
sc_bv<32> __xlx_tmp_lv = ((int*)__xlx_apatb_param_out_r)[j];

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVIN_out_r, __xlx_sprintf_buffer.data()); 
  }
}}
  tcl_file.set_num(3, &tcl_file.out_r_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVIN_out_r, __xlx_sprintf_buffer.data());
}
CodeState = CALL_C_DUT;
foo_hw_stub(__xlx_apatb_param_in_r, __xlx_apatb_param_a, __xlx_apatb_param_b, __xlx_apatb_param_c, __xlx_apatb_param_out_r);
CodeState = DUMP_OUTPUTS;
// print out_r Transactions
{
  sprintf(__xlx_sprintf_buffer.data(), "[[transaction]] %d\n", AESL_transaction);
  aesl_fh.write(AUTOTB_TVOUT_out_r, __xlx_sprintf_buffer.data());
  {{for (int j = 0, e = 3; j != e; ++j) {
sc_bv<32> __xlx_tmp_lv = ((int*)__xlx_apatb_param_out_r)[j];

    sprintf(__xlx_sprintf_buffer.data(), "%s\n", __xlx_tmp_lv.to_string(SC_HEX).c_str());
    aesl_fh.write(AUTOTB_TVOUT_out_r, __xlx_sprintf_buffer.data()); 
  }
}}
  tcl_file.set_num(3, &tcl_file.out_r_depth);
  sprintf(__xlx_sprintf_buffer.data(), "[[/transaction]] \n");
  aesl_fh.write(AUTOTB_TVOUT_out_r, __xlx_sprintf_buffer.data());
}
CodeState = DELETE_CHAR_BUFFERS;
AESL_transaction++;
tcl_file.set_num(AESL_transaction , &tcl_file.trans_num);
}
