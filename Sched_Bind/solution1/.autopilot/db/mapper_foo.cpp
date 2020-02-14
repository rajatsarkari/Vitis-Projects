#include <systemc>
#include <vector>
#include <iostream>
#include "hls_stream.h"
#include "ap_int.h"
#include "ap_fixed.h"
using namespace std;
using namespace sc_dt;
class AESL_RUNTIME_BC {
  public:
    AESL_RUNTIME_BC(const char* name) {
      file_token.open( name);
      if (!file_token.good()) {
        cout << "Failed to open tv file " << name << endl;
        exit (1);
      }
      file_token >> mName;//[[[runtime]]]
    }
    ~AESL_RUNTIME_BC() {
      file_token.close();
    }
    int read_size () {
      int size = 0;
      file_token >> mName;//[[transaction]]
      file_token >> mName;//transaction number
      file_token >> mName;//pop_size
      size = atoi(mName.c_str());
      file_token >> mName;//[[/transaction]]
      return size;
    }
  public:
    fstream file_token;
    string mName;
};
extern "C" int foo(char, char, char, char);
extern "C" int apatb_foo_hw(char __xlx_apatb_param_x, char __xlx_apatb_param_a, char __xlx_apatb_param_b, char __xlx_apatb_param_c) {
  // DUT call
  int ap_return = foo(__xlx_apatb_param_x, __xlx_apatb_param_a, __xlx_apatb_param_b, __xlx_apatb_param_c);
return ap_return;
}
