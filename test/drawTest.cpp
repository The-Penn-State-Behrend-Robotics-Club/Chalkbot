#include "draw.h"

#include <string>
#include <fstream>
#include <streambuf>
#include <mutex>

using namespace gpr;
using namespace std;

// This example program shows how to create your own blocks and print them out,
// and how to use the parser. To create a custom block and print it out just type:
//     ./parse-gcode
// at the command line
// To parse a G-code file type:
//     ./parse-gcode <path-to-gcode-file>
int main(int argc, char** argv) {
  
  mtrControl_t ctrl(3);

  if(argc == 1) return -1;
  else draw(&ctrl, argv[1]);

  return 0;
}
