
#include "draw.h"


bool draw(mtrControl_t *control, std::string gcodeFileName){

    if (gcodeFileName.empty()) {
        std::cout << "No gcode specified.";

    } else {
        std::cout << "We are going to cycle through the gcode\n";
    
        std::ifstream file(gcodeFileName);
        std::string gcodeContents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

        gpr::gcode_program program = gpr::parse_gcode(gcodeContents);

        std::cout << program << std::endl;
    }

    return true;
}