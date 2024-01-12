#include "frc_robot_JniInterface.h"
#include <iostream>

// export JAVA_HOME=/Library/Java/JavaVirtualMachines/temurin-17.jdk/Contents/Home
// g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/darwin -std=c++11 [cpp filename].cpp -o [cpp filename].o 
// g++ -shared -fPIC -o libnative.so [cpp filename].o -lc

JNIEXPORT void JNICALL Java_frc_robot_JniInterface_sayHello
  (JNIEnv * env, jobject thisObject){
    std::cout << "hello" << std::endl;
  }