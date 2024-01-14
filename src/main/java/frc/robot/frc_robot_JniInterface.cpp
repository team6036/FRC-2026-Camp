#include "frc_robot_JniInterface.h"
#include <iostream>
#include <stdio.h>

// export JAVA_HOME=/Library/Java/JavaVirtualMachines/temurin-17.jdk/Contents/Home
// export JAVA_HOME=/Users/yiyunzhu/wpilib/2024/jdk
// g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/darwin -std=c++11 frc_robot_JniInterface.cpp -o frc_robot_JniInterface.o 
// g++ -shared -fPIC -o libnative.so [cpp filename].o -lc

// https://gist.github.com/disposedtrolley/06d37e1db82b80ccf8c5d801eaa29373
// /Applications/ArmGNUToolchain/13.2.Rel1/arm-none-eabi/bin
// export PATH=$PATH:/Applications/ArmGNUToolchain/13.2.Rel1/arm-none-eabi/bin
// arm-none-eabi-g++ -c -fPIC -march=armv7-a -I${JAVA_HOME}/include -I${JAVA_HOME}/include/darwin -std=c++11 frc_robot_JniInterface.cpp -o frc_robot_JniInterface.o
// arm-none-eabi-g++ -shared -fPIC -o libnative.so frc_robot_JniInterface.o -lc

//https://www.chiefdelphi.com/t/guide-to-using-jni-to-make-custom-libraries/347930
//https://github.com/wpilibsuite/native-utils

//https://nilrt-docs.ni.com/cross_compile/introduction.html


JNIEXPORT void JNICALL Java_frc_robot_JniInterface_sayHello
  (JNIEnv * env, jobject thisObject){
    std::cout << "hello" << std::endl;
  }