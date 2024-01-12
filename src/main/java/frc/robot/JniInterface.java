package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

public class JniInterface {

    //https://stackoverflow.com/questions/2937406/how-to-bundle-a-native-library-and-a-jni-library-inside-a-jar
    //
    static {
        // String relative = "src/main/deploy/libnative.so";
        // Path absolutePath = Paths.get(relative).toAbsolutePath();
        // String absolutePathString = absolutePath.toString();

        // String relative = "/libnative.so";
        // System.load(Filesystem.getDeployDirectory().getAbsolutePath().concat(relative));

        // System.load("/home/lvuser/deploy/libnative.so");
        // String relative = "lib/libnative.so";
        // Path absolutePath = Paths.get(relative).toAbsolutePath();
        // String absolutePathString = absolutePath.toString();
        // System.load(absolutePathString);

        // File test = new File(Filesystem.getDeployDirectory(), "libnative.so");
        // System.out.println(test.getAbsolutePath());
        File test = new File(Filesystem.getDeployDirectory(), "libnative.so");
        System.load(test.getAbsolutePath());
        
    }

    public native void sayHello();
}
