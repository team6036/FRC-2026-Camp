package frc.robot;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.io.File;
import java.io.IOException;


import edu.wpi.first.util.RuntimeLoader;

import java.util.concurrent.atomic.AtomicBoolean;

public class JniInterface {

      static boolean libraryLoaded = false;
  static RuntimeLoader<JniInterface> loader = null;

  /**
   * Helper class for determining whether or not to load the driver on static initialization.
   */
  public static class Helper {
    private static AtomicBoolean extractOnStaticLoad = new AtomicBoolean(true);

    /**
     * Get whether to load the driver on static init.
     * @return true if the driver will load on static init
     */
    public static boolean getExtractOnStaticLoad() {
      return extractOnStaticLoad.get();
    }

    /**
     * Set whether to load the driver on static init.
     * @param load the new value
     */
    public static void setExtractOnStaticLoad(boolean load) {
      extractOnStaticLoad.set(load);
    }
  }

  /**
   * Force load the library.
   * @throws java.io.IOException thrown if the native library cannot be found
   */
  public static synchronized void forceLoad() throws IOException {
    if (libraryLoaded) {
      return;
    }
    loader = new RuntimeLoader<>("native", "/home/lvuser/deploy", JniInterface.class);
    loader.loadLibrary();
    libraryLoaded = true;
  }

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
        // File test = new File(Filesystem.getDeployDirectory(), "libnative.so");
        // System.load(test.getAbsolutePath());

        // System.loadLibrary("native");

        if (Helper.getExtractOnStaticLoad()) {
            try {
                loader = new RuntimeLoader<>("native", "/home/lvuser/deploy", JniInterface.class);
                loader.loadLibrary();
            } catch (IOException ex) {
                ex.printStackTrace();
                System.exit(1);
            }
            libraryLoaded = true;
        }
        
    }

    public native void sayHello();
}
