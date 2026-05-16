package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.RobotConstants;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

/** Logger class that logs to EpilogueBackend manually. */
@SuppressWarnings({"unchecked", "rawtypes"})
public class Logger extends Thread {
  private static final Map<String, Publisher> entries = new HashMap<>();
  private static final Map<String, StructPublisher> structEntries = new HashMap<>();
  private static final Map<String, StructArrayPublisher> structArrayEntries = new HashMap<>();

  static {
    DataLogManager.start(); // Automatically hijacks NT stream and logs under NT:{path}
    DriverStation.startDataLog(DataLogManager.getLog()); // Allows DS to report joysticks/other info
  }

  // Taken from DogLog
  private static final Map<Class<?>, Struct<?>> resolvedStructs = new HashMap<>();

  private static Struct<?> getStruct(Class<?> entryClass) {
    if (resolvedStructs.containsKey(entryClass)) {
      return resolvedStructs.get(entryClass);
    }
    var struct = getStructRaw(entryClass);
    resolvedStructs.put(entryClass, struct);
    return struct;
  }

  private static Struct<?> getStructRaw(Class<?> classObj) {
    try {
      var field = classObj.getDeclaredField("struct");
      return (Struct<?>) field.get(null);
    } catch (Exception e) {
      return null;
    }
  }

  private static class Entry {
    public final String path;
    public final long timestamp;

    public Entry(String path) {
      this.path = path;
      this.timestamp = WPIUtilJNI.now();
    }
  }

  private static class DataEntry extends Entry {
    private enum Type {
      LONG("int64"),
      LONG_A("int64[]"),
      FLOAT("float"),
      FLOAT_A("float[]"),
      DOUBLE("double"),
      DOUBLE_A("double[]"),
      BOOL("boolean"),
      BOOL_A("boolean[]"),
      STR("string"),
      STR_A("string[]");

      public final String datalogType;

      Type(String datalogType) {
        this.datalogType = datalogType;
      }
    }

    public final Type type;
    public final Object value;

    public DataEntry(String path, Type type, Object value) {
      super(path);

      this.type = type;
      this.value = value;
    }
  }

  private static class StructEntry extends Entry {
    public final StructSerializable value;

    public StructEntry(String path, StructSerializable value) {
      super(path);

      this.value = value;
    }
  }

  private static class StructArrayEntry extends Entry {
    public final StructSerializable[] value;

    public StructArrayEntry(String path, StructSerializable[] value) {
      super(path);

      this.value = value;
    }
  }

  private static final BlockingQueue<Entry> queue = new LinkedBlockingQueue<>(2000);

  private static void log(Entry entry) {
    queue.offer(entry);
  }

  public static void log(String path, int value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new DataEntry(path, DataEntry.Type.LONG, (long) value));
  }

  public static void log(String path, int[] value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    long[] valueLongs = new long[value.length];
    for (int i = 0; i < value.length; i++) {
      valueLongs[i] = value[i];
    }
    log(new DataEntry(path, DataEntry.Type.LONG_A, valueLongs));
  }

  public static void log(String path, long value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new DataEntry(path, DataEntry.Type.LONG, value));
  }

  public static void log(String path, long[] value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new DataEntry(path, DataEntry.Type.LONG_A, value));
  }

  public static void log(String path, float value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new DataEntry(path, DataEntry.Type.FLOAT, value));
  }

  public static void log(String path, float[] value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new DataEntry(path, DataEntry.Type.FLOAT_A, value));
  }

  public static void log(String path, double value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new DataEntry(path, DataEntry.Type.DOUBLE, value));
  }

  public static void log(String path, double[] value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new DataEntry(path, DataEntry.Type.DOUBLE_A, value));
  }

  public static void log(String path, boolean value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new DataEntry(path, DataEntry.Type.BOOL, value));
  }

  public static void log(String path, boolean[] value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new DataEntry(path, DataEntry.Type.BOOL_A, value));
  }

  public static void log(String path, String value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new DataEntry(path, DataEntry.Type.STR, value));
  }

  public static void log(String path, String[] value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new DataEntry(path, DataEntry.Type.STR_A, value));
  }

  public static void log(String path, Translation2d value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new StructEntry(path, value));
  }

  public static void log(String path, Rotation2d value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new StructEntry(path, value));
  }

  public static void log(String path, Pose2d value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new StructEntry(path, value));
  }

  public static void log(String path, Translation3d value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new StructEntry(path, value));
  }

  public static void log(String path, Rotation3d value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new StructEntry(path, value));
  }

  public static void log(String path, Pose3d value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new StructEntry(path, value));
  }

  public static void log(String path, Enum<?> value) {
    log(path, value == null ? "null" : value.name());
  }

  public static <T extends StructSerializable> void log(String path, T value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new StructEntry(path, value));
  }

  public static <T extends StructSerializable> void log(String path, T[] value) {
    if (!RobotConstants.enableLogging) {
      return;
    }
    log(new StructArrayEntry(path, value));
  }

  public static void init() {
    if (!RobotConstants.enableLogging) {
      return;
    }
    new Logger().start();
  }

  private Logger() {
    super("logger-thread");
  }

  @Override
  public void run() {
    while (!interrupted()) {
      try {
        Entry entry = queue.take();
        if (entry instanceof DataEntry dataEntry) {
          dumpData(dataEntry);
        } else if (entry instanceof StructEntry structEntry) {
          dumpStruct(structEntry);
        } else if (entry instanceof StructArrayEntry structArrayEntry) {
          dumpStructArray(structArrayEntry);
        }
      } catch (InterruptedException e) {
        interrupt();
      }
    }
  }

  private void dumpData(DataEntry entry) {
    String key = "/" + entry.path;
    var publisher = entries.get(key);
    if (publisher == null) {
      publisher =
          switch (entry.type) {
            case LONG -> NetworkTableInstance.getDefault().getIntegerTopic(key).publish();
            case LONG_A -> NetworkTableInstance.getDefault().getIntegerArrayTopic(key).publish();
            case FLOAT -> NetworkTableInstance.getDefault().getFloatTopic(key).publish();
            case FLOAT_A -> NetworkTableInstance.getDefault().getFloatArrayTopic(key).publish();
            case DOUBLE -> NetworkTableInstance.getDefault().getDoubleTopic(key).publish();
            case DOUBLE_A -> NetworkTableInstance.getDefault().getDoubleArrayTopic(key).publish();
            case BOOL -> NetworkTableInstance.getDefault().getBooleanTopic(key).publish();
            case BOOL_A -> NetworkTableInstance.getDefault().getBooleanArrayTopic(key).publish();
            case STR -> NetworkTableInstance.getDefault().getStringTopic(key).publish();
            case STR_A -> NetworkTableInstance.getDefault().getStringArrayTopic(key).publish();
          };
      entries.put(key, publisher);
    }
    switch (entry.type) {
      case LONG -> ((IntegerPublisher) publisher).set((long) entry.value, entry.timestamp);
      case LONG_A -> ((IntegerArrayPublisher) publisher).set((long[]) entry.value, entry.timestamp);
      case FLOAT -> ((FloatPublisher) publisher).set((float) entry.value, entry.timestamp);
      case FLOAT_A -> ((FloatArrayPublisher) publisher).set((float[]) entry.value, entry.timestamp);
      case DOUBLE -> ((DoublePublisher) publisher).set((double) entry.value, entry.timestamp);
      case DOUBLE_A -> ((DoubleArrayPublisher) publisher)
          .set((double[]) entry.value, entry.timestamp);
      case BOOL -> ((BooleanPublisher) publisher).set((boolean) entry.value, entry.timestamp);
      case BOOL_A -> ((BooleanArrayPublisher) publisher)
          .set((boolean[]) entry.value, entry.timestamp);
      case STR -> ((StringPublisher) publisher).set((String) entry.value, entry.timestamp);
      case STR_A -> ((StringArrayPublisher) publisher).set((String[]) entry.value, entry.timestamp);
    }
  }

  private void dumpStruct(StructEntry entry) {
    Struct<?> struct = getStruct(entry.value.getClass());
    if (struct == null) {
      return;
    }
    String key = "/" + entry.path;
    var publisher = structEntries.get(key);
    if (publisher == null) {
      publisher = NetworkTableInstance.getDefault().getStructTopic(key, struct).publish();
      structEntries.put(key, publisher);
    }
    publisher.set(entry.value, entry.timestamp);
  }

  private void dumpStructArray(StructArrayEntry entry) {
    Struct<?> struct = getStruct(entry.value.getClass().getComponentType());
    if (struct == null) {
      return;
    }
    String key = "/" + entry.path;
    var publisher = structArrayEntries.get(key);
    if (publisher == null) {
      publisher = NetworkTableInstance.getDefault().getStructArrayTopic(key, struct).publish();
      structArrayEntries.put(key, publisher);
    }
    publisher.set(entry.value, entry.timestamp);
  }
}
