package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {

  public static final double gravity = 9.81;

  public static final double fieldLenMeters = Units.inchesToMeters(651.2);
  public static final double fieldWidthMeters = Units.inchesToMeters(317.7);

  public static class Hub {
    public static final double hubToDSDistance = Units.inchesToMeters(158.597470);
    public static final double hubSize = Units.inchesToMeters(47);

    public static final double hubHeight = Units.inchesToMeters(72);

    public static final Translation3d blueHubPosition =
        new Translation3d(hubToDSDistance + hubSize / 2, fieldWidthMeters / 2, hubHeight);
    public static final Translation3d redHubPosition =
        new Translation3d(Units.inchesToMeters(469.102530 + 22), fieldWidthMeters / 2, hubHeight);
  }
}
