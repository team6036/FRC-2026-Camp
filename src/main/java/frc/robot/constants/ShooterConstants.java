package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
  /* CAN */
  public static final CANBus bus = RobotConstants.rio;
  public static final int bottomLeftMotorId = 30;
  public static final int bottomRightMotorId = 31;
  public static final int topLeftMotorId = 32;
  public static final int topRightMotorId = 33;

  /* Control */
  public static final double kP = 0.5;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0.3;
  public static final double kV = 0.12;
  public static final double kA = 0;

  /* Shotmap */
  public static final InterpolatingDoubleTreeMap shooterVelocityMap =
      new InterpolatingDoubleTreeMap();

  static {
    shooterVelocityMap.put(1.0, 25.0);
    shooterVelocityMap.put(2.0, 35.0);
    shooterVelocityMap.put(4.0, 48.0);
    shooterVelocityMap.put(5.0, 60.0);
    shooterVelocityMap.put(8.0, 90.0);
  }
}
