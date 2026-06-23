package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.util.NTFullGains;

public class ShooterConstants {
  /* CAN */
  public static final CANBus bus = RobotConstants.rio;
  public static final int bottomLeftMotorId = 30;
  public static final int bottomRightMotorId = 31;
  public static final int topLeftMotorId = 32;
  public static final int topRightMotorId = 33;

  /* Control */
  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kG = 0;
  public static final double kV = 0.;
  public static final double kA = 0;

  public static final NTFullGains shooterGains =
      new NTFullGains("Shooter", kP, kI, kD, kS, kV, kA, kG);

  /* Shotmap */
  public static final InterpolatingDoubleTreeMap shooterVelocityMap =
      new InterpolatingDoubleTreeMap();

  static {
    shooterVelocityMap.put(1.0, 25.0);
    shooterVelocityMap.put(2.5, 32.0);
    shooterVelocityMap.put(3.5, 42.0);
    shooterVelocityMap.put(5.0, 52.0);
    shooterVelocityMap.put(8.0, 80.0);
  }
}
