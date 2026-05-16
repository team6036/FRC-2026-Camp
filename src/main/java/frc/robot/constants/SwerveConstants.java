package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;

public class SwerveConstants {
  /* CAN */
  public static final CANBus bus = RobotConstants.rio;

  /* Drivetrain */
  public static final double wheelbaseMeters = Units.inchesToMeters(20.75);
  public static final double trackWidthMeters = Units.inchesToMeters(20.75);

  /* Swerve Modules */
  public static final double moduleOffsetX = wheelbaseMeters / 2;
  public static final double moduleOffsetY = trackWidthMeters / 2;

  public static Translation2d[] modulePositions = {
    new Translation2d(moduleOffsetX, moduleOffsetY), // Front Left
    new Translation2d(moduleOffsetX, -moduleOffsetY), // Front Right
    new Translation2d(-moduleOffsetX, moduleOffsetY), // Back Left
    new Translation2d(-moduleOffsetX, -moduleOffsetY) // Back Right
  };

  /* Steer Motor PID Values */
  public static final double steerKP = 1000;
  public static final double steerKI = 0;
  public static final double steerKD = 40;
  public static final double steerKS = 0;
  public static final double steerKV = 0;
  public static final double steerKA = 0;
  public static final double steerKG = 0;

  /* Drive Motor PID Values */
  public static final double driveKP = 0.9993214286;
  public static final double driveKI = 0;
  public static final double driveKD = 0;
  public static final double driveKS = 0.19528;
  public static final double driveKV = 0.6474107143;
  public static final double driveKA = 0.0331076786;
  public static final double driveKG = 0;

  public static final int[] driveIds = {1, 3, 5, 7};
  public static final int[] steerIds = {2, 4, 6, 8};
  public static final int[] encoderIds = {9, 10, 11, 12};

  public static final double steerGearRatio = 287d / 11; // MK5n
  public static final double driveGearRatio = 1d / ((14d / 54) * (32d / 25) * (15d / 30)); // MK5n
  public static final double wheelDiameterMeters = 0.049782 * 2;
  public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;

  /**
   * Create module constants using explicit motor/encoder configuration objects. This variant lets
   * callers provide tuned gain/config objects per-module.
   */
  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      createModuleConstants(
          int i,
          TalonFXConfiguration driveConfig,
          TalonFXConfiguration steerConfig,
          CANcoderConfiguration encoderConfig) {

    return new SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
        .withDriveMotorId(driveIds[i])
        .withSteerMotorId(steerIds[i])
        .withEncoderId(encoderIds[i])
        .withDriveMotorGearRatio(driveGearRatio)
        .withSteerMotorGearRatio(steerGearRatio)
        .withWheelRadius(wheelDiameterMeters / 2)
        .withLocationX(modulePositions[i].getX())
        .withLocationY(modulePositions[i].getY())
        .withDriveMotorGains(driveConfig.Slot0)
        .withSteerMotorGains(steerConfig.Slot0)
        .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
        .withDriveMotorInitialConfigs(driveConfig)
        .withSteerMotorInitialConfigs(steerConfig)
        .withEncoderInitialConfigs(encoderConfig);
  }

  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      createModuleConstants(int i) {
    TalonFXConfiguration driveConfig = getDriveConfiguration();
    TalonFXConfiguration steerConfig = getSteerConfiguration(encoderIds[i]);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    return createModuleConstants(i, driveConfig, steerConfig, encoderConfig);
  }

  private static TalonFXConfiguration getSteerConfiguration(int encoderId) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = steerKP;
    config.Slot0.kI = steerKI;
    config.Slot0.kD = steerKD;
    config.Slot0.kS = steerKS;
    config.Slot0.kV = steerKV;
    config.Slot0.kA = steerKA;
    config.Slot0.kG = steerKG;

    if (RobotBase.isReal()) {
      config.Feedback.FeedbackRemoteSensorID = encoderId;
    }

    return config;
  }

  private static TalonFXConfiguration getDriveConfiguration() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = driveKP;
    config.Slot0.kI = driveKI;
    config.Slot0.kD = driveKD;
    config.Slot0.kS = driveKS;
    config.Slot0.kV = driveKV;
    config.Slot0.kA = driveKA;
    config.Slot0.kG = driveKG;

    return config;
  }

  public static final List<
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>>
      moduleConstants =
          List.of(
              createModuleConstants(0),
              createModuleConstants(1),
              createModuleConstants(2),
              createModuleConstants(3));

  /* IMU */
  public static final int imuId = 50;

  /* Limits */
  public static final double maxLinearSpeed = 1d;
  public static final double maxAngularSpeed = 2 * Math.PI;
  public static final double joystickDeadband = 0.05;
}
