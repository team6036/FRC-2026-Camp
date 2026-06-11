package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.util.NTFullGains;
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

  public static Translation2d[] moduleTranslations = {
    new Translation2d(moduleOffsetX, moduleOffsetY), // Front Left
    new Translation2d(moduleOffsetX, -moduleOffsetY), // Front Right
    new Translation2d(-moduleOffsetX, moduleOffsetY), // Back Left
    new Translation2d(-moduleOffsetX, -moduleOffsetY) // Back Right
  };

  /* Steer Motor PID Values */
  public static final double steerKP = 50;
  public static final double steerKI = 0;
  public static final double steerKD = 0;
  public static final double steerKS = 0;
  public static final double steerKV = 0;
  public static final double steerKA = 0;
  public static final double steerKG = 0;

  public static final NTFullGains steerGains =
      new NTFullGains("Steer", steerKP, steerKI, steerKD, steerKS, steerKV, steerKA, steerKG);

  /* Drive Motor PID Values */
  public static final double driveKP = 0.03;
  //  public static final double driveKP = 0.9993214286;
  public static final double driveKI = 0;
  public static final double driveKD = 0;
  public static final double driveKS = 0;
  //  public static final double driveKS = 0.19528;
  public static final double driveKV = 0.11;
  //  public static final double driveKV = 0.6474107143;
  public static final double driveKA = 0;
  //  public static final double driveKA = 0.0331076786;
  public static final double driveKG = 0;

  public static final NTFullGains driveGains =
      new NTFullGains("Drive", driveKP, driveKI, driveKD, driveKS, driveKV, driveKA, driveKG);

  public static final int[] steerIds = {1, 3, 5, 7};
  public static final int[] driveIds = {2, 4, 6, 8};
  public static final int[] encoderIds = {9, 10, 11, 12};

  //  public static final double steerGearRatio = 287d / 11; // MK5n
  //  public static final double driveGearRatio = 1d / ((14d / 54) * (32d / 25) * (15d / 30)); //
  // MK5n
  //  public static final double wheelDiameterMeters = 0.049782 * 2;

  public static final double steerGearRatio =
      switch (RobotConstants.swerveModuleType) {
        case MK4n_L2 -> 18.75;
        case MK5n_L2 -> 287d / 11;
        case MK4i_L2 -> 150d / 7;
        default -> 18.75; // MK4n L2
      };

  public static final double driveGearRatio =
      switch (RobotConstants.swerveModuleType) {
        case MK4n_L2 -> 1d / ((16d / 50) * (27d / 17) * (15d / 45));
        case MK5n_L2 -> 1d / ((14d / 54) * (32d / 25) * (15d / 30));
        case MK4i_L2 -> 1d / ((14d / 50) * (27d / 17) * (15d / 45));
        default -> 1d / ((16d / 50) * (27d / 17) * (15d / 45)); // MK4n L2
      };

  public static final double wheelDiameterMeters =
      switch (RobotConstants.swerveModuleType) {
        case MK4n_L2 -> Units.inchesToMeters(4);
        case MK5n_L2 -> Units.inchesToMeters(4);
        case MK4i_L2 -> Units.inchesToMeters(4);
        default -> Units.inchesToMeters(4);
      }; // Prolly don't need this kind of thing cuz afaik they're all 4"

  public static final boolean driveMotorInverted =
      switch (RobotConstants.swerveModuleType) {
        case MK4n_L2 -> true;
        case MK5n_L2 -> false;
        case MK4i_L2 -> true;
        default -> true;
      }; // be most weary of drive motor inverted lowkey as results are inconclusive online

  public static final boolean steerMotorInverted =
      switch (RobotConstants.swerveModuleType) {
        case MK4n_L2 -> true;
        case MK5n_L2 -> false; // unsure - maybe, gemini says true
        case MK4i_L2 -> true;
        default -> true;
      }; // this being wrong but having correct encoder offsets results in the frantic back and
  // forth rather than align on enable

  public static final boolean encoderInverted =
      switch (RobotConstants.swerveModuleType) {
        case MK4n_L2 -> false;
        case MK5n_L2 -> false; // sure
        case MK4i_L2 -> false;
        default -> false;
      };

  public static final double[] encoderOffsets =
      switch (RobotConstants.robotType) {
        case CAMP_A -> new double[] {
          0.224121 + 1 + 2 * .125,
          -0.244873 + +0.5 + 2 * 0.125,
          -0.321777 + 1 + 2 * .125,
          -0.040771 + .5 + 2 * 0.125
        };
        case CAMP_B -> new double[] {-0.118408, -0.415039 + 0.5, 0.318115, -0.248779};
        case CAMP_D -> new double[] {0.176025, -0.25, -0.265137, -0.027832};
        case CAMP_C -> new double[] {0.354492, -.436035, -.182373, -.303955};
        default -> new double[] {0, 0, 0, 0};
      };

  private static SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      createModuleConstants(int i) {
    TalonFXConfiguration driveConfig = getDriveConfiguration();
    TalonFXConfiguration steerConfig = getSteerConfiguration(encoderIds[i]);
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    return new SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
        .withDriveMotorId(driveIds[i])
        .withSteerMotorId(steerIds[i])
        .withEncoderId(encoderIds[i])
        .withDriveMotorGearRatio(driveGearRatio)
        .withSteerMotorGearRatio(steerGearRatio)
        .withDriveMotorInverted(driveMotorInverted)
        .withSteerMotorInverted(steerMotorInverted)
        .withEncoderInverted(encoderInverted)
        .withWheelRadius(wheelDiameterMeters / 2)
        .withLocationX(moduleTranslations[i].getX())
        .withLocationY(moduleTranslations[i].getY())
        .withDriveMotorGains(driveConfig.Slot0)
        .withSteerMotorGains(steerConfig.Slot0)
        .withEncoderOffset(encoderOffsets[i])
        .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
        .withDriveMotorInitialConfigs(driveConfig)
        .withSteerMotorInitialConfigs(steerConfig)
        .withEncoderInitialConfigs(encoderConfig);
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

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.Feedback.FeedbackRemoteSensorID = encoderId;
    config.Feedback.RotorToSensorRatio = steerGearRatio;
    config.ClosedLoopGeneral.ContinuousWrap = true;

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

  /* Pose Estimator */
  public static final Matrix<N3, N1> stateStDev = VecBuilder.fill(0.1, 0.1, 0.05);

  /* IMU */
  public static final int imuId = 50;

  /* Limits */
  public static final double maxLinearSpeed = 1.5d;
  public static final double maxAngularSpeed = Math.PI;
  public static final double joystickDeadband = 0.05;
}
