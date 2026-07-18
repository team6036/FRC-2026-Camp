package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import java.util.Optional;

public class RobotConstants {

  public enum RobotType {
    CAMP_A,
    CAMP_B,
    CAMP_C,
    CAMP_D,
    UNKNOWN
  }

  public enum SwerveModuleType {
    MK4n_L2,
    MK5n_L2,
    MK4i_L2
  }

  public static final String serialNumber =
      Robot.isReal() ? RobotController.getSerialNumber() : "SIMULATION";

  public static final RobotType robotType =
      switch (serialNumber) {
        case "023FF3E1" -> RobotType.CAMP_B;
        case "032381B0" -> RobotType.CAMP_C;
        case "0326F27E" -> RobotType.CAMP_A;
        case "0251EF71" -> RobotType.CAMP_B; // rio moved, D was taken apart and moved to bot B
        default -> {
          if (Robot.isReal()) {
            new Alert(
                    "Unknown roboRIO Serial: "
                        + serialNumber
                        + ", some functions may not work correctly",
                    Alert.AlertType.kError)
                .set(true);
            yield RobotType.UNKNOWN;
          } else {
            yield RobotType.UNKNOWN;
          }
        }
      };

  public static final SwerveModuleType swerveModuleType =
      switch (robotType) {
        case CAMP_A -> SwerveModuleType.MK4i_L2;
        case CAMP_B -> SwerveModuleType.MK4n_L2;
        case CAMP_C -> SwerveModuleType.MK5n_L2;
        case CAMP_D -> SwerveModuleType.MK4n_L2;
        case UNKNOWN -> SwerveModuleType.MK4n_L2;
      };

  /* Start Pose (we have no cameras :c) */
  public static double redHubX = FieldConstants.Hub.redHubPosition.getX();
  public static double redHubY = FieldConstants.Hub.redHubPosition.getY();

  public enum StartPosition {
    LEFT(new Pose2d(new Translation2d(redHubX + 2, redHubY - 4), Rotation2d.kCCW_90deg)),
    RIGHT(new Pose2d(new Translation2d(redHubX + 2, redHubY + 4), Rotation2d.kCW_90deg)),
    CENTER(new Pose2d(new Translation2d(redHubX + 4.7, redHubY), Rotation2d.k180deg));

    public Pose2d pose;

    StartPosition(Pose2d pose) {
      this.pose = pose;
    }
  }

  public static final StartPosition startPosition =
      switch (robotType) {
        case CAMP_A -> StartPosition.CENTER;
        case CAMP_B -> StartPosition.CENTER;
        case CAMP_C -> StartPosition.CENTER;
        case CAMP_D -> StartPosition.CENTER;
        case UNKNOWN -> StartPosition.CENTER;
      };

  /* CAN Buses */
  public static final CANBus rio = new CANBus("rio");

  /* Logging */
  public static final boolean enableLogging = true;

  /* Alliance */
  public static boolean onBlue() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;
  }

  public static boolean onRed() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }
}
