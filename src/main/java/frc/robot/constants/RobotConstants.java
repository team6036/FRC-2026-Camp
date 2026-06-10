package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
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

  public enum DriveDirection {
    NORMAL,
    LEFT,
    RIGHT
  }

  public static final String serialNumber =
      Robot.isReal() ? RobotController.getSerialNumber() : "SIMULATION";

  public static final RobotType robotType =
      switch (serialNumber) {
        case "023FF3E1" -> RobotType.CAMP_A;
        case "032381B0" -> RobotType.CAMP_B;
        case "0326F275" -> RobotType.CAMP_C;
        case "0251EF71" -> RobotType.CAMP_D;
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
        case CAMP_A -> SwerveModuleType.MK4n_L2;
        case CAMP_B -> SwerveModuleType.MK4n_L2;
        case CAMP_C -> SwerveModuleType.MK4i_L2;
        case CAMP_D -> SwerveModuleType.MK5n_L2;
        case UNKNOWN -> SwerveModuleType.MK4n_L2;
      };

  public static final DriveDirection driveDirection =
      switch (robotType) {
        case CAMP_A -> DriveDirection.LEFT;
        case CAMP_B -> DriveDirection.LEFT;
        case CAMP_C -> DriveDirection.LEFT;
        case CAMP_D -> DriveDirection.LEFT;
        case UNKNOWN -> DriveDirection.NORMAL;
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
