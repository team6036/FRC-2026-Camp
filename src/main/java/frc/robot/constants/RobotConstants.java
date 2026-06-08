package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Robot;
import java.util.Optional;

public class RobotConstants {

  public enum RobotType {
    CAMP_1,
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
        case "0251EF71" -> RobotType.CAMP_1;
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
            yield RobotType.CAMP_1;
          }
        }
      };

  public static final SwerveModuleType swerveModuleType =
      switch (robotType) {
        case CAMP_1 -> SwerveModuleType.MK4n_L2;
        case UNKNOWN -> SwerveModuleType.MK4n_L2;
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
