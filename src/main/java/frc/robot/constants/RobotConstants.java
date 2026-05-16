package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

public class RobotConstants {
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
