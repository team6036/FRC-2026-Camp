package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.util.Logger;

public class ShooterSubsystem extends SubsystemBase {

  public enum Mode {
    OFF,
    SHOOTING,
  }

  private final ShooterIO io;
  private final ShooterIOInputs inputs = new ShooterIOInputs();
  public Mode wantedMode = Mode.OFF;
  private double wantedVelocityRPS = 0;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public void setWantedVelocityRPS(double wantedVelocityRPS) {
    this.wantedVelocityRPS = wantedVelocityRPS;
  }

  @Override
  public void periodic() {
    Logger.log("Subsystems/Shooter/WantedMode", wantedMode);
    Logger.log("Subsystems/Shooter/WantedVelocityRPS", wantedVelocityRPS);

    io.updateInputs(inputs);
    Logger.log("Subsystems/Shooter/BottomLeftVelocity", inputs.bottomLeftVelocityRPS);
    Logger.log("Subsystems/Shooter/BottomRightVelocity", inputs.bottomRightVelocityRPS);
    Logger.log("Subsystems/Shooter/TopLeftVelocity", inputs.topLeftVelocityRPS);
    Logger.log("Subsystems/Shooter/TopRightVelocity", inputs.topRightVelocityRPS);

    switch (wantedMode) {
      case SHOOTING:
        io.setTopVelocity(wantedVelocityRPS);
        if (inputs.topLeftVelocityRPS >= wantedVelocityRPS * 0.9) {
          io.setBottomVelocity(wantedVelocityRPS);
        }
        break;
      case OFF:
      default:
        io.stop();
    }
  }
}
