package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.util.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputs inputs = new ShooterIOInputs();
  private double wantedVelocityRPS = 0;
  private boolean isShooting = false;

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  public void configurePID(
      double kP, double kI, double kD, double kS, double kV, double kG, double kA) {
    io.configurePID(kP, kI, kD, kS, kV, kG, kA);
  }

  public void addShot(double distance, double velocityRPS) {
    ShooterConstants.shooterVelocityMap.put(distance, velocityRPS);
  }

  public double getVelocityForDistance(double distanceMeters) {
    return ShooterConstants.shooterVelocityMap.get(distanceMeters);
  }

  public void shoot(double velocityRPS) {
    this.wantedVelocityRPS = velocityRPS;
    this.isShooting = true;
  }

  public void stop() {
    this.wantedVelocityRPS = 0;
    this.isShooting = false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.log("Subsystems/Shooter/WantedVelocityRPS", wantedVelocityRPS);
    Logger.log("Subsystems/Shooter/BottomLeftVelocity", inputs.bottomLeftVelocityRPS);
    Logger.log("Subsystems/Shooter/BottomRightVelocity", inputs.bottomRightVelocityRPS);
    Logger.log("Subsystems/Shooter/TopLeftVelocity", inputs.topLeftVelocityRPS);
    Logger.log("Subsystems/Shooter/TopRightVelocity", inputs.topRightVelocityRPS);

    Logger.log("Subsystems/Shooter/BottomLeftPresent", inputs.bottomLeftPresent);
    Logger.log("Subsystems/Shooter/BottomRightPresent", inputs.bottomRightPresent);

    if (isShooting) {
      io.setTopVelocity(wantedVelocityRPS);
      if (inputs.topLeftVelocityRPS >= wantedVelocityRPS * 0.7) {
        io.setBottomVelocity(wantedVelocityRPS);
      }
    } else {
      io.stop();
    }
  }
}
