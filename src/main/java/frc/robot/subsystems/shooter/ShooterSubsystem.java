package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
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

  public void updateNT() {
    boolean update = ShooterConstants.shooterGains.shouldUpdate();

    if (!update) return;
    TalonFXConfigurator tlConfig = io.topLeftMotor.getConfigurator();
    TalonFXConfigurator trConfig = io.topRightMotor.getConfigurator();
    TalonFXConfigurator blConfig = io.bottomLeftMotor.getConfigurator();
    TalonFXConfigurator brConfig = io.bottomRightMotor.getConfigurator();

    ShooterConstants.shooterGains.update(tlConfig);
    ShooterConstants.shooterGains.update(trConfig);
    ShooterConstants.shooterGains.update(blConfig);
    ShooterConstants.shooterGains.update(brConfig);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.log("Subsystems/Shooter/WantedVelocityRPS", wantedVelocityRPS);
    Logger.log("Subsystems/Shooter/BottomLeftVelocity", Math.abs(inputs.bottomLeftVelocityRPS));
    Logger.log("Subsystems/Shooter/BottomRightVelocity", Math.abs(inputs.bottomRightVelocityRPS));
    Logger.log("Subsystems/Shooter/TopLeftVelocity", Math.abs(inputs.topLeftVelocityRPS));
    Logger.log("Subsystems/Shooter/TopRightVelocity", Math.abs(inputs.topRightVelocityRPS));

    Logger.log("Subsystems/Shooter/BottomLeftPresent", inputs.bottomLeftPresent);
    Logger.log("Subsystems/Shooter/BottomRightPresent", inputs.bottomRightPresent);

    if (isShooting) {
      io.setTopVelocity(-wantedVelocityRPS);
      if (Math.abs(inputs.topLeftVelocityRPS) >= wantedVelocityRPS * 0.8) {
        io.setBottomVelocity(-wantedVelocityRPS);
      }
    } else {
      io.stop();
    }
  }
}
