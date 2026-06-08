package frc.robot.commands;

import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ShootCommand extends AimCommand {

  private final ShooterSubsystem shooter;
  private final SwerveSubsystem swerve;

  public ShootCommand(ShooterSubsystem shooter, SwerveSubsystem swerve) {
    super(swerve);
    this.swerve = swerve;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    super.initialize();
    shooter.wantedMode = ShooterSubsystem.Mode.SHOOTING;
  }

  @Override
  public void execute() {
    super.execute();
    shooter.setWantedVelocityRPS(
        ShooterConstants.shooterVelocityMap.get(swerve.getDistanceFromHub()));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooter.wantedMode = ShooterSubsystem.Mode.OFF;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
