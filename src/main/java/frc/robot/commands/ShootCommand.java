package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ShootCommand extends Command {

  private final ShooterSubsystem shooter;
  private final SwerveSubsystem swerve;

  public ShootCommand(ShooterSubsystem shooter, SwerveSubsystem swerve) {
    this.swerve = swerve;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.wantedMode = ShooterSubsystem.Mode.SHOOTING;
  }

  @Override
  public void execute() {
    shooter.setWantedVelocityRPS(
        ShooterConstants.shooterVelocityMap.get(swerve.getDistanceFromHub()));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.wantedMode = ShooterSubsystem.Mode.OFF;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
