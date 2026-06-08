package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AimCommand extends Command {

  private final SwerveSubsystem swerve;

  public AimCommand(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    swerve.wantedMode = SwerveSubsystem.Mode.AIM;
  }

  @Override
  public void end(boolean interrupted) {
    swerve.wantedMode = SwerveSubsystem.Mode.NORMAL;
  }
}
