// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Logger;

public class Robot extends TimedRobot {

  private final SwerveSubsystem swerve;
  private final ShooterSubsystem shooter;
  //  private final VisionFuelSubsystem visionFuel;
  private final XboxController controller = new XboxController(0);

  private final AimCommand aimCommand;
  private final ShootCommand shootCommand;

  public Robot() {
    super(0.02);
    swerve = new SwerveSubsystem(new SwerveIO());
    shooter = new ShooterSubsystem(new ShooterIO());
    //    visionFuel = new VisionFuelSubsystem(new VisionFuelIO(swerve.getPoseBuffer()));

    aimCommand = new AimCommand(swerve);
    shootCommand = new ShootCommand(shooter, swerve);
  }

  @Override
  public void robotInit() {
    Logger.init();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    new Trigger(controller::getYButton).whileTrue(shootCommand);
    new Trigger(controller::getAButton).whileTrue(aimCommand);
  }

  @Override
  public void teleopPeriodic() {
    double vx = controller.getLeftY() * SwerveConstants.maxLinearSpeed;
    double vy = controller.getLeftX() * SwerveConstants.maxLinearSpeed;
    double omega = controller.getRightX() * SwerveConstants.maxAngularSpeed;

    if (swerve.wantedMode == SwerveSubsystem.Mode.AIM) {
      Pose2d pose = swerve.getPose();
      double angleToHub =
          Math.atan2(
              FieldConstants.Hub.redHubPosition.getY() - pose.getY(),
              FieldConstants.Hub.redHubPosition.getX() - pose.getX());
      omega = Rotation2d.fromRadians(angleToHub).minus(pose.getRotation()).getRadians();
    }
    swerve.driveFieldRelative(new ChassisSpeeds(vx, vy, omega));
    Logger.log("SerialNumber", RobotController.getSerialNumber());
    Logger.log("SwerveType", RobotConstants.swerveModuleType);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    swerve.stop();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    swerve.simulationPeriodic();
  }
}
