// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Logger;

public class Robot extends TimedRobot {

  private final SwerveSubsystem swerve;
  //  private final VisionFuelSubsystem visionFuel;
  private final XboxController controller = new XboxController(0);

  public Robot() {
    super(0.02);
    swerve = new SwerveSubsystem(new SwerveIO());
    //    visionFuel = new VisionFuelSubsystem(new VisionFuelIO(swerve.getPoseBuffer()));
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
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double vx = controller.getLeftY() * SwerveConstants.maxLinearSpeed;
    double vy = controller.getLeftX() * SwerveConstants.maxLinearSpeed;
    double omega = controller.getRightX() * SwerveConstants.maxAngularSpeed;

    swerve.driveFieldRelative(new ChassisSpeeds(vx, vy, omega));
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
