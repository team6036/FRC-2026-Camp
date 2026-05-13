// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveIOSim;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class CampRobot extends TimedRobot {

  private SwerveSubsystem swerve;

  public CampRobot() {
    super(0.02);
  }

  @Override
  public void robotInit() {
    if (isReal()) {
      swerve = new SwerveSubsystem(new SwerveIO());
    } else {
      swerve = new SwerveSubsystem(new SwerveIOSim());
    }
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
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    if (swerve != null) {
      swerve.simulationPeriodic();
    }
  }
}
