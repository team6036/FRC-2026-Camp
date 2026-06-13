// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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

  public Robot() {
    super(0.02);
    swerve = new SwerveSubsystem(new SwerveIO());
    shooter = new ShooterSubsystem(new ShooterIO());
    //    visionFuel = new VisionFuelSubsystem(new VisionFuelIO(swerve.getPoseBuffer()));

    // ===== YOUR JOB: Tune the shooter ===========================
    shooter.configurePID(0, 0, 0, 0, 0, 0, 0);

    shooter.addShot(0.0, 20.0);
    // ============================================================
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
  public void teleopPeriodic() {
    double vx = controller.getLeftY();
    double vy = controller.getLeftX();
    double omega = controller.getRightX();

    swerve.driveFieldRelative(new ChassisSpeeds(vx, vy, omega));

    // ===== YOUR JOB: Make shooter shoot at correct velocity =====
    if (controller.getYButton()) {
      //      double distance = swerve.getDistanceFromHub();
      //      double velocity = shooter.getVelocityForDistance(distance);
      //      shooter.shoot(velocity);
    } else {
      //      shooter.stop();
    }
    // ============================================================
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
