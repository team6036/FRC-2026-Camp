// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
  // private final IntakeSubsystem intake;
  private final XboxController controller = new XboxController(0);

  // ===== PULL UP SHOOTING NetworkTables subscribers/de-bouncer HERE =====

  public Robot() {
    super(0.02);
    swerve = new SwerveSubsystem(new SwerveIO());
    shooter = new ShooterSubsystem(new ShooterIO());
    // intake = new IntakeSubsystem(new IntakeIO());

    // ===== YOUR JOB: Tune the shooter ===========================
    //    shooter.configurePID(0, 0, 0, 0, 0, 0, 0);
    //    shooter.addShot(0.0, 20.0);
    // ============================================================
    shooter.configurePID(0.5, 0, 0, 0.275, 0.12, 0, 0);
    shooter.addShot(3.7, 50);
    shooter.addShot(2.2, 32);
    shooter.addShot(3, 38);
    shooter.addShot(4, 54);
    Logger.log("SerialNumber", RobotConstants.serialNumber);
  }

  @Override
  public void robotInit() {
    Logger.init();

    swerve.resetPose(RobotConstants.startPosition.pose);
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
    double vx = controller.getLeftY() * SwerveConstants.maxLinearSpeed;
    double vy = controller.getLeftX() * SwerveConstants.maxLinearSpeed;
    double omega = controller.getRightX() * SwerveConstants.maxAngularSpeed;

    if (controller.getRawButton(7) && controller.getRawButton(8)) { // + and -
      swerve.zeroGyro();
    }
    // ===== YOUR JOB: Make shooter shoot at correct velocity =====
    // HINT: swerve.getDistanceFromHub()
    // HINT: shooter.getVelocityForDistance()
    // HINT: shooter.shoot(...)
    if (controller.getYButton() || controller.getRawButton(4)) {
      // double distance = swerve.getDistanceFromHub();
      // shooter.shoot(shooter.getVelocityForDistance(distance));
      shooter.shoot(40);
    }

    // ===== YOUR JOB: Autonomous driving! ========================
    // HINT: swerve.hasTarget()
    // HINT: swerve.chaseTarget()
    // HINT: swerve.aimAtHub(vx, vy)
    // HINT: swerve.isAimedAtHub()
    // HINT: the three methods we used in the exercise above!
    else if (controller.getAButton()) {

    }

    // ============================================================
    else {
      shooter.stop();
      //      intake.stop();
      swerve.drive(new ChassisSpeeds(vx, vy, omega));
    }
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
