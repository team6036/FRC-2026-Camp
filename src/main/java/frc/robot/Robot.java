// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.Logger;

public class Robot extends TimedRobot {

  private final SwerveSubsystem swerve;
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private final XboxController controller = new XboxController(0);

  private double shooterTrim;
  private int lastPOV = -1;

  // ===== PULL UP SHOOTING NetworkTables subscribers/de-bouncer HERE =====

  public Robot() {
    super(0.02);
    swerve = new SwerveSubsystem(new SwerveIO());
    shooter = new ShooterSubsystem(new ShooterIO());
    intake = new IntakeSubsystem(new IntakeIO());

    // ===== YOUR JOB: Tune the shooter ===========================
    shooter.configurePID(0.5, 0, 0, 0.3, 0.12, 0, 0);
    shooter.addShot(1.4, 37);
    shooter.addShot(1.9, 41);
    shooter.addShot(2.3, 44);
    shooter.addShot(2.7, 47);
    shooter.addShot(3.0, 50);
    // ============================================================
  }

  @Override
  public void robotInit() {
    Logger.init();

    boolean gotVisionPose = swerve.resetPoseFromVision();
    Logger.log("StartupVisionPoseSuccess", gotVisionPose);
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
    // Shooter trim stuff (dpad is under POV https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html)
    int pov = controller.getPOV();
    if (pov != lastPOV) {
      if (pov == 0) shooterTrim += 1;
      else if (pov == 180) shooterTrim -= 1;
      lastPOV = pov;
    }

    Logger.log("Joysticks/POV", pov);
    Logger.log("Subsystems/Shooter/Trim", shooterTrim);

    double vx = controller.getLeftY() * SwerveConstants.maxLinearSpeed;
    double vy = controller.getLeftX() * SwerveConstants.maxLinearSpeed;
    double omega = controller.getRightX() * SwerveConstants.maxAngularSpeed;

    // ===== YOUR JOB: Make shooter shoot at correct velocity =====
    // HINT: swerve.getDistanceFromHub()
    // HINT: shooter.getVelocityForDistance()
    // HINT: shooter.shoot(...)
    if (controller.getYButton()) {
      swerve.aimAtHub(vx, vy);
      if (swerve.isAimedAtHub()) {
        double distance = swerve.getDistanceFromHub();
        double velocity = shooter.getVelocityForDistance(distance);
        shooter.shoot(velocity, shooterTrim);
      }
    } else if (controller.getBButton()) {
      intake.run();
    }

    // ===== YOUR JOB: Autonomous driving! ========================
    // HINT: swerve.driveTowardTarget()
    // HINT: swerve.chaseTarget()
    // HINT: swerve.aimAtHub(vx, vy)
    // HINT: swerve.isAimedAtHub()
    // HINT: the three methods we used in the exercise above!
    else if (controller.getAButton()) {
      if (!swerve.ballCollected()) {
        swerve.driveTowardTarget();

        if (swerve.isNearBall()) {
          intake.run();
        } else {
          intake.stop();
        }
      } else {
        swerve.aimAtHub(vx, vy);
        Logger.log("WantToAimAtHub", true);
        if (swerve.isAimedAtHub()) {
          shooter.shoot(shooter.getVelocityForDistance(swerve.getDistanceFromHub()), shooterTrim);
        }
      }
    }

    // ============================================================
    else {
      swerve.resetCollectSequence();
      shooter.stop();
      intake.stop();
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
