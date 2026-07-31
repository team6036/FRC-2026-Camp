// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;
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
  private double omegaTrim;
  private int lastPOV = -1;

  private Timer intakeCommitTimer = new Timer();

  private enum CollectState {
    SEEKING,
    INTAKING,
    AIMING,
    SHOOTING
  };

  private CollectState state = CollectState.SEEKING;

  // ===== PULL UP SHOOTING NetworkTables subscribers/de-bouncer HERE =====
  private BooleanSubscriber shootSubscriber;
  private final Debouncer shootDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kBoth);

  public Robot() {
    super(0.02);
    swerve = new SwerveSubsystem(new SwerveIO());
    shooter = new ShooterSubsystem(new ShooterIO());
    intake = new IntakeSubsystem(new IntakeIO());

    // ===== YOUR JOB: Tune the shooter ===========================
    //    shooter.configurePID(0, 0, 0, 0, 0, 0, 0);
    //    shooter.addShot(0.0, 20.0);
    // ============================================================
    shooter.configurePID(0.5, 0, 0, 0.275, 0.12, 0, 0);
    shooter.addShot(3.7, 50);
    shooter.addShot(2.2, 32);
    shooter.addShot(3, 38);
    shooter.addShot(4, 54);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("Vision");
    shootSubscriber = table.getBooleanTopic("Shoot").subscribe(false);
    Logger.log("SerialNumber", RobotConstants.serialNumber);
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
    // Shooter trim stuff (dpad is under POV
    // https://docs.wpilib.org/en/stable/docs/software/basic-programming/joystick.html)
    int pov = controller.getPOV();
    if (pov != lastPOV) {
      if (pov == 0) shooterTrim += 1;
      else if (pov == 180) shooterTrim -= 1;
      else if (pov == 90) omegaTrim += 0.1;
      else if (pov == 270) omegaTrim -= 0.1;
      lastPOV = pov;
    }

    Logger.log("Joysticks/POV", pov);
    Logger.log("Subsystems/Shooter/Trim", shooterTrim);
    Logger.log("Subsystems/Swerve/OmegaTrim", omegaTrim);

    double vx = controller.getLeftY() * SwerveConstants.maxLinearSpeed;
    double vy = controller.getLeftX() * SwerveConstants.maxLinearSpeed;
    double omega = -controller.getRightX() * SwerveConstants.maxAngularSpeed;

    if (controller.getRawButton(7) && controller.getRawButton(8)) { // + and -
      swerve.zeroGyro();
    }
    // ===== YOUR JOB: Make shooter shoot at correct velocity =====
    // HINT: swerve.getDistanceFromHub()
    // HINT: shooter.getVelocityForDistance()
    // HINT: shooter.shoot(...)
    boolean shouldShoot = shootDebouncer.calculate(shootSubscriber.get());
    if (controller.getYButton() || controller.getRawButton(4) || shouldShoot) {
      swerve.aimAtHub(vx, vy, omegaTrim);
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
      switch (state) {
        case SEEKING:
          swerve.chaseTarget();
          if (swerve.ballClose()) {
            intakeCommitTimer.restart();
            state = CollectState.INTAKING;
          }
          break;
        case INTAKING:
          swerve.driveForwardBlind();
          intake.run();
          if (intakeCommitTimer.hasElapsed(1.5)) {
            state = CollectState.AIMING;
          }
          break;
        case AIMING:
          swerve.aimAtHub(vx, vy);
          if (swerve.isAimedAtHub()) {
            intake.stop();
            state = CollectState.SHOOTING;
          }
          break;
        case SHOOTING:
          swerve.aimAtHub(vx, vy);
          double distance = swerve.getDistanceFromHub();
          double velocity = shooter.getVelocityForDistance(distance);
          shooter.shoot(velocity, shooterTrim);
          break;
      }
    }

    // ============================================================
    else {
      state = CollectState.SEEKING;
      shooter.stop();
      intake.stop();
      swerve.drive(new ChassisSpeeds(vx, vy, omega));
    }

    Logger.log("Tracking/CollectState", state.toString());
    Logger.log("Tracking/IntakeCommitTimer", intakeCommitTimer.get());
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
