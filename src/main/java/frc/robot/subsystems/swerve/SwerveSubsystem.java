package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveIO.SwerveIOInputs;
import frc.robot.util.Logger;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveIO io;
  private final SwerveIOInputs inputs = new SwerveIOInputs();

  private final PIDController hubAimController = new PIDController(SwerveConstants.aimKp, 0, 0);
  private final PIDController fuelAimController = new PIDController(SwerveConstants.chaseKp, 0, 0);

  private final BooleanSubscriber hasTargetSub;
  private final DoubleSubscriber targetXSub;
  private final DoubleSubscriber targetYSub;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(SwerveConstants.maxLinearSpeed * SwerveConstants.joystickDeadband)
          .withRotationalDeadband(
              SwerveConstants.maxAngularSpeed * SwerveConstants.joystickDeadband)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
          .withSteerRequestType(SwerveModule.SteerRequestType.Position);

  private final SwerveRequest.RobotCentric chaseRequest =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
          .withSteerRequestType(SwerveModule.SteerRequestType.Position);

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  public SwerveSubsystem(SwerveIO io) {
    this.io = io;

    hubAimController.enableContinuousInput(-Math.PI, Math.PI);
    fuelAimController.setTolerance(VisionConstants.fuelAimTolerancePixels);

    NetworkTable visionTable = NetworkTableInstance.getDefault().getTable("Vision");
    hasTargetSub = visionTable.getBooleanTopic("hasTarget").subscribe(false);
    targetXSub = visionTable.getDoubleTopic("targetX").subscribe(0.0);
    targetYSub = visionTable.getDoubleTopic("targetY").subscribe(0.0);
  }

  public Pose2d getPose() {
    return inputs.pose;
  }

  public void drive(ChassisSpeeds speeds) {
    double vx, vy, omega;
    switch (RobotConstants.startPosition) {
      case LEFT:
        vx = speeds.vyMetersPerSecond;
        vy = -speeds.vxMetersPerSecond;
        omega = speeds.omegaRadiansPerSecond;
        break;
      case RIGHT:
        vx = -speeds.vyMetersPerSecond;
        vy = speeds.vxMetersPerSecond;
        omega = speeds.omegaRadiansPerSecond;
        break;
      default:
        vx = speeds.vxMetersPerSecond;
        vy = speeds.vyMetersPerSecond;
        omega = speeds.omegaRadiansPerSecond;
    }

    if (RobotConstants.onBlue()) {
      vx = -vx;
      vy = -vy;
    }

    double magnitude = Math.hypot(vx, vy);
    if (magnitude > SwerveConstants.maxLinearSpeed) {
      double scale = SwerveConstants.maxLinearSpeed / magnitude;
      vx *= scale;
      vy *= scale;
    }

    targetSpeeds = new ChassisSpeeds(vx, vy, omega);
    Logger.log("Subsystems/Swerve/Speeds/Target", targetSpeeds);

    io.setControl(driveRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
  }

  public boolean hasFuelTarget() {
    return hasTargetSub.getAsBoolean();
  }

  public void driveForwardBlind() {
    io.setControl(
        chaseRequest
            .withVelocityX(SwerveConstants.maxLinearSpeed / 3)
            .withVelocityY(0)
            .withRotationalRate(0));
  }

  public void chaseTarget() {
    double error = targetXSub.getAsDouble() - (VisionConstants.frameWidth / 2d);
    double omega =
        MathUtil.clamp(
            -fuelAimController.calculate(-error, 0),
            -SwerveConstants.maxAngularSpeed / 4,
            SwerveConstants.maxAngularSpeed / 4);
    io.setControl(
        chaseRequest
            .withVelocityX(SwerveConstants.maxLinearSpeed / 4)
            .withVelocityY(0.0)
            .withRotationalRate(omega));
  }

  public boolean ballClose() {
    return targetYSub.getAsDouble() > 600;
  }

  public boolean isAimedAtTarget() {
    return fuelAimController.atSetpoint();
  }

  public double getDistanceFromHub() {
    return getPose()
        .getTranslation()
        .getDistance(FieldConstants.Hub.redHubPosition.toTranslation2d());
  }

  public void aimAtHub(double vx, double vy) {
    aimAtHub(vx, vy, 0);
  }

  public void aimAtHub(double vx, double vy, double omegaTrim) {
    Translation2d hubPosition = FieldConstants.Hub.redHubPosition.toTranslation2d();
    Translation2d toHub = hubPosition.minus(inputs.pose.getTranslation());
    Rotation2d targetAngle = new Rotation2d(toHub.getX(), toHub.getY()).plus(Rotation2d.kPi);

    double currentAngle = inputs.pose.getRotation().getRadians();
    double omega =
        MathUtil.clamp(
            hubAimController.calculate(currentAngle, targetAngle.getRadians() + omegaTrim),
            -SwerveConstants.maxAngularSpeed,
            SwerveConstants.maxAngularSpeed);

    drive(new ChassisSpeeds(vx, vy, omega));
  }

  public boolean isAimedAtHub() {
    return hubAimController.atSetpoint();
  }

  private void updateNT() {
    boolean updateDrive = SwerveConstants.driveGains.shouldUpdate();
    boolean updateSteer = SwerveConstants.steerGains.shouldUpdate();
    if (!updateDrive && !updateSteer) {
      return;
    }

    for (int i = 0; i < 4; i++) {
      TalonFXConfigurator steerConfigurator = io.getModule(i).getSteerMotor().getConfigurator();
      TalonFXConfigurator driveConfigurator = io.getModule(i).getDriveMotor().getConfigurator();

      if (updateDrive) SwerveConstants.driveGains.update(driveConfigurator);
      if (updateSteer) SwerveConstants.steerGains.update(steerConfigurator);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.log("Subsystems/Swerve/SwerveModuleStates", inputs.moduleStates);
    Logger.log("Subsystems/Swerve/Pose", getPose());
    Logger.log("Subsystems/Swerve/Speeds/Actual", inputs.speeds);

    Logger.log("Subsystems/Swerve/DistanceFromHub", getDistanceFromHub());
    Logger.log("Subsystems/Swerve/Vision/HasFuelTarget", hasFuelTarget());
    Logger.log("Subsystems/Swerve/Vision/TargetX", targetXSub.getAsDouble());
    Logger.log("Subsystems/Swerve/Vision/TargetY", targetYSub.getAsDouble());

    Logger.log("Subsystems/Swerve/BallClose", ballClose());

    Logger.log("Subsystems/Swerve/Swerve/AimedAtFuel", isAimedAtTarget());
    Logger.log("Subsystems/Swerve/AimedAtHub", isAimedAtHub());

    updateNT();
  }

  public void simulationPeriodic() {
    io.updateSimState(0.02, 12.0);
  }

  public void stop() {
    drive(new ChassisSpeeds());
  }

  public boolean resetPoseFromVision() {
    return io.resetPoseFromVision();
  }

  public void resetPose(Pose2d pose) {
    io.resetPose(pose);
  }

  public void zeroGyro() {
    io.seedFieldCentric();
  }
}
