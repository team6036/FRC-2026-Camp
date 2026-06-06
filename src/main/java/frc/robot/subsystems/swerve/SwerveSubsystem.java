package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveIO.SwerveIOInputs;
import frc.robot.util.Logger;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveIO io;
  private final SwerveIOInputs inputs = new SwerveIOInputs();

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(VisionConstants.pieceStaleTime);

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(SwerveConstants.maxLinearSpeed * SwerveConstants.joystickDeadband)
          .withRotationalDeadband(
              SwerveConstants.maxAngularSpeed * SwerveConstants.joystickDeadband)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
          .withSteerRequestType(SwerveModule.SteerRequestType.Position);

  private ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  public SwerveSubsystem(SwerveIO io) {
    this.io = io;
  }

  public void driveFieldRelative(ChassisSpeeds speeds) {
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;
    double omega = speeds.omegaRadiansPerSecond;
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

  public Pose2d getPose() {
    return inputs.pose;
  }

  public TimeInterpolatableBuffer<Pose2d> getPoseBuffer() {
    return poseBuffer;
  }

  public void updateNT() {
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
    poseBuffer.addSample(Timer.getFPGATimestamp(), inputs.pose);

    Logger.log("Subsystems/Swerve/SwerveModuleStates", inputs.moduleStates);
    Logger.log("Subsystems/Swerve/Pose", inputs.pose);
    Logger.log("Subsystems/Swerve/Speeds/Actual", inputs.speeds);

    updateNT();
  }

  public void simulationPeriodic() {
    io.updateSimState(0.02, 12.0);
  }

  public void stop() {
    driveFieldRelative(new ChassisSpeeds());
  }
}
