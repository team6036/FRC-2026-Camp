package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveIO.SwerveIOInputs;
import frc.robot.util.Logger;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveIO io;

  private final SwerveIOInputs inputs = new SwerveIOInputs();

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDeadband(SwerveConstants.maxLinearSpeed * SwerveConstants.joystickDeadband)
          .withRotationalDeadband(
              SwerveConstants.maxAngularSpeed * SwerveConstants.joystickDeadband)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
          .withSteerRequestType(SwerveModule.SteerRequestType.Position);

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

    io.setControl(driveRequest.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.log("Subsystems/Swerve/Pose", inputs.pose);
  }

  public void simulationPeriodic() {
    io.updateSimState(0.02, 12.0);
  }

  public void stop() {
    driveFieldRelative(new ChassisSpeeds());
  }
}
