package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveIO.SwerveIOInputs;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveIO io;

  private final SwerveIOInputs inputs = new SwerveIOInputs();

  private final SwerveRequest.ApplyRobotSpeeds robotRelativeRequest =
      new SwerveRequest.ApplyRobotSpeeds();

  public SwerveSubsystem(SwerveIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {

    io.setControl(robotRelativeRequest.withSpeeds(speeds));
  }

  public void simulationPeriodic() {

    if (io instanceof SwerveIOSim sim) {

      sim.updateSim();
    }
  }

  public void stop() {

    driveRobotRelative(new ChassisSpeeds());
  }
}
