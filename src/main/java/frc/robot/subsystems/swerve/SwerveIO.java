package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.DartCamera;
import java.util.List;

public class SwerveIO extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

  private final DartCamera camera;
  private double lastProcessedTimestamp = -1;

  public static class SwerveIOInputs {
    public Pose2d pose = new Pose2d();
    public ChassisSpeeds speeds = new ChassisSpeeds();
    public SwerveModuleState[] moduleStates;
  }

  public SwerveIO() {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        new SwerveDrivetrainConstants()
            .withCANBusName(SwerveConstants.bus.getName())
            .withPigeon2Id(SwerveConstants.imuId),
        250,
        SwerveConstants.stateStDev,
        VisionConstants.visionStDev,
        SwerveConstants.moduleConstants.get(0),
        SwerveConstants.moduleConstants.get(1),
        SwerveConstants.moduleConstants.get(2),
        SwerveConstants.moduleConstants.get(3));
    this.camera = new DartCamera(VisionConstants.camera.name);
  }

  public void updateInputs(SwerveIOInputs inputs) {
    List<DartCamera.AprilTagResult> aprilTagResults = camera.readAprilTagResults();
    DartCamera.PnPResult latestPnPResult = null;
    double latestTimestamp = -1;
    for (DartCamera.AprilTagResult result : aprilTagResults) {
      if (result.numPnpResults > 0 && result.captureTimestamp > latestTimestamp) {
        latestTimestamp = result.captureTimestamp;
        latestPnPResult = result.pnpResults[0];
      }
    }
    if (latestPnPResult != null && latestTimestamp > lastProcessedTimestamp) {
      this.addVisionMeasurement(
          latestPnPResult.pose.toPose2d(), latestTimestamp, VisionConstants.visionStDev);
      lastProcessedTimestamp = latestTimestamp;
    }

    var state = getStateCopy();

    inputs.pose = state.Pose;
    inputs.speeds = state.Speeds;
    inputs.moduleStates = state.ModuleStates;
  }
}
