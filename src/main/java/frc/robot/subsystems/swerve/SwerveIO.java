package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.SwerveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.Logger;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class SwerveIO extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

  private final SwerveModule<TalonFX, TalonFX, CANcoder>[] modules;

  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
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
    this.modules = getModules();

    this.camera = new PhotonCamera(VisionConstants.camera.name);
    this.poseEstimator =
        new PhotonPoseEstimator(VisionConstants.fieldLayout, VisionConstants.camera.robotToCamera);
  }

  public void updateInputs(SwerveIOInputs inputs) {
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      if (result.getTimestampSeconds() <= lastProcessedTimestamp) {
        continue;
      }

      Optional<EstimatedRobotPose> estimate;
      if (result.getMultiTagResult().isPresent()) {
        estimate = poseEstimator.estimateCoprocMultiTagPose(result);
      } else {
        estimate = poseEstimator.estimateLowestAmbiguityPose(result);
      }

      if (estimate.isPresent()) {
        EstimatedRobotPose est = estimate.get();
        this.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds);
        lastProcessedTimestamp = est.timestampSeconds;
      }
    }
    var state = getStateCopy();

    inputs.pose = state.Pose;
    inputs.speeds = state.Speeds;
    inputs.moduleStates = state.ModuleStates;

    for (int i = 0; i < modules.length; i++) {
      Logger.log(
          "Subsystems/Swerve/Voltages/Drive/" + i,
          modules[i].getDriveMotor().getMotorVoltage().getValueAsDouble());
      Logger.log(
          "Subsystems/Swerve/Voltages/Steer/" + i,
          modules[i].getSteerMotor().getMotorVoltage().getValueAsDouble());
      // todo use status signals instead
    }
  }
}
