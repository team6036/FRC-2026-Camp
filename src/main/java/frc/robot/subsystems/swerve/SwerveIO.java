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
import java.util.List;
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

    this.camera = new PhotonCamera(VisionConstants.shooterCamera.name);
    this.poseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.fieldLayout, VisionConstants.shooterCamera.robotToCamera);
  }

  public boolean resetPoseFromVision() {
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      if (!result.hasTargets()) {
        continue;
      }

      Optional<EstimatedRobotPose> estimate =
          result.getMultiTagResult().isPresent()
              ? poseEstimator.estimateCoprocMultiTagPose(result)
              : poseEstimator.estimateLowestAmbiguityPose(result);

      if (estimate.isPresent()) {
        resetPose(estimate.get().estimatedPose.toPose2d());
        lastProcessedTimestamp = estimate.get().timestampSeconds;
        return true;
      }
    }
    return false;
  }

  public void updateInputs(SwerveIOInputs inputs) {
    Logger.log("Subsystems/Swerve/Vision/CameraConfigured", camera.isConnected());

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    Logger.log("Subsystems/Swerve/Vision/NumResults", results.size());
    for (PhotonPipelineResult result : results) {
      if (result.getTimestampSeconds() <= lastProcessedTimestamp) {
        continue;
      }

      Logger.log("Subsystems/Swerve/Vision/HasTargets", result.hasTargets());
      Optional<EstimatedRobotPose> estimate;
      if (result.getMultiTagResult().isPresent()) {
        estimate = poseEstimator.estimateCoprocMultiTagPose(result);
      } else {
        estimate = poseEstimator.estimateLowestAmbiguityPose(result);
      }
      Logger.log("Subsystems/Swerve/Vision/HasEstimate", estimate.isPresent());

      if (estimate.isPresent()) {
        EstimatedRobotPose est = estimate.get();
        Logger.log("Subsystems/Swerve/Vision/Estimate", est.estimatedPose.toPose2d());
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
