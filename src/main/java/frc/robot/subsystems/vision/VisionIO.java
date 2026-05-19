package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.CameraConfiguration;
import frc.robot.constants.VisionConstants.Piece;
import frc.robot.util.DartCamera;
import java.util.ArrayList;
import java.util.List;

public class VisionIO {

  private final CameraConfiguration configuration;
  private final DartCamera camera;

  public static class VisionIOInputs {
    public boolean hasMeasurement = false;
    public Pose2d robotPose = new Pose2d();
    public double timestamp = 0d;

    public List<Piece> targets = new ArrayList<>();
  }

  public VisionIO(CameraConfiguration configuration) {
    this.configuration = configuration;
    this.camera = new DartCamera(configuration.name);
  }

  public void updateInputs(VisionIOInputs inputs) {
    List<DartCamera.AprilTagResult> aprilTagResults = camera.aprilTag3d().readAprilTagResults();

    DartCamera.PnPResult latestPnPResult = null;
    double latestTimestamp = -1;

    for (DartCamera.AprilTagResult result : aprilTagResults) {
      if (result.numPnpResults > 0 && result.captureTimestamp > latestTimestamp) {
        latestTimestamp = result.captureTimestamp;
        latestPnPResult = result.pnpResults[0];
      }
    }

    if (latestPnPResult != null) {
      inputs.hasMeasurement = true;
      inputs.timestamp = latestTimestamp;
      inputs.robotPose =
          new Pose2d(
              latestPnPResult.pose.getTranslation().toTranslation2d(),
              latestPnPResult.pose.getRotation().toRotation2d());
    } else {
      inputs.hasMeasurement = false;
    }

    List<DartCamera.ColorResult> colorResults = camera.colorThreshold().readColorResults();
    inputs.targets.clear();

    double currentTime = Timer.getFPGATimestamp();
    for (DartCamera.ColorResult result : colorResults) {
      if (currentTime - result.captureTimestamp < VisionConstants.pieceStaleTime) {
        for (DartCamera.ColorTarget target : result.targets) {
          Pose2d targetPose =
              new Pose2d(
                  getPositionFromDetection(
                      VisionConstants.pieceHeight,
                      Math.toRadians(target.pitch),
                      Math.toRadians(target.yaw),
                      inputs.robotPose),
                  inputs.robotPose.getRotation());
          inputs.targets.add(new Piece(target, targetPose));
        }
      }
    }
  }

  public Translation2d getPositionFromDetection(
      double pieceHeight, double pitchRadians, double yawRadians, Pose2d robotPose) {
    Translation3d cameraToDetection3dNorm =
        new Translation3d(1, new Rotation3d(0, -pitchRadians, -yawRadians))
            .rotateBy(this.configuration.robotToCamera.getRotation());
    Translation2d cameraToDetection =
        cameraToDetection3dNorm
            .toTranslation2d()
            .times(
                -(this.configuration.robotToCamera.getZ() - pieceHeight)
                    / cameraToDetection3dNorm.getZ())
            .rotateBy(robotPose.getRotation());
    Translation2d fieldToCamera =
        robotPose
            .getTranslation()
            .plus(
                this.configuration
                    .robotToCamera
                    .getTranslation()
                    .toTranslation2d()
                    .rotateBy(robotPose.getRotation()));
    return fieldToCamera.plus(cameraToDetection);
  }
}
