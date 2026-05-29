package frc.robot.subsystems.visionfuel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.Piece;
import frc.robot.util.DartCamera;
import java.util.ArrayList;
import java.util.List;

public class VisionFuelIO {

  private final DartCamera camera;
  private final TimeInterpolatableBuffer<Pose2d> robotPoseBuffer;

  public static class VisionFuelInputs {
    public List<Piece> targets = new ArrayList<>();
  }

  public VisionFuelIO(TimeInterpolatableBuffer<Pose2d> robotPoseBuffer) {
    this.camera = new DartCamera(VisionConstants.camera.name);
    this.robotPoseBuffer = robotPoseBuffer;
  }

  public void updateInputs(VisionFuelInputs inputs) {
    double now = Timer.getFPGATimestamp();

    List<DartCamera.ColorResult> colorResults = camera.readColorResults();
    inputs.targets.clear();

    for (DartCamera.ColorResult result : colorResults) {
      if (now - result.captureTimestamp < VisionConstants.pieceStaleTime) {
        final Pose2d robotPoseAtCapture =
            robotPoseBuffer.getSample(result.captureTimestamp).orElse(Pose2d.kZero);

        for (DartCamera.ColorTarget target : result.targets) {
          Pose2d targetPose =
              new Pose2d(
                  getPositionFromDetection(
                      VisionConstants.pieceHeight,
                      Math.toRadians(target.pitch),
                      Math.toRadians(target.yaw),
                      robotPoseAtCapture),
                  robotPoseAtCapture.getRotation());

          inputs.targets.add(new Piece(target, targetPose, result.captureTimestamp));
        }
      }
    }
  }

  public Translation2d getPositionFromDetection(
      double pieceHeight, double pitchRadians, double yawRadians, Pose2d robotPose) {
    Translation3d cameraToDetection3dNorm =
        new Translation3d(1, new Rotation3d(0, -pitchRadians, -yawRadians))
            .rotateBy(VisionConstants.camera.robotToCamera.getRotation());
    Translation2d cameraToDetection =
        cameraToDetection3dNorm
            .toTranslation2d()
            .times(
                -(VisionConstants.camera.robotToCamera.getZ() - pieceHeight)
                    / cameraToDetection3dNorm.getZ())
            .rotateBy(robotPose.getRotation());
    Translation2d fieldToCamera =
        robotPose
            .getTranslation()
            .plus(
                VisionConstants.camera
                    .robotToCamera
                    .getTranslation()
                    .toTranslation2d()
                    .rotateBy(robotPose.getRotation()));
    return fieldToCamera.plus(cameraToDetection);
  }
}
