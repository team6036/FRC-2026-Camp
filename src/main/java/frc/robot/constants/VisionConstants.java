package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.DartCamera;

public class VisionConstants {
  public static final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final CameraConfiguration camera =
      new CameraConfiguration("Camera", new Transform3d());

  public static final double pieceHeight = Units.inchesToMeters(5.91);
  public static final double pieceStaleTime = 0.5;

  public static class Piece {
    public final DartCamera.ColorTarget target;
    public final Pose2d pose;

    public Piece(DartCamera.ColorTarget target, Pose2d pose) {
      this.target = target;
      this.pose = pose;
    }
  }

  public static class CameraConfiguration {
    public final String name;
    public final Transform3d robotToCamera;

    public CameraConfiguration(String name, Transform3d robotToCamera) {
      this.name = name;
      this.robotToCamera = robotToCamera;
    }
  }
}
