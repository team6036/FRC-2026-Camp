package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  /* Field Layout */
  public static final AprilTagFieldLayout fieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  /* Pose Estimator */
  public static final Matrix<N3, N1> visionStDev =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(6));

  /* Camera Configuration */
  public static class CameraConfiguration {
    public final String name;
    public final Transform3d robotToCamera;

    public CameraConfiguration(String name, Transform3d robotToCamera) {
      this.name = name;
      this.robotToCamera = robotToCamera;
    }
  }

  public static final CameraConfiguration camera =
      new CameraConfiguration("Camera", new Transform3d());

  public static final double frameWidth = 1200d;
}
