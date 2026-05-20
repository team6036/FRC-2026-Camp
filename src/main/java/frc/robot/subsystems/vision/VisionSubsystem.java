package frc.robot.subsystems.vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.Piece;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.Logger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

  private final List<Pair<VisionIO, VisionIOInputs>> inputsList = new ArrayList<>();
  Optional<Piece> bestTarget = Optional.empty();

  public VisionSubsystem(VisionIO... visionIOList) {
    for (VisionIO visionIO : visionIOList) {
      inputsList.add(new Pair<>(visionIO, new VisionIOInputs()));
    }
  }

  public void addVisionEstimates(SwerveSubsystem swerve) {
    for (Pair<VisionIO, VisionIOInputs> camera : inputsList) {
      VisionIOInputs inputs = camera.getSecond();
      if (inputs.hasMeasurement) {
        swerve.poseEstimator.addVisionMeasurement(inputs.robotPose, inputs.timestamp);
      }
    }
  }

  @Override
  public void periodic() {
    for (Pair<VisionIO, VisionIOInputs> camera : inputsList) {
      camera.getFirst().updateInputs(camera.getSecond());
    }

    if (bestTarget.isPresent()
        && (inputsList.get(0).getSecond().timestamp - bestTarget.get().timestamp)
            > VisionConstants.pieceStaleTime) {
      bestTarget = Optional.empty();
    }

    double maxArea = -1;

    for (Pair<VisionIO, VisionIOInputs> camera : inputsList) {
      VisionIOInputs inputs = camera.getSecond();
      for (Piece piece : inputs.targets) {
        if (piece.target.area > maxArea) {
          maxArea = piece.target.area;
          bestTarget = Optional.of(piece);
        }
      }
    }

    if (bestTarget.isPresent()) {
      Logger.log("Subsystems/Vision/BestTarget/Pose", bestTarget.get().pose);
    }
  }
}
