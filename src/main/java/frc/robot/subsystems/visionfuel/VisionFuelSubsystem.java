package frc.robot.subsystems.visionfuel;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.VisionConstants.Piece;
import frc.robot.subsystems.visionfuel.VisionFuelIO.VisionFuelInputs;
import frc.robot.util.Logger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionFuelSubsystem extends SubsystemBase {

  private final List<Pair<VisionFuelIO, VisionFuelInputs>> inputsList = new ArrayList<>();
  Optional<Piece> bestTarget = Optional.empty();

  public VisionFuelSubsystem(VisionFuelIO... visionFuelIOList) {
    for (VisionFuelIO visionFuelIO : visionFuelIOList) {
      inputsList.add(new Pair<>(visionFuelIO, new VisionFuelInputs()));
    }
  }

  @Override
  public void periodic() {
    for (Pair<VisionFuelIO, VisionFuelInputs> camera : inputsList) {
      camera.getFirst().updateInputs(camera.getSecond());
    }

    if (bestTarget.isPresent()
        && (Timer.getFPGATimestamp() - bestTarget.get().timestamp)
            > VisionConstants.pieceStaleTime) {
      bestTarget = Optional.empty();
    }

    double maxArea = -1;

    for (Pair<VisionFuelIO, VisionFuelInputs> camera : inputsList) {
      VisionFuelInputs inputs = camera.getSecond();
      for (Piece piece : inputs.targets) {
        if (piece.target.area > maxArea) {
          maxArea = piece.target.area;
          bestTarget = Optional.of(piece);
        }
      }
    }

    if (bestTarget.isPresent()) {
      Logger.log("Subsystems/VisionFuel/BestTarget/Pose", bestTarget.get().pose);
    }
  }
}
