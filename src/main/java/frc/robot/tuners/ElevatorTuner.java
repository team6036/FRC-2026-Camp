package frc.robot.tuners;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanism;

public class ElevatorTuner extends TickingTuner {
  private enum Stage {
    WAIT,
    RAMP_KG_MAX,
    ELEVATE,
    RAMP_KG_MIN,
  }

  private Stage stage;

  private int step;
  private double kGMax;
  private int elevateTimer;

  public ElevatorTuner(Mechanism mech) {
    super(mech, 0.25);
  }

  @Override
  public void reset() {
    super.reset();

    mech.setBaseSignal(0);

    stage = Stage.WAIT;
    step = 0;
    kGMax = 0;
    elevateTimer = 0;
  }

  @Override
  public void update() {
    super.update();

    switch (stage) {
      case WAIT -> {
        if (getNext()) stage = Stage.RAMP_KG_MAX;
        mech.setSignal(0);
      }
      case RAMP_KG_MAX -> {
        if (trigger) step++;
        if (getNext() || (trigger && mech.isMoving())) {
          stage = Stage.ELEVATE;
          kGMax = mech.getWholeSignal();
          SmartDashboard.putNumber("ElevatorTuner/kGMax", kGMax);
          break;
        }
        mech.setSignal(step * mech.output.step);
      }
      case ELEVATE -> {
        if (trigger) elevateTimer++;
        if (getNext() || elevateTimer > 2) stage = Stage.RAMP_KG_MIN;
      }
      case RAMP_KG_MIN -> {
        if (trigger) step--;
        if (getNext() || (trigger && mech.isStatic())) {
          stage = Stage.WAIT;
          double kGMin = mech.getWholeSignal();
          SmartDashboard.putNumber("ElevatorTuner/kGMin", kGMin);
          SmartDashboard.putNumber("ElevatorTuner/kG", (kGMax + kGMin) / 2);
          SmartDashboard.putNumber("ElevatorTuner/kS", (kGMax - kGMin) / 2);
          break;
        }
        mech.setSignal(step * mech.output.step);
      }
    }

    mech.update();
  }
}
