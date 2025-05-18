package frc.robot.tuners;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LineEst;
import frc.robot.Mechanism;
import java.util.ArrayList;

public class FlywheelTuner extends TickingTuner {
  private enum Stage {
    WAIT,
    RAMP_KS,
    RAMP_KV,
  }

  private Stage stage;

  private int step;
  private ArrayList<Double> x, y;

  public FlywheelTuner(Mechanism mech) {
    super(mech, 0.1);
  }

  @Override
  public void reset() {
    super.reset();

    mech.setBaseSignal(0);

    stage = Stage.WAIT;
    step = 0;
    x = new ArrayList<>();
    y = new ArrayList<>();
  }

  @Override
  public void update() {
    super.update();

    switch (stage) {
      case WAIT -> {
        if (getNext()) stage = Stage.RAMP_KS;
        mech.setSignal(0);
      }
      case RAMP_KS -> {
        if (trigger) step++;
        if (getNext() || (trigger && mech.isMoving())) {
          stage = Stage.RAMP_KV;
          SmartDashboard.putNumber("FlywheelTuner/kS", mech.getWholeSignal());
          break;
        }
        mech.setSignal(step * mech.output.step);
      }
      case RAMP_KV -> {
        if (trigger) {
          x.add(mech.getSpeed());
          y.add(mech.getWholeSignal());
          step++;
        }
        if (getNext() || (trigger && step * mech.output.step >= mech.output.max)) {
          stage = Stage.WAIT;
          LineEst.Line line = LineEst.estimate(x, y);
          SmartDashboard.putNumber("FlywheelTuner/kSAlt", line.b());
          SmartDashboard.putNumber("FlywheelTuner/kV", line.m());
          break;
        }
        mech.setSignal(step * mech.output.step);
      }
    }

    mech.update();
  }
}
