package frc.robot.tuners;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Mechanism;

public class ManualTuner extends TickingTuner {
  private int step;

  public ManualTuner(Mechanism mech) {
    super(mech);
  }

  @Override
  public void reset() {
    super.reset();

    step = 0;
  }

  @Override
  public void update() {
    super.update();

    if (controller.getPOV() == 0) step++;
    if (controller.getPOV() == 180) step--;
    SmartDashboard.putNumber("CustomTuner/step", step);
    mech.setSignal(MathUtil.clamp(step * mech.output.step, -mech.output.max, mech.output.max));
    mech.update();
  }
}
