package frc.robot.tuners;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Mechanism;

public class TickingTuner {
  protected final XboxController controller = new XboxController(0);

  public final Mechanism mech;

  private final Timer triggerTimer = new Timer();
  protected boolean trigger;
  protected double triggerPeriod;

  public TickingTuner(Mechanism mech, double triggerPeriod) {
    this.mech = mech;

    triggerTimer.restart();
    trigger = false;
    this.triggerPeriod = triggerPeriod;
  }

  public TickingTuner(Mechanism mech) {
    this(mech, 1);
  }

  public final boolean getNext() {
    return controller.getRightBumperButtonPressed();
  }

  public void reset() {
    triggerTimer.restart();
    trigger = false;
  }

  public void update() {
    trigger = false;
    if (triggerTimer.hasElapsed(triggerPeriod)) {
      triggerTimer.restart();
      trigger = true;
    }
  }
}
