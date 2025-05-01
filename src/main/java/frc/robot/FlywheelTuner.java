package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlywheelTuner {
  private final Mechanism mech;
  private final Mechanism.TuningType tuningType;

  private int stage;

  public FlywheelTuner(Mechanism mech, Mechanism.TuningType tuningType) {
    this.mech = mech;
    this.tuningType = tuningType;
    stage = 0;
  }

  public void update() {
    if (stage == 0) {
      stage++;
      mech.startTuning(tuningType, Mechanism.Tuning.FLYWHEEL_KS_KV);
      return;
    }
    if (stage == 1) {
      mech.update();
      if (mech.isTuning()) {
        stage++;
      }
      return;
    }
    SmartDashboard.putNumber("Flywheel/kS", mech.kS);
    SmartDashboard.putNumber("Flywheel/kV", mech.kV);
    stage = stage % 2;
  }
}
