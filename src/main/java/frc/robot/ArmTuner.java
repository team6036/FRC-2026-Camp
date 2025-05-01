package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmTuner {
  private final Mechanism mech;
  private final Mechanism.TuningType tuningType;

  private int stage;

  public ArmTuner(Mechanism mech, Mechanism.TuningType tuningType) {
    this.mech = mech;
    this.tuningType = tuningType;
    stage = 0;
  }

  public void update() {
    if (stage == 0) {
      stage++;
      mech.startTuning(tuningType, Mechanism.Tuning.ARM_KS);
      return;
    }
    if (stage == 1) {
      mech.update();
      if (!mech.isTuning()) {
        stage++;
      }
      return;
    }
    if (stage == 2) {
      stage++;
      mech.startTuning(tuningType, Mechanism.Tuning.ARM_KG, mech.kS);
      return;
    }
    if (stage == 3) {
      mech.update();
      if (!mech.isTuning()) {
        stage++;
      }
      return;
    }
    SmartDashboard.putNumber("Arm/kS", mech.kS);
    SmartDashboard.putNumber("Arm/kV", mech.kV);
    stage = stage % 4;
  }
}
