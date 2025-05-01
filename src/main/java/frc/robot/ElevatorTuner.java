package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorTuner {
  private final Mechanism mech;
  private final Mechanism.TuningType tuningType;

  private int stage;

  public ElevatorTuner(Mechanism mech, Mechanism.TuningType tuningType) {
    this.mech = mech;
    this.tuningType = tuningType;
    stage = 0;
  }

  public void update() {
    if (stage == 0) {
      stage++;
      mech.startTuning(tuningType, Mechanism.Tuning.ELEVATOR_KG_KS);
      return;
    }
    if (stage == 1) {
      mech.update();
      if (!mech.isTuning()) {
        stage++;
      }
      return;
    }
    SmartDashboard.putNumber("Elevator/kS", mech.kS);
    SmartDashboard.putNumber("Elevator/kG", mech.kG);
    stage = stage % 2;
  }
}
