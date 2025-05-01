package frc.robot;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import java.util.ArrayList;

public class Mechanism {
  private final XboxController controller = new XboxController(0);

  private final TalonFX motor;
  private final int leaderId;
  private final String leaderBus;
  private final ArrayList<TalonFX> followers;

  private TuningType tuningType;
  private Tuning tuning;
  private int stage;
  private double signal;
  public double baseSignal;

  private final ArrayList<Double> x = new ArrayList<>();
  private final ArrayList<Double> y = new ArrayList<>();

  public double kS, kG, kV;

  public enum TuningType {
    VOLTAGE(12, 0.01),
    CURRENT(120, 0.1);
    public final double max, step;

    TuningType(double max, double step) {
      this.max = max;
      this.step = step;
    }
  }

  public enum Tuning {
    NONE,
    FLYWHEEL_KS_KV,
    ARM_KS,
    ARM_KG,
    ELEVATOR_KG_KS,
  }

  public Mechanism(int id, String bus) {
    leaderId = id;
    leaderBus = bus;
    motor = new TalonFX(leaderId, leaderBus);
    followers = new ArrayList<>();
  }

  public Mechanism addFollower(int id, boolean oppositeLeader) {
    TalonFX motor = new TalonFX(id, leaderBus);
    motor.setControl(new Follower(leaderId, oppositeLeader));
    followers.add(motor);
    return this;
  }

  public Mechanism clearFollowers() {
    for (TalonFX follower : followers) {
      follower.setControl(new CoastOut());
    }
    followers.clear();
    return this;
  }

  public boolean isTuning() {
    return tuning == Tuning.NONE;
  }

  public Mechanism startTuning(TuningType tuningType, Tuning tuning, double baseSignal) {
    this.tuningType = tuningType;
    this.tuning = tuning;
    stage = 0;
    this.baseSignal = baseSignal;
    return this;
  }

  public Mechanism startTuning(TuningType tuningType, Tuning tuning) {
    return startTuning(tuningType, tuning, 0);
  }

  private boolean getStart() {
    return controller.getRightBumperButton();
  }

  private boolean getCancel() {
    return controller.getLeftBumperButton();
  }

  private void setSignal(double signal) {
    motor.setControl(
        switch (tuningType) {
          case VOLTAGE -> new VoltageOut(signal);
          case CURRENT -> new TorqueCurrentFOC(signal);
        });
  }

  public void update() {
    if (tuning == Tuning.NONE) {
      motor.setControl(new CoastOut());
      return;
    }
    switch (tuning) {
      case FLYWHEEL_KS_KV -> updateFlywheelKSKV();
      case ARM_KS -> updateArmKS();
      case ARM_KG -> updateArmKG();
      case ELEVATOR_KG_KS -> updateElevatorKGKS();
    }
  }

  private void updateFlywheelKSKV() {
    if (stage == 0) {
      if (!getStart()) {
        return;
      }
      stage++;
      signal = 0;
      x.clear();
      y.clear();
      return;
    }
    if (stage == 1) {
      double realSignal = signal + baseSignal;
      setSignal(realSignal);
      x.add(motor.getRotorVelocity(true).getValue().in(Units.RotationsPerSecond));
      y.add(signal);
      if (Math.abs(realSignal) >= tuningType.max || getCancel()) {
        stage++;
        return;
      }
      signal += tuningType.step;
      return;
    }
    if (stage == 2) {
      stage++;
      LineEst.Line line = LineEst.estimate(x, y);
      kS = line.b();
      kV = line.m();
      return;
    }
    tuning = Tuning.NONE;
  }

  private void updateArmKS() {
    if (stage == 0) {
      if (!getStart()) {
        return;
      }
      stage++;
      signal = 0;
      return;
    }
    if (stage == 1) {
      double realSignal = signal + baseSignal;
      setSignal(realSignal);
      if (Math.abs(motor.getRotorVelocity(true).getValue().in(Units.RotationsPerSecond)) > 1e-3
          || Math.abs(realSignal) >= tuningType.max
          || getCancel()) {
        kS = signal;
        stage++;
        return;
      }
      signal += tuningType.step;
      return;
    }
    tuning = Tuning.NONE;
  }

  private void updateArmKG() {
    if (stage == 0) {
      if (!getStart()) {
        return;
      }
      stage++;
      signal = 0;
      return;
    }
    if (stage == 1) {
      double realSignal = signal + baseSignal;
      setSignal(realSignal);
      if (Math.abs(realSignal) >= tuningType.max || getCancel()) {
        kG = signal;
        stage++;
        return;
      }
      if (controller.getPOV() == 0) {
        signal += tuningType.step;
      }
      if (controller.getPOV() == 180) {
        signal -= tuningType.step;
      }
      return;
    }
    tuning = Tuning.NONE;
  }

  private void updateElevatorKGKS() {
    if (stage == 0) {
      if (!getStart()) {
        return;
      }
      stage++;
      signal = 0;
      x.clear();
      y.clear();
      return;
    }
    if (stage == 1) {
      double realSignal = signal + baseSignal;
      setSignal(realSignal);
      if (Math.abs(motor.getRotorVelocity(true).getValue().in(Units.RotationsPerSecond)) > 1e-3
          || Math.abs(realSignal) >= tuningType.max
          || getCancel()) {
        x.add(signal);
        stage++;
        return;
      }
      signal += tuningType.step;
      return;
    }
    if (stage == 2) {
      double realSignal = signal + baseSignal;
      setSignal(realSignal);
      if (Math.abs(motor.getRotorVelocity(true).getValue().in(Units.RotationsPerSecond)) <= 1e-3
          || Math.abs(realSignal) >= tuningType.max
          || getCancel()) {
        x.add(signal);
        stage++;
        return;
      }
      signal -= tuningType.step;
      return;
    }
    if (stage == 3) {
      stage++;
      double min = Math.min(x.get(0), x.get(1));
      double max = Math.max(x.get(0), x.get(1));
      kG = (min + max) / 2;
      kS = (max - min) / 2;
      return;
    }
    tuning = Tuning.NONE;
  }
}
