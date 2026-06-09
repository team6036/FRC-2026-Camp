package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import java.util.function.Consumer;

public class NTFullGains {
  private final NTDouble kP;
  private final NTDouble kI;
  private final NTDouble kD;
  private final NTDouble kS;
  private final NTDouble kV;
  private final NTDouble kA;
  private final NTDouble kG;

  public NTFullGains(
      String name, double p, double i, double d, double s, double v, double a, double g) {
    name = name + '/';
    kP = new NTDouble(name + "kP", p);
    kI = new NTDouble(name + "kI", i);
    kD = new NTDouble(name + "kD", d);
    kS = new NTDouble(name + "kS", s);
    kV = new NTDouble(name + "kV", v);
    kA = new NTDouble(name + "kA", a);
    kG = new NTDouble(name + "kG", g);
  }

  public boolean shouldUpdate() {
    boolean update =
        kP.shouldUpdate()
            || kI.shouldUpdate()
            || kD.shouldUpdate()
            || kS.shouldUpdate()
            || kV.shouldUpdate()
            || kA.shouldUpdate()
            || kG.shouldUpdate();
    Logger.log("Test/ShouldUpdate", update);
    return update;
  }

  public void update(TalonFXConfiguration config) {
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kS = kS.get();
    config.Slot0.kV = kV.get();
    config.Slot0.kA = kA.get();
    config.Slot0.kG = kG.get();
  }

  public void update(TalonFXConfigurator configurator) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    configurator.refresh(config);
    update(config);
    configurator.apply(config);
  }

  public void update(TalonFXConfigurator configurator, Consumer<TalonFXConfiguration> updater) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    configurator.refresh(config);
    update(config); // ok so I think I have this right but it firstly yoinks values from our NT
    updater.accept(
        config); // and then with those values, this scales kP by the appropriate amount for the
    // module type
    configurator.apply(config); // and then we apply this one again
  }
}
