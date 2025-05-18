package frc.robot;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

public class Mechanism {
  private final TalonFX motor;
  private final int leaderId;
  private final String leaderBus;
  private final ArrayList<TalonFX> followers;

  public enum Output {
    VOLTAGE(12, 0.01),
    TORQUE(120, 0.1);
    public final double max, step;

    Output(double max, double step) {
      this.max = max;
      this.step = step;
    }
  }

  public final Output output;
  private final boolean invert;
  private double baseSignal = 0;
  private double signal = 0;

  private final CoastOut coastControl = new CoastOut();
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final TorqueCurrentFOC torqueCurrentControl = new TorqueCurrentFOC(0);

  public Mechanism(int id, String bus, Output output, boolean invert, boolean foc) {
    leaderId = id;
    leaderBus = bus;
    motor = new TalonFX(leaderId, leaderBus);
    followers = new ArrayList<>();

    this.output = output;
    this.invert = invert;

    this.voltageControl.EnableFOC = foc;
  }

  public Mechanism addFollower(int id, boolean oppositeLeader) {
    TalonFX motor = new TalonFX(id, leaderBus);
    motor.setControl(new Follower(leaderId, oppositeLeader));
    followers.add(motor);
    return this;
  }

  public Mechanism clearFollowers() {
    for (TalonFX follower : followers) follower.setControl(coastControl);
    followers.clear();
    return this;
  }

  public double getBaseSignal() {
    return baseSignal;
  }

  public void setBaseSignal(double baseSignal) {
    this.baseSignal = baseSignal;
  }

  public double getSignal() {
    return signal;
  }

  public void setSignal(double signal) {
    this.signal = signal;
  }

  public double getWholeSignal() {
    return baseSignal + signal;
  }

  public void update() {
    SmartDashboard.putNumber("Mechanism/baseSignal", baseSignal);
    SmartDashboard.putNumber("Mechanism/signal", signal);
    SmartDashboard.putNumber("Mechanism/wholeSignal", getWholeSignal());
    SmartDashboard.putBoolean("Mechanism/isMoving", isMoving());
    double wholeSignal = (invert ? -1 : 1) * getWholeSignal();
    motor.setControl(
        switch (output) {
          case VOLTAGE -> voltageControl.withOutput(wholeSignal);
          case TORQUE -> torqueCurrentControl.withDeadband(wholeSignal);
        });
  }

  public double getSpeed() {
    return Math.abs(motor.getVelocity().getValue().in(Units.RotationsPerSecond));
  }

  public boolean isMoving() {
    return getSpeed() > 1e-3;
  }

  public boolean isStatic() {
    return !isMoving();
  }
}
