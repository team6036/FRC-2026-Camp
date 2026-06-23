package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.ShooterConstants;

public class ShooterIO {

  public final TalonFX topLeftMotor;
  public final TalonFX topRightMotor;
  public final TalonFX bottomLeftMotor;
  public final TalonFX bottomRightMotor;

  private final boolean bottomLeftPresent;
  private final boolean bottomRightPresent;

  private final StatusSignal<AngularVelocity> topLeftVelocity;
  private final StatusSignal<AngularVelocity> topRightVelocity;
  private final StatusSignal<AngularVelocity> bottomLeftVelocity;
  private final StatusSignal<AngularVelocity> bottomRightVelocity;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public static class ShooterIOInputs {
    public double topLeftVelocityRPS = 0.0;
    public double topRightVelocityRPS = 0.0;
    public double bottomLeftVelocityRPS = 0.0;
    public double bottomRightVelocityRPS = 0.0;

    public boolean bottomLeftPresent = false;
    public boolean bottomRightPresent = false;
  }

  public ShooterIO() {
    topLeftMotor = new TalonFX(ShooterConstants.topLeftMotorId, ShooterConstants.bus);
    topRightMotor = new TalonFX(ShooterConstants.topRightMotorId, ShooterConstants.bus);
    bottomLeftMotor = new TalonFX(ShooterConstants.bottomLeftMotorId, ShooterConstants.bus);
    bottomRightMotor = new TalonFX(ShooterConstants.bottomRightMotorId, ShooterConstants.bus);

    bottomLeftPresent = bottomLeftMotor.isConnected();
    bottomRightPresent = bottomRightMotor.isConnected();

    topLeftVelocity = topLeftMotor.getVelocity();
    topRightVelocity = topRightMotor.getVelocity();
    bottomLeftVelocity = bottomLeftMotor.getVelocity();
    bottomRightVelocity = bottomRightMotor.getVelocity();

    configurePID(
        ShooterConstants.kP,
        ShooterConstants.kI,
        ShooterConstants.kD,
        ShooterConstants.kS,
        ShooterConstants.kV,
        ShooterConstants.kG,
        ShooterConstants.kA);

    MotorOutputConfigs motorAlignment = new MotorOutputConfigs();
    motorAlignment.Inverted = InvertedValue.Clockwise_Positive;
    topLeftMotor.getConfigurator().apply(motorAlignment);
    bottomRightMotor.getConfigurator().apply(motorAlignment);
    motorAlignment.Inverted = InvertedValue.CounterClockwise_Positive;
    topRightMotor.getConfigurator().apply(motorAlignment);
    bottomLeftMotor.getConfigurator().apply(motorAlignment);

    topRightMotor.setControl(
        new Follower(ShooterConstants.topLeftMotorId, MotorAlignmentValue.Opposed));
    if (bottomLeftPresent && bottomRightPresent) {
      bottomRightMotor.setControl(
          new Follower(ShooterConstants.bottomLeftMotorId, MotorAlignmentValue.Opposed));
    }
  }

  public void configurePID(
      double kP, double kI, double kD, double kS, double kV, double kG, double kA) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    topLeftMotor.getConfigurator().refresh(config); // Need to read current config into config first
    topRightMotor.getConfigurator().refresh(config);
    bottomLeftMotor.getConfigurator().refresh(config);
    bottomRightMotor.getConfigurator().refresh(config);

    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kG = kG;
    config.Slot0.kA = kA;

    topLeftMotor.getConfigurator().apply(config);
    topRightMotor.getConfigurator().apply(config);
    bottomLeftMotor.getConfigurator().apply(config);
    bottomRightMotor.getConfigurator().apply(config);
  }

  public TalonFXConfigurator[] getConfigurators() {
    return new TalonFXConfigurator[] {
      topLeftMotor.getConfigurator(),
      topRightMotor.getConfigurator(),
      bottomLeftMotor.getConfigurator(),
      bottomRightMotor.getConfigurator()
    };
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.topLeftVelocityRPS = topLeftVelocity.refresh().getValueAsDouble();
    inputs.topRightVelocityRPS = topRightVelocity.refresh().getValueAsDouble();
    inputs.bottomLeftVelocityRPS = bottomLeftVelocity.refresh().getValueAsDouble();
    inputs.bottomRightVelocityRPS = bottomRightVelocity.refresh().getValueAsDouble();

    inputs.bottomLeftPresent = bottomLeftPresent;
    inputs.bottomRightPresent = bottomRightPresent;
  }

  public void setTopVelocity(double velocityRPS) {
    topLeftMotor.setControl(velocityRequest.withVelocity(-velocityRPS));
  }

  public void setBottomVelocity(double velocityRPS) {
    bottomLeftMotor.setControl(velocityRequest.withVelocity(-velocityRPS));
    if (!bottomLeftPresent && bottomRightPresent)
      bottomRightMotor.setControl(velocityRequest.withVelocity(-velocityRPS));
  }

  public void stop() {
    topLeftMotor.stopMotor();
    if (bottomLeftPresent) {
      bottomLeftMotor.stopMotor();
    } else if (bottomRightPresent) {
      bottomRightMotor.stopMotor();
    }
  }
}
