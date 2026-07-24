package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.constants.IntakeConstants;

public class IntakeIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final boolean intakeLeftPresent;
  private final boolean intakeRightPresent;

  private final CoastOut coastRequest;
  private final VoltageOut voltageRequest;

  public static class IntakeIOInputs {
    public double voltage;

    public boolean intakeLeftPresent;
    public boolean intakeRightPresent;
  }

  public IntakeIO() {
    leftMotor = new TalonFX(IntakeConstants.leftMotor, IntakeConstants.bus);
    rightMotor = new TalonFX(IntakeConstants.rightMotor, IntakeConstants.bus);

    intakeLeftPresent = leftMotor.isConnected();
    intakeRightPresent = rightMotor.isConnected();

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = IntakeConstants.gearRatio;
    config.Slot0.kP = IntakeConstants.kP;
    config.Slot0.kI = IntakeConstants.kI;
    config.Slot0.kD = IntakeConstants.kD;
    config.Slot0.kS = IntakeConstants.kS;
    config.Slot0.kV = IntakeConstants.kV;
    config.Slot0.kG = IntakeConstants.kG;
    config.Slot0.kA = IntakeConstants.kA;
    config.MotorOutput.Inverted =
        leftMotor.isConnected()
            ? InvertedValue.CounterClockwise_Positive
            : InvertedValue.Clockwise_Positive;

    leftMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);

    coastRequest = new CoastOut();
    voltageRequest = new VoltageOut(0);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.voltage = leftMotor.getMotorVoltage().getValueAsDouble();
    if (inputs.intakeRightPresent && !intakeLeftPresent) {
      inputs.voltage = rightMotor.getMotorVoltage().getValueAsDouble();
    }

    inputs.intakeLeftPresent = intakeLeftPresent;
    inputs.intakeRightPresent = intakeRightPresent;
  }

  public void setVoltage(double voltage) {
    leftMotor.setControl(voltageRequest.withOutput(-voltage));
    if (intakeRightPresent && !intakeLeftPresent) {
      rightMotor.setControl(voltageRequest.withOutput(voltage));
    }
  }

  public void setCoast() {
    leftMotor.setControl(coastRequest);
    if (intakeRightPresent && !intakeLeftPresent) {
      rightMotor.setControl(coastRequest);
    }
  }
}
