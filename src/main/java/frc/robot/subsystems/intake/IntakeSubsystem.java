package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import frc.robot.util.Logger;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIO.IntakeIOInputs inputs = new IntakeIOInputs();
  private double wantedVoltage = 0;

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public void run() {
    this.wantedVoltage = IntakeConstants.intakeVoltage;
  }

  public void stop() {
    this.wantedVoltage = 0;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.log("Subsystems/Intake/Actual/Voltage", inputs.voltage);
    Logger.log("Subsystems/Intake/Wanted/Voltage", wantedVoltage);

    io.setVoltage(wantedVoltage);
  }
}
