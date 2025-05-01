// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private final FlywheelTuner flywheelTuner;
  private final ArmTuner armTuner;
  private final ElevatorTuner elevatorTuner;

  public Robot() {
    super(0.02);

    flywheelTuner = new FlywheelTuner(new Mechanism(0, ""), Mechanism.TuningType.VOLTAGE);
    armTuner = new ArmTuner(new Mechanism(0, ""), Mechanism.TuningType.VOLTAGE);
    elevatorTuner = new ElevatorTuner(new Mechanism(0, ""), Mechanism.TuningType.VOLTAGE);
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    flywheelTuner.update();
    armTuner.update();
    elevatorTuner.update();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
