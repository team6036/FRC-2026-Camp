package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class IntakeConstants {
  /* CAN */
  public static final CANBus bus = RobotConstants.rio;
  public static final int leftMotor = 20;
  public static final int rightMotor = 21;

  public static final double gearRatio = 1;

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kG = 0;
  public static final double kA = 0;

  public static final double intakeVoltage = 10;
}
