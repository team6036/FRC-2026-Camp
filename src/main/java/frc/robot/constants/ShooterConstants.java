package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class ShooterConstants {
    /* CAN */
    public static final CANBus bus = RobotConstants.rio;
    public static final int bottomLeftMotorId = 101;
    public static final int bottomRightMotorId = 102;
    public static final int topLeftMotorId = 103;
    public static final int topRightMotorId = 104;

    /* Control */
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    /* Setpoints */
    public static final double desiredVelocity = 30d;
}
