package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.constants.ShooterConstants;

public class ShooterIO {

    private final TalonFX bottomLeftMotor = new TalonFX(ShooterConstants.bottomLeftMotorId,  ShooterConstants.bus);
    private final TalonFX bottomRightMotor = new TalonFX(ShooterConstants.bottomRightMotorId, ShooterConstants.bus);
    private final TalonFX topLeftMotor = new TalonFX(ShooterConstants.topLeftMotorId,     ShooterConstants.bus);
    private final TalonFX topRightMotor = new TalonFX(ShooterConstants.topRightMotorId,    ShooterConstants.bus);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public static class ShooterIOInputs {
        public double bottomLeftVelocityRPS = 0.0;
        public double bottomRightVelocityRPS = 0.0;
        public double topLeftVelocityRPS = 0.0;
        public double topRightVelocityRPS = 0.0;
    }

    public ShooterIO() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;
        config.Slot0.kS = ShooterConstants.kS;
        config.Slot0.kV = ShooterConstants.kV;
        config.Slot0.kA = ShooterConstants.kA;

        bottomLeftMotor.getConfigurator().apply(config);
        topLeftMotor.getConfigurator().apply(config);

        bottomRightMotor.setControl(new Follower(ShooterConstants.bottomLeftMotorId, MotorAlignmentValue.Opposed));
        topRightMotor.setControl(new Follower(ShooterConstants.topLeftMotorId, MotorAlignmentValue.Opposed));

    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.bottomLeftVelocityRPS = bottomLeftMotor.getVelocity().getValueAsDouble();
        inputs.bottomRightVelocityRPS = bottomRightMotor.getVelocity().getValueAsDouble();
        inputs.topLeftVelocityRPS = topLeftMotor.getVelocity().getValueAsDouble();
        inputs.topRightVelocityRPS = topRightMotor.getVelocity().getValueAsDouble();
    }

    public void setVelocity(double velocityRPS) {
        bottomLeftMotor.setControl(velocityRequest.withVelocity(velocityRPS));
        topLeftMotor.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    public void stop() {
        bottomLeftMotor.stopMotor();
        topLeftMotor.stopMotor();
    }
}
