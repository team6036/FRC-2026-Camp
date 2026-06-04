package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.util.Logger;

public class ShooterSubsystem extends SubsystemBase {

    public enum Mode {
        OFF,
        SHOOTING,
    }

    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();
    private Mode wantedMode = Mode.OFF;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    public void setMode(Mode mode) {
        this.wantedMode = mode;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.log("Subsystems/Shooter/BottomLeftVelocity", inputs.bottomLeftVelocityRPS);
        Logger.log("Subsystems/Shooter/BottomRightVelocity", inputs.bottomRightVelocityRPS);
        Logger.log("Subsystems/Shooter/TopLeftVelocity", inputs.topLeftVelocityRPS);
        Logger.log("Subsystems/Shooter/TopRightVelocity", inputs.topRightVelocityRPS);

        switch (wantedMode) {
            case SHOOTING:
                io.setVelocity(ShooterConstants.desiredVelocity);
                break;
            case OFF:
            default:
                io.stop();
        }
    }
}