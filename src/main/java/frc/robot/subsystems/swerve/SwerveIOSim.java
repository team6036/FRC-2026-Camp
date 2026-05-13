package frc.robot.subsystems.swerve;

public class SwerveIOSim extends SwerveIO {

  public void updateSim() {
    updateSimState(0.02, 12.0);
  }
}
