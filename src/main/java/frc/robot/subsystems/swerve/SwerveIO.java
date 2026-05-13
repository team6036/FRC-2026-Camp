package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.SwerveConstants;

public class SwerveIO extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {

  public static class SwerveIOInputs {
    public Pose2d pose = new Pose2d();
    public ChassisSpeeds speeds = new ChassisSpeeds();
  }

  public SwerveIO() {
    super(
        TalonFX::new,
        TalonFX::new,
        CANcoder::new,
        new SwerveDrivetrainConstants()
            .withCANBusName(SwerveConstants.bus.getName())
            .withPigeon2Id(SwerveConstants.imuId),
        SwerveConstants.moduleConstants.get(0),
        SwerveConstants.moduleConstants.get(1),
        SwerveConstants.moduleConstants.get(2),
        SwerveConstants.moduleConstants.get(3));
  }

  public void updateInputs(SwerveIOInputs inputs) {
    var state = getStateCopy();

    inputs.pose = state.Pose;
    inputs.speeds = state.Speeds;
  }
}
