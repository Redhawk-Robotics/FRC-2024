package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public double driveVoltage = 0;
    public double driveEncoderPose = 0;
    public double driveVelocity = 0;
    public double driveTemp = 0;
    public double driveCurrentSetSpeed = 0;
    public double driveBusVoltage = 0;
    public double driveOutputCurrent = 0;
    public double driveVoltageCompensation = 0;

    public double angleVoltage = 0;
    public double angleEncoderPose = 0;
    public double angleVelocity = 0;
    public double angleTemp = 0;
    public double angleCurrentSetSpeed = 0;
    public double angleBusVoltage = 0;
    public double angleOutputCurrent = 0;
    public double angleVoltageCompensation = 0;
  }

  public default void updateInputs(ModuleIOInputs inputs) {}

  public default void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {}

  public Rotation2d getCanCoder();

  public SwerveModuleState getState();

  public SwerveModulePosition getPosition();
}
