package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double roll = 0.0;
    public double pitch = 0.0;
    public double yaw = 0.0;
    public Rotation2d rotation = new Rotation2d();
    public double angle = 0;
  }

  public default void updateInputs(GyroIOInputs inputs) {
  }

  public default void reset() {
  }

  public default void setYaw(double deg) {
  }
}
