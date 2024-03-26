package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double roll = 0.0;
    public double pitch = 0.0;
    public double yaw = 0.0;
    public double angle = 0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void reset() {}

  public default void setYaw(double deg) {}

  public Rotation2d getRotation2d();
}
