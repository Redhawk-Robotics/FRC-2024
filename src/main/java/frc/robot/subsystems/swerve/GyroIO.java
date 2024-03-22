package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double rollPosition = 0.0;
    public double pitchPosition = 0.0;
    public double yawPosition = 0.0;
    public double rollVelocity = 0.0;
    public double pitchVelocity = 0.0;
    public double yawVelocity = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {
  }
}
