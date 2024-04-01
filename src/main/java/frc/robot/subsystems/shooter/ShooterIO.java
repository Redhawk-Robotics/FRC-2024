package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterInputs {
    boolean on = false;
  }

  public default void updateInputs(ShooterInputs inputs) {}

  public default void applyShooterSpeed(double power) {}

  public default void applySupportWheelSpeeds(double guardPower, double uptakePower) {}

  public default void indexerApplySpeed(double power) {}

  public boolean shooterSensorsEnabled();
}
