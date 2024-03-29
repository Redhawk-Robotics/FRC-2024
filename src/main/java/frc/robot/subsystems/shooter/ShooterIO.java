package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {}

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void shooterApplySpeed(double power) {}

  public default void indexerApplySpeed(double power) {}

  public default void shooterStop() {}

  public default void fullShot() {}

  public default void halfShot() {}

  public default void quarterShot() {}

  public default void threeQuarterShot() {}

  public default void sourceIntake() {}

  public default void chamferShot() {}

  public default void indexerStop() {}

  public default void indexerForward() {}

  public default void indexerInverse() {}
}
