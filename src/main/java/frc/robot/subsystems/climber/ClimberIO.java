package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberInputs {}

  public default void updateInputs(ClimberInputs inputs) {}

  public default void setReference(double targetPosition) {}

  public boolean atReference();

  public default void climberApplySpeed(double power) {}
}
