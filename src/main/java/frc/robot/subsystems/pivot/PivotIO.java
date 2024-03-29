package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotInputs {}

  public default void updateInputs(PivotInputs inputs) {}

  public default void setReference(double targetPosition) {}

  public default void pivotApplySpeed(double power) {}

  public default void pivotUp() {}

  public default void pivotDown() {}

  public default void pivotStop() {}
}
