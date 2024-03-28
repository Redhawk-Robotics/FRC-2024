package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {}

  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setReference() {}

  public default void pivotApplySpeed(double power) {}

  public default void pivotUp(){}

  public default void pivotDown(){}

  public default void pivotStop(){}
  
}
