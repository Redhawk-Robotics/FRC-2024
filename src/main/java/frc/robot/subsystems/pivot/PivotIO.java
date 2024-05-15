package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotInputs {
    public double leftPivotVoltage = 0;
    public double leftPivotEncoderPose = 0;
    public double leftPivotVelocity = 0;
    public double leftPivotTemp = 0;
    public double leftPivotCurrentSetSpeed = 0;
    public double leftPivotBusVoltage = 0;
    public double leftPivotOutputCurrent = 0;
    public double leftPivotVoltageCompensation = 0;

    public double rightPivotVoltage = 0;
    public double rightPivotEncoderPose = 0;
    public double rightPivotVelocity = 0;
    public double rightPivotTemp = 0;
    public double rightPivotCurrentSetSpeed = 0;
    public double rightPivotBusVoltage = 0;
    public double rightPivotOutputCurrent = 0;
    public double rightPivotVoltageCompensation = 0;

    public boolean isPivotAtRef = false;
    public double isPivotAtRefCalculation = 0;

    public double boreEncoderPose = 0;
  }

  public default void updateInputs(PivotInputs inputs) {}

  public default void setReference(double targetPosition) {}

  public boolean atReference();

  public default void pivotApplySpeed(double power) {}

  public default void enableLimits(boolean enable) {}
}
