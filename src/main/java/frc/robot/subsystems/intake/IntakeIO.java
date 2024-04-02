package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {
    public double leftIntakeVoltage = 0;
    public double leftIntakeVelocity = 0;
    public double leftIntakeTemp = 0;
    public double leftIntakeCurrentSetSpeed = 0;
    public double leftIntakeBusVoltage = 0;
    public double leftIntakeOutputCurrent = 0;
    public double leftIntakeVoltageCompensation = 0;

    public double rightIntakeVoltage = 0;
    public double rightIntakeVelocity = 0;
    public double rightIntakeTemp = 0;
    public double rightIntakeCurrentSetSpeed = 0;
    public double rightIntakeBusVoltage = 0;
    public double rightIntakeOutputCurrent = 0;
    public double rightIntakeVoltageCompensation = 0;

    public boolean isIntakesEntranceIRSensorOn = false;
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void intakeApplySpeed(double speed) {}

  public boolean intakeEntranceSensorsEnabled();
}
