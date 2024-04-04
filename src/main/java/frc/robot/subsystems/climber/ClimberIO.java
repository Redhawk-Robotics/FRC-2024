package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberInputs {
    public double leftClimberVoltage = 0;
    public double leftClimberVelocity = 0;
    public double leftClimberTemp = 0;
    public double leftClimberCurrentSetSpeed = 0;
    public double leftClimberBusVoltage = 0;
    public double leftClimberOutputCurrent = 0;
    public double leftClimberVoltageCompensation = 0;

    public double rightClimberVoltage = 0;
    public double rightClimberVelocity = 0;
    public double rightClimberTemp = 0;
    public double rightClimberCurrentSetSpeed = 0;
    public double rightClimberBusVoltage = 0;
    public double rightClimberOutputCurrent = 0;
    public double rightClimberVoltageCompensation = 0;
  }

  public default void updateInputs(ClimberInputs inputs) {}

  public default void setReference(double targetPosition) {}

  public boolean atReference();

  public default void climberApplySpeed(double leftPower, double rightPower) {}
}
