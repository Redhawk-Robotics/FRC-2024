package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterInputs {
    public double topShooterVoltage = 0;
    public double topShooterVelocity = 0;
    public double topShooterTemp = 0;
    public double topShooterCurrentSetSpeed = 0;
    public double topShooterBusVoltage = 0;
    public double topShooterOutputCurrent = 0;
    public double topShooterVoltageCompensation = 0;

    public double bottomShooterVoltage = 0;
    public double bottomShooterVelocity = 0;
    public double bottomShooterTemp = 0;
    public double bottomShooterCurrentSetSpeed = 0;
    public double bottomShooterBusVoltage = 0;
    public double bottomShooterOutputCurrent = 0;
    public double bottomShooterVoltageCompensation = 0;

    public double guardVoltage = 0;
    public double guardVelocity = 0;
    public double guardTemp = 0;
    public double guardCurrentSetSpeed = 0;
    public double guardBusVoltage = 0;
    public double guardOutputCurrent = 0;
    public double guardVoltageCompensation = 0;

    public double uptakeVoltage = 0;
    public double uptakeVelocity = 0;
    public double uptakeTemp = 0;
    public double uptakeCurrentSetSpeed = 0;
    public double uptakeBusVoltage = 0;
    public double uptakeOutputCurrent = 0;
    public double uptakeVoltageCompensation = 0;

    boolean isShooterIRSensorOn = false;
  }

  public default void updateInputs(ShooterInputs inputs) {}

  public default void applyShooterSpeed(double power) {}

  public default void applySupportWheelSpeeds(double guardPower, double uptakePower) {}

  public default void indexerApplySpeed(double power) {}

  public boolean shooterSensorsEnabled();
}
