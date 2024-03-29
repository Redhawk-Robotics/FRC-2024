package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeInputs {}

  public default void updateInputs(IntakeInputs inputs) {}

  public default void intakeFloorNote() {}

  public default void intakeStop() {}

  public default void intakeApplySpeed(double speed) {}

  public boolean intakeEntranceSensorsEnabled();
}
