package frc.robot.subsystems.intake;

public enum IntakeState {
  kIntakeStop(0),
  kIntakeNote(1),
  kIntakeReverse(-1);

  public double intakePower;

  private IntakeState(double intakePower) {
    this.intakePower = intakePower;
  }
}
