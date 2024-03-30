package frc.robot.subsystems.shooter;

public enum ShooterSupportWheelStates {
  kSWStop(0, 0),
  kSWUptake(0, 1),
  kSWFeedShooter(1, 1),
  kSWIntake(-1, -1);

  public double guardPower;
  public double uptakePower;

  private ShooterSupportWheelStates(double guardPower, double uptakePower) {
    this.guardPower = guardPower;
    this.uptakePower = uptakePower;
  }
}
