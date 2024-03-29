package frc.robot.subsystems.shooter;

public enum ShooterSupportWheelStates {
  kStop(0, 0),
  kUptake(0, 1),
  kFeedShooter(1, 1),
  kIntake(-1, -1);

  public double guardPower;
  public double uptakePower;

  private ShooterSupportWheelStates(double guardPower, double uptakePower) {
    this.guardPower = guardPower;
    this.uptakePower = uptakePower;
  }
}
