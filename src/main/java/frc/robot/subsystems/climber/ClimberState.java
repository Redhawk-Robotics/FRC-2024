package frc.robot.subsystems.climber;

public enum ClimberState {
  kStop(0),
  kHome(0),
  kClimb(0);

  public final double encoderPose;

  private ClimberState(double encoderPose) {
    this.encoderPose = encoderPose;
  }
}
