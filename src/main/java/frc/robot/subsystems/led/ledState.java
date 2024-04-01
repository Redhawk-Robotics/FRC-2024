package frc.robot.subsystems.led;

public enum ledState {
  RED(0),
  BLUE(0),
  YELLOW(0),
  PURPLE(0);

  public final double colorState;

  private ledState(double colorState) {
    this.colorState = colorState;
  }
}
