package frc.robot.subsystems.pivot;

public enum PivotStates {
  kPivotHome(0),
  kPivotSubwoofer(0),
  kPivotNoteMid(0),
  kPivotAmp(0),
  kPivotPodium(0),
  kPivotSource(0),
  kPivotNoteSides(0),
  kPivotStageMidForwardToSpeaker(0),
  kPivotStageMidSideToSpeaker(0);

  public final double encoderPose;

  private PivotStates(double encoderPose) {
    this.encoderPose = encoderPose;
  }
}
