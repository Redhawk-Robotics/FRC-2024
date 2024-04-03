package frc.robot.subsystems.pivot;

public enum PivotStates {
  kPivotHome(0.015),
  kPivotSubwoofer(.123),
  kPivotNoteMid(.084),
  kCenterLine(.015),
  kEject(0),
  kPivotAmp(0),
  kPivotPodium(.120),
  kPivotSource(.1),
  kPivotNoteSides(.082),
  kPivotStageMidForwardToSpeaker(0),
  kPivotStageMidSideToSpeaker(0);

  public final double encoderPose;

  private PivotStates(double encoderPose) {
    this.encoderPose = encoderPose;
  }
}
