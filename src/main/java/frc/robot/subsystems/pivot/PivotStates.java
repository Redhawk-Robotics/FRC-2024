package frc.robot.subsystems.pivot;

public enum PivotStates {
  kHome(0),
  kSpeakerMid(0),
  kSpeakerLeft(0),
  kSpeakerRight(0),
  kNoteMid(0), kNoteSides(0),
  kStageMidForwardToSpeaker(0),
  kStageMidSideToSpeaker(0);

  public final double encoderPose;

  private PivotStates(double encoderPose) {
    this.encoderPose = encoderPose;
  }
}
