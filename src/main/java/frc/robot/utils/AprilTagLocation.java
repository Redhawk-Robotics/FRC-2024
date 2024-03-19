package frc.robot.utils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public enum AprilTagLocation {
  kTag1(593.68, 9.68, 53.38, 120),
  kTag2(637.21, 34.79, 53.38, 120),
  kTag3(652.73, 196.17, 57.13, 180),
  kTag4(652.73, 218.42, 57.13, 180),
  kTag5(578.77, 323.00, 53.38, 270),
  kTag6(72.5, 323.00, 53.38, 270),
  kTag7(-1.50, 218.42, 57.13, 0),
  kTag8(-1.50, 196.17, 57.13, 0),
  kTag9(14.02, 34.79, 53.38, 60),
  kTag10(57.54, 9.68, 53.38, 60),
  kTag11(468.69, 146.19, 52.00, 300),
  kTag12(468.69, 177.10, 52.00, 60),
  kTag13(441.74, 161.62, 52.00, 180),
  kTag14(209.48, 161.62, 52.00, 0),
  kTag15(182.73, 177.10, 52.00, 120),
  kTag16(182.73, 146.19, 52.00, 240),
  kTagNULL(0, 0, 0, 0);

  public final double xPos;
  public final double yPos;
  public final double zPos;
  public final double rotation;

  private static final Map<Integer, AprilTagLocation> tagIdMap = new HashMap<>();
  private static final List<AprilTag> tagIdList = new ArrayList<>();

  static {
    for (AprilTagLocation tag : values()) {
      tagIdMap.put(tag.ordinal() + 1, tag); // Assuming tag IDs start from 1
      tagIdList.add(
          new AprilTag(
              tag.ordinal() + 1,
              new Pose3d(tag.xPos, tag.yPos, tag.yPos, new Rotation3d(0, 0, tag.rotation))));
    }
  }

  private AprilTagLocation(double xPos, double yPos, double zPos, double rotation) {
    this.xPos = Units.inchesToMeters(xPos);
    this.yPos = Units.inchesToMeters(yPos);
    this.zPos = Units.inchesToMeters(zPos);
    this.rotation = Units.degreesToRadians(rotation);
  }

  public static AprilTagLocation getPositionById(int tagId) {
    return tagIdMap.get(tagId);
  }

  public static List<AprilTag> getTagIdList() {
    return tagIdList;
  }
}
