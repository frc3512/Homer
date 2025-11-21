package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class VisionConstants {

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.15;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 1.5; // Meters
  public static double angularStdDevBaseline = 0.12; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final List<Integer> kRedReefTagIDs = List.of(6, 7, 8, 9, 10, 11);

  public static final List<Integer> kBlueReefTagIDs = List.of(17, 18, 19, 20, 21, 22);

  private static final List<AprilTag> kBlueReefTags =
      List.of(
          kTagLayout.getTags().get(16),
          kTagLayout.getTags().get(17),
          kTagLayout.getTags().get(18),
          kTagLayout.getTags().get(19),
          kTagLayout.getTags().get(20),
          kTagLayout.getTags().get(21));

  private static final List<AprilTag> kRedReefTags =
      List.of(
          kTagLayout.getTags().get(5),
          kTagLayout.getTags().get(6),
          kTagLayout.getTags().get(7),
          kTagLayout.getTags().get(8),
          kTagLayout.getTags().get(9),
          kTagLayout.getTags().get(10));

  private static final List<AprilTag> kReefTags =
      List.of(
          kTagLayout.getTags().get(5),
          kTagLayout.getTags().get(6),
          kTagLayout.getTags().get(7),
          kTagLayout.getTags().get(8),
          kTagLayout.getTags().get(9),
          kTagLayout.getTags().get(10),
          kTagLayout.getTags().get(16),
          kTagLayout.getTags().get(17),
          kTagLayout.getTags().get(18),
          kTagLayout.getTags().get(19),
          kTagLayout.getTags().get(20),
          kTagLayout.getTags().get(21));

  public static final AprilTagFieldLayout kBlueTagLayout =
      new AprilTagFieldLayout(
          kBlueReefTags, kTagLayout.getFieldLength(), kTagLayout.getFieldWidth());
  public static final AprilTagFieldLayout kRedTagLayout =
      new AprilTagFieldLayout(
          kRedReefTags, kTagLayout.getFieldLength(), kTagLayout.getFieldWidth());
  public static final AprilTagFieldLayout kReefTagLayout =
      new AprilTagFieldLayout(kReefTags, kTagLayout.getFieldLength(), kTagLayout.getFieldWidth());

  public static class CameraInfo {

    public String cameraName;
    public Transform3d robotToCamera;
    public Rotation2d diagFOV;
    public int[] cameraRes;

    public CameraInfo(
        String cameraName, Transform3d robotToCamera, Rotation2d diagFOV, int[] cameraRes) {
      this.cameraName = cameraName;
      this.robotToCamera = robotToCamera;
      this.diagFOV = diagFOV;
      this.cameraRes = cameraRes;
    }
  }

  public static CameraInfo BlackReefInfo =
      new CameraInfo(
          "Black_Reef",
          new Transform3d(
              new Translation3d(-0.320048, -0.300306, 0.299816),
              new Rotation3d(0, Units.degreesToRadians(0), Math.PI - Units.degreesToRadians(55))),
          Rotation2d.fromDegrees(79.19),
          new int[] {1280, 800});

  public static CameraInfo WhiteReefInfo =
      new CameraInfo(
          "White_Reef",
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(3), Units.inchesToMeters(4), Units.inchesToMeters(9)),
              new Rotation3d(
                  Math.PI, Units.degreesToRadians(0), Math.PI + Units.degreesToRadians(20))),
          Rotation2d.fromDegrees(79.19),
          new int[] {1280, 800});
}
