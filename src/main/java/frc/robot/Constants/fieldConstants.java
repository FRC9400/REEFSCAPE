package frc.robot.Constants;

import java.io.File;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;

public class fieldConstants {
  public static final double fieldLengthMeters = 17.548;
  public static final double fieldWidthMeters = 8.052;

  public Translation3d[] translations = new Translation3d[22];
  AprilTagFieldLayout aprilTagFieldLayout;

  public fieldConstants(){
    try{
        aprilTagFieldLayout = new AprilTagFieldLayout(
              Filesystem.getDeployDirectory()
                  .toPath()
                  .resolve("AprilTagLayout" + File.separator + "2025-reefscape.json"));
        } catch (Exception e) {
             System.err.println("Failed to load apriltag map :|");
        }

  translations[0] = new Translation3d(16.697198, 0.65532, 1.4859); // ID1
  translations[1] = new Translation3d(16.697198, 7.39648, 1.4859); // ID2
  translations[2] = new Translation3d(11.56081, 8.05561, 1.30175); // ID3
  translations[3] = new Translation3d(9.27608, 6.137656, 1.867916); // ID4
  translations[4] = new Translation3d(9.27608, 1.914906, 1.867916); // ID5
  translations[5] = new Translation3d(13.474446, 3.306318, 0.308102); // ID6
  translations[6] = new Translation3d(13.890498, 4.0259, 0.308102); // ID7
  translations[7] = new Translation3d(13.474446, 4.745482, 0.308102); // ID8
  translations[8] = new Translation3d(12.643358, 4.745482, 0.308102); // ID9
  translations[9] = new Translation3d(12.227306, 4.0259, 0.308102); // ID10
  translations[10] = new Translation3d(12.643358, 3.306318, 0.308102); // ID11
  translations[11] = new Translation3d(0.851154, 0.65532, 1.4859); // ID12
  translations[12] = new Translation3d(0.851154, 7.39648, 1.4859); // ID13
  translations[13] = new Translation3d(8.272272, 6.137656, 1.867916); // ID14
  translations[14] = new Translation3d(8.272272, 1.914906, 1.867916); // ID15
  translations[15] = new Translation3d(5.987542, -0.00381, 1.30175); // ID16
  translations[16] = new Translation3d(4.073906, 3.306318, 0.308102); // ID17
  translations[17] = new Translation3d(3.6576, 4.0259, 0.308102); // ID18
  translations[18] = new Translation3d(4.073906, 4.745482, 0.308102); // ID19
  translations[19] = new Translation3d(4.90474, 4.745482, 0.308102); // ID20
  translations[20] = new Translation3d(5.321046, 4.0259, 0.308102); // ID21
  translations[21] = new Translation3d(4.90474, 3.306318, 0.308102); // ID22
  }

  public Translation3d getTranslation3d(Short ID){
    return translations[(int) ID - 1 ];
  }

}
