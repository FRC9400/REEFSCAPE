package frc.robot.Subsystems.Swerve;

import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.VisionUpdate;
import frc.robot.Constants.fieldConstants;

public class Vision extends SubsystemBase{
    private Timer timer = new Timer();
    private final static Transform3d[] cameraOffsets = {new Transform3d(), new Transform3d(), new Transform3d()};
    private List<PhotonPipelineResult>[] results = new List[3];
    private double[] timestamps = new double[3];
    private PhotonCamera[] photonCameras = new PhotonCamera[3];
    private List<Short>[] targetIDs = new List[3];
    private Pose2d visionPose;
    private Deque<VisionUpdate> poseQueue = new LinkedList<>();
    private fieldConstants field = new fieldConstants(); //this is so yucky why did i do this
    private Matrix<N3, N1> visionStdDevs = VecBuilder.fill(9400, 9400, 9400);
    private double xyStdev = 9400;
    private double thetaStdev = 9400;
  
    public Vision(){
        visionPose = new Pose2d();
        photonCameras[0] = new PhotonCamera("camera1");
        photonCameras[1] = new PhotonCamera("camera2");
        photonCameras[2] = new PhotonCamera("camera3");
        for (int i = 0; i < 3; i++) {
            results[i] =  photonCameras[i].getAllUnreadResults();
            targetIDs[i] = new ArrayList<>();  
        }
        timer.start();
    }

    @Override
    public void periodic(){
        for(int i = 0; i < results.length; i++){

            if (!poseQueue.isEmpty() && (timer.get() - poseQueue.peek().timestamp > 5)) {
                poseQueue.poll();
            }
        
            results[i] = photonCameras[i].getAllUnreadResults();
            PhotonPipelineResult latestResult = results[i].get(results[i].size() - 1);
            timestamps[i] = latestResult.getTimestampSeconds();
            double ambiguity = 9400;
            List<Translation3d> tagPoses = new ArrayList<Translation3d>();

            if(!latestResult.hasTargets()) continue;
            else{
                Pose3d robotPose = new Pose3d();
                double totalDistance = 0;
                double avgDistance = 0;
                double nonReefTags = 0;
                if(latestResult.getMultiTagResult().isPresent()){

                    robotPose = new Pose3d(latestResult.getMultiTagResult().get().estimatedPose.best.getTranslation(), latestResult.getMultiTagResult().get().estimatedPose.best.getRotation()).transformBy(cameraOffsets[i]);

                    targetIDs[i] = latestResult.getMultiTagResult().get().fiducialIDsUsed; 
                }    
                else{
                    ambiguity = latestResult.getBestTarget().getPoseAmbiguity();
                    targetIDs[i].clear();
                    targetIDs[i].add((short) latestResult.getBestTarget().fiducialId);

                    if(latestResult.getBestTarget().getBestCameraToTarget().getTranslation().getNorm() > 5) continue;

                    if(ambiguity > 0.15) continue;
                    
                    robotPose = new Pose3d(latestResult.getBestTarget().getBestCameraToTarget().getTranslation(), latestResult.getBestTarget().getBestCameraToTarget().getRotation()).transformBy(cameraOffsets[i]);  
                    
                }

                if(robotPose.getMeasureX().baseUnitMagnitude() > fieldConstants.fieldLengthMeters + 0.1 || robotPose.getMeasureX().baseUnitMagnitude()  < 0 || robotPose.getMeasureY().baseUnitMagnitude()  > fieldConstants.fieldWidthMeters + 0.1|| robotPose.getMeasureY().baseUnitMagnitude()  < 0 || robotPose.getMeasureZ().baseUnitMagnitude()  > 0.125) continue;

                for(Short ID : targetIDs[i]){
                    tagPoses.add(field.getTranslation3d(ID));
                   
                    if((ID > 0 && ID < 6) || (ID > 11 && ID < 17)){
                        nonReefTags++;
                    }
                }

                for (Translation3d tagPose : tagPoses) {
                    totalDistance += tagPose.getDistance(cameraOffsets[i].getTranslation());
                }
                avgDistance = totalDistance / targetIDs[i].size();
                xyStdev = Math.pow(avgDistance, 2) / targetIDs[i].size() + 0.05 * nonReefTags; // inverse relationship
                thetaStdev = xyStdev;  

                if(targetIDs[i].size() == 1){
                    xyStdev *= 2;
                    thetaStdev *= 2;
                }
                
                visionStdDevs = VecBuilder.fill(xyStdev, xyStdev, thetaStdev);
                visionPose = robotPose.toPose2d();
                poseQueue.add(new VisionUpdate(timestamps[i], visionPose, visionStdDevs));}

                Logger.recordOutput("Vision/Camera", photonCameras[i].getName());
                Logger.recordOutput("Vision/Pose", visionPose);
                Logger.recordOutput("Vision/Timestamp", timestamps[i]);
                Logger.recordOutput("Vision/Strategy", latestResult.getMultiTagResult().isPresent() ? "MultiTagCoproc" : "LowestAmbiguity");
                logTargetIDs(targetIDs[i]);
                Logger.recordOutput("Vision/stdDevs", visionStdDevs);
                Logger.recordOutput("Vision/Ambiguity", ambiguity);
            }
    }
    

    public Optional<VisionUpdate> getVisionUpdate() {
        return Optional.ofNullable(poseQueue.peekLast()); // in case posequeue is empty
    }

    public void logTargetIDs(List<Short> target){
        int[] targets = new int[target.size()];
        for(int i = 0; i < targets.length; i++){
            targets[i] = (int) target.get(i);
        }
        Logger.recordOutput("Vision/TargetIDs", targets);
    } 

}

    

