package frc.robot.Subsystems.Swerve;

import java.util.List;

import org.opencv.photo.Photo;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    private final static Transform3d[] cameraOffsets = {new Transform3d(), new Trans(), new Pose3d()};
    private List<PhotonPipelineResult>[] results = new List[3];
    private PhotonCamera[] photonCameras = new PhotonCamera[3];
    

    public Vision(){
        photonCameras[0] = new PhotonCamera("camera1");
        photonCameras[1] = new PhotonCamera("camera2");
        photonCameras[2] = new PhotonCamera("camera3");
        results[0] = photonCameras[0].getAllUnreadResults();
        results[1] = photonCameras[1].getAllUnreadResults();
        results[2] = photonCameras[2].getAllUnreadResults();
    }

    @Override
    public void periodic(){
        for(int i = 0; i < results.length; i++){
            results[i] = photonCameras[i].getAllUnreadResults();
        }
    }
    
}
