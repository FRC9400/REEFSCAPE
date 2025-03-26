package frc.commons;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionUpdate {
    public double timestamp;
    public Pose2d pose;
    public Matrix<N3, N1> stdDevs;

    public VisionUpdate(double timestamp, Pose2d pose, Matrix<N3,N1> stdDevs){
        this.timestamp = timestamp;
        this.pose = pose;
        this.stdDevs = stdDevs;
    }
    public VisionUpdate(){
        this.timestamp = 0;
        this.pose = new Pose2d();
        this.stdDevs = VecBuilder.fill(9400, 9400, 9400);
    }
}
