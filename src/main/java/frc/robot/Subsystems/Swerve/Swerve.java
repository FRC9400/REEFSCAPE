package frc.robot.Subsystems.Swerve;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.commons.VisionUpdate;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.swerveConstants.kinematicsConstants;

public class Swerve extends SubsystemBase{
    Vision vision = new Vision();
    VisionUpdate latestVisionUpdate;
    public SwerveDrivePoseEstimator poseEstimator;
    Matrix<N3, N1> stateSTDevs = VecBuilder.fill(0, 0, 0); // how to tune how to tune
    Pigeon2 pigeon = new Pigeon2(canIDConstants.pigeon, "canivore");
    private StatusSignal<Angle> m_heading = pigeon.getYaw();
    private StatusSignal<AngularVelocity> m_angularVelocity = pigeon.getAngularVelocityZDevice();

    public final ModuleIOTalonFX[] Modules = new ModuleIOTalonFX[4];
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(kinematicsConstants.FL, kinematicsConstants.FR, kinematicsConstants.BL,
        kinematicsConstants.BR);
    OdometryThread m_OdometryThread;
    BaseStatusSignal[] m_allSignals;
    Timer timer = new Timer();
    double collisionTimestamp = 0;

    private final PIDController xController = new PIDController(4.6,0,0.023);
    private final PIDController yController = new PIDController(4.205,0,0.015);
    private final PIDController thetaController = new PIDController(3.57,0,0.005);

    // from swervestate class
    public Pose2d poseRaw;
    public ChassisSpeeds Speeds;
    public SwerveModuleState[] currentModuleStates;
    public SwerveModuleState[] setpointModuleStates;
    public SwerveModulePosition[] currentModulePositions;
    public Rotation2d heading = new Rotation2d();
    public int SuccessfulDaqs = 0;
    public int FailedDaqs = 0;
    private SwerveDriveOdometry odometry;

    private final SysIdRoutine driveRoutine = new SysIdRoutine(new SysIdRoutine.Config(
        null, 
        Volts.of(3), 
        Seconds.of(2), 

    
        (state) -> SignalLogger.writeString("state", state.toString())), 
        new SysIdRoutine.Mechanism((
            Voltage volts) -> driveVoltage(volts.in(Volts)),
             null, 
             this)
    );

    private final SysIdRoutine steerRoutine = new SysIdRoutine(new SysIdRoutine.Config(
        null, 
        Volts.of(5), 
        Seconds.of(6), 
        (state) -> SignalLogger.writeString("state", state.toString())), 
        new SysIdRoutine.Mechanism((
            Voltage volts) -> Modules[0].steerVoltage(volts.in(Volts)),
             null, 
             this)
        );
     
    public class OdometryThread {
            private boolean m_running = false;
            private Thread m_thread;
    
            /** Constructor for OdometryThread */
            public OdometryThread() {
                m_thread = new Thread(this::run);
                m_thread.setDaemon(true); // Marks the thread as a background process
    
                /* 4 signals for each module + 2 for Pigeon2 */
                m_allSignals = new BaseStatusSignal[(4 * 4) + 2];
                for (int i = 0; i < 4; ++i) {
                    var signals = Modules[i].getSignals();
                    m_allSignals[(i * 4) + 0] = signals[0];
                    m_allSignals[(i * 4) + 1] = signals[1];
                    m_allSignals[(i * 4) + 2] = signals[2];
                    m_allSignals[(i * 4) + 3] = signals[3];
                }
    
                // Ensure these fields exist in Swerve or pass them as parameters
                m_allSignals[m_allSignals.length - 2] = m_heading; // Replace with correct signal
                m_allSignals[m_allSignals.length - 1] = m_angularVelocity; // Replace with correct signal
            }
    
            /** Starts the odometry thread. */
            public void start() {
                if (!m_running) {
                    m_running = true;

                    m_thread.start();
                }
            }
    
            /** Stops the odometry thread. */
            public void stop() {
                stop(0);
            }
    
            /** Stops the odometry thread with a timeout. */
            public void stop(long millis) {
                m_running = false;
                try {
                    m_thread.join(millis);
                } catch (InterruptedException ex) {
                    Thread.currentThread().interrupt();
                }
            }
    
            /** The method that runs in the background thread */
            private void run() {
                StatusCode status;
                BaseStatusSignal.setUpdateFrequencyForAll(250, m_allSignals);
                while (m_running) {
                    status = BaseStatusSignal.waitForAll(2.0/250, m_allSignals);  //synchornously waits for all signals with a timeout of double its update freq and then refreshes all singals
                
                 /* Get status of first element */
                    if (status.isOK()) {
                          SuccessfulDaqs++;
                     } else {
                          FailedDaqs++;
                    }

                for (int i = 0; i < 4; ++i) {
                    currentModulePositions[i] = Modules[i].getPosition(false);
                    currentModuleStates[i] = Modules[i].getState();
                  }
                  heading =
                  new Rotation2d(m_heading.getValueAsDouble() * Math.PI/180.0); //need to add latency compesnation

                  odometry.update(heading, currentModulePositions); //update odoemtry thread
                  poseEstimator.update(heading, currentModulePositions); //add timestamp?
                
            }
    }
    
}
public Swerve() {
    Modules[0] = new ModuleIOTalonFX(canIDConstants.driveMotor[0], canIDConstants.steerMotor[0], canIDConstants.CANcoder[0],swerveConstants.moduleConstants.CANcoderOffsets[0],
    swerveConstants.moduleConstants.driveMotorInverts[0], swerveConstants.moduleConstants.steerMotorInverts[0], swerveConstants.moduleConstants.CANcoderInverts[0]);

    Modules[1] = new ModuleIOTalonFX(canIDConstants.driveMotor[1], canIDConstants.steerMotor[1], canIDConstants.CANcoder[1], swerveConstants.moduleConstants.CANcoderOffsets[1],
   swerveConstants.moduleConstants.driveMotorInverts[1], swerveConstants.moduleConstants.steerMotorInverts[1], swerveConstants.moduleConstants.CANcoderInverts[1]);

   Modules[2] = new ModuleIOTalonFX(canIDConstants.driveMotor[2], canIDConstants.steerMotor[2], canIDConstants.CANcoder[2], swerveConstants.moduleConstants.CANcoderOffsets[2],
    swerveConstants.moduleConstants.driveMotorInverts[2], swerveConstants.moduleConstants.steerMotorInverts[2], swerveConstants.moduleConstants.CANcoderInverts[2]);

    Modules[3] = new ModuleIOTalonFX(canIDConstants.driveMotor[3], canIDConstants.steerMotor[3], canIDConstants.CANcoder[3], swerveConstants.moduleConstants.CANcoderOffsets[3],
    swerveConstants.moduleConstants.driveMotorInverts[3], swerveConstants.moduleConstants.steerMotorInverts[3], swerveConstants.moduleConstants.CANcoderInverts[3]);
    
    currentModulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; ++i) {
        currentModulePositions[i] = Modules[i].getPosition(true);
    }

    poseRaw = new Pose2d();
    Speeds = new ChassisSpeeds();
    currentModuleStates = new SwerveModuleState[4];
    setpointModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        0,
        new Rotation2d()));


    for (int i = 0; i < 4; i++) {
        Modules[i].setDriveBrakeMode(true);
        Modules[i].setTurnBrakeMode(false);
    }
    latestVisionUpdate = vision.getVisionUpdate().isPresent() ? vision.getVisionUpdate().get() : new VisionUpdate();

    m_OdometryThread = new OdometryThread();
    m_OdometryThread.start();
    odometry = new SwerveDriveOdometry(kinematics, pigeon.getRotation2d(),
        currentModulePositions); 
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, heading, currentModulePositions, latestVisionUpdate.pose, VecBuilder.fill(0.1, 0.1, 0.1), latestVisionUpdate.stdDevs);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    timer.start();
  }


@Override
public void periodic(){
    /* 
    if(checkCollision()){
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, heading, currentModulePositions, poseRaw, VecBuilder.fill(9400, 9400, 9400), latestVisionUpdate.stdDevs);
        collisionTimestamp = timer.get();
    else if(checkSkid()){
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, heading, currentModulePositions, poseRaw, VecBuilder.fill(0.5, 0.5, 0.5), latestVisionUpdate.stdDevs);
        collisionTimestamp = timer.get();
    }
    }else if(timer.get() - collisionTimestamp > 0.5){
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, heading, currentModulePositions, poseRaw, VecBuilder.fill(0.1, 0.1, 0.1), latestVisionUpdate.stdDevs);
    }
    */

    if(vision.getVisionUpdate().isPresent()){
    latestVisionUpdate = vision.getVisionUpdate().get();
    poseEstimator.setVisionMeasurementStdDevs(latestVisionUpdate.stdDevs);
    poseEstimator.addVisionMeasurement(latestVisionUpdate.pose, latestVisionUpdate.timestamp);}

    Logger.recordOutput("Swerve/GyroDeg", m_heading.getValueAsDouble());
    for (int i = 0; i < 4; i++){
        Logger.recordOutput("Swerve/Module/ModuleNum[" + i + "]DriveStator", Modules[i].getCurrentSignals()[0].getValueAsDouble());
        Logger.recordOutput("Swerve/Module/ModuleNum[" + i + "]DriveSupply", Modules[i].getCurrentSignals()[1].getValueAsDouble());
        Logger.recordOutput("Swerve/Module/ModuleNum[" + i + "]SteerStator", Modules[i].getCurrentSignals()[2].getValueAsDouble());
        Logger.recordOutput("Swerve/Module/ModuleNum[" + i + "]AbsoluteAngle", Modules[i].getCurrentSignals()[3].getValueAsDouble());
    }

    logModuleStates("Swerve/SwerveModuleStates/MeasuredStates", currentModuleStates);
    logModuleStates("Swerve/SwerveModuleStates/DesiredStates", setpointModuleStates);
    Logger.recordOutput("Swerve/Pose/FusedPose", poseEstimator.getEstimatedPosition());
    Logger.recordOutput("Swerve/Pose/Odometry", odometry.getPoseMeters());
    Logger.recordOutput("Swerve/Pose/VisionPose", latestVisionUpdate.pose);
    Logger.recordOutput("Swerve/Pose/Timestamp", latestVisionUpdate.timestamp);
    Logger.recordOutput("Swerve/Pose/visionStdevs", latestVisionUpdate.stdDevs);
    Logger.recordOutput("Swerve/SuccessfulDaqs", SuccessfulDaqs);
    Logger.recordOutput("Swerve/FailedDaqs", FailedDaqs);

}


public void requestDesiredState(double x_speed, double y_speed, double rot_speed, boolean fieldRelative, boolean isOpenLoop){
    Rotation2d gyroPosition = new Rotation2d(m_heading.getValueAsDouble() * Math.PI * 2);
    if (fieldRelative && isOpenLoop){
        setpointModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed,
            y_speed,
            rot_speed,
            gyroPosition));
        kinematics.desaturateWheelSpeeds(setpointModuleStates, 12);
        for (int i = 0; i < 4; i++) {
            Modules[i].setDesiredState(setpointModuleStates[i], true, false);
        }
    }
    else if(fieldRelative && !isOpenLoop){
        setpointModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
            x_speed,
            y_speed,
            rot_speed,
            gyroPosition));
        kinematics.desaturateWheelSpeeds(setpointModuleStates, swerveConstants.moduleConstants.maxSpeedMeterPerSecond);
        for (int i = 0; i < 4; i++) {
            Modules[i].setDesiredState(setpointModuleStates[i], false, false); //optimizes within module
        }
    }
    else if(!fieldRelative && !isOpenLoop){
        setpointModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(
            x_speed,
            y_speed,
            rot_speed
            ));
        kinematics.desaturateWheelSpeeds(setpointModuleStates, swerveConstants.moduleConstants.maxSpeedMeterPerSecond);
        for (int i = 0; i < 4; i++) {
            Modules[i].setDesiredState(setpointModuleStates[i], false, false);
        }
    }
    
}

/* 
public boolean checkSkid(){

}

public boolean checkCollision(){

}
*/

public Pose2d getEstimatedPose(){
    return poseEstimator.getEstimatedPosition();
}

public void resetGyro(double yawDeg){
    pigeon.setYaw(yawDeg);
}

public void resetPoseEstimator(Pose2d pose){
    poseEstimator.resetPose(pose);

}

public void resetPose(Pose2d pose){
    odometry.resetPosition(heading, currentModulePositions, pose);

}

public Pose2d getPoseRaw(){
    return odometry.getPoseMeters();
}

public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
    ChassisSpeeds desiredChassisSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    double x_speed = desiredChassisSpeeds.vxMetersPerSecond;
    double y_speed = desiredChassisSpeeds.vyMetersPerSecond;
    double rot_speed = desiredChassisSpeeds.omegaRadiansPerSecond;

    requestDesiredState(x_speed, y_speed, rot_speed, false, false);

}

public ChassisSpeeds getRobotRelativeSpeeds(){
    return kinematics.toChassisSpeeds(currentModuleStates);
}


public void driveVoltage(double volts){
    for( int i = 0; i < 4; i++){
        Modules[i].setDriveVoltage(volts);
    }
    
}

public Command driveSysIdCmd(){
    return Commands.sequence(
        this.runOnce(() -> SignalLogger.start()),
        driveRoutine
            .quasistatic(Direction.kForward),
            this.runOnce(() -> driveVoltage(0)),
            Commands.waitSeconds(1),
        driveRoutine
            .quasistatic(Direction.kReverse),
            this.runOnce(() -> driveVoltage(0)),
            Commands.waitSeconds(1),  

        driveRoutine
            .dynamic(Direction.kForward),
            this.runOnce(() -> driveVoltage(0)),
            Commands.waitSeconds(1),  

        driveRoutine
            .dynamic(Direction.kReverse),
            this.runOnce(() -> driveVoltage(0)),
            Commands.waitSeconds(1), 
        this.runOnce(() -> SignalLogger.stop())
    );
}

public Command steerSysIdCmd(){
    return Commands.sequence(
    this.runOnce(() -> SignalLogger.start()),
        steerRoutine
            .quasistatic(Direction.kForward),
            this.runOnce(() -> Modules[0].steerVoltage(0)),
            Commands.waitSeconds(1),
        steerRoutine
            .quasistatic(Direction.kReverse),
            this.runOnce(() -> Modules[0].steerVoltage(0)),
            Commands.waitSeconds(1),  

        steerRoutine
            .dynamic(Direction.kForward),
            this.runOnce(() -> Modules[0].steerVoltage(0)),
            Commands.waitSeconds(1),  

        steerRoutine
            .dynamic(Direction.kReverse),
            this.runOnce(() -> Modules[0].steerVoltage(0)),
            Commands.waitSeconds(1), 
        this.runOnce(() -> SignalLogger.stop())
    );
}
public void zeroWheels(){
    for(int i = 0; i < 4; i++){
        Modules[i].resetToAbsolute();
    }
}

public void followChoreoTraj(SwerveSample sample) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
        sample.vx + xController.calculate(odometry.getPoseMeters().getX(), sample.x),
        sample.vy + yController.calculate(odometry.getPoseMeters().getY(), sample.y),
        sample.omega + thetaController.calculate(odometry.getPoseMeters().getRotation().getRadians(), sample.heading)
    ), odometry.getPoseMeters().getRotation()
    );
    driveRobotRelative(speeds);
}

private void logModuleStates(String key, SwerveModuleState[] states) {
    List<Double> dataArray = new ArrayList<Double>();
    for (int i = 0; i < 4; i++) {
        dataArray.add(states[i].angle.getRadians());
        dataArray.add(states[i].speedMetersPerSecond);
    }
    Logger.recordOutput(key, dataArray.stream().mapToDouble(Double::doubleValue).toArray());
}
}
