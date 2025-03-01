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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.commons.Conversions;
import frc.robot.Constants.canIDConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.swerveConstants.kinematicsConstants;
import frc.robot.autons.AutoConstants;

/*
 * Changes
 * Removed zero wheels /
 * Gyro added / 
 * Added Odometry Thread class, synchronoulsy wait for signals/
 * Retained logging swervemodulestates and pose /
 * Added Oculus pose 
 * 
 * To do
 * add opi stuff (yikes!)
 * offset oculus pose
 * does this even work????
 */

public class Swerve extends SubsystemBase{
    ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0, null); //create pose constants
    Pose2d initialPose = new Pose2d();
    private QuestNav questNav = new QuestNav();
    Pigeon2 pigeon = new Pigeon2(canIDConstants.pigeon, "canivore");
    private StatusSignal<Angle> m_heading = pigeon.getYaw();
    private StatusSignal<AngularVelocity> m_angularVelocity = pigeon.getAngularVelocityZDevice();

    public final ModuleIOTalonFX[] Modules = new ModuleIOTalonFX[4];
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(kinematicsConstants.FL, kinematicsConstants.FR, kinematicsConstants.BL,
        kinematicsConstants.BR);
    OdometryThread m_OdometryThread;
    BaseStatusSignal[] m_allSignals;

    // from swervestate class
    public Pose2d poseRaw;
    public ChassisSpeeds Speeds;
    public SwerveModuleState[] currentModuleStates;
    public SwerveModuleState[] setpointModuleStates;
    public SwerveModulePosition[] currentModulePositions;
    public SwerveModuleState[] desiredModuleStates;
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
                  new Rotation2d(BaseStatusSignal.getLatencyCompensatedValue(m_heading, m_angularVelocity).magnitude() * Math.PI * 2);

                  odometry.update(heading, currentModulePositions); //update odoemtry threa
                
            }
    }
    
}
public Swerve() {
    // initialPose = getVisionPose()
    Modules[0] = new ModuleIOTalonFX(canIDConstants.driveMotor[0], canIDConstants.steerMotor[0], canIDConstants.CANcoder[0],swerveConstants.moduleConstants.CANcoderOffsets[0],
    swerveConstants.moduleConstants.driveMotorInverts[0], swerveConstants.moduleConstants.steerMotorInverts[0], swerveConstants.moduleConstants.CANcoderInverts[0]);

    Modules[1] = new ModuleIOTalonFX(canIDConstants.driveMotor[1], canIDConstants.steerMotor[1], canIDConstants.CANcoder[1], swerveConstants.moduleConstants.CANcoderOffsets[1],
   swerveConstants.moduleConstants.driveMotorInverts[1], swerveConstants.moduleConstants.steerMotorInverts[1], swerveConstants.moduleConstants.CANcoderInverts[1]);

   Modules[2] = new ModuleIOTalonFX(canIDConstants.driveMotor[2], canIDConstants.steerMotor[2], canIDConstants.CANcoder[2], swerveConstants.moduleConstants.CANcoderOffsets[2],
    swerveConstants.moduleConstants.driveMotorInverts[2], swerveConstants.moduleConstants.steerMotorInverts[2], swerveConstants.moduleConstants.CANcoderInverts[2]);

    Modules[3] = new ModuleIOTalonFX(canIDConstants.driveMotor[3], canIDConstants.steerMotor[3], canIDConstants.CANcoder[3], swerveConstants.moduleConstants.CANcoderOffsets[3],
    swerveConstants.moduleConstants.driveMotorInverts[3], swerveConstants.moduleConstants.steerMotorInverts[3], swerveConstants.moduleConstants.CANcoderInverts[3]);

    poseRaw = new Pose2d();
    Speeds = new ChassisSpeeds();
    currentModuleStates = new SwerveModuleState[4];
    desiredModuleStates = new SwerveModuleState[4];
    currentModulePositions = new SwerveModulePosition[4];
    setpointModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
        0,
        0,
        0,
        new Rotation2d()));


    for (int i = 0; i < 4; i++) {
        Modules[i].setDriveBrakeMode(true);
        Modules[i].setTurnBrakeMode(false);
    }

    for (int i = 0; i < 4; ++i) {
        currentModulePositions[i] = Modules[i].getPosition(true);
    }

    RobotConfig config;
    try{
        config = RobotConfig.fromGUISettings();
        AutoBuilder.configure(
            this::getPoseRaw,
            this::resetPose,
            this::getRobotRelativeSpeeds, 
            (speeds, feedforwards) -> driveRobotRelative(speeds), 
            new PPHolonomicDriveController(
                    new PIDConstants(0.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(0.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, 
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this 
    );
    } catch (Exception e) {
    e.printStackTrace();
        }

    m_OdometryThread = new OdometryThread();
    m_OdometryThread.start();
    odometry = new SwerveDriveOdometry(kinematics, pigeon.getRotation2d(),
        currentModulePositions); 

  }


@Override
public void periodic(){
    Logger.recordOutput("Swerve/GyroDeg", Conversions.RotationsToDegrees(m_heading.getValueAsDouble(), 1));
    for (int i = 0; i < 4; i++){
        Logger.recordOutput("Swerve/Module/ModuleNum[" + i + "]DriveStator", Modules[i].getCurrentSignals()[0].getValueAsDouble());
        Logger.recordOutput("Swerve/Module/ModuleNum[" + i + "]DriveSupply", Modules[i].getCurrentSignals()[1].getValueAsDouble());
        Logger.recordOutput("Swerve/Module/ModuleNum[" + i + "]SteerStator", Modules[i].getCurrentSignals()[2].getValueAsDouble());
        Logger.recordOutput("Swerve/Module/ModuleNum[" + i + "]AbsoluteAngle", Modules[i].getCurrentSignals()[3].getValueAsDouble());
    }
    
    logModuleStates("SwerveModuleStates/setpointStates", desiredModuleStates);
    logModuleStates("SwerveModuleStates/MeasuredStates", currentModuleStates);
    Logger.recordOutput("OculusPosituion", questNav.getPose());
    Logger.recordOutput("Odometry/PoseRaw", odometry.getPoseMeters());

}

public void requestDesiredState(double x_speed, double y_speed, double rot_speed, boolean fieldRelative, boolean isOpenLoop){
    Rotation2d gyroPosition = new Rotation2d(m_heading.getValueAsDouble() * Math.PI * 2);
    if (fieldRelative && isOpenLoop){
        desiredModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
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
        desiredModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
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
        desiredModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(
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
public Pose2d getPoseRaw(){
    return odometry.getPoseMeters();
}

public void resetGyro(double yawDeg){
    pigeon.setYaw(yawDeg);
    questNav.zeroHeading();
}


public void resetPose(Pose2d pose){
    odometry.resetPosition(heading, currentModulePositions, pose);

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

private void logModuleStates(String key, SwerveModuleState[] states) {
    List<Double> dataArray = new ArrayList<Double>();
    for (int i = 0; i < 4; i++) {
        dataArray.add(states[i].angle.getRadians());
        dataArray.add(states[i].speedMetersPerSecond);
    }
    Logger.recordOutput(key, dataArray.stream().mapToDouble(Double::doubleValue).toArray());
}

public void followChoreoTraj(SwerveSample sample) {
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
        sample.vx + AutoConstants.xController.calculate(poseRaw.getX(), sample.x),
        sample.vy + AutoConstants.yController.calculate(poseRaw.getY(), sample.y),
        sample.omega + AutoConstants.headingController.calculate(poseRaw.getRotation().getRadians(), sample.heading)
    ), poseRaw.getRotation()
    );
    driveRobotRelative(speeds);
}
}

