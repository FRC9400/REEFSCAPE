package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.commons.LoggedTunableNumber;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;

public class SnapToPose extends Command{
    double maxAngularVel = 1;
    double maxAngularAccel = 4;
    double maxTransVel = 2;
    double maxTransAccel = 4;
    LoggedTunableNumber headingP = new LoggedTunableNumber("headingP", 3);
    LoggedTunableNumber headingD = new LoggedTunableNumber("headingD", 0);
    LoggedTunableNumber xP = new LoggedTunableNumber("xP", 3);
    LoggedTunableNumber xD = new LoggedTunableNumber("xD", 0);
    LoggedTunableNumber yP = new LoggedTunableNumber("yP", 3);
    LoggedTunableNumber yD = new LoggedTunableNumber("yD", 0);
    private final Swerve swerve;
    private final Superstructure superstructure;
    private ProfiledPIDController headingController = new ProfiledPIDController(headingP.get(), 0, headingD.get(), new TrapezoidProfile.Constraints(maxAngularVel, maxAngularAccel));
    private ProfiledPIDController xController= new ProfiledPIDController(xP.get(), 0, xD.get(), new TrapezoidProfile.Constraints(maxTransVel, maxTransAccel));
    private ProfiledPIDController yController= new ProfiledPIDController(yP.get(), 0, yD.get(), new TrapezoidProfile.Constraints(maxTransVel, maxTransAccel));

    private Pose2d goalPose;

    
    public SnapToPose(Swerve swerve, Superstructure superstructure, Pose2d goalPose){
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.goalPose = goalPose;
        addRequirements(swerve, superstructure);
       
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
  
       
        double thetaFeedback = headingController.calculate(
            swerve.getEstimatedPose().getRotation().getRadians(),
           goalPose.getRotation().getRadians());
        double xFeedback = xController.calculate(swerve.getEstimatedPose().getX(), goalPose.getX());
        double yFeedback = yController.calculate(swerve.getEstimatedPose().getY(), goalPose.getY());
    
        thetaFeedback = MathUtil.clamp(thetaFeedback, -5, 5);
        xFeedback = MathUtil.clamp(thetaFeedback, -3, 3);
        yFeedback = MathUtil.clamp(thetaFeedback, -3, 3);
        /* 
        if(Math.abs(swerve.getEstimatedPose().getX() - goalPose.getX()) < 0.001 && Math.abs(swerve.getEstimatedPose().getX() - goalPose.getX()) < 0.001
        && Math.abs(swerve.getEstimatedPose().getDegrees() - goalPose.getDegrees())) < 0.001))
        swerve.requestDesiredState(xFeedback, yFeedback, thetaFeedback, true, false);*/
        
    }

    @Override
    public void end(boolean interrupted){
        
    }
}