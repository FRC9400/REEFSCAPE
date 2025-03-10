package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.commons.LoggedTunableNumber;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;

public class SnapToPose extends Command{
    LoggedTunableNumber kP = new LoggedTunableNumber("PassingkP", 3);
    LoggedTunableNumber kD = new LoggedTunableNumber("PassingkD", 0);
    private final Swerve swerve;
    private final Superstructure superstructure;
    private ProfiledPIDController controller = new ProfiledPIDController(kP.get(), 0, kD.get(), null);
    private Pose2d goalPose;

    
    public SnapToPose(Swerve swerve, Superstructure superstructure, Pose2d goalPose){
        controller.enableContinuousInput(-Math.PI, Math.PI);
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
        double x = MathUtil.applyDeadband(RobotContainer.driver.getLeftY(), 0.1);
        double y = MathUtil.applyDeadband(RobotContainer.driver.getLeftX(), 0.1);
        double dx;
        double dy;

        dx = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? x * -1 : x;
        dy =  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? y : y *-1;
       
        double thetaFeedback = controller.calculate(
            swerve.getEstimatedPose().getRotation().getRadians(),
           goalPose.getRotation().getRadians());
        
        double xFeedback = controller.calculate(swerve.getEstimatedPose().getX(), goalPose.getX());
        double yFeedback = controller.calculate(swerve.getEstimatedPose().getY(), goalPose.getY());
    
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
