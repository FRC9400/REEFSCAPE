package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.commons.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.swerveConstants;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;


public class SnapToPose extends Command{
    ScoringPosts targetPost = ScoringPosts.A;
    Pose2d startingPose;
    double maxAngularVel = 3;
    double maxAngularAccel = 4;
    double maxTransVel = 3;
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

    public enum ScoringPosts{
        NOT_SCORING,
        LEFT_SOURCE,
        RIGHT_SOURCE,
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K, 
        L,
        FRONT,
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK,
        BACK_LEFT,
        BACK_RIGHT, 
    }
    
    
    public SnapToPose(Swerve swerve, Superstructure superstructure, Pose2d goalPose){
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        this.superstructure = superstructure;
        startingPose = swerve.getEstimatedPose();
        targetPost = findNearestPost();
        switch(targetPost){
            case NOT_SCORING:
                this.goalPose = new Pose2d();
            case LEFT_SOURCE:
                this.goalPose = new Pose2d(0.6543587446212769, 6.804286956787109, new Rotation2d(-0.938047201237074));
            case RIGHT_SOURCE:
                this.goalPose = new Pose2d(1.4565401077270508, 0.6411857008934021, new Rotation2d(0.9380476238634191));
            case A:
                this.goalPose = new Pose2d(3.256556749343872, 4.202088832855225, new Rotation2d(0));
            case B: 
                this.goalPose = new Pose2d(3.256556749343872, 3.84991192817688, new Rotation2d(0));
            case C: 
                this.goalPose = new Pose2d(3.7261264324188232, 3.0477304458618164, new Rotation2d(1.0472));
            case D:
                this.goalPose = new Pose2d(4.0196075439453125, 2.891207218170166,  new Rotation2d(1.0472));
            case E: 
                this.goalPose = new Pose2d(4.958746910095215, 2.8716418743133545, new Rotation2d(2.0944));
            case F: 
                this.goalPose = new Pose2d(5.232662200927734, 3.0477304458618164, new Rotation2d(2.0944));
            case G:
                this.goalPose = new Pose2d(5.741362571716309, 4.182523727416992, new Rotation2d(Math.PI));
            case H: 
                this.goalPose = new Pose2d(5.721797466278076, 3.8694772720336914, new Rotation2d(Math.PI));
            case I: 
                this.goalPose = new Pose2d(5.232662200927734, 5.004270553588867, new Rotation2d(-2.0944));
            case J: 
                this.goalPose = new Pose2d(4.958746910095215, 5.160793781280518, new Rotation2d(-2.0944));
            case K:
                this.goalPose = new Pose2d(4.000041961669922, 5.18035888671875, new Rotation2d(-1.0472));
            case L: 
                this.goalPose = new Pose2d(3.7261264324188232, 5.023836135864258, new Rotation2d(-1.0472));
            case FRONT: 
                this.goalPose = new Pose2d(3.2714266777038574, 4.0053911209106445,  new Rotation2d(0));
            case FRONT_LEFT: 
                this.goalPose = new Pose2d(3.8826496601104736, 5.082531929016113, new Rotation2d(-1.0472));
            case FRONT_RIGHT: 
                this.goalPose = new Pose2d(3.863084316253662, 2.9499034881591797, new Rotation2d(1.0472));
            case BACK: 
                this.goalPose = new Pose2d(5.721797466278076, 4.0053911209106445, new Rotation2d(Math.PI));
            case BACK_LEFT: 
                this.goalPose = new Pose2d(5.095704555511475, 5.082531929016113, new Rotation2d(-2.0944));
            case BACK_RIGHT: 
                this.goalPose = new Pose2d(5.095704555511475, 2.969468832015991, new Rotation2d(2.0944));
            
        }

        
        Logger.recordOutput("GoalPose", goalPose);
        Logger.recordOutput("ScoringState", targetPost);
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
        
        if(targetPost == ScoringPosts.NOT_SCORING){
            double x = Math.pow(MathUtil.applyDeadband(RobotContainer.driver.getLeftY(), 0.1),3) * swerveConstants.moduleConstants.maxSpeedMeterPerSecond / 2;
            double y =Math.pow( MathUtil.applyDeadband(RobotContainer.driver.getLeftX(), 0.1), 3) * swerveConstants.moduleConstants.maxSpeedMeterPerSecond/ 2;
            double rot = Math.pow(MathUtil.applyDeadband(RobotContainer.driver.getRightX(), 0.1), 3) * swerveConstants.moduleConstants.maxAngularVelocity/ 2;

            swerve.requestDesiredState(x, y, rot, true, false);
        } 
        else if(Math.abs(swerve.getEstimatedPose().getX() - goalPose.getX()) < 0.01 && Math.abs(swerve.getEstimatedPose().getY() - goalPose.getY()) < 0.01
        && Math.abs(swerve.getEstimatedPose().getRotation().getDegrees() - goalPose.getRotation().getDegrees()) < 0.5) {
            //led.set smth
        }
        else{
            swerve.requestDesiredState(xFeedback, yFeedback, thetaFeedback, true, false);
        } 
    }

    @Override
    public void end(boolean interrupted){
        
    }

    public ScoringPosts findNearestPost(){
        //units are in meters?
        double closest = 100;
        double bounds = 0.95;
        double intakingBounds = 1;
        int index = 0;
        // front, front left, front right, back, back left, back right, left source, right source
        Translation2d[] translations = {new Translation2d(3.2714266777038574, 4.0053911209106445), new Translation2d(3.8826496601104736, 5.082531929016113), new Translation2d(3.863084316253662, 2.9499034881591797), new Translation2d(5.721797466278076, 4.0053911209106445), new Translation2d(5.095704555511475, 5.082531929016113), new Translation2d(5.095704555511475, 2.969468832015991), new Translation2d(0.6543587446212769, 6.804286956787109), new Translation2d(1.4565401077270508, 0.6411857008934021)};

        for(int i = 0; i < translations.length; i++){
            double distance = Math.hypot(startingPose.getX() - translations[i].getX(), startingPose.getY() - translations[i].getY());
            if (distance < closest){
                closest = distance;
                index = i;
            }
        }
        if (index == 6 || index == 7) {
            if (closest > intakingBounds) {
                return (index == 6) ? ScoringPosts.LEFT_SOURCE : ScoringPosts.RIGHT_SOURCE;
            }
            return ScoringPosts.NOT_SCORING;
        }
        else if(closest > bounds ){
            return ScoringPosts.NOT_SCORING;
        }
        else if(RobotContainer.selectedNode.equals("left")){
            if(index == 0){
                return ScoringPosts.A;
            }
            else if(index == 1){
                return ScoringPosts.K;
            }
            else if(index == 2){
                return ScoringPosts.C;
            }
            else if(index == 3){
                return ScoringPosts.G;
            }
            else if(index == 4){
                return ScoringPosts.J;
            }
            else if(index == 5){
                return ScoringPosts.F;
            }
            
        }
        else if(RobotContainer.selectedNode.equals("right")){
            if(index == 0){
                return ScoringPosts.B;
            }
            else if(index == 1){
                return ScoringPosts.L;
            }
            else if(index == 2){
                return ScoringPosts.D;
            }
            else if(index == 3){
                return ScoringPosts.K;
            }
            else if(index == 4){
                return ScoringPosts.I;
            }
            else if(index == 5){
                return ScoringPosts.E;
            
            }
        }else if (RobotContainer.selectedNode.equals("middle")){
            if(index == 0){
                return ScoringPosts.FRONT;
            }
            else if(index == 1){
                return ScoringPosts.FRONT_LEFT;

            }
            else if(index == 2){
                return ScoringPosts.FRONT_RIGHT;
            
            }
            else if(index == 3){
                return ScoringPosts.BACK;
            
            }
            else if(index == 4){
                return ScoringPosts.BACK_LEFT;
            
            }
            else if(index == 5){
                return ScoringPosts.BACK_RIGHT;
            }
        }
        return ScoringPosts.NOT_SCORING;
        
    }

}