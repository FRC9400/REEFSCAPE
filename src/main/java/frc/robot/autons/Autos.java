package frc.robot.autons;

import javax.naming.InitialContext;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.Swerve;

public class Autos {
    private final Swerve s_Swerve;
    private final AutoFactory autoFactory;

    public Autos(Swerve swerve){
        this.s_Swerve = swerve;
        autoFactory = new AutoFactory(
            s_Swerve::getPoseRaw,
            s_Swerve::resetPose,
            s_Swerve::followChoreoTraj,
            true,
            s_Swerve);    
        }
    public AutoFactory getFactory() {
            return autoFactory;
        }

    public Command tune(String name){
        final AutoRoutine routine = autoFactory.newRoutine(name);
        final AutoTrajectory trajectory = routine.trajectory(name);
        routine.active().whileTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
        return routine.cmd();
    }

    public Command runAuto(String name){
        final AutoRoutine routine = autoFactory.newRoutine(name);
        final AutoTrajectory trajectory = routine.trajectory(name);
        if(trajectory.getInitialPose().isPresent()){
            s_Swerve.feedInitialPose(trajectory.getInitialPose().get());
        }
        else{
            s_Swerve.feedInitialPose(new Pose2d());
        }
        
        routine.active().whileTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
        return routine.cmd();
    }

    public Command none(){
        return new InstantCommand();
    }
}