package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.commons.LoggedTunableNumber;
import frc.robot.Constants.canIDConstants;
import frc.robot.Subsystems.BeamBreak.BeamBreakIO;
import frc.robot.Subsystems.BeamBreak.BeamBreakIOInputsAutoLogged;
import frc.robot.Subsystems.Dealgae.Dealgae;
import frc.robot.Subsystems.Dealgae.DealgaeIO;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorIO;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffectorIO;
import frc.robot.Subsystems.Funnel.Funnel;
import frc.robot.Subsystems.Funnel.FunnelIO;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.IntakeIO;
import frc.robot.Subsystems.LEDs.LEDs;
import frc.robot.Subsystems.LEDs.LEDs.LEDStates;

public class Superstructure extends SubsystemBase {
    private Dealgae s_dealgae;
    private Elevator s_elevator;
    private EndEffector s_endeffector;
    private LEDs led;
    private BeamBreakIO beambreak;
    private Intake s_intake;
    private Funnel s_funnel;
    private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

    private boolean hasAlgae = false;
    private boolean hasCoral = false;

    LoggedTunableNumber setpoint = new LoggedTunableNumber("Setpoint", 0.1);

    private double stateStartTime = 0;
    private SuperstructureStates systemState = SuperstructureStates.IDLE;

    public Superstructure(DealgaeIO dealgaeIO, ElevatorIO elevatorIO, EndEffectorIO endEffectorIO, LEDs led, FunnelIO funnelIO, IntakeIO intakeIO, BeamBreakIO beamBreakIO){
        this.s_dealgae = new Dealgae(dealgaeIO);
        this.s_elevator = new Elevator(elevatorIO);
        this.s_endeffector = new EndEffector(endEffectorIO);
        this.s_funnel = new Funnel(funnelIO);
        this.s_intake = new Intake(intakeIO);
        this.beambreak = beamBreakIO;
        this.led = led; 
        
    }

    public enum SuperstructureStates {
        IDLE,
        INTAKE_A,
        INTAKE_B,
        INTAKE_C,
        POST_INTAKE,
        OUTTAKE,
        SCORE_A,
        SCORE_B,
        DEALGAE_A,
        DEALGAE_B,
        DEALGAED,
        HOLD_ALGAE,
        HOLD_CORAL,
        PROCESSOR,
        ELEVATOR_DOWN,
        ELEVATOR_DOWN_B,
        PULSE_INTAKE,
        GROUND_INTAKE_CORAL,
        GROUND_INTAKE_ALGAE,
        GROUND_SCORE_CORAL,
        GROUND_PRCOESS_ALGAE
    }

    @Override
    public void periodic(){
        s_dealgae.Loop();
        s_elevator.Loop();
        s_endeffector.Loop();
        s_funnel.Loop();
        led.Loop();
        beambreak.updateInputs(beamBreakInputs);
        Logger.processInputs("BeamBreak", beamBreakInputs);
        Logger.recordOutput("SuperstructureState", this.systemState);
        Logger.recordOutput("State start time", stateStartTime);
        switch(systemState){
            case IDLE:
                if (DriverStation.isDisabled()) {
                    led.setState(LEDStates.DISABLED);
                 
                } else {
                    if (s_elevator.selectedHeight == "L2"){
                        led.requestL2();
                    } if (s_elevator.selectedHeight == "L3"){
                        led.requestL3();
                    } if (s_elevator.selectedHeight == "L4"){
                        led.requestL4();
                    } if (s_elevator.selectedHeight == "L1"){
                        led.requestIdleLED();
                    }
                }
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIdle();
                s_endeffector.requestIdle();
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                break;
            case INTAKE_A:
                led.requestFunnelIntakingLED();
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIntake(2);
                s_endeffector.requestIntake(4.5);
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (isBeamBroken()){
                    setState(SuperstructureStates.INTAKE_B);
                }
                break;
            case INTAKE_B:
                led.requestFunnelIntakingLED();
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIntake(0.75);
                s_endeffector.requestIntake(2);
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (!isBeamBroken()){
                    setState(SuperstructureStates.INTAKE_C);
                }
                break;
            case INTAKE_C:
                led.requestFunnelIntakingLED();
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIdle();
                s_endeffector.requestIntake(-1.25);
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (isBeamBroken()){
                    setState(SuperstructureStates.POST_INTAKE);
                }
                break;
            case POST_INTAKE:
                led.requestFunnelIntookLED();
                s_dealgae.requestIdle();
                s_funnel.requestIdle();
                s_elevator.requestIdle();
                s_endeffector.requestIdle();
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (!isBeamBroken()){
                    setState(SuperstructureStates.INTAKE_C);
                }
                break;
       
            case OUTTAKE:
                led.requestFunnelIntakingLED();
                s_dealgae.requestIdle();
                s_funnel.requestIntake(-1);
                s_elevator.requestIdle();
                s_endeffector.requestIntake(-3);
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                break;
            case SCORE_A:
                led.requestScoringLED();
                s_dealgae.requestIdle();
                s_elevator.requestMotionMagicCoral();
                s_funnel.requestIdle();
                s_endeffector.requestIdle();
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.SCORE_B);
                }
                break;
            case SCORE_B:
                led.requestScoringLED();
                s_dealgae.requestIdle();
                s_elevator.requestHold();
                s_endeffector.requestScore(3);
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.ELEVATOR_DOWN);
                }
                break;
            case DEALGAE_A:
                led.requestDealgaingLED();
                s_dealgae.requestIdle();
                s_elevator.requestMotionMagicAlgae();
                s_endeffector.requestIdle();
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.DEALGAE_B);
                }
                break;
            case DEALGAE_B:
                led.requestDealgaingLED();
                s_dealgae.requestDealgae(3);
                s_elevator.requestHold();
                s_endeffector.requestIdle();
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (s_dealgae.getDealgaeCurrent() > 27 && RobotController.getFPGATime() / 1.0E6 - stateStartTime > 0.5){
                    setState(SuperstructureStates.DEALGAED);
                }
                break;
            case DEALGAED:
                led.requestDealgaedLED();
                s_dealgae.requestIdle();
                s_elevator.requestElevatorDown();
                s_endeffector.requestIdle();
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.HOLD_ALGAE);
                }
                break;
            case HOLD_ALGAE: // this cancels
                led.requestDealgaedLED();
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_endeffector.requestHoldAlgae(0.5);
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                break;
            case PROCESSOR:
                led.requestProcessingLED();
                s_dealgae.requestProcessor(-3);
                s_elevator.requestElevatorDown();
                s_endeffector.requestIdle();
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (RobotController.getFPGATime() / 1.0E6 - stateStartTime > 1) {
                    setState(SuperstructureStates.IDLE);
                }
                break;            
            case ELEVATOR_DOWN:
                if (s_elevator.selectedHeight == "L2"){
                    led.requestL2();
                } if (s_elevator.selectedHeight == "L3"){
                    led.requestL3();
                } if (s_elevator.selectedHeight == "L4"){
                    led.requestL4();
                } if (s_elevator.selectedHeight == "L1"){
                    led.requestIdleLED();
                }   
                s_dealgae.requestIdle();
                s_elevator.requestElevatorDown();
                s_endeffector.requestIdle();
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.ELEVATOR_DOWN_B);
                }
                break;
            case ELEVATOR_DOWN_B:
                if (s_elevator.selectedHeight == "L2"){
                    led.requestL2();
                } if (s_elevator.selectedHeight == "L3"){
                    led.requestL3();
                } if (s_elevator.selectedHeight == "L4"){
                    led.requestL4();
                } if (s_elevator.selectedHeight == "L1"){
                    led.requestIdleLED();
                }   
                s_dealgae.requestIdle();
                s_elevator.requestSlow();
                s_endeffector.requestIdle();
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if (s_elevator.atSetpoint()){
                    setState(SuperstructureStates.IDLE);
                }

            case PULSE_INTAKE:
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIntake(3);
                s_endeffector.requestIntake(5);
                if (hasCoral == true){
                    s_intake.requestHoldCoral();
                } else if (hasAlgae = true){
                    s_intake.requestHoldAlgae();
                } else {
                    s_intake.requestSetpoint();
                }
                if(RobotController.getFPGATime() / 1.056 - stateStartTime > 7){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case GROUND_INTAKE_CORAL:
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIdle();
                s_endeffector.requestIdle();
                s_intake.requestIntakeCoral();
                if(RobotController.getFPGATime() / 1.056 - stateStartTime > 0.5 && s_intake.getRollerCurrent() > 26){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case GROUND_INTAKE_ALGAE:
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIdle();
                s_endeffector.requestIdle();
                s_intake.requestIntakeAlgae();
                if(RobotController.getFPGATime() / 1.056 - stateStartTime > 0.5 && s_intake.getRollerCurrent() > 26){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case GROUND_SCORE_CORAL:
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIdle();
                s_endeffector.requestIdle();
                s_intake.requestScoreCoral();
                if(RobotController.getFPGATime() / 1.056 - stateStartTime > 2){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            case GROUND_PRCOESS_ALGAE:
                s_dealgae.requestIdle();
                s_elevator.requestIdle();
                s_funnel.requestIdle();
                s_endeffector.requestIdle();
                s_intake.requestPrcoessAlgae();
                if(RobotController.getFPGATime() / 1.056 - stateStartTime > 2){
                    setState(SuperstructureStates.IDLE);
                }
                break;
            default:
                break;
        }
    }

    public void requestIdle(){
        setState(SuperstructureStates.IDLE);
    }

    public void requestScore(){
        setState(SuperstructureStates.SCORE_A);
    }

    public void requestGroundIntakeCoral(){
        setState(SuperstructureStates.GROUND_INTAKE_CORAL);
    }

    public void requestGroundIntakeAlgae(){
        setState(SuperstructureStates.GROUND_INTAKE_ALGAE);
    }

    public void requestGroundScoreCoral(){
        setState(SuperstructureStates.GROUND_SCORE_CORAL);
    }

    public void requestGroundProcessAlgae(){
        setState(SuperstructureStates.GROUND_PRCOESS_ALGAE);
    }

    public void requestIntake(){

        setState(SuperstructureStates.INTAKE_A);
    }

    public void requestDealgae(){
        setState(SuperstructureStates.DEALGAE_A);
    }
    
    public void requestProcessor(){
        setState(SuperstructureStates.PROCESSOR);
    }

    public void requestElevatorDown(){
        setState(SuperstructureStates.ELEVATOR_DOWN);
    }

    public void setL1(){
        s_elevator.setHeight("L1");
    }

    public void setL2(){
        s_elevator.setHeight("L2");
    }

    public void setL3(){
        s_elevator.setHeight("L3");
    }

    public void setL4(){
        s_elevator.setHeight("L4");
    }

    public void requestOuttake(){
        setState(SuperstructureStates.OUTTAKE);
    }

    public void requestPulseIntake(){
        setState(SuperstructureStates.PULSE_INTAKE);
    }

    public boolean isBeamBroken(){
        return beamBreakInputs.beamBroken;
    }

    public void setState(SuperstructureStates nextState){
        systemState = nextState;
        stateStartTime = RobotController.getFPGATime() / 1E6;
    }

    public SuperstructureStates getState(){
        return systemState;
    }
}