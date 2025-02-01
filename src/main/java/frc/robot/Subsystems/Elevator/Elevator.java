package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

public class Elevator {
    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private ElevatorStates elevatorState = ElevatorStates.IDLE;
    private double elevatorSetpoint = 0;
    private double elevatorVoltage = 0;

    public enum ElevatorStates{
        IDLE,
        L1,
        L2,
        L3,
        L4
    }

    public void Loop(){
        elevatorIO.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Elevator", this.elevatorState);
        switch(elevatorState){
            case IDLE:
                elevatorIO.requestElevatorVoltage(0);
                elevatorIO.requestMotionMagicConfigs(false);
                break;
            case L1:
            case L2:
            case L3:
            case L4:
        }
    }

    public void requestIdle(){
        setState(elevatorState.IDLE);
    }

    public void requestL1(){
        setState(elevatorState.L1);
    }

    public void requestL2(){
        setState(elevatorState.L2);
    }

    public void requestL3(){
        setState(elevatorState.L3);
    }

    public void requestL4(){
        setState(elevatorState.L4);
    }

    public void setState(ElevatorStates nextState){
        this.elevatorState = elevatorState;
    }

    public double getElevatorVelMPS(){
        return inputs.elevatorVelMPS;
    }

    public Elevator(ElevatorIO elevatorIO){
        this.elevatorIO = elevatorIO;
    }

    public void elevatorConfiguration(){
        elevatorIO.elevatorConfiguration();
    }

    public ElevatorStates getState(){
        return this.elevatorState;
    }

}
