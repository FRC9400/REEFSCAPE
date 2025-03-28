package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.SignalLogger;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class Intake{
    private final IntakeIO intakeIO;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private IntakeStates intakeState = IntakeStates.IDLE;
    private double pivotSetpoint = 0;
   private double rollerSetpoint = 0;
 
 enum IntakeStates{
        IDLE,
        INTAKE,
        UP,
        SETPOINT,
        SCORE
    }

    public Intake(IntakeIO intakeIO){
        this.intakeIO = intakeIO;
    }

    public void Loop(){
        intakeIO.updateTunableNumbers();
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("IntakeState", intakeState);

        switch(intakeState){
            case IDLE:
            intakeIO.resetMotionMagicConfigs(false);
                intakeIO.requestPivotVoltage(0);
                intakeIO.requestRollerVoltage(0);
                break;
            case SETPOINT:
            intakeIO.resetMotionMagicConfigs(false);
                intakeIO.requestMotionMagic(pivotSetpoint);
                intakeIO.requestRollerVoltage(0);
                break;
            case INTAKE:
            
                intakeIO.requestMotionMagic(pivotSetpoint);
                intakeIO.requestRollerVoltage(rollerSetpoint);
                break;
            case UP:
            intakeIO.resetMotionMagicConfigs(true);
            intakeIO.requestMotionMagic(pivotSetpoint);
            intakeIO.requestRollerVoltage(rollerSetpoint);
            case SCORE:
            intakeIO.resetMotionMagicConfigs(false);
                intakeIO.requestMotionMagic(pivotSetpoint);
                intakeIO.requestRollerVoltage(rollerSetpoint);
                break;
            default:
                break;
                
        }
    }

    
    public void requestIdle(){
        setState(IntakeStates.IDLE);
    }

    public void requestSetpoint(double degrees){
        pivotSetpoint = degrees;
        setState(IntakeStates.SETPOINT);
    }

    public void requestIntakeCoral(){
        pivotSetpoint = 135;
        rollerSetpoint = 3;
        setState(IntakeStates.INTAKE);
    }

    public void requestUp(){
        rollerSetpoint = 5;
        setState(IntakeStates.UP);
    }

    public void requestIntakeAlgae(){
        intakeIO.resetMotionMagicConfigs(false);
        pivotSetpoint = 60;
        rollerSetpoint = -3;
        setState(IntakeStates.INTAKE);
    }

    public void requestScoreCoral(){
        pivotSetpoint = 25;
        rollerSetpoint = -3;
        setState(IntakeStates.SCORE);
    }

    public void requestPrcoessAlgae(){
        pivotSetpoint = 40;
        rollerSetpoint = 3;
        setState(IntakeStates.SCORE);
    }

    public void requestHoldCoral(){
        pivotSetpoint = 0;
        rollerSetpoint = 3;
        setState(IntakeStates.INTAKE);
    }

    public void requestHoldAlgae(){
        intakeIO.resetMotionMagicConfigs(true);
        pivotSetpoint = 33;
        rollerSetpoint = -2;
        setState(IntakeStates.INTAKE);
    }

    public void setState(IntakeStates nextState){
        this.intakeState = nextState;
    }

    public double getRollerCurrent(){
        return inputs.rollerCurrentAmps;
    }

    public IntakeStates getIntakeState(){
        return this.intakeState;
    }


}