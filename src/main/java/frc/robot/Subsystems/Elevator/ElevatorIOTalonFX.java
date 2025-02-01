package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.KeyDeserializer;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import frc.commons.LoggedTunableNumber;

public class ElevatorIOTalonFX implements ElevatorIO{
    // canID numbers are random
    private final TalonFX leftMotor = new TalonFX(1, "canivore");
    private final TalonFX rightMotor = new TalonFX(2, "canivore");
    
    private final TalonFXConfiguration leftMotorConfigs;
    private final TalonFXConfiguration rightMotorConfigs;
    private final TalonFXConfigurator leftMotorConfigurator; 
    private final TalonFXConfigurator rightMotorConfigurator;
    
    private final StatusSignal<Current> leftElevatorCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Current> rightElevatorCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Temperature> leftElevatorTemp = leftMotor.getDeviceTemp();
    private final StatusSignal<Temperature> rightElevatorTemp = rightMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> leftElevatorSpeedRPS = leftMotor.getRotorVelocity();
    private final StatusSignal<Angle> leftElevatorPos = leftMotor.getPosition();
    
    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private VoltageOut voltageOutRequest = new VoltageOut(0).withEnableFOC(true);

    double setPointMeters;
    private boolean climbDown;

    LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 1);
    LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 1);
    LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 1);
    LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 1);
    LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 1);
    LoggedTunableNumber kMotionCruiseVelocity = new LoggedTunableNumber("Elevator/kMotionCruiseVelocity", 1);
    LoggedTunableNumber kMotionAcceleration = new LoggedTunableNumber("Elevator/kMotionAcceleration", 1);
    LoggedTunableNumber kMotionJerk = new LoggedTunableNumber("Elevator/kMotionJerk", 1);


    public ElevatorIOTalonFX(){
        leftMotorConfigurator = leftMotor.getConfigurator();
        rightMotorConfigurator = rightMotor.getConfigurator();
        leftMotorConfigs = new TalonFXConfiguration();
        rightMotorConfigs = new TalonFXConfiguration();
        setPointMeters = 0;
        climbDown = false;

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            leftElevatorCurrent,
            rightElevatorCurrent,
            leftElevatorTemp,
            rightElevatorTemp,
            leftElevatorSpeedRPS,
            leftElevatorPos);

            leftMotor.optimizeBusUtilization();
            rightMotor.optimizeBusUtilization();
    }

    public void updateInputs(ElevatorIOInputs inputs){
        BaseStatusSignal.refreshAll(
            leftElevatorCurrent,
            rightElevatorCurrent,
            leftElevatorTemp,
            rightElevatorTemp,
            leftElevatorSpeedRPS,
            leftElevatorPos);

        inputs.appliedVolts = voltageOutRequest.Output;
        inputs.setPointMeters = setPointMeters;
        inputs.currentAmps = new double[] {leftElevatorCurrent.getValue(), rightElevatorCurrent.getValue()};


    }



    
}
