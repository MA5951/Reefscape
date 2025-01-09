
package frc.robot.Subsystem.Elevator.IOs;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.Subsystem.Elevator.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

    protected TalonFX rightMotor;
    protected TalonFXConfiguration rightConfig;

    protected TalonFX leftMotor;
    protected TalonFXConfiguration leftConfig;

    private DigitalInput limitSwitch;

    private MotionMagicVoltage MotionMagic = new MotionMagicVoltage(0);

    private StatusSignal<Current> rightMotorCurrent;
    private StatusSignal<Angle> rightMotorPosition;
    private StatusSignal<AngularVelocity> rightMotorVelocity;
    private StatusSignal<Voltage> rightMotorAppliedVoltage;
    private StatusSignal<Double> rightError;
    
    private StatusSignal<Current> leftMotorCurrent;
    private StatusSignal<Angle> leftMotorPosition;
    private StatusSignal<AngularVelocity> leftMotorVelocity;
    private StatusSignal<Voltage> leftMotorappliedVoltage;
    private StatusSignal<Double> leftError;

    public ElevatorIOReal() {
        rightMotor = new TalonFX(PortMap.Elevator.elevatorRightMotor, PortMap.CanBus.RioBus);
        rightConfig = new TalonFXConfiguration();

        leftMotor = new TalonFX(PortMap.Elevator.elevatorLeftMotor, PortMap.CanBus.RioBus);
        leftConfig = new TalonFXConfiguration();

        limitSwitch = new DigitalInput(PortMap.Elevator.elevatorLimitSwich);

        rightMotorCurrent = rightMotor.getStatorCurrent();
        rightMotorPosition = rightMotor.getPosition();
        rightMotorVelocity = rightMotor.getVelocity();
        rightMotorAppliedVoltage = rightMotor.getMotorVoltage();
        rightError = rightMotor.getClosedLoopError();

        leftMotorCurrent = leftMotor.getStatorCurrent();
        leftMotorPosition = leftMotor.getPosition();
        leftMotorVelocity = leftMotor.getVelocity();
        leftMotorappliedVoltage = leftMotor.getMotorVoltage();
        leftError = leftMotor.getClosedLoopError();

        rightConfig();
        leftConfig();
    }

    public void rightConfig() {
        rightConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rightConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR;

        rightConfig.Voltage.PeakForwardVoltage = 12;
        rightConfig.Voltage.PeakReverseVoltage = -12;
        
        
        rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rightConfig.Slot0.kP = ElevatorConstants.kP;
        rightConfig.Slot0.kI = ElevatorConstants.kI;
        rightConfig.Slot0.kD = ElevatorConstants.kD;
        
        rightConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kACCELERATION;
        rightConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kCRUSIE_VELOCITY;
        rightConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.kJERK;

        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = ElevatorConstants.RIGHT_IS_CURRENT_LIMIT_ENABLED;
        rightConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.RIGHT_PEAK_CURRENT_LIMIT;
        rightConfig.CurrentLimits.SupplyCurrentLowerLimit = ElevatorConstants.RIGHT_CONTINUES_CURRENT_LIMIT;
        rightConfig.CurrentLimits.SupplyCurrentLowerTime = ElevatorConstants.RIGHT_PEAK_CURRENT_TIME;

        rightMotor.getConfigurator().apply(rightConfig);
    }

    public void leftConfig() {
        leftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        leftConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR;

        leftConfig.Voltage.PeakForwardVoltage = 12;
        leftConfig.Voltage.PeakReverseVoltage = -12;
        
        
        leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftConfig.Slot0.kP = ElevatorConstants.kP;
        leftConfig.Slot0.kI = ElevatorConstants.kI;
        leftConfig.Slot0.kD = ElevatorConstants.kD;
        
        leftConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.kACCELERATION;
        leftConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.kCRUSIE_VELOCITY;
        leftConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.kJERK;

        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = ElevatorConstants.LEFT_IS_CURRENT_LIMIT_ENABLED;
        leftConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.LEFT_PEAK_CURRENT_LIMIT;
        leftConfig.CurrentLimits.SupplyCurrentLowerLimit = ElevatorConstants.LEFT_CONTINUES_CURRENT_LIMIT;
        leftConfig.CurrentLimits.SupplyCurrentLowerTime = ElevatorConstants.LEFT_PEAK_CURRENT_TIME;

        leftMotor.getConfigurator().apply(leftConfig);
    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public double getCurrent() {
        return (rightMotorCurrent.getValueAsDouble() + leftMotorCurrent.getValueAsDouble())/2;
    }

    public double getPosition() {
        return (rightMotorPosition.getValueAsDouble() + leftMotorPosition.getValueAsDouble())/2;
    }

    public double getVelocity() {
        return (rightMotorVelocity.getValueAsDouble() + leftMotorVelocity.getValueAsDouble())/2;
    }

    public double getAppliedVolts() {
        return (rightMotorAppliedVoltage.getValueAsDouble() + leftMotorappliedVoltage.getValueAsDouble())/2;
    }

    public double getError() {
        return (rightError.getValueAsDouble() + leftError.getValueAsDouble())/2;
    }

    public void updatePID(double Kp , double Ki , double Kd) {
        leftConfig.Slot0.kP = Kp;
        leftConfig.Slot0.kI = Ki;
        leftConfig.Slot0.kD = Kd;

        rightConfig.Slot0.kP = Kp;
        rightConfig.Slot0.kI = Ki;
        rightConfig.Slot0.kD = Kd;
    }

    public void setNutralMode(boolean isbrake) {
        if (isbrake) {
            rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        rightMotor.getConfigurator().apply(rightConfig);
        leftMotor.getConfigurator().apply(leftConfig);
    }

    public void setHight(double angle) {
        leftMotor.setControl(MotionMagic.withPosition(angle).withSlot(ElevatorConstants.CONTROL_SLOT));  
    }
}

