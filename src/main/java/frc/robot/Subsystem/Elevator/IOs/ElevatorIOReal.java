
package frc.robot.Subsystem.Elevator.IOs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.LoggedDouble;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Elevator.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

    protected TalonFX masterMotor;
    protected TalonFX slaveMotor;

    private DigitalInput limitSwitch;

    private MotionMagicVoltage MotionMagic; 
    protected TalonFXConfiguration masterConfig;
    private StrictFollower masterFollower;

    private StatusSignal<Current> masterMotorCurrent;
    private StatusSignal<Angle> masterMotorPosition;
    private StatusSignal<AngularVelocity> masterMotorVelocity;
    private StatusSignal<Voltage> masterMotorAppliedVoltage;
    private StatusSignal<Double> masterError;
    private StatusSignal<Double> masterSetPoint;

    private LoggedDouble masterMotorCurrentLog;
    private LoggedDouble masterMotorPositionLog;
    private LoggedDouble masterMotorVelocityLog;
    private LoggedDouble masterMotorAppliedVoltageLog;
    private LoggedDouble masterErrorLog;
    private LoggedDouble masterSetPointLog;
    
    public ElevatorIOReal() {
        masterMotor = new TalonFX(PortMap.Elevator.elevatorMasterMotor, PortMap.CanBus.CANivoreBus);
        slaveMotor = new TalonFX(PortMap.Elevator.elevatorSlaveMotor, PortMap.CanBus.CANivoreBus);
        masterConfig = new TalonFXConfiguration();
        MotionMagic = new MotionMagicVoltage(0);
        masterFollower = new StrictFollower(PortMap.Elevator.elevatorMasterMotor); 

        limitSwitch = new DigitalInput(PortMap.Elevator.elevatorLimitSwich);

        masterMotorCurrent = masterMotor.getStatorCurrent();
        masterMotorPosition = masterMotor.getPosition();
        masterMotorVelocity = masterMotor.getVelocity();
        masterMotorAppliedVoltage = masterMotor.getMotorVoltage();
        masterError = masterMotor.getClosedLoopError();
        masterSetPoint = masterMotor.getClosedLoopReference();

        masterMotorCurrentLog = new LoggedDouble("/Subsystems/Elevator/IO/Master Motor Current");
        masterMotorPositionLog = new LoggedDouble("/Subsystems/Elevator/IO/Master Motor Position");
        masterMotorVelocityLog = new LoggedDouble("/Subsystems/Elevator/IO/Master Motor Velocity");
        masterMotorAppliedVoltageLog = new LoggedDouble("/Subsystems/Elevator/IO/Master Motor Applied Voltage");
        masterErrorLog = new LoggedDouble("/Subsystems/Elevator/IO/Master Error");
        masterSetPointLog = new LoggedDouble("/Subsystems/Elevator/IO/Master Set Point");

        configMotors();

        masterMotor.setPosition(0);
    }

    public void configMotors() {
        masterConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR * ElevatorConstants.SPROKET_CIRCUMFERENCE;

        masterConfig.Voltage.PeakForwardVoltage = RobotConstants.NOMINAL_VOLTAGE;
        masterConfig.Voltage.PeakReverseVoltage = -RobotConstants.NOMINAL_VOLTAGE;
        
        masterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        masterConfig.Slot0.kP = ElevatorConstants.kP;
        masterConfig.Slot0.kI = ElevatorConstants.kI;
        masterConfig.Slot0.kD = ElevatorConstants.kD;
        masterConfig.Slot0.kS = ElevatorConstants.kS;
        
        masterConfig.MotionMagic.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;
        masterConfig.MotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.CRUSIE_VELOCITY;
        masterConfig.MotionMagic.MotionMagicJerk = ElevatorConstants.JERK;

        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = ElevatorConstants.ENABLE_CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLimit = ElevatorConstants.CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLowerLimit = ElevatorConstants.CONTINUOUS_LOWER_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLowerTime = ElevatorConstants.CONTINUOUS_CURRENT_DURATION;

        masterMotor.getConfigurator().apply(masterConfig);
        slaveMotor.getConfigurator().apply(masterConfig);
    }


    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public double getCurrent() {
        return masterMotorCurrent.getValueAsDouble();
    }

    public double getPosition() {
        return masterMotorPosition.getValueAsDouble()  * 2 / 100;
    }

    public double getVelocity() {
        return masterMotorVelocity.getValueAsDouble();
    }

    public double getAppliedVolts() {
        return masterMotorAppliedVoltage.getValueAsDouble();
    }

    public double getError() {
        return masterError.getValueAsDouble() * 2 / 100; 
    }

    
    public double getSetPoint() {
        return masterSetPoint.getValueAsDouble()* 2 / 100; 
    }

    public void resetPosition(double newHight) {
        masterMotor.setPosition(newHight);
    }

    public void setVoltage(double volt) {
        masterMotor.setVoltage(volt);
        slaveMotor.setControl(masterFollower); 
    }

    public void updatePID(double Kp , double Ki , double Kd) {
        masterConfig.Slot0.kP = Kp;
        masterConfig.Slot0.kI = Ki;
        masterConfig.Slot0.kD = Kd;

        masterMotor.getConfigurator().apply(masterConfig);
    }

    public void setNutralMode(boolean isbrake) {
        if (isbrake) {
            masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }

        masterMotor.getConfigurator().apply(masterConfig);
        slaveMotor.getConfigurator().apply(masterConfig);
    }

    public void setHight(double hight) {
        masterMotor.setControl(MotionMagic.withPosition((hight / 2 * 100 ) ).withSlot(ElevatorConstants.CONTROL_SLOT).withFeedForward(
            ElevatorConstants.FEED_FORWARD
        ));
        slaveMotor.setControl(masterFollower);
    }

    public void updatePeriodic() {
        BaseStatusSignal.refreshAll(
            masterMotorCurrent,
            masterMotorPosition,
            masterMotorVelocity,
            masterMotorAppliedVoltage,
            masterError,
            masterSetPoint
        );


        masterMotorCurrentLog.update(getCurrent());
        masterMotorPositionLog.update(getPosition());
        masterMotorVelocityLog.update(getVelocity());
        masterMotorAppliedVoltageLog.update(getAppliedVolts());
        masterErrorLog.update(getError());
        masterSetPointLog.update(getSetPoint());

    }
}

