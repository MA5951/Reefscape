
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
import frc.robot.Subsystem.Elevator.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO {

    protected TalonFX masterMotor;
    protected TalonFX slaveMotor;

    private DigitalInput limitSwitch;

    private MotionMagicVoltage MotionMagic = new MotionMagicVoltage(0); //TODO change to the home positiosn value and move to the constractor
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
        masterFollower = new StrictFollower(PortMap.Elevator.elevatorSlaveMotor); //TODO change to the master id

        limitSwitch = new DigitalInput(PortMap.Elevator.elevatorLimitSwich);

        masterMotorCurrent = masterMotor.getStatorCurrent();
        masterMotorPosition = masterMotor.getPosition();
        masterMotorVelocity = masterMotor.getVelocity();
        masterMotorAppliedVoltage = masterMotor.getMotorVoltage();
        masterError = masterMotor.getClosedLoopError();
        masterSetPoint = masterMotor.getClosedLoopReference();

        masterMotorCurrentLog = new LoggedDouble("Subsystems/Elevator/IO/Master Motor Current");
        masterMotorPositionLog = new LoggedDouble("/Subsystems/Elevator/IO/Master Motor Position");
        masterMotorVelocityLog = new LoggedDouble("/Subsystem/Elevator/IO/Master Motor Velocity");
        masterMotorAppliedVoltageLog = new LoggedDouble("/Subsystem/Elevator/IO/Master Motor Applied Voltage");
        masterErrorLog = new LoggedDouble("/Subsystem/Elevator/IO/Master Error");
        masterSetPointLog = new LoggedDouble("Subsystem/Elevator/IO/Master Set Point");

        configMotors();
    }

    public void configMotors() {
        masterConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR;

        masterConfig.Voltage.PeakForwardVoltage = 12; //TODO use the global constance
        masterConfig.Voltage.PeakReverseVoltage = -12;
        
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
        slaveMotor.getConfigurator().apply(masterConfig); //TODO chek if the need to rotat in the same diraction?
    }


    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    public double getCurrent() {
        return masterMotorCurrent.getValueAsDouble();
    }

    public double getPosition() {
        return masterMotorPosition.getValueAsDouble() * ElevatorConstants.SPROKET_CIRCUMFERENCE; //TODO why not to includ in the GEAR
    }

    public double getVelocity() {
        return masterMotorVelocity.getValueAsDouble() * 2 * Math.PI * (ElevatorConstants.SPROKET_PITCH_DIAMETER / 2);
    }

    public double getAppliedVolts() {
        return masterMotorAppliedVoltage.getValueAsDouble();
    }

    public double getError() {
        return masterError.getValueAsDouble() * ElevatorConstants.SPROKET_CIRCUMFERENCE;
    }

    
    public double getSetPoint() {
        return masterSetPoint.getValueAsDouble() * ElevatorConstants.SPROKET_CIRCUMFERENCE;
    }

    public void resetPosition(double newHight) {
        masterMotor.setPosition(newHight / ElevatorConstants.SPROKET_CIRCUMFERENCE);
    }

    public void resetPosition() {
        masterMotor.setPosition(0); //TODO change to the home positiosn value and use the resetPosition(double newHight) func
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
        masterMotor.setControl(MotionMagic.withPosition(hight / ElevatorConstants.SPROKET_CIRCUMFERENCE).withSlot(ElevatorConstants.CONTROL_SLOT));  
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
        masterMotorAppliedVoltageLog.update(getVelocity());
        masterErrorLog.update(getError());
        masterSetPointLog.update(getSetPoint());
    }
}

