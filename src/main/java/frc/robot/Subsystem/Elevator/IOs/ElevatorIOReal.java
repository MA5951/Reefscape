
package frc.robot.Subsystem.Elevator.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    private MotionMagicVoltage MotionMagic = new MotionMagicVoltage(0);
    private TalonFXConfiguration masterConfig;
    private StrictFollower masterFollower;

    private StatusSignal<Current> masterMotorCurrent;
    private StatusSignal<Angle> masterMotorPosition;
    private StatusSignal<AngularVelocity> masterMotorVelocity;
    private StatusSignal<Voltage> masterMotorAppliedVoltage;
    private StatusSignal<Double> masterError;
    
    public ElevatorIOReal() {
        masterMotor = new TalonFX(PortMap.Elevator.elevatorMasterMotor, PortMap.CanBus.CANivoreBus);
        slaveMotor = new TalonFX(PortMap.Elevator.elevatorSlaveMotor, PortMap.CanBus.CANivoreBus);
        masterConfig = new TalonFXConfiguration();
        masterFollower = new StrictFollower(PortMap.Elevator.elevatorSlaveMotor);

        limitSwitch = new DigitalInput(PortMap.Elevator.elevatorLimitSwich);

        masterMotor.getStatorCurrent();
        masterMotorPosition = masterMotor.getPosition();
        masterMotorVelocity = masterMotor.getVelocity();
        masterMotorAppliedVoltage = masterMotor.getMotorVoltage();
        masterError = masterMotor.getClosedLoopError();

        configMotors();

        

    }

    public void configMotors() {
        masterConfig.Feedback.SensorToMechanismRatio = ElevatorConstants.GEAR;

        masterConfig.Voltage.PeakForwardVoltage = 12;
        masterConfig.Voltage.PeakReverseVoltage = -12;
        
        masterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        masterConfig.Slot0.kP = ElevatorConstants.kP;
        masterConfig.Slot0.kI = ElevatorConstants.kI;
        masterConfig.Slot0.kD = ElevatorConstants.kD;
        
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
        return masterMotorPosition.getValueAsDouble();
    }

    public double getVelocity() {
        return masterMotorVelocity.getValueAsDouble();
    }

    public double getAppliedVolts() {
        return masterMotorAppliedVoltage.getValueAsDouble();
    }

    public double getError() {
        return masterError.getValueAsDouble();
    }

    
    public double getSetPoint() {
        return mas
    }

    @Override
    public void resetPosition(double newPose) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'resetPosition'");
    }

    @Override
    public void setVoltage(double volt) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    public void updatePID(double Kp , double Ki , double Kd) {
        masterConfig.Slot0.kP = Kp;
        masterConfig.Slot0.kI = Ki;
        masterConfig.Slot0.kD = Kd;
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

    public void setHight(double angle) {
        masterMotor.setControl(MotionMagic.withPosition(angle).withSlot(ElevatorConstants.CONTROL_SLOT));  
    }
}

