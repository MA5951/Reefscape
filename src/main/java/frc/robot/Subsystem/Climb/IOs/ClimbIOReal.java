
package frc.robot.Subsystem.Climb.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.Subsystem.Climb.ClimbConstants;

public class ClimbIOReal implements ClimbIO{
    
    protected TalonFX masterMotor;
    protected TalonFX slaveMotor;

    protected TalonFXConfiguration masterConfig;

    private DigitalInput limitSensor;

    private StatusSignal<Current> masterMotorCurrent; 
    private StatusSignal<AngularVelocity> masterMotorVelocity;
    private StatusSignal<Voltage> masterMotorapliedVolts;
    private StatusSignal<Angle> masterPosition;

    private LoggedDouble masterMotorCurrentLog;
    private LoggedDouble masterMotorVelocityLog;
    private LoggedDouble masterMotorapliedVoltsLog;
    private LoggedDouble masterPositionLog;

    private StrictFollower masterFollwer;


    public ClimbIOReal() {
        masterMotor = new TalonFX(PortMap.Climb.masterMotor, PortMap.CanBus.CANivoreBus);
        slaveMotor = new TalonFX(PortMap.Climb.slaveMotor, PortMap.CanBus.CANivoreBus);

        masterConfig = new TalonFXConfiguration();

        limitSensor = new DigitalInput(PortMap.Climb.limitSensor);

        masterFollwer = new StrictFollower(PortMap.Climb.masterMotor);

        masterMotorCurrent = masterMotor.getStatorCurrent();
        masterMotorVelocity = masterMotor.getVelocity();
        masterMotorapliedVolts = masterMotor.getMotorVoltage();
        masterPosition = masterMotor.getPosition();

        masterMotorCurrentLog = new LoggedDouble("/Subsystems/Climb/IO/master Motor Current");
        masterMotorVelocityLog = new LoggedDouble("/Subsystems/Climb/IO/master Motor Velocity");
        masterMotorapliedVoltsLog = new LoggedDouble("/Subsystems/Climb/IO/master Motor aplied Volts");
        masterPositionLog = new LoggedDouble("/Subsystems/Climb/IO/Position");
    }

    public void masterConfig() {
        masterConfig.Feedback.RotorToSensorRatio = ClimbConstants.GEAR;

        masterConfig.Voltage.PeakForwardVoltage = 12;
        masterConfig.Voltage.PeakReverseVoltage = -12;

        masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = ClimbConstants.kENABLE_CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLimit = ClimbConstants.kCURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLowerLimit = ClimbConstants.kCONTINUOUS_LOWER_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLowerTime = ClimbConstants.kCONTINUOUS_CURRENT_TIME;

        masterMotor.getConfigurator().apply(masterConfig);
        slaveMotor.getConfigurator().apply(masterConfig);
    }


    public void setVoltage(double volt) {
        masterMotor.setVoltage(volt);
        slaveMotor.setControl(masterFollwer);
    }

    public void setMasterNutralMode(boolean isBreak) {
        if (isBreak) {
            masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
        else {
            masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        masterMotor.getConfigurator().apply(masterConfig);
        slaveMotor.getConfigurator().apply(masterConfig);
    }


    public double getMasterCurrent() {
        return masterMotorCurrent.getValueAsDouble();
    }

    public double getMasterVelocity() {
        return ConvUtil.RotationsToDegrees(masterMotorVelocity.getValueAsDouble());
    }

    public double getMasterAppliedVolts() {
        return masterMotorapliedVolts.getValueAsDouble();
    }

    public Boolean getLimitSensor() {
        return limitSensor.get();
    }

    public double getPosition() {
        return masterPosition.getValueAsDouble() * 360;
    }

    public void updatePeriodic() {
        StatusSignal.refreshAll(
        masterMotorCurrent,
        masterMotorVelocity,
        masterMotorapliedVolts,
        masterPosition
        );
        
        masterMotorCurrentLog.update(getMasterCurrent());
        masterMotorVelocityLog.update(getMasterVelocity());
        masterMotorapliedVoltsLog.update(getMasterAppliedVolts());
        masterPositionLog.update(getPosition());
        
    }

}
