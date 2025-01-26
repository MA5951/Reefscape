
package frc.robot.Subsystem.climb.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.Subsystem.climb.climbConstants;

public class ClimbIOReal implements ClimbIO{
    
    protected TalonFX masterMotor;
    protected TalonFX slaveMotor;

    protected TalonFXConfiguration masterConfig;

    private DigitalInput firstSensor;
    private DigitalInput secondSensor;

    private StatusSignal<Current> masterMotorCurrent; 

    private StatusSignal<AngularVelocity> masterMotorVelocity;

    private StatusSignal<Voltage> masterMotorapliedVolts;

    private LoggedDouble masterMotorCurrentLog;
    private LoggedDouble masterMotorVelocityLog;
    private LoggedDouble masterMotorapliedVoltsLog;

    private StrictFollower masterFollwer;


    public ClimbIOReal() {
        masterMotor = new TalonFX(PortMap.Climb.ClimbMasterMotor, PortMap.CanBus.CANivoreBus);
        slaveMotor = new TalonFX(PortMap.Climb.ClimbSlaveMotor, PortMap.CanBus.CANivoreBus);

        masterConfig = new TalonFXConfiguration();

        firstSensor = new DigitalInput(PortMap.Climb.ClimbFirstSensor);
        secondSensor = new DigitalInput(PortMap.Climb.ClimbSecondSensor);

        masterFollwer = new StrictFollower(PortMap.Climb.ClimbMasterMotor);

        masterMotorCurrent = masterMotor.getStatorCurrent();

        masterMotorVelocity = masterMotor.getVelocity();

        masterMotorapliedVolts = masterMotor.getMotorVoltage();

        masterMotorCurrentLog = new LoggedDouble("/Subsystems/Climb/IO/master Motor Current");
        masterMotorVelocityLog = new LoggedDouble("/Subsystems/Climb/IO/master Motor Velocity");
        masterMotorapliedVoltsLog = new LoggedDouble("/Subsystems/Climb/IO/master Motor aplied Volts");
    }

    public void masterConfig() {
        masterConfig.Feedback.RotorToSensorRatio = climbConstants.GEAR;

        masterConfig.Voltage.PeakForwardVoltage = 12;
        masterConfig.Voltage.PeakReverseVoltage = -12;

        masterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        masterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        masterConfig.CurrentLimits.SupplyCurrentLimitEnable = climbConstants.kENABLE_CURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLimit = climbConstants.kCURRENT_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLowerLimit = climbConstants.kCONTINUOUS_LOWER_LIMIT;
        masterConfig.CurrentLimits.SupplyCurrentLowerTime = climbConstants.kCONTINUOUS_CURRENT_TIME;

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
        return ConvUtil.RadiansToDegrees(masterMotorVelocity.getValueAsDouble());
    }

    public double getMasterAppliedVolts() {
        return masterMotorapliedVolts.getValueAsDouble();
    }

    public Boolean getFirstSensor() {
        return firstSensor.get();
    }

    public Boolean getSecondSensor() {
        return secondSensor.get();
    }

    public void updatePeriodic() {
        masterMotorCurrentLog.update(getMasterCurrent());
        masterMotorVelocityLog.update(getMasterVelocity());
        masterMotorapliedVoltsLog.update(getMasterAppliedVolts());

        StatusSignal.refreshAll(
        masterMotorCurrent,
        masterMotorVelocity,
        masterMotorapliedVolts
        );
    }

}
