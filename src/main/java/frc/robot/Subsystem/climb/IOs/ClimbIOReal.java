
package frc.robot.Subsystem.climb.IOs;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
    
    protected TalonFX rightMotor;
    protected TalonFX leftMotor;

    protected TalonFXConfiguration rightConfig;
    protected TalonFXConfiguration leftConfig;

    private DigitalInput firstSensor;
    private DigitalInput secondSensor;

    private StatusSignal<Current> RightMotorCurrent; 
    private StatusSignal<Current> LeftMotorCurrent;

    private StatusSignal<AngularVelocity> RightMotorVelocity;
    private StatusSignal<AngularVelocity> LeftMotorVelocity;

    private StatusSignal<Voltage> RightMotorapliedVolts;
    private StatusSignal<Voltage> LeftMotorapliedVolts;

    private LoggedDouble RightMotorCurrentLog;
    private LoggedDouble LeftMotorCurrentLog;
    private LoggedDouble RightMotorVelocityLog;
    private LoggedDouble LeftMotorVelocityLog;
    private LoggedDouble RightMotorapliedVoltsLog;
    private LoggedDouble LeftMotorapliedVoltsLog;


    public ClimbIOReal() {
        rightMotor = new TalonFX(PortMap.Climb.ClimbRightMotor, PortMap.CanBus.CANivoreBus);
        leftMotor = new TalonFX(PortMap.Climb.ClimbLeftMotor, PortMap.CanBus.CANivoreBus);

        rightConfig = new TalonFXConfiguration();
        leftConfig = new TalonFXConfiguration();

        firstSensor = new DigitalInput(PortMap.Climb.ClimbFirstSensor);
        secondSensor = new DigitalInput(PortMap.Climb.ClimbSecondSensor);

        RightMotorCurrent = rightMotor.getStatorCurrent();
        LeftMotorCurrent = leftMotor.getStatorCurrent();

        RightMotorVelocity = rightMotor.getVelocity();
        LeftMotorVelocity = rightMotor.getVelocity();

        RightMotorapliedVolts = rightMotor.getMotorVoltage();
        LeftMotorapliedVolts = leftMotor.getMotorVoltage();

        RightMotorCurrentLog = new LoggedDouble("/Subsystems/Climb/IO/Right Motor Current");
        LeftMotorCurrentLog = new LoggedDouble("/Subsystems/Climb/IO/Left Motor Current");
        RightMotorVelocityLog = new LoggedDouble("/Subsystems/Climb/IO/Right Motor Velocity");
        LeftMotorVelocityLog = new LoggedDouble("/Subsystems/Climb/IO/Left Motor Velocity");
        RightMotorapliedVoltsLog = new LoggedDouble("/Subsystems/Climb/IO/Right Motor aplied Volts");
        LeftMotorapliedVoltsLog = new LoggedDouble("/Subsystems/Climb/IO/Left Motor aplied Volts");
    }

    public void RightConfig() {
        rightConfig.Feedback.RotorToSensorRatio = climbConstants.GEAR;

        rightConfig.Voltage.PeakForwardVoltage = 12;
        rightConfig.Voltage.PeakReverseVoltage = -12;

        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = climbConstants.kENABLE_CURRENT_LIMIT;
        rightConfig.CurrentLimits.SupplyCurrentLimit = climbConstants.kCURRENT_LIMIT;
        rightConfig.CurrentLimits.SupplyCurrentLowerLimit = climbConstants.kCONTINUOUS_LOWER_LIMIT;
        rightConfig.CurrentLimits.SupplyCurrentLowerTime = climbConstants.kCONTINUOUS_CURRENT_TIME;

        rightMotor.getConfigurator().apply(rightConfig);
    }

    public void LeftConfig() {
        leftConfig.Feedback.RotorToSensorRatio = climbConstants.GEAR;

        leftConfig.Voltage.PeakForwardVoltage = 12;
        leftConfig.Voltage.PeakReverseVoltage = -12;

        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = climbConstants.kENABLE_CURRENT_LIMIT;
        leftConfig.CurrentLimits.SupplyCurrentLimit = climbConstants.kCURRENT_LIMIT;
        leftConfig.CurrentLimits.SupplyCurrentLowerLimit = climbConstants.kCONTINUOUS_LOWER_LIMIT;
        leftConfig.CurrentLimits.SupplyCurrentLowerTime = climbConstants.kCONTINUOUS_CURRENT_TIME;

        leftMotor.getConfigurator().apply(leftConfig);
    }

    public void setVoltage(double volt) {
        leftMotor.setVoltage(volt);
    }

    public void setLeftNutralMode(boolean isBreak) {
        if (isBreak) {
            leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
        else {
            leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        leftMotor.getConfigurator().apply(leftConfig);
    }

    public void setRightNutralMode(boolean isBreak) {
        if (isBreak) {
            rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
        else {
            rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        rightMotor.getConfigurator().apply(leftConfig);
    }

    public double getRightCurrent() {
        return RightMotorCurrent.getValueAsDouble();
    }

    public double getRightVelocity() {
        return ConvUtil.RadiansToDegrees(RightMotorVelocity.getValueAsDouble());
    }

    public double getRightAppliedVolts() {
        return RightMotorapliedVolts.getValueAsDouble();
    }

    public double getLeftCurrent() {
        return LeftMotorCurrent.getValueAsDouble();
    }

    public double getLeftVelocity() {
        return ConvUtil.RadiansToDegrees(LeftMotorVelocity.getValueAsDouble());
    }

    public double getLeftAppliedVolts() {
        return LeftMotorapliedVolts.getValueAsDouble();
    }

    public Boolean getFirstSensor() {
        return firstSensor.get();
    }

    public Boolean getSecondSensor() {
        return secondSensor.get();
    }

    public void updatePeriodic() {
        RightMotorCurrentLog.update(getRightCurrent());
        LeftMotorCurrentLog.update(getLeftCurrent());
        RightMotorVelocityLog.update(getRightVelocity());
        LeftMotorVelocityLog.update(getLeftVelocity());
        RightMotorapliedVoltsLog.update(getRightAppliedVolts());
        LeftMotorapliedVoltsLog.update(getLeftAppliedVolts());
    }

}
