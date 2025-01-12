
package frc.robot.Subsystem.Intake.IOs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.PortMap;
import frc.robot.Subsystem.Intake.IntakeConstants;

public class IntakeIOReal implements IntakeIO {

    protected TalonFX intakeMotor;
    protected TalonFXConfiguration motorConfig;

    private StatusSignal<Current> motorCurrent;
    private StatusSignal<Angle> motorPosition;
    private StatusSignal<AngularVelocity> motorVelocity;
    private StatusSignal<Voltage> apliedVolts;
    private StatusSignal<ForwardLimitValue> forwardLimit;
    private StatusSignal<ReverseLimitValue> reversLimit;

    private LoggedDouble currentLog;
    private LoggedDouble positionLog;
    private LoggedDouble velocityLog;
    private LoggedDouble apliedVoltsLog;
    private LoggedBool forwardLimitLog;
    private LoggedBool reversLimitLog;

    public IntakeIOReal() {
        intakeMotor = new TalonFX(PortMap.Intake.intakeMotor , PortMap.CanBus.RioBus);
        motorConfig = new TalonFXConfiguration();

        motorCurrent = intakeMotor.getStatorCurrent();
        motorPosition = intakeMotor.getPosition();
        motorVelocity = intakeMotor.getVelocity();
        apliedVolts = intakeMotor.getMotorVoltage();
        forwardLimit = intakeMotor.getForwardLimit();
        reversLimit = intakeMotor.getReverseLimit();

        currentLog = new LoggedDouble("/Subsystems/Intake/IO/Current");
        positionLog = new LoggedDouble("/Subsystems/Intake/IO/Position");
        velocityLog = new LoggedDouble("/Subsystems/Intake/IO/Velocity");
        apliedVoltsLog = new LoggedDouble("/Subsystems/Intake/IO/Applied Volts");
        forwardLimitLog = new LoggedBool("/Subsystems/Intake/IO/Forward Limit");
        reversLimitLog = new LoggedBool("/Subsystems/Intake/IO/Revers Limit");

        config();   
    }

    private void config() {
        motorConfig.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR;

        motorConfig.Voltage.PeakForwardVoltage = 12;
        motorConfig.Voltage.PeakReverseVoltage = -12;

        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstants.IS_CURRENT_LIMIT_ENABLED;
        motorConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PEAK_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLowerLimit = IntakeConstants.CONTINUES_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLowerTime = IntakeConstants.PEAK_CURRENT_TIME;

        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        motorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        motorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

        intakeMotor.getConfigurator().apply(motorConfig);
    }

    public boolean getFrontSensor() {
        return forwardLimit.getValueAsDouble() == 1;
    }

    public boolean getRearSensor() {
        return reversLimit.getValueAsDouble() == 1;
    }

    public double getCurrent() {
        return motorCurrent.getValueAsDouble();
    }

    public double getPosition() {
        return ConvUtil.RotationsToDegrees(motorPosition.getValueAsDouble());
    }

    public double getVelocity() {
        return ConvUtil.RPStoRPM(motorVelocity.getValueAsDouble());
    }

    public double getAppliedVolts() {
        return apliedVolts.getValueAsDouble();
    }

    public void setNutralMode(boolean isbrake) {
        if (isbrake) {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        } else {
            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
        intakeMotor.getConfigurator().apply(motorConfig);
    }

    public void setVoltage(double volt) {
        intakeMotor.setVoltage(volt);
    }

    public void updatePeriodic() {
        BaseStatusSignal.refreshAll(
            motorCurrent,
            motorPosition,
            motorVelocity,
            apliedVolts,
            forwardLimit,
            reversLimit
        );

        currentLog.update(getCurrent());
        positionLog.update(getPosition());
        velocityLog.update(getVelocity());
        apliedVoltsLog.update(getAppliedVolts());
        forwardLimitLog.update(getFrontSensor());
        reversLimitLog.update(getRearSensor());
    }
}