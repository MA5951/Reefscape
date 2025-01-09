
package frc.robot.Subsystem.Intake.IOs;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import frc.robot.Subsystem.Intake.IntakeConstants;

public class IntakeIOreal implements IntakeIO {

    protected TalonFX intakeMotor;
    protected TalonFXConfiguration motorConfig;

    private DigitalInput leftFrontIR;
    private DigitalInput rightFrontIR;
    private DigitalInput leftRearIR;
    private DigitalInput rightRearIR;

    private StatusSignal<Current> motorCurrent;
    private StatusSignal<Angle> motorPosition;
    private StatusSignal<AngularVelocity> motorVelocity;
    private StatusSignal<Voltage> apliedVolts;

    private LoggedDouble currentLog;
    private LoggedDouble positionLog;
    private LoggedDouble velocityLog;
    private LoggedDouble apliedVoltsLog;

    public IntakeIOreal() {
        intakeMotor = new TalonFX(PortMap.Intake.intakeMotor , PortMap.CanBus.RioBus);
        motorConfig = new TalonFXConfiguration();

        leftFrontIR = new DigitalInput(PortMap.Intake.leftFrontIR);
        rightFrontIR = new DigitalInput(PortMap.Intake.rightFrontIR);
        leftRearIR = new DigitalInput(PortMap.Intake.leftRearIR);
        rightRearIR = new DigitalInput(PortMap.Intake.rightRearIR);

        motorCurrent = intakeMotor.getSupplyCurrent();
        motorPosition = intakeMotor.getPosition();
        motorVelocity = intakeMotor.getVelocity();
        apliedVolts = intakeMotor.getMotorVoltage();

        currentLog = new LoggedDouble("/Subsystem/Intake/IO/Current");
        positionLog = new LoggedDouble("/Subsystem/Intake/IO/Position");
        velocityLog = new LoggedDouble("/Subsystem/Intake/IO/Velocity");
        apliedVoltsLog = new LoggedDouble("/Subsystem/Intake/IO/aplied Volts");

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
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        motorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;

        intakeMotor.getConfigurator().apply(motorConfig);
    }

    public boolean getFrontSensor() {
        return leftFrontIR.get() || rightFrontIR.get();
    }

    public boolean getRearSensor() {
        return leftRearIR.get() || rightRearIR.get();
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
            apliedVolts
        );

        currentLog.update(getCurrent());
        positionLog.update(getPosition());
        velocityLog.update(getVelocity());
        apliedVoltsLog.update(getAppliedVolts());
    }
}
