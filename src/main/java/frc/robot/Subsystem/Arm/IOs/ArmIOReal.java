package frc.robot.Subsystem.Arm.IOs;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.PortMap;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Arm.ArmConstants;

public class ArmIOReal implements ArmIO {

    protected TalonFX armMotor;
    protected TalonFXConfiguration armConfig;
    private PositionVoltage positionControl;

    protected CANcoder absEncoder;

    private StatusSignal<Angle> motorPosition;
    private StatusSignal<AngularVelocity> motorVelocity;
    private StatusSignal<Current> motorCurrent;
    private StatusSignal<Voltage> motorVoltage;
    private StatusSignal<Double> error;
    private StatusSignal<Double> setPoint;
    private StatusSignal<Angle> encoderPosition;

    private LoggedDouble motorPositionLog;
    private LoggedDouble motorVelocityLog;
    private LoggedDouble motorCurrentLog;
    private LoggedDouble motorVoltageLog;
    private LoggedDouble absPositionLog;
    private LoggedDouble errorLog;
    private LoggedDouble setPointLog;

    public ArmIOReal() {
        armMotor = new TalonFX(PortMap.Arm.armMotor, PortMap.CanBus.RioBus);
        armConfig = new TalonFXConfiguration();
        absEncoder = new CANcoder(PortMap.Arm.armEncoder, PortMap.CanBus.RioBus);
        positionControl = new PositionVoltage(0); 


        motorPosition = armMotor.getPosition();
        motorVelocity = armMotor.getVelocity();
        motorCurrent = armMotor.getStatorCurrent();
        motorVoltage = armMotor.getMotorVoltage();
        error = armMotor.getClosedLoopError();
        setPoint = armMotor.getClosedLoopReference();
        encoderPosition = absEncoder.getAbsolutePosition();

        motorPositionLog = new LoggedDouble("/Subsystems/Arm/IO/Motor Position");
        motorVelocityLog = new LoggedDouble("/Subsystems/Arm/IO/Motor Velocity");
        motorCurrentLog = new LoggedDouble("/Subsystems/Arm/IO/Motor Current");
        motorVoltageLog = new LoggedDouble("Subsystems/Arm/IO/Motor Voltage");
        errorLog = new LoggedDouble("/Subsystems/Arm/IO/Error");
        setPointLog = new LoggedDouble("/Subsystems/Arm/IO/Set Point");
        absPositionLog = new LoggedDouble("Subsystems/Arm/IO/Abs Position");

        encoderPosition.refresh();
        armMotor.setPosition(0);

        configMotor();
    }

    private void configMotor() {
        armConfig.Feedback.SensorToMechanismRatio = ArmConstants.GEAR_RATIO;

        armConfig.Voltage.PeakForwardVoltage = RobotConstants.NOMINAL_VOLTAGE; 
        armConfig.Voltage.PeakReverseVoltage = -RobotConstants.NOMINAL_VOLTAGE; 

        armConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;


        armConfig.Slot0.kP = ArmConstants.kP;
        armConfig.Slot0.kI = ArmConstants.kI;
        armConfig.Slot0.kD = ArmConstants.kD;

        armConfig.CurrentLimits.SupplyCurrentLimitEnable = ArmConstants.ENABLE_CURRENT_LIMIT;
        armConfig.CurrentLimits.SupplyCurrentLimit = ArmConstants.PEAK_CURRENT_LIMIT;
        armConfig.CurrentLimits.SupplyCurrentLowerLimit = ArmConstants.CONTINUOUS_CURRENT_LIMIT;
        armConfig.CurrentLimits.SupplyCurrentLowerTime = ArmConstants.CONTINUOUS_CURRENT_DURATION;

        armMotor.getConfigurator().apply(armConfig);
    }

    public double getAbsolutePosition() {
            return ConvUtil.RotationsToDegrees(encoderPosition.getValueAsDouble() / 3) + ArmConstants.ABS_ENCODER_OFFSET; 
       
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
        return motorVoltage.getValueAsDouble();
    }

    public double getError() {
        return ConvUtil.RotationsToDegrees(error.getValueAsDouble());
    }

    public double getSetPoint() {
        return ConvUtil.RotationsToDegrees(setPoint.getValueAsDouble());
    }

    public void resetPosition(double newAngle) {
        armMotor.setPosition(ConvUtil.DegreesToRotations(newAngle));
    }

    public void updatePID(double Kp, double Ki, double Kd) {
        armConfig.Slot0.kP = Kp;
        armConfig.Slot0.kI = Ki;
        armConfig.Slot0.kD = Kd;
        armMotor.getConfigurator().apply(armConfig);
    }

    public void setNeutralMode(boolean isBrake) {
        armConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        armMotor.getConfigurator().apply(armConfig);
    }

    public void setVoltage(double volt) {
        armMotor.setVoltage(volt);
    }

    public void setAngle(double angle , double FEED_FORWARD) {
        armMotor.setControl(
                positionControl.withPosition(ConvUtil.DegreesToRotations(angle)).withSlot(ArmConstants.CONTROL_SLOT)
                .withFeedForward(FEED_FORWARD));
    }

    public void updatePeriodic() {
        BaseStatusSignal.refreshAll(
                motorPosition,
                motorVelocity,
                motorCurrent,
                motorVoltage,
                error,
                setPoint,
                encoderPosition);

        motorPositionLog.update(getPosition());
        motorVelocityLog.update(getVelocity());
        motorCurrentLog.update(getCurrent());
        motorVoltageLog.update(getAppliedVolts());
        errorLog.update(getError());
        setPointLog.update(getSetPoint());
        absPositionLog.update(getAbsolutePosition());
    }
}
