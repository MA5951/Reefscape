package frc.robot.Subsystem.Arm;

import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.RobotControl.StatesTypes.StatesConstants;
import com.ma5951.utils.RobotControl.Subsystems.StateControlledSubsystem;
import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.Subsystem.Arm.IOs.ArmIO;

public class Arm extends StateControlledSubsystem {
    private static Arm arm;

    private LoggedBool atPointLog;

    private ArmIO armIO = ArmConstants.getArmIO();
    private Debouncer atPointDebouncer = new Debouncer(RobotConstants.kDELTA_TIME * 2);

    private Arm() {
        super(ArmConstants.SUBSYSTEM_STATES, "Arm");
        atPointLog = new LoggedBool("/Subsystems/Arm/AtPoint");

        resetPose();
    }

    public double getFeedForwardVoltage() {
        return Math.sin(ConvUtil.DegreesToRadians(getPosition())) * ArmConstants.FEED_FORWARD_VOLTAGE;
    }

    public double getAbsolutePosition() {
        return armIO.getAbsolutePosition();
    }

    public double getCurrent() {
        return armIO.getCurrent();
    }

    public double getPosition() {
        return armIO.getPosition();
    }

    public double getVelocity() {
        return armIO.getVelocity();
    }

    public double getAppliedVolts() {
        return armIO.getAppliedVolts();
    }

    public double getSetPoint() {
        return armIO.getSetPoint();
    }

    public void resetPose() {
        armIO.resetPosition(getAbsolutePosition());
    }

    public void setNeutralMode(boolean isBrake) {
        armIO.setNeutralMode(isBrake);
    }

    public void setVoltage(double volt) {
        armIO.setVoltage(volt);
    }

    public void setAngle(double angle) {
        armIO.setAngle(angle , getFeedForwardVoltage());
    }

    public boolean atPoint() {
        return atPointDebouncer.calculate(Math.abs(armIO.getError()) <= ArmConstants.TOLERANCE) ;
    }

    public boolean atMinPose() {
        return Math.abs(armIO.getError()) <= ArmConstants.TOLERANCE && getPosition() < 2;
    }

    public boolean BallRemovingCanMove() {
        return (RobotContainer.currentRobotState == RobotConstants.BALLREMOVING 
                && RobotContainer.elevator.atPoint()) || RobotContainer.currentRobotState != RobotConstants.BALLREMOVING  ;
    }

    @Override
    public boolean canMove() {
        return (BallRemovingCanMove()  && (RobotContainer.currentRobotState != RobotConstants.CLIMB
                && Math.abs(getCurrent()) < ArmConstants.kCAN_MOVE_CURRENT_LIMIT

                && getSetPoint() >= ArmConstants.MIN_ANGLE && getSetPoint() <= ArmConstants.MAX_ANGLE))
                || getSystemFunctionState() == StatesConstants.MANUEL;
    }

    public static Arm getInstance() {
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

    @Override
    public void periodic() {
        super.periodic();
        armIO.updatePeriodic();
        atPointLog.update(atPoint());
    }
}
