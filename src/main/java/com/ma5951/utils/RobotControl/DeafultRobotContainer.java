package com.ma5951.utils.RobotControl;

import java.util.function.Supplier;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

import com.ma5951.utils.DashBoard.AutoOption;
import com.ma5951.utils.DashBoard.AutoSelector;
import com.ma5951.utils.Logger.LoggedBool;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedPose2d;
import com.ma5951.utils.Logger.LoggedString;
import com.ma5951.utils.RobotControl.StatesTypes.State;
import com.ma5951.utils.Utils.DriverStationUtil;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveConstants;

public class DeafultRobotContainer {

    private double IsAtStartingPoseDistance = 0.15;

    public static PS5Controller driverController;
    public static PS5Controller operatorController;
    public static XboxController driverControllerRumble;
    public static XboxController operatorControllerRumble;

    protected static AutoSelector autoSelector;
    protected static Supplier<Pose2d> robotPoseSupplier;
    private Supplier<Double> batteryVoltagSupplier;

    public static State currentRobotState = RobotConstants.IDLE;
    public static State lastRobotState = currentRobotState;

    private LoggedString currentRobotStateLog;
    private LoggedString lastRobotStateLog;
    private LoggedBool isStartingPoseLog;
    private LoggedString currentSelectedAuto;
    private LoggedPose2d startingPoseLog;
    private LoggedDouble batteryVoltageLog;
    private LoggedDouble matchTimeLog;

    public DeafultRobotContainer(int DriverControllerID, int OperatorControllerID, int DriverControllerRumbleID,
            int OperatorControllerRumbleID) {
        driverController = new PS5Controller(DriverControllerID);
        operatorController = new PS5Controller(OperatorControllerID);
        driverControllerRumble = new XboxController(DriverControllerRumbleID);
        operatorControllerRumble = new XboxController(OperatorControllerRumbleID);
        autoSelector = new AutoSelector(() -> PoseEstimator.getInstance().getEstimatedRobotPose());

        robotPoseSupplier = () -> PoseEstimator.getInstance().getEstimatedRobotPose();
        batteryVoltagSupplier = () -> RobotController.getBatteryVoltage();
        if (Robot.isReal()) {
            batteryVoltagSupplier = () -> RobotController.getBatteryVoltage();
        } else {
            batteryVoltagSupplier = () -> SimulatedBattery.getBatteryVoltage().baseUnitMagnitude();
        }
        
        DriverStation.silenceJoystickConnectionWarning(true);

        currentRobotStateLog = new LoggedString("/RobotControl/Current Robot State");
        lastRobotStateLog = new LoggedString("/RobotControl/Last Robot State");
        currentSelectedAuto = new LoggedString("/Auto/Selected Auto");
        isStartingPoseLog = new LoggedBool("/Auto/Is Starting Pose");
        startingPoseLog = new LoggedPose2d("/Auto/Starting Pose");
        batteryVoltageLog = new LoggedDouble("/Dash/Battery Vlotage");
        matchTimeLog = new LoggedDouble("/Dash/Match Time");
    }

    public void setCurrentState(State state) {
        lastRobotState = currentRobotState;
        currentRobotState = state;
    }

    // Autonomuse  ------------------------------------------------

    protected void setAutoOptions(AutoOption[] autoOptions) {
        autoSelector.setAutoOptions(autoOptions, true);
    }

    public AutoOption getSelectedAuto() {
        return autoSelector.getSelectedAuto();
    }

    public String getAutonomousName() {
        return getSelectedAuto().getName();
    }

    public boolean getIsPathPLannerAuto() {
        return getSelectedAuto().isPathPlannerAuto();
    }

    public AutoOption getCurrentSelectedAutoOption() {
        return getSelectedAuto();
    }

    public Command getSelectedAutoCommand() {
        return getSelectedAuto().getCommand();
    }

    public Command getAutonomousCommand() {
        return getSelectedAuto().getCommand();
    }

    public boolean isAtStartingPose() {
        if (DriverStationUtil.getAlliance() == Alliance.Red) {
            
            return getSelectedAuto().getStartPose().getTranslation().getDistance(robotPoseSupplier.get().getTranslation()) < IsAtStartingPoseDistance;
        }
        
        return FlippingUtil.flipFieldPose(getSelectedAuto().getStartPose()).getTranslation().getDistance(robotPoseSupplier.get().getTranslation()) < IsAtStartingPoseDistance;
    }




    





    // Updates  ------------------------------------------------

    public void updateAutoInit() {
        if(!Robot.isReal()) {
            SwerveConstants.SWERVE_DRIVE_SIMULATION.setSimulationWorldPose(getSelectedAuto().getStartPose());
            PoseEstimator.getInstance().resetPose(getSelectedAuto().getStartPose());
        }
    }

    public void updatePeriodic() {
        currentRobotStateLog.update(currentRobotState.getName());
        lastRobotStateLog.update(lastRobotState.getName());

        batteryVoltageLog.update(batteryVoltagSupplier.get());
        matchTimeLog.update(DriverStation.getMatchTime());
    }

    public void updateDisablePeriodic() {
        autoSelector.updateViz();
        currentSelectedAuto.update(getAutonomousName());
        startingPoseLog.update(getSelectedAuto().getStartPose());
        isStartingPoseLog.update(isAtStartingPose());
    }

}
