// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve.IOs;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.Util.SwerveModuleData;
import frc.robot.Utils.PhoenixUtil;

public class SwerveModuleSim extends SwerveModuleTalonFX {




    private SwerveModuleData moduleData = new SwerveModuleData();
    private final SwerveModuleSimulation moduleSimulation;

    public SwerveModuleSim(String moduleNameN , int driveID, int turningID, int absoluteEncoderID, boolean isDriveMotorReversed, boolean isTurningMotorReversed,SwerveModuleSimulation simulation) {
        super(moduleNameN, driveID, turningID, absoluteEncoderID, isDriveMotorReversed, isTurningMotorReversed);
        
        
        moduleSimulation = simulation;
        moduleSimulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveMotor ,isDriveMotorReversed ));
        moduleSimulation.useSteerMotorController(new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turningMotor, isTurningMotorReversed, absoluteEcoder, false, edu.wpi.first.units.Units.Degrees.of(0)));


        
    }

    public SwerveModuleData update() {
        refreshAll();


        moduleData.updateData(
                getDrivePosition(),
                getDriveVelocity(),
                getDriveCurrent(),
                getDriveVolts(),
                getSteerPosition(),
                getSteerCurrent(),
                getSteerVolts(),
                getAbsolutePosition(),
                getSteerVelocity(),
                Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
                        .mapToDouble(angle -> angle.in(Rotation) * SwerveConstants.WHEEL_CIRCUMFERENCE)
                        .toArray(),
                moduleSimulation.getCachedSteerAbsolutePositions());

        DrivePosition.update(getDrivePosition());
        DriveVelocity.update(getDriveVelocity());
        DriveCurrent.update(getDriveCurrent());
        DriveVolts.update(getDriveVolts());
        SteerPosition.update(getSteerPosition());
        SteerCurrent.update(getSteerCurrent());
        SteerVolts.update(getSteerVolts());
        AbsAngle.update(getAbsolutePosition());

        return moduleData;

    }

}