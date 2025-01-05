// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve.Util;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class SwerveModuleData {

    private double drivePosition;
    private double driveVelocity;
    private double driveCurrent;
    private double driveVolts;
    private double steerPosition;
    private double steerCurrent;
    private double steerVolts;
    private double absAngle;
    private double steerVelocity; 
    private double[] drivePositionQueue;
    private Rotation2d[] turnPositionQueue;

    public SwerveModuleData () {
        drivePosition = 0;
        driveVelocity = 0;
        driveCurrent = 0;
        driveVolts = 0;
        steerPosition = 0;
        steerCurrent = 0;
        steerVolts = 0;
        absAngle = 0;
        steerVelocity = 0; 
        drivePositionQueue = new double[0];
        turnPositionQueue = new Rotation2d[0];
    }

    public SwerveModuleData (
        double DrivePosition,
        double DriveVelocity,
        double DriveCurrent,
        double DriveVolts,
        double SteerPosition,
        double SteerCurrent,
        double SteerVolts,
        double AbsoluteAngle,
        double SteerVelocity,
        double[] DrivePositionQueue,
        Rotation2d[] TurnPositionQueue
    ) {
        drivePosition = DrivePosition;
        driveVelocity = DriveVelocity;
        driveCurrent = DriveCurrent;
        driveVolts = DriveVolts;
        steerPosition = SteerPosition;
        steerCurrent = SteerCurrent;
        steerVolts = SteerVolts;
        absAngle = AbsoluteAngle;
        steerVelocity = SteerVelocity; 
        drivePositionQueue = DrivePositionQueue;
        turnPositionQueue = TurnPositionQueue;
    }

    public void updateData(
        double DrivePosition,
        double DriveVelocity,
        double DriveCurrent,
        double DriveVolts,
        double SteerPosition,
        double SteerCurrent,
        double SteerVolts,
        double AbsoluteAngle,
        double SteerVelocity,
        double[] DrivePositionQueue,
        Rotation2d[] TurnPositionQueue
    ) {
        drivePosition = DrivePosition;
        driveVelocity = DriveVelocity;
        driveCurrent = DriveCurrent;
        driveVolts = DriveVolts;
        steerPosition = SteerPosition;
        steerCurrent = SteerCurrent;
        steerVolts = SteerVolts;
        absAngle = AbsoluteAngle;
        steerVelocity = SteerVelocity; 
        drivePositionQueue = DrivePositionQueue;
        turnPositionQueue = TurnPositionQueue;
    }

    public double getDrivePosition() {
        return drivePosition;
    }

    public double getDriveVelocity() {
        return driveVelocity;
    }

    public double getDriveVolts() {
        return driveVolts;
    }

    public double getDriveCurrent() {
        return driveCurrent;
    }

   

    public double getSteerPosition() {
        return steerPosition;
    }

    public double getSteerCurrent() {
        return steerCurrent;
    }

    public double getSteerVolts() {
        return steerVolts;
    }

    public double getSteerVelocity() {
        return steerVelocity;
    }

    public double getAbsoluteAngle() {
        return absAngle;
    }

    public double[] getDrivePositionQueue() {
        return drivePositionQueue;
    }

    public Rotation2d[] getSteerPositionQueue() {
        return turnPositionQueue;
    }

}
