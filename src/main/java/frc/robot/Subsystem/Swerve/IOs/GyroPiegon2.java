// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve.IOs;


import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Utils.DriverStationUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Subsystem.Swerve.PhoenixOdometryThread;
import frc.robot.Subsystem.Swerve.SwerveConstants;
import frc.robot.Subsystem.Swerve.Util.Gyro;
import frc.robot.Subsystem.Swerve.Util.GyroData;


public class GyroPiegon2 implements Gyro{

    private String name;

    private Pigeon2 gyro;

    private StatusSignal<Angle> gyroYaw;
    private StatusSignal<Angle> gyroPitch;
    private StatusSignal<Angle> gyroRoll;
    private StatusSignal<AngularVelocity> gyroVeloPitch;
    private StatusSignal<AngularVelocity> gyroVeloRoll;
    private StatusSignal<AngularVelocity> gyroVeloYaw;
    private StatusSignal<LinearAcceleration> gyroAccelX;
    private StatusSignal<LinearAcceleration> gyroAccelY;

    private final Queue<Double> yawPositionQueue;
    private GyroData gyroData = new GyroData();
    private double gyroOffset = 0;

    private LoggedDouble yaw;
    private LoggedDouble pitch;
    private LoggedDouble roll;
    private LoggedDouble veloRoll;
    private LoggedDouble veloPitch;
    private LoggedDouble veloYaw;
    private LoggedDouble accelX;
    private LoggedDouble accelY;
    private LoggedDouble yawNormal;
    
    public GyroPiegon2(String type, CANBus canBus ,int id) {
        name = type;

        gyro = new Pigeon2(id, canBus);

        gyroYaw = gyro.getYaw();
        gyroPitch = gyro.getPitch();
        gyroRoll = gyro.getRoll();
        gyroAccelX = gyro.getAccelerationX();
        gyroAccelY = gyro.getAccelerationY();
        gyroVeloPitch = gyro.getAngularVelocityXDevice();
        gyroVeloYaw = gyro.getAngularVelocityZDevice();
        gyroVeloRoll = gyro.getAngularVelocityYDevice();

        yaw = new LoggedDouble("/Subsystems/Swerve/" + name + "/Yaw");
        pitch = new LoggedDouble("/Subsystems/Swerve/" + name + "/Pitch");
        roll = new LoggedDouble("/Subsystems/Swerve/" + name + "/Roll");
        accelX = new LoggedDouble("/Subsystems/Swerve/" + name + "/Accel X");
        accelY = new LoggedDouble("/Subsystems/Swerve/" + name + "/Accel Y");
        yawNormal = new LoggedDouble("/Subsystems/Swerve/" + name + "/Yaw Normal");
        veloRoll = new LoggedDouble("/Subsystems/Swerve/" + name + "/Roll Accel");
        veloPitch = new LoggedDouble("/Subsystems/Swerve/" + name + "/Pitch Accel");
        veloYaw = new LoggedDouble("/Subsystems/Swerve/" + name + "/Yaw Accel");

        gyro.getConfigurator().setYaw(0);

        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro, gyro.getYaw());
        BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.ODOMETRY_UPDATE_RATE, gyroYaw);
        
    }

    public void reset() {
        gyro.reset();
    }

    public void updateOffset() {
        gyroOffset = gyroYaw.getValueAsDouble();
    }

    public double getYaw() {
        return gyroYaw.getValueAsDouble() - gyroOffset;
    }

    public double getAbsYaw() {
        if (DriverStationUtil.getAlliance() == Alliance.Blue) {
            return getYaw() + 180;
        } else {
            return getYaw();
        }
    }

    public double getPitch() {
        return gyroPitch.getValueAsDouble();
    }

    public double getRoll() {
        return gyroRoll.getValueAsDouble();
    }

    public double getAccelX() {
        return gyroAccelX.getValueAsDouble();
    }

    public double getAccelY() {
        return gyroAccelY.getValueAsDouble();
    }

    public double getVeloYaw() {
        return gyroVeloYaw.getValueAsDouble();
    }

    public double getVeloPitch() {
        return gyroVeloPitch.getValueAsDouble();
    }

    public double getVeloRoll() {
        return gyroVeloRoll.getValueAsDouble();
    }

    public void logData(GyroData data) {
        yaw.update(data.getYaw());
        pitch.update(data.getPitch());
        roll.update(data.getRoll());
        accelX.update(data.getAccelX());
        accelY.update(data.getAccelY());
        yawNormal.update(data.getYaw() % 360);
        veloRoll.update(data.getVeloRoll());
        veloPitch.update(data.getVeloPitch());
        veloYaw.update(data.getVeloYaw());
    }

    public GyroData update(ChassisSpeeds robotSpeeds) {
        gyroData.updateData(
            getPitch(), 
            getYaw(), 
            getRoll(),
            getVeloYaw(),
            getVeloPitch(),
            getVeloRoll(), 
            getAccelX(), 
            getAccelY(), 
            getAbsYaw(), 
            yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new));

        logData(gyroData);

        yawPositionQueue.clear();
        
        return gyroData;
    }

}
