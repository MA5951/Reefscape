// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve;

import java.util.function.Supplier;

import com.ma5951.utils.Utils.VectorUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class SkidDetector {
    private SwerveDriveKinematics kinematics;
    private Supplier<SwerveModuleState[]> statesSupplier;
    private ChassisSpeeds measurSpeeds;
    private SwerveModuleState[] swerveStatesRotationalPart;
    private double[] swerveStatesTranslationalPartMagnitudes = new double[4];
    private Translation2d swerveStateMeasuredAsVector;
    private Translation2d swerveStatesRotationalPartAsVector;
    private Translation2d swerveStatesTranslationalPartAsVector;
    private double maximumTranslationalSpeed = 0;
    private double minimumTranslationalSpeed = 0;

    public SkidDetector(SwerveDriveKinematics kinematics,Supplier<SwerveModuleState[]> states) {
        this.kinematics = kinematics;
        statesSupplier = states;


    }

    public double getSkiddingRatio() {
        measurSpeeds = kinematics.toChassisSpeeds(statesSupplier.get());
        swerveStatesRotationalPart = kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, measurSpeeds.omegaRadiansPerSecond));
        
        for (int i = 0; i < statesSupplier.get().length; i++) {
            swerveStateMeasuredAsVector = VectorUtil.getVectorFromSwerveState(statesSupplier.get()[i]);
            swerveStatesRotationalPartAsVector = VectorUtil.getVectorFromSwerveState(swerveStatesRotationalPart[i]);
            swerveStatesTranslationalPartAsVector = swerveStateMeasuredAsVector.minus(swerveStatesRotationalPartAsVector);
            swerveStatesTranslationalPartMagnitudes[i] = swerveStatesTranslationalPartAsVector.getNorm();
        }

        
        for (double translationalSpeed : swerveStatesTranslationalPartMagnitudes) {
            maximumTranslationalSpeed = Math.max(maximumTranslationalSpeed, translationalSpeed);
            minimumTranslationalSpeed = Math.min(minimumTranslationalSpeed, translationalSpeed);
        }

        return maximumTranslationalSpeed / minimumTranslationalSpeed;
    }

}
