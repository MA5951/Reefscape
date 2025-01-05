// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem.Swerve;


import com.ma5951.utils.Logger.LoggedDouble;
import com.ma5951.utils.Logger.LoggedSwerveStates;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//The orde of te modules is a STANDART and it is
//Front Left
//Front Right
//Rear Left
//Raer Right
import frc.robot.Utils.ModuleLimits;
import frc.robot.Utils.SwerveSetpoint;
import frc.robot.Subsystem.Swerve.Util.SwerveModule;
import frc.robot.Subsystem.Swerve.Util.SwerveModuleData;
import frc.robot.Subsystem.Swerve.Util.SwerveOdometry;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Swerve.Util.Gyro;
import frc.robot.Subsystem.Swerve.Util.GyroData;

public class SwerveSubsystem extends SubsystemBase {
  private static SwerveSubsystem swerveSubsystem;

  private SwerveSetpointGenerator setpointGenerator;
  private LoggedSwerveStates currenStatesLog;
  private LoggedSwerveStates setPoinStatesLog;
  private LoggedDouble swerevXvelocityLog;
  private LoggedDouble swerevYvelocityLog;
  private LoggedDouble swerevTheatavelocityLog;
  

  private final SwerveModule[] modulesArry = SwerveConstants.getModulesArry();
  private final Gyro gyro = SwerveConstants.getGyro();
  private static SwerveOdometry odometry;
  private final SwerveDriveKinematics kinematics = SwerveConstants.kinematics;
  private SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
  private SwerveModuleState[] currentStates = new SwerveModuleState[4];
  private SwerveModulePosition[] currentPositions = new  SwerveModulePosition[4];
  private SwerveModuleData[] modulesData = new SwerveModuleData[4];
  private GyroData gyroData = new GyroData();
  private ModuleLimits currentLimits = SwerveConstants.DEFUALT;
  private ChassisSpeeds currentChassisSpeeds;
  private SwerveModuleState[] states;
  private SwerveModuleState[] Optistates = new SwerveModuleState[4];

  private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
    new ChassisSpeeds(),
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
  });

  public SwerveSubsystem() {

    setpointGenerator = new SwerveSetpointGenerator(kinematics , new Translation2d[] {
      SwerveConstants.frontLeftLocation,
      SwerveConstants.frontRightLocation,
      SwerveConstants.rearLeftLocation,
      SwerveConstants.rearRightLocation
    });


    currenStatesLog = new LoggedSwerveStates("/Subsystems/Swerve/States/Current States");
    setPoinStatesLog = new LoggedSwerveStates("/Subsystems/Swerve/States/SetPoint States");
    swerevXvelocityLog = new LoggedDouble("/Subsystems/Swerve/Chassis Speed/X Velocity");
    swerevYvelocityLog = new LoggedDouble("/Subsystems/Swerve/Chassis Speed/Y Velocity");
    swerevTheatavelocityLog = new LoggedDouble("/Subsystems/Swerve/Chassis Speed/Theat Velocity");

    for (int i = 0; i < 4 ; i++) {
      modulesArry[i].setNeutralModeDrive(true);
      modulesArry[i].setNeutralModeTurn(true);
    }

    

    gyro.reset();

  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    for (int i = 0; i<4 ; i++) {
      currentPositions[i] = modulesArry[i].getPosition(modulesData[i]);
    }

    return currentPositions;
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    for (int i = 0; i<4 ; i++) {
      currentStates[i] = modulesArry[i].getState(modulesData[i]);
    }

    return currentStates;
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getFusedHeading() {
    return gyroData.getYaw();
  }

  public double getRoll() {
    return gyroData.getRoll();
  }

  public double getPitch() {
    return gyroData.getPitch();
  }

  public double getAbsYaw() {
    return gyroData.getAbsoluteYaw();
  }

  public double getVelocityVector(){ 
    ChassisSpeeds speeds = getRobotRelativeSpeeds();
    return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) +
      Math.pow(speeds.vyMetersPerSecond, 2));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public Rotation2d getRotation2d() {
    return new Rotation2d(Math.toRadians(getFusedHeading()));
  }

  public SwerveModuleState[] generateStates(ChassisSpeeds chassiSpeeds , boolean optimize) {

    if (optimize) {
      currentSetpoint =
      setpointGenerator.generateSetpoint(
       getCurrentLimits(), currentSetpoint, chassiSpeeds, RobotConstants.kDELTA_TIME);

    for (int i = 0; i < modulesArry.length; i++) {
      optimizedSetpointStates[i] = currentSetpoint.moduleStates()[i];

    }

    return optimizedSetpointStates;
    } else {
      return kinematics
        .toSwerveModuleStates(chassiSpeeds);
    }
  }

  public void setModules(SwerveModuleState[] states) {
    modulesArry[0].setDesiredState(states[0]);
    modulesArry[1].setDesiredState(states[1]);
    modulesArry[2].setDesiredState(states[2]);
    modulesArry[3].setDesiredState(states[3]);
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    states = generateStates(chassisSpeeds, SwerveConstants.optimize);

    Optistates[0] = states[1];
    Optistates[1] = states[3];
    Optistates[2] = states[0];
    Optistates[3] = states[2];
    setPoinStatesLog.update(Optistates);
    setModules(Optistates);
    setPoinStatesLog.update(Optistates);
    setModules(Optistates);
  }

  public void drive(ChassisSpeeds chassisSpeeds , DriveFeedforwards feedforwards) {
    drive(chassisSpeeds);
  }

  public ModuleLimits getCurrentLimits() {
    return currentLimits;
  }

  public void setCurrentLimits(ModuleLimits newLimits) {
    currentLimits = newLimits;
  }

  public SwerveModule[] getModulesArry() {
    return modulesArry;
  }

  public Gyro getGyro() {
    return gyro;
  }

  public GyroData getGyroData() {
    return gyroData;
  }

  public SwerveModuleData[] getModulesData() {
    return modulesData;
  }

  public void updateHardwereData() {
    for (int i = 0; i < 4 ; i++) {
      modulesData[i] = modulesArry[i].update();
    }
    currentChassisSpeeds = kinematics.toChassisSpeeds(getSwerveModuleStates());
    gyroData = gyro.update(currentChassisSpeeds);
  }

  public static SwerveSubsystem getInstance() {
    if (swerveSubsystem == null) {
      swerveSubsystem = new SwerveSubsystem();
      odometry = SwerveConstants.getOdometry();
    }
    return swerveSubsystem;
    }

  @Override
  public void periodic() {
    odometry.updateOdometry();

    currenStatesLog.update(currentStates);
    
    
    swerevXvelocityLog.update(currentChassisSpeeds.vxMetersPerSecond);
    swerevYvelocityLog.update(currentChassisSpeeds.vyMetersPerSecond);
    swerevTheatavelocityLog.update(currentChassisSpeeds.omegaRadiansPerSecond);
   

    
  }
}