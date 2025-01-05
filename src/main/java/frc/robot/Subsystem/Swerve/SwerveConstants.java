package frc.robot.Subsystem.Swerve;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.PortMap;
import frc.robot.Robot;
import frc.robot.Utils.ModuleLimits;
import frc.robot.Utils.PPHolonomicDriveController;
import frc.robot.Subsystem.Swerve.IOs.GyroPiegon2;
import frc.robot.Subsystem.Swerve.IOs.GyroSim;
import frc.robot.Subsystem.Swerve.IOs.SwerveModuleSim;
import frc.robot.Subsystem.Swerve.IOs.SwerveModuleTalonFX;
import frc.robot.Subsystem.Swerve.IOs.SwerveThreadOdometry;
import frc.robot.Subsystem.Swerve.Util.Gyro;
import frc.robot.Subsystem.Swerve.Util.OdometryConfig;
import frc.robot.Subsystem.Swerve.Util.SwerveModule;
import frc.robot.Subsystem.Swerve.Util.SwerveOdometry;

public class SwerveConstants {
        public final static boolean optimize = true;

        // swerve constants
        public final static double WIDTH = 0.58;
        public final static double LENGTH = 0.59;
        public final static double RADIUS = Math.sqrt(
                        Math.pow(WIDTH, 2) + Math.pow(LENGTH, 2)) / 2.0;
        public final static double BUMPER_WIDTH = WIDTH + 0.16;
        public final static double BUMPER_LENGTH = LENGTH + 0.16;
        public final static double ROBOT_MASS = 54;

        // Modules constants
        public final static double TURNING_GEAR_RATIO = 150d / 7;
        public final static double DRIVE_GEAR_RATIO = 6.12;
        public final static double WHEEL_RADIUS = 0.0508;
        public final static double WHEEL_CIRCUMFERENCE = 2 * WHEEL_RADIUS * Math.PI;

        public final static double VELOCITY_TIME_UNIT_IN_SECONDS = 1;

        // front left module
        public final static boolean FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = true;
        public final static boolean FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean FRONT_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // front right module
        public final static boolean FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = false;
        public final static boolean FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean FRONT_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // rear left module
        public final static boolean REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = true;
        public final static boolean REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean REAR_LEFT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // rear right module
        public final static boolean REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED = false;
        public final static boolean REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED = true;
        public final static boolean REAR_RIGHT_MODULE_IS_ABSOLUTE_ENCODER_REVERSED = false;

        // Module locations
        public static final Translation2d frontLeftLocation = new Translation2d(
                        -WIDTH / 2,
                        LENGTH / 2);

        public static final Translation2d frontRightLocation = new Translation2d(
                        WIDTH / 2,
                        LENGTH / 2);

        public static final Translation2d rearLeftLocation = new Translation2d(
                        -WIDTH / 2,
                        -LENGTH / 2);

        public static final Translation2d rearRightLocation = new Translation2d(
                        WIDTH / 2,
                        -LENGTH / 2);

        public final static Translation2d[] modulesLocationArry = new Translation2d[] { frontLeftLocation,
                        frontRightLocation, rearLeftLocation, rearRightLocation };

        // IOs
        public static final SwerveModule[] getModulesArry() {
                if (Robot.isReal()) {
                        final SwerveModule frontLeftModule = new SwerveModuleTalonFX(
                                        "Front Left",
                                        PortMap.Swerve.leftFrontDriveID,
                                        PortMap.Swerve.leftFrontTurningID,
                                        PortMap.Swerve.leftFrontAbsoluteEncoder,
                                        SwerveConstants.FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                        SwerveConstants.FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED);

                        final SwerveModule frontRightModule = new SwerveModuleTalonFX(
                                        "Front Right",
                                        PortMap.Swerve.rightFrontDriveID,
                                        PortMap.Swerve.rightFrontTurningID,
                                        PortMap.Swerve.rightFrontAbsoluteEncoder,
                                        SwerveConstants.FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                        SwerveConstants.FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED);

                        final SwerveModule rearLeftModule = new SwerveModuleTalonFX(
                                        "Rear Left",
                                        PortMap.Swerve.leftBackDriveID,
                                        PortMap.Swerve.leftBackTurningID,
                                        PortMap.Swerve.leftBackAbsoluteEncoder,
                                        SwerveConstants.REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                        SwerveConstants.REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED);

                        final SwerveModule rearRightModule = new SwerveModuleTalonFX(
                                        "Rear Right",
                                        PortMap.Swerve.rightBackDriveID,
                                        PortMap.Swerve.rightBackTurningID,
                                        PortMap.Swerve.rightBackAbsoluteEncoder,
                                        SwerveConstants.REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                        SwerveConstants.REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED);
                        return new SwerveModule[] {
                                        frontLeftModule, frontRightModule, rearLeftModule, rearRightModule };
                }

                final SwerveModule frontLeftModule = new SwerveModuleSim("Front Left",
                                PortMap.Swerve.leftFrontDriveID,
                                PortMap.Swerve.leftFrontTurningID,
                                PortMap.Swerve.leftFrontAbsoluteEncoder,
                                SwerveConstants.FRONT_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.FRONT_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                SWERVE_DRIVE_SIMULATION.getModules()[0]);
                final SwerveModule frontRightModule = new SwerveModuleSim("Front Right",
                                PortMap.Swerve.rightFrontDriveID,
                                PortMap.Swerve.rightFrontTurningID,
                                PortMap.Swerve.rightFrontAbsoluteEncoder,
                                SwerveConstants.FRONT_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.FRONT_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                SWERVE_DRIVE_SIMULATION.getModules()[1]);
                final SwerveModule rearLeftModule = new SwerveModuleSim("Raer Left",
                                PortMap.Swerve.leftBackDriveID,
                                PortMap.Swerve.leftBackTurningID,
                                PortMap.Swerve.leftBackAbsoluteEncoder,
                                SwerveConstants.REAR_LEFT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.REAR_LEFT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                SWERVE_DRIVE_SIMULATION.getModules()[2]);
                final SwerveModule rearRightModule = new SwerveModuleSim("Rear Right",
                                PortMap.Swerve.rightBackDriveID,
                                PortMap.Swerve.rightBackTurningID,
                                PortMap.Swerve.rightBackAbsoluteEncoder,
                                SwerveConstants.REAR_RIGHT_MOUDLE_IS_DRIVE_MOTOR_REVERSED,
                                SwerveConstants.REAR_RIGHT_MODULES_IS_TURNING_MOTOR_REVERSED,
                                SWERVE_DRIVE_SIMULATION.getModules()[3]);

                return new SwerveModule[] {
                                frontLeftModule, frontRightModule, rearLeftModule, rearRightModule };

        }

        public static final Gyro getGyro() {
                if (Robot.isReal()) {
                        return new GyroPiegon2("Piegon 2", PortMap.CanBus.CANivoreBus, PortMap.Swerve.Pigeon2ID);
                }

                return new GyroSim("Gyro Sim");
        }

        public static final SwerveOdometry getOdometry() {
                return new SwerveThreadOdometry(ODOMETRY_CONFIG);
        }

        public static final PPHolonomicDriveController getHolonomicController() {
                if (Robot.isReal()) {
                        return new PPHolonomicDriveController(
                                        new PIDConstants(0.09, 0, 0),//TODO
                                        new PIDConstants(0.4, 0, 0),//TODO
                                        new PIDConstants(0.25, 0, 0));//TODO
                }

                return new PPHolonomicDriveController(
                                        new PIDConstants(0.09, 0, 0),
                                        new PIDConstants(0.4, 0, 0),
                                        new PIDConstants(0.25, 0, 0));
        }

        // Modules config
        public final static int SLOT_CONFIG = 0;

        // MotionMagic PID
        public final static double TURNING_kP = 90;
        public final static double TURNING_kI = 0;
        public final static double TURNING_kD = 0;
        public final static double TURNING_kS = 0;
        public final static double TURNING_kV = 0;
        public final static double TURNING_kA = 0;
        public final static double TURNING_CTUISE_VELOCITY = 224 * (1 / 0.02); // RPS
        public final static double TURNING_ACCELERATION = 150;
        public final static double TURNING_JERK = 0;

        // Ramp
        public final static double OPEN_LOOP_RAMP = 0.25;
        public final static double CLOSED_LOOP_RAMP = 0;
        // Current Limit
        public final static int TURNING_CURRENT_LIMIT = 50;
        public final static int TURNING_CONTINUOUS_LOWER_LIMIT = 25;
        public final static double TURNING_CONTINUOUS_CURRENT_DURATION = 0.1;
        public final static boolean TURNING_ENABLE_CURRENT_LIMIT = true;

        // Modules drive config
        // PID
        public final static double DRIVE_kP = 0;
        public final static double DRIVE_kI = 0;
        public final static double DRIVE_kD = 0;
        public final static double DRIVE_kS = 0;
        public final static double DRIVE_kV = 0.800;// 857
        public final static double DRIVE_kA = 0;

        // Current Limit
        public final static int DRIVE_CURRENT_LIMIT = 80;
        public final static int DRIVE_CONTINUOS_LOWER__LIMIT = 35;
        public final static double DRIVE_CONTINUOUS_CURRENT_DURATION = 0.15;
        public final static boolean DRIVE_ENBLE_CURRENT_LIMIT = true;

        // Swerve physics
        public final static double MAX_VELOCITY = 5.2;
        public final static double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / RADIUS;// Radians
        public final static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        SwerveConstants.frontLeftLocation, SwerveConstants.frontRightLocation,
                        SwerveConstants.rearLeftLocation, SwerveConstants.rearRightLocation);

        public final static DriveTrainSimulationConfig DRIVE_TRAIN_SIMULATION_CONFIG = DriveTrainSimulationConfig
                        .Default()
                        .withGyro(COTS.ofPigeon2())
                        .withRobotMass(edu.wpi.first.units.Units.Kilogram.of(ROBOT_MASS))
                        .withSwerveModule(COTS.ofMark4i(
                                        DCMotor.getKrakenX60(1),
                                        DCMotor.getFalcon500(1),
                                        COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                                        3 // Gear ratio (l3 gear ratio)
                        ))
                        .withTrackLengthTrackWidth(edu.wpi.first.units.Units.Meter.of(LENGTH),
                                        edu.wpi.first.units.Units.Meter.of(WIDTH))
                        .withBumperSize(edu.wpi.first.units.Units.Meter.of(BUMPER_WIDTH),
                                        edu.wpi.first.units.Units.Meter.of(BUMPER_LENGTH));

        public final static SwerveDriveSimulation SWERVE_DRIVE_SIMULATION = new SwerveDriveSimulation(
                        DRIVE_TRAIN_SIMULATION_CONFIG, new Pose2d(2, 2, new Rotation2d()));

        // Module Limits
        public final static ModuleLimits DEFUALT = new ModuleLimits(5.2, Units.feetToMeters(65),
                        Units.degreesToRadians(700));

        // Odometry
        public final static double ODOMETRY_UPDATE_RATE = 250;
        public final static OdometryConfig ODOMETRY_CONFIG = new OdometryConfig(
                        true, true, 0.15, 2d, false, 85d);

        // Swerve controllers

        // Swerve AngleAdjust
        public final static double THATA_KP = 0.45;
        public final static double THATA_KI = 0;
        public final static double THATA_KD = 0.0;
        public final static double ANGLE_PID_TOLORANCE = Math.toRadians(3);

        // Swerve AutoAdjustXY
        public final static double XY_KP = 0.45;
        public final static double XY_KI = 0;
        public final static double XY_KD = 0.0;
        public final static Constraints XY_CONSTRAINTS = new Constraints(3.5, 3);
        public final static double XY_TOLORANCE = 0.05;

        // Swerve RelativAngleAdjust
        public final static double RELATIV_THATA_KP = 0.00615;
        public final static double RELATIV_THATA_KI = 0;
        public final static double RELATIV_THATA_KD = 0.0016;
        public final static double RELATIV_ANGLE_PID_TOLORANCE = 3;

        // Swerve DRIVE CONTROLLER
        public final static double DRIVER_XY_SCALER = 1;
        public final static double DRIVER_THATA_SCALER = 0.7;

}