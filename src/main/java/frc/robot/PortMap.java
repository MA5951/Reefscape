
package frc.robot;


import java.util.Calendar;

import com.ctre.phoenix6.CANBus;

public class PortMap {

    public static class CanBus {
        public static final CANBus CANivoreBus= new CANBus("*");
        public static final CANBus RioBus = new CANBus("rio");
    }
    
    public static class Controllers {
        public static final int driveID = 0;
        public static final int operatorID = 1;
        public static final int driveRumbleID = 2;
        public static final int operatorRumbleID = 3;  
    }
    

    public class Swerve {
        public static final CANBus SwervBus = CanBus.CANivoreBus;

        public static final int leftFrontAbsoluteEncoder = 22;
        public static final int leftFrontDriveID = 8;
        public static final int leftFrontTurningID = 5;

        public static final int leftBackAbsoluteEncoder = 21;
        public static final int leftBackDriveID = 4;
        public static final int leftBackTurningID = 9;

        public static final int rightFrontAbsoluteEncoder = 23;
        public static final int rightFrontDriveID = 7;
        public static final int rightFrontTurningID = 6;

        public static final int rightBackAbsoluteEncoder = 24;
        public static final int rightBackDriveID = 2;
        public static final int rightBackTurningID = 3;

        public static final int Pigeon2ID = 12;
    }

    public class Elevator {
        public static final int elevatorMasterMotor = 18;
        public static final int elevatorSlaveMotor = 17;
        public static final int elevatorLimitSwich = 6;
    }

    public class Intake {
        public static final int intakeMotor = 16;
        public static final int frontSensorSim = 4;
        public static final int rearSensorSim = 5;

    }

    public class Vision {
        public final static String FRONT_CAMERA_NAME = "limelight-front";
        public final static String BACK_CAMERA_NAME = "limelight-camera-BACK";
    }

    public class Arm {
        public static final int armMotor = 15;
        public static final int armEncoder = 19;
    }

    public class Led {
        public static final int ledPort = 0;
    }
}

