
package frc.robot;


import com.ctre.phoenix6.CANBus;

public class PortMap {

    public static class CanBus {
        public static final CANBus CANivoreBus= new CANBus("Swerve");
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

    public class Climb {
        public static final int ClimbMasterMotor = 50;
        public static final int ClimbSlaveMotor = 52;

        public static final int ClimbFirstSensor = 1;
        public static final int ClimbSecondSensor = 2;
    }

    public class Vision {
        public final static String CAMERA_NAME = "limelight-camera";
    }

}
