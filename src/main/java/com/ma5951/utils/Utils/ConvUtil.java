
package com.ma5951.utils.Utils;


public class ConvUtil {


    /////////////////////////////////////////Rotations///////////////////////////////////

    /**
   * Convert Round Per Secound to Round Per Minute.
   *
   * @param RPS to conver
   * @return RPM after convertion
   */
    public static double RPStoRPM(double RPS) {
        return RPS * 60;
    }

    /**
     * Convert Round Per Minute to Round Per Secound.
     * 
     * @param RPM to conver
     * @return RPS after convertion
     */
    public static double RPMtoRPS(double RPM) {
        return RPM / 60;
    }

    /**
   * Convert Radians to Degrees.
   *
   * @param Radians to conver
   * @return Degrees after convertion
   */
    public static double RadiansToDegrees(double Radians) {
        return Math.toDegrees(Radians);
    }

    /**
   * Convert Degrees to Rdians.
   *
   * @param Degrees to conver
   * @return Radians after convertion
   */
    public static double DegreesToRadians(double Degrees) {
        return Math.toRadians(Degrees);
    }

    /**
   * Convert Rotations To Degrees.
   *
   * @param Rotations to conver
   * @return Degrees after convertion
   */
     public static double RotationsToDegrees(double Rotations) {
        return Rotations * 360;
    }

    /**
   * Convert Degrees To Rotations.
   *
   * @param Degrees to conver
   * @return Rotations after convertion
   */
    public static double DegreesToRotations(double Degrees) {
        return Degrees / 360;
    }

    /**
   * Convert Radians To Rotations.
   *
   * @param Radians to conver
   * @return Rotations after convertion
   */
    public static double RadiansToRotations(double Radians) {
        return Radians / (2 * Math.PI);
    }

    /**
   * Convert Rotations To Radians.
   *
   * @param Rotations to conver
   * @return Radians after convertion
   */
    public static double RotationsToRadians(double Rotations) {
        return Rotations * (2 * Math.PI);
    }


    /////////////////////////////////////////Distance////////////////////////////////
    
    /**
   * Convert Rotations To Distance.
   *
   * @param Rotations to conver
   * @param Radius of the wheel
   * @return Distance after convertion
   */
    public static double RotationsToDistance(double Rotations, double Radius) {
        double circumference = 2 * Math.PI * Radius;
        return Rotations * circumference;
    }
    
    /**
   * Convert Meters To Millimeters.
   *
   * @param Meters to conver
   * @return Millimeters after convertion
   */
    public static double MetersToMillimeters(double Meters) {
        return Meters * 1000;
    }

    /**
   * Convert Meters To Centimeters.
   *
   * @param Meters to conver
   * @return Centimeters after convertion
   */
    public static double MetersToCentimeters(double Meters) {
        return Meters * 100;
    }

    /**
   * Convert Centimeters To Meters.
   *
   * @param Centimeters to conver
   * @return Meters after convertion
   */
    public static double centimetersToMeters(double Centimeters) {
        return Centimeters / 100;
    }

    /**
   * Convert Millimeters To Centimeters.
   *
   * @param Millimeters to conver
   * @return Centimeters after convertion
   */
    public static double MillimetersToCentimeters(double Millimeters) {
        return Millimeters / 10;
    }

    /**
   * Convert Millimeters To Meters.
   *
   * @param Millimeters to conver
   * @return Meters after convertion
   */
    public static double MillimetersToMeters(double Millimeters) {
        return Millimeters / 1000;
    }


    ///////////////////////////////////////////////////Time////////////////////////////////////

    /**
   * Convert Seconds To Milliseconds.
   *
   * @param Seconds to conver
   * @return Milliseconds after convertion
   */
    public static double SecondsToMilliseconds(double Seconds) {
        return Seconds * 1000;
    }

    
}
