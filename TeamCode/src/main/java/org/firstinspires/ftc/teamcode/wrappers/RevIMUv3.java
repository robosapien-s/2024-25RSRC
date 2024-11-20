package org.firstinspires.ftc.teamcode.wrappers;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.interfaces.IImu;

public class RevIMUv3 extends GyroEx implements IImu {

    private IMU revIMU;

    /***
     * Heading relative to starting position
     */
    double globalHeading;

    /**
     * Heading relative to last offset
     */
    double relativeHeading;

    /**
     * Offset between global heading and relative heading
     */
    double offset;

    private int multiplier;

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub
     *
     * @param hw      Hardware map
     * @param imuName Name of sensor in configuration
     */
    public RevIMUv3(HardwareMap hw, String imuName) {
        revIMU = hw.get(IMU.class, imuName);
        multiplier = 1;
    }

    /**
     * Create a new object for the built-in gyro/imu in the Rev Expansion Hub with the default configuration name of "imu"
     *
     * @param hw Hardware map
     */
    public RevIMUv3(HardwareMap hw) {
        this(hw, "imu");
    }

    /**
     * Initializes gyro with default parameters.
     */
    public void init() {

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        init(new IMU.Parameters(orientationOnRobot));
    }


    public void init(RevHubOrientationOnRobot.LogoFacingDirection logoDirection, RevHubOrientationOnRobot.UsbFacingDirection usbDirection) {
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        init(new IMU.Parameters(orientationOnRobot));
    }

    /**
     * Initializes gyro with custom parameters.
     */
    public void init(IMU.Parameters parameters) {
        boolean didInit = revIMU.initialize(parameters);

        globalHeading = 0;
        relativeHeading = 0;
        offset = 0;
    }

    /**
     * Inverts the ouptut of gyro
     */
    public void invertGyro() {
        multiplier *= -1;
    }

    /**
     * @return Relative heading of the robot
     */
    public double getHeading() {
        // Return yaw
        return getAbsoluteHeading() - offset;
    }

    /**
     * @return Absolute heading of the robot
     */
    @Override
    public double getAbsoluteHeading() {

        YawPitchRollAngles orientation = revIMU.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = revIMU.getRobotAngularVelocity(AngleUnit.DEGREES);
        return orientation.getYaw() * multiplier;
    }

    @Override
    public double[] getAngles() {
        return new double[0];
    }

    /**
     * @return X, Y, Z angles of gyro
     */
//    public double[] getAngles() {
//        // make a singular hardware call
//        Orientation orientation = revIMU.getAngularOrientation();
//
//        return new double[]{orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle};
//    }

    /**
     * @return Transforms heading into {@link Rotation2d}
     */
    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void disable() {
        revIMU.close();
    }

    @Override
    public void reset() {
        offset += getHeading();
    }

    @Override
    public String getDeviceType() {
        return "Rev Expansion Hub IMU";
    }

    /**
     * @return the internal sensor being wrapped
     */
    public IMU getRevIMU() {
        return revIMU;
    }

}