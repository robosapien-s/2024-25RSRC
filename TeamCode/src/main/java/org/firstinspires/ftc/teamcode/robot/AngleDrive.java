package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@Disabled
public class AngleDrive implements IDrive {

    double normalizedHeadingError = 0;

    IMU imu;

    boolean cosineThing = false;

    HardwareMap hardwareMap;

    double pitch;
    double roll;
    double yaw;

    double PGain = .03;

    double targetHeading;

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    boolean isAutoMode = false;
    double autoModeX = 0;
    double autoModeY = 0;

    double rotateAngleOffset = 180;


    public AngleDrive(HardwareMap hardwareMap) {
        InitializeResetImu(hardwareMap);
    }

    public void Initialize(HardwareMap hardwareMap) {

        //0
        frontLeftMotor = hardwareMap.dcMotor.get("fL");
        //1
        frontRightMotor = hardwareMap.dcMotor.get("fR");
        //2
        backLeftMotor = hardwareMap.dcMotor.get("bL");
        //3
        backRightMotor = hardwareMap.dcMotor.get("bR");

        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //imu.resetYaw();
    }

    public void InitializeResetImu(HardwareMap hardwareMap) {
        rotateAngleOffset = 0;
        Initialize(hardwareMap);
        imu.resetYaw();
    }

    @Override
    public void update(Telemetry telemetry, JoystickWrapper joystickWrapper, double speed, double rotSpeed) {

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        pitch = orientation.getPitch(AngleUnit.DEGREES);
        roll = orientation.getRoll(AngleUnit.DEGREES);
        yaw = orientation.getYaw(AngleUnit.DEGREES);

        normalizedHeadingError = -yaw;



// Drive forward code
//        if (length(joystickWrapper.gamepad1GetLeftStickX(), joystickWrapper.gamepad1GetLeftStickY())>0.1) {
//
//            //TODO: how to determine side
//            if(true/*!RedOrBlue.isRed*/) {
//                if (normalize(Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetLeftStickY(), joystickWrapper.gamepad1GetLeftStickX())) + 90) > 45 || normalize(Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetLeftStickY(), joystickWrapper.gamepad1GetLeftStickX())) + 90) < -135) {
//                    targetHeading = normalize(Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetLeftStickY(), joystickWrapper.gamepad1GetLeftStickX())) + 90);
//                } else {
//                    targetHeading = normalize(Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetLeftStickY(), joystickWrapper.gamepad1GetLeftStickX())) - 90);
//                }
//            }else {
//                if (normalize(Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetLeftStickY(), -joystickWrapper.gamepad1GetLeftStickX())) + 90) > 45 || normalize(Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetLeftStickY(), -joystickWrapper.gamepad1GetLeftStickX())) + 90) < -135) {
//                    targetHeading = normalize(Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetLeftStickY(), joystickWrapper.gamepad1GetLeftStickX())) + 90);
//                } else {
//                    targetHeading = normalize(Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetLeftStickY(), joystickWrapper.gamepad1GetLeftStickX())) - 90);
//                }
//            }
//        }

        if (length(joystickWrapper.gamepad1GetRightStickX(), joystickWrapper.gamepad1GetRightStickY()) > .5) {

            /*if(joystickWrapper.gamepad1.left_stick_button){
                if (joystickWrapper.gamepad1.right_stick_button){
                    targetHeading = Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetLeftStickY(), joystickWrapper.gamepad1GetLeftStickX())) + 90;
                }else{
                    targetHeading = Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetLeftStickY(), joystickWrapper.gamepad1GetLeftStickX())) + - 90;
                }*/
            /*}else {*/
            targetHeading = normalize(Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetRightStickY(), joystickWrapper.gamepad1GetRightStickX())) - 90);
            //}
        }   // Save for telemetry


        /* TODO have this be a function that the states can call or set
        if(joystickWrapper.gamepad1.b){
            targetHeading = -90;
        }
        if(joystickWrapper.gamepad1.x){
            targetHeading = 90;
        }
         */

        // Determine the heading current error
        double headingError = normalize((targetHeading - yaw)+rotateAngleOffset);





        //telemetry.addData("Joystick Left Angle", targetHeading);
        //telemetry.addData("Joystick Right Angle", Math.toDegrees(Math.atan2(joystickWrapper.gamepad1GetRightStickY(), joystickWrapper.gamepad1GetRightStickX())));
        //telemetry.addData("Pitch", pitch);
        //telemetry.addData("Roll", roll);
        //telemetry.addData("Yaw", yaw);
        //telemetry.addData("Error", headingError);
        //telemetry.addData("Range",Range.clip(headingError * PGain, -1, 1));
        //telemetry.addData("correction angle",yaw);
        //telemetry.addData("Cosine Movement",cosineThing);


        Translation2d translation2d;

        translation2d =RotateAngle(joystickWrapper.gamepad1GetLeftStickX(),joystickWrapper.gamepad1GetLeftStickY(),yaw);


        if(isAutoMode && joystickWrapper.gamepad1GetLeftStick()) {
            translation2d =RotateAngle(autoModeX,autoModeY,yaw);
        }


        /* TODO
        if (joystickWrapper.gamepad1GetA()) {
            cosineThing = !cosineThing;
        }
        */

        if (cosineThing) {
            MoveMecanum(-translation2d.getX(),translation2d.getY()*Math.cos(Math.toRadians(headingError)),Range.clip(headingError * PGain, -1, 1));
        } else {
            MoveMecanum(-translation2d.getX(),translation2d.getY(),Range.clip(headingError * PGain, -1, 1));
        }


        telemetry.update();
    }

    double length(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    void MoveMecanum(double x,double y,double rx){

        /* TODO: is there a similar constraint
        if(armWrapper.ExtensionMotorEx1.getCurrentPosition()>800){
            y=y/1.5;
            x=x/1.5;
            rx= Math.min(rx,Math.max(-.5, Math.min(.5, rx)));
        }
        */

        frontLeftMotor.setPower(y + x + rx);
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);
    }

    Translation2d RotateAngle(double x,double y,double angle){

        double rAngle = Math.toRadians(angle+rotateAngleOffset);

        double rotatedX = x*Math.cos(rAngle) - y*Math.sin(rAngle);
        double rotatedY = x*Math.sin(rAngle) + y*Math.cos(rAngle);
        return new Translation2d(rotatedX,rotatedY);
    }

    public double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setAutoMode(double inX, double inY) {
        isAutoMode = true;
        autoModeX = inX;
        autoModeY = inY;
    }

    public void disableAutoMode() {
        isAutoMode = false;
        autoModeX = 0;
        autoModeY = 0;
    }

    public double getNormalizedHeadingError() {
        return normalizedHeadingError;
    }

    public double normalize(double angle) {
        if (angle > 180) {
            return angle-360;
        } else if (angle < -180) {
            return angle+360;
        } else{
            return angle;
        }
    }

}
