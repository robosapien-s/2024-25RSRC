package org.firstinspires.ftc.teamcode.interfaces;

import android.graphics.Point;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.localization.Pose;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public interface IDrive {

//    public void init();
    public void update(Telemetry telemetry, JoystickWrapper joystickWrapper, double speed, double rotSpeed);

    public void updateRaw(Telemetry telemetry, boolean isLeftStickPressed, double leftStickX, double leftStickY, double rightStickX, double rightStickY, double speed, double rotSpeed);

    public void setTargetHeading(double heading);

    public Vector3D getRobotPosition();

    public void setAutoMode(double inX, double inY);

    public void disableAutoMode();

    public double getYaw();

    public void setPose(Pose pose);

    public Pose getPose() ;

}
