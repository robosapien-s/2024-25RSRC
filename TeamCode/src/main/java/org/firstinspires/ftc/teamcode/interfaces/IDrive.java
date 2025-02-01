package org.firstinspires.ftc.teamcode.interfaces;

import android.graphics.Point;

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

}
