package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public interface IDrive {

//    public void init();
    public void update(Telemetry telemetry, JoystickWrapper joystickWrapper, double speed, double rotSpeed);
}
