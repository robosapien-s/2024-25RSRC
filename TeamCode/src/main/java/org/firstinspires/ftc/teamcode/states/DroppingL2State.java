package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.CallBackTask;
import org.firstinspires.ftc.teamcode.controllers.RobotTaskSeries;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class DroppingL2State extends DroppingL1State {

    public DroppingL2State(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public int getHeight() {
        return DriveTest.Params.VERTICAL_SLIDE_DROP_L2;
    }

    @Override
    public State getState() {
        return State.DROPPING_L2;
    }

}
