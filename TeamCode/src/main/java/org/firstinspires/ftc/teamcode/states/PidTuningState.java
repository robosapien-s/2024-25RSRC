/*
red -
blue -
yellow - rotation
green -
 */
package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.DriveTest;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

public class PidTuningState extends BaseState {


    public PidTuningState(JoystickWrapper joystick) {
        super(joystick);
    }

    HashMap<String, Servo> servoHashMap;
    ArrayList<String> servoNames = new ArrayList<>();
    int index = -1;


    @Override
    public void initialize(Robot robot, IRobot prevState) {

    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {

        if (joystick.gamepad1GetDUp()) {
            robot.setVerticalSlideTargetPosition(DriveTest.Params.VERTICAL_SLIDE_PID_target);
        }
    }

    @Override
    public State getState() {
        return State.PID_TUNING;
    }
}
