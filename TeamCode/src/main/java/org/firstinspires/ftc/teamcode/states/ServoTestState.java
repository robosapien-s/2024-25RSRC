/*
red -
blue -
yellow - rotation
green -
 */
package org.firstinspires.ftc.teamcode.states;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

public class ServoTestState extends BaseState {

    public ServoTestState(JoystickWrapper joystick) {
        super(joystick);
    }

    HashMap<String, Servo> servoHashMap;
    ArrayList<String> servoNames = new ArrayList<>();
    int index = -1;


    @Override
    public void initialize(Robot robot, IRobot prevState) {
        servoHashMap = robot.getServoForTesting();
        servoNames = new ArrayList<>(servoHashMap.keySet());
        if(!servoNames.isEmpty()) {
            index = 0;
        }
    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {


        if(joystick.gamepad1GetDLeft()) {
            index--;
            if(index < 0) {
                index = servoNames.size()-1;
            }
        } else if (joystick.gamepad1GetDRight()) {
            index++;
            if(index>servoNames.size()-1) {
                index = 0;
            }
        }
        else if(joystick.gamepad1GetDUp()) {

            String servoName = servoNames.get(index);
            Servo servo = Objects.requireNonNull(servoHashMap.get(servoName));
            if(servo != null) {
                double position = servo.getPosition();
                position += .02;
                if (position > 1) {
                    position = 1;
                }
                servo.setPosition(position);
            }

        } else if (joystick.gamepad1GetDDown()) {
            String servoName = servoNames.get(index);
            Servo servo = Objects.requireNonNull(servoHashMap.get(servoName));
            if(servo != null) {
                double position = servo.getPosition();
                position -= .02;
                if (position < 0 ) {
                    position = 0;
                }

                servo.setPosition(position);
            }
        }

        if (joystick.gamepad1GetA()) {
            robot.setDualSlideTargetPosition(50);
        }
        if (joystick.gamepad1GetB()) {
            robot.setDualSlideTargetPosition(0);
        }

        String servoName = servoNames.get(index);
        telemetry.addData(servoName, Objects.requireNonNull(servoHashMap.get(servoName)).getPosition());
    }

    @Override
    public State getState() {
        return State.SERVO_TEST;
    }
}
