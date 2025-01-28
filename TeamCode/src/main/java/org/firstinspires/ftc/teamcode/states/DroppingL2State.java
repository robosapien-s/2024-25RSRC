package org.firstinspires.ftc.teamcode.states;

import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class DroppingL2State extends DroppingL1State {

    public DroppingL2State(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public int getHeight() {
        return RoboSapiensTeleOp.Params.VERTICAL_SLIDE_DROP_L2;
    }

    @Override
    public State getState() {
        return State.DROPPING_L2;
    }

}
