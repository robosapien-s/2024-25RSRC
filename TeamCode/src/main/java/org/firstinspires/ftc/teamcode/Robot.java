package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.states.StateManager;

public class Robot implements IRobot {
    public void getState(){
        // pass
    }

    @Override
    public void execute() {
        IRobot drive = new StateManager();
        drive.execute();
    }
}
