package org.firstinspires.ftc.teamcode.controllers;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class RobotTaskParallel extends RobotTaskListImpl {

    @Override
    public void execute(Telemetry telemetry) {

        for (IRobotTask task : tasks) {
//            if(!task.hasStarted()) {
//                task.execute(telemetry);
//            }
            task.execute(telemetry);
        };
    }

    @Override
    public boolean isBlocking() {
        return false;
    }


}
