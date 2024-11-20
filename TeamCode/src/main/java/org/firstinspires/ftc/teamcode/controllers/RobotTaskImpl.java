package org.firstinspires.ftc.teamcode.controllers;


abstract class RobotTaskImpl implements IRobotTask {
    ETaskBlockingType blockingType = ETaskBlockingType.series;

    @Override
    public void stopTask() {

    }
}
