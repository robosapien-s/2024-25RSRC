package org.firstinspires.ftc.teamcode.controllers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface IRobotTask {

    void execute(Telemetry telemetry);
    boolean isBlocking();
    boolean hasStarted();
    boolean isRunning();
    boolean isComplete();
    void stopTask();

    void dispose();
}
