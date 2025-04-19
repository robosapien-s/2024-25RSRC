package org.firstinspires.ftc.teamcode.states;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.controllers.ConditionalTask;
import org.firstinspires.ftc.teamcode.controllers.ExecuteOnceTask;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.opmodes.RoboSapiensTeleOp;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;


@Config
public class HangState extends BaseState {

    boolean lvl2Reached = false;


    public static int HANG_SLIDE_ANGLE_ROTATION_PREP = 723;
    public static int HANG_SLIDE_EXTENSION_PREP = 1200;

    public static int HANG_SLIDE_ROTATION_INITIAL = 242;

    public static int HANG_SLIDE_ANGLE_ROTATION_L2 = 600;
    public static int HANG_SLIDE_EXTENSTION_L2 = 0;

    public static int HANG_SLIDE_HOOKS_ENGAGED = 242;


    public static int HANG_SLIDE_ROTATION_L3_INITIAL = 1635;
    public static int HANG_SLIDE_L3_INTIAL = 1190;

    double lastPitch = 10000;

    double lastTime = 0L;


    double targetPos;


    public HangState(JoystickWrapper joystick) {
        super(joystick);
    }

    @Override
    public void start(Robot robot, Telemetry telemetry) {
        //robot.setIntakeAnglePosition(RoboSapiensTeleOp.Params.INTAKE_ANGLE_READY);
    }


    @Override
    public void initialize(Robot robot, IRobot prevState) {
        robot.setSlideMinPosition(0);
        robot.setSlideMaxPosition(1500);
        robot.setSlideRotationMaxPosition(2600);
        robot.setDriveTrainEnabled(false);
        taskArrayList.add(createSlideRotationTask(robot, HANG_SLIDE_ANGLE_ROTATION_PREP, 500, "slide rotation", false));
        taskArrayList.add(createSlideTask(robot, HANG_SLIDE_EXTENSION_PREP, 0, "slide", false));
        taskArrayList.add(createRotationAndAngleTask(robot, new double[]{0.055, 0.945}, 0, "rot and angle", false));
        taskArrayList.add(createIntakeAngleServoTask(robot, .275, 250, "intake angle", false));
        taskArrayList.add(new ExecuteOnceTask(
                new ExecuteOnceTask.ExecuteListener() {
                    @Override
                    public void execute() {
                        robot.setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_OPEN);
                        targetPos = RoboSapiensTeleOp.Params.HANG_SERVO_OPEN;
                    }
                }, "hang servo"
        ));

    }

    @Override
    public void execute(Robot robot, Telemetry telemetry) {


        if (joystick.gamepad1GetA()) {
            robot.switchState(State.INTAKINGCLAW);
        } else if (joystick.gamepad1GetY()) {
            if (taskArrayList.isEmpty()) {
                lvl2hang(robot);
//                lvl3hang(robot, telemetry);
            }

        } else if (joystick.gamepad1GetB() || joystick.gamepad1GetX()) {
            if (taskArrayList.isEmpty() && !lvl2Reached) {
                    lvl2hang(robot);
                }  else if (taskArrayList.isEmpty()) {
//                    lvl3hang(robot, telemetry);

                }

            }

            if (joystick.gamepad1GetRightBumperRaw()) {
                robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger() * -100));
            } else {
                robot.increaseSlideTargetPosition((int) (joystick.gamepad1GetRightTrigger() * 100));
            }

            if (joystick.gamepad1GetLeftBumperRaw()) {
                robot.increaseSlideRotationTargetPosition((int) (joystick.gamepad1GetLeftTrigger() * -20));
            } else {
                robot.increaseSlideRotationTargetPosition((int) (joystick.gamepad1GetLeftTrigger() * 20));
            }

            if (joystick.gamepad1GetDUp()) {
                targetPos = RoboSapiensTeleOp.Params.HANG_SERVO_OPEN;
                robot.setHangServo(targetPos);
            }

            if (joystick.gamepad1GetDDown()) {
                targetPos = RoboSapiensTeleOp.Params.HANG_SERVO_HOOKED;
                robot.setHangServo(targetPos);
            }

            if (joystick.gamepad1GetDLeft()) {
                targetPos -= 0.05;
                robot.setHangServo(targetPos);
            }

            if (joystick.gamepad1GetDRight()) {
                targetPos += 0.05;
                robot.setHangServo(targetPos);
            }

            if (joystick.gamepad1GetLeftStickDown()) {
                robot.disableHangServo();
            }

            if (joystick.gamepad1GetRightStickDown()) {
                robot.enableHangServo();
            }

            robot.updateDriveTrainsRaw(telemetry, false, joystick.gamepad1GetLeftStickX(), joystick.gamepad1GetLeftStickY(), joystick.gamepad1GetRightStickX(), joystick.gamepad1GetRightStickY(), 1, 1);

            telemetry.addData("Hang Servo Pos", targetPos);

            executeTasks(telemetry);

        }


        @Override
        public void dispose (Robot robot){
            robot.setDriveTrainEnabled(true);
            robot.setSlideMinPosition(RoboSapiensTeleOp.Params.SLIDE_MIN_POSITION);
        }

        @Override
        public State getState () {
            return State.ROBOT_HANG;
        }


        public void lvl2hang(Robot robot) {
            taskArrayList.add(createSlideRotationTask(robot, HANG_SLIDE_ROTATION_INITIAL, 500, "rotation", false));

            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_FULLY_OPEN);
                        }
                    }, "hang servo"
            ));

            taskArrayList.add(createSlideTask(robot, HANG_SLIDE_EXTENSTION_L2, 500, "slide", false));

            taskArrayList.add(createSlideRotationTask(robot, HANG_SLIDE_ANGLE_ROTATION_L2, 1000, "rotation", false));
            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_HOOKED);
                        }
                    }, "hang servo"
            ));

            taskArrayList.add(createWaitTask(robot, 500, "engaging hooks"));

            taskArrayList.add(createSlideTask(robot, HANG_SLIDE_HOOKS_ENGAGED, 100, "sliding out", false));

            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.disableHangServo();
                        }
                    }, "disable hang servo"
            ));


            taskArrayList.add(createWaitTask(robot, 100, "wait"));

            taskArrayList.add(createSlideRotationTask(robot, 950, 2000, "slide rotation more", false));

            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            lvl2Reached = true;
                        }
                    }, "lvl2Reached = true"
            ));
        }


        public void lvl3hang(Robot robot, Telemetry telemetry) {
            /**
             * hang state
             * close angle to 242
             * simultaneously close angle to 600 and close slide to 0, and open hooks more
             * engage hooks,
             * 200 on slide
             * unengage hooks
             * 950 on rotation
             * 1620 on rotation
             * 1190 on slide
             * slide to 917
             * open rotation 2500
             * close slide to 0
             * close rotation to 1500
             * open slide to 180
             * close rotation to 972
             * bring slide to 0
             * engage hooks
             */
            taskArrayList.add(createSlideRotationTask(robot, HANG_SLIDE_ROTATION_L3_INITIAL, 0, "slide rotation", false));
            taskArrayList.add(new ConditionalTask(
                    new ConditionalTask.ConditionalTaskListener() {
                        @Override
                        public boolean conditionMet() {
                            double pitch = robot.getPitch();

                            telemetry.addData("pitch", pitch);

                            if (lastPitch > 360) {
                                lastPitch = pitch;
                                lastTime = System.currentTimeMillis();
                                return false;
                            } else {
                                double time = System.currentTimeMillis();
                                double velocity = (pitch - lastPitch) / (time - lastTime);
                                lastTime = time;
                                lastPitch = pitch;
                                telemetry.addData("velocity", velocity);
                                return (pitch > -20) && (velocity <= -.025);
                            }


                        }

                        @Override
                        public void executeTask() {
                            robot.setSlideTargetPosition(HANG_SLIDE_L3_INTIAL);
                        }
                    }
            ));
            taskArrayList.add(createSlideRotationTask(robot, HANG_SLIDE_ROTATION_L3_INITIAL - 200, 1250, "slide rotation", false));
            taskArrayList.add(createSlideTask(robot, 917, 250, "slide in", false));
            taskArrayList.add(createSlideRotationTask(robot, 2600, 750, "slide rotation", false));
            taskArrayList.add(createSlideTask(robot, 0, 1750, "slide in", false));
            taskArrayList.add(createSlideRotationTask(robot, 1200, 1000, "slide rotation", false));
            taskArrayList.add(createSlideTask(robot, 150, 250, "slide in", true));
            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_FULLY_OPEN);
                            robot.enableHangServo();
                        }
                    }, "enable hang servo"
            ));
            taskArrayList.add(createWaitTask(robot, 500, "wait for servo to engage"));
            taskArrayList.add(createSlideRotationTask(robot, 630, 750, "slide rotation", false));
            taskArrayList.add(createSlideTask(robot, 0, 250, "slide in", false));
            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.setHangServo(RoboSapiensTeleOp.Params.HANG_SERVO_HOOKED);
                        }
                    }, "enable hang servo"
            ));
            taskArrayList.add(createWaitTask(robot, 500, "wait for servo to engage"));
            taskArrayList.add(createSlideTask(robot, HANG_SLIDE_HOOKS_ENGAGED, 100, "slide in", false));
            taskArrayList.add(new ExecuteOnceTask(
                    new ExecuteOnceTask.ExecuteListener() {
                        @Override
                        public void execute() {
                            robot.disableHangServo();
                        }
                    }, "enable hang servo"
            ));
            taskArrayList.add(createWaitTask(robot, 100, "wait for servo to disengage"));

        }
}