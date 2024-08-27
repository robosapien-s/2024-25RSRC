package org.firstinspires.ftc.teamcode.wrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

// REMOVED STICK ANGLE BECAUSE OF ROADRUNNER DEPENDENCY ADD BACK WHEN POSSIBLE
public class JoystickWrapper {

    public Gamepad gamepad1;  //setting a variable gamepad of type Gamepad
    Gamepad gamepad2;

    boolean gamepad1xPressed = false;
    boolean gamepad1yPressed = false;
    boolean gamepad1aPressed = false;
    boolean gamepad1bPressed = false;

    boolean gamepad1dUpPressed = false;
    boolean gamepad1dDownPressed = false;
    boolean gamepad1dRightPressed = false;
    boolean gamepad1dLeftPressed = false;

    boolean gamepad1leftBumperPressed = false;
    boolean gamepad1rightBumperPressed = false;

    boolean gamepad1leftStickPressed = false;
    boolean gamepad1rightStickPressed = false;

    boolean gamepad1leftTriggerPressed = false;
    boolean gamepad1rightTriggerPressed = false;

    double gamepad1leftStickLastDirection;
    double gamepad1rightStickLastDirection;

    boolean gamepad2xPressed = false;
    boolean gamepad2yPressed = false;
    boolean gamepad2aPressed = false;
    boolean gamepad2bPressed = false;

    boolean gamepad2dUpPressed = false;
    boolean gamepad2dDownPressed = false;
    boolean gamepad2dRightPressed = false;
    boolean gamepad2dLeftPressed = false;

    boolean gamepad2leftBumperPressed = false;
    boolean gamepad2rightBumperPressed = false;

    boolean gamepad2leftStickPressed = false;
    boolean gamepad2rightStickPressed = false;

    boolean gamepad2leftTriggerPressed = false;
    boolean gamepad2rightTriggerPressed = false;

    double gamepad2leftStickLastDirection;
    double gamepad2rightStickLastDirection;


    public JoystickWrapper(Gamepad inGamepad1, Gamepad inGamepad2) {
        gamepad1 = inGamepad1; //Making a reference to the type Gampad in opmodes
        gamepad2 = inGamepad2;
    }

    /*Everything below is what happens in certain cases of keys being pressed to prevent them from
     being pressed multiple times when we don't want them to
     */

    //Gamepad 1

    public boolean gamepad1GetX() {
        if (!gamepad1xPressed && gamepad1.x) {
            gamepad1xPressed = true;
            return true;
        } else if (gamepad1xPressed && !gamepad1.x) {
            gamepad1xPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetY() {
        if (!gamepad1yPressed && gamepad1.y) {
            gamepad1yPressed = true;
            return true;
        } else if (gamepad1yPressed && !gamepad1.y) {
            gamepad1yPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetA() {
        if (!gamepad1aPressed && gamepad1.a) {
            gamepad1aPressed = true;
            return true;
        } else if (gamepad1aPressed && !gamepad1.a) {
            gamepad1aPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetB() {
        if (!gamepad1bPressed && gamepad1.b) {
            gamepad1bPressed = true;
            return true;
        } else if (gamepad1bPressed && !gamepad1.b) {
            gamepad1bPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetDUp() {
        if (!gamepad1dUpPressed && gamepad1.dpad_up) {
            gamepad1dUpPressed = true;
            return true;
        } else if (gamepad1dUpPressed && !gamepad1.dpad_up) {
            gamepad1dUpPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetDDown() {
        if (!gamepad1dDownPressed && gamepad1.dpad_down) {
            gamepad1dDownPressed = true;
            return true;
        } else if (gamepad1dDownPressed && !gamepad1.dpad_down) {
            gamepad1dDownPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetDRight() {
        if (!gamepad1dRightPressed && gamepad1.dpad_right) {
            gamepad1dRightPressed = true;
            return true;
        } else if (gamepad1dRightPressed && !gamepad1.dpad_right) {
            gamepad1dRightPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetDLeft() {
        if (!gamepad1dLeftPressed && gamepad1.dpad_left) {
            gamepad1dLeftPressed = true;
            return true;
        } else if (gamepad1dLeftPressed && !gamepad1.dpad_left) {
            gamepad1dLeftPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetLeftBumperDown() {
        if (!gamepad1leftBumperPressed && gamepad1.left_bumper) {
            gamepad1leftBumperPressed = true;
            return true;
        } else if (gamepad1leftBumperPressed && !gamepad1.left_bumper) {
            gamepad1leftBumperPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetRightBumperDown() {
        if (!gamepad1rightBumperPressed && gamepad1.right_bumper) {
            gamepad1rightBumperPressed = true;
            return true;
        } else if (gamepad1rightBumperPressed && !gamepad1.right_bumper) {
            gamepad1rightBumperPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetLeftStickDown() {
        if (!gamepad1leftStickPressed && gamepad1.left_stick_button) {
            gamepad1leftStickPressed = true;
            return true;
        } else if (gamepad1leftStickPressed && !gamepad1.left_stick_button) {
            gamepad1leftStickPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetRightStickDown() {
        if (!gamepad1rightStickPressed && gamepad1.right_stick_button) {
            gamepad1rightStickPressed = true;
            return true;
        } else if (gamepad1rightStickPressed && !gamepad1.right_stick_button) {
            gamepad1rightStickPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetLeftTriggerDown() {
        if (!gamepad1leftTriggerPressed && gamepad1GetLeftTriggerPressed()) {
            gamepad1leftTriggerPressed = true;
            return true;
        } else if (gamepad1leftTriggerPressed && !gamepad1GetLeftTriggerPressed()) {
            gamepad1leftTriggerPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetRightTriggerDown() {
        if (!gamepad1rightTriggerPressed && gamepad1GetRightTriggerPressed()) {
            gamepad1rightTriggerPressed = true;
            return true;
        } else if (gamepad1rightTriggerPressed && !gamepad1GetRightTriggerPressed()) {
            gamepad1rightTriggerPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad1GetLeftTriggerPressed() {
        return gamepad1.left_trigger > .5;
    }

    public boolean gamepad1GetRightTriggerPressed() {
        return gamepad1.right_trigger > .5;
    }

    public boolean gamepad1GetRightBumperRaw() {
        return gamepad1.right_bumper;
    }

    public boolean gamepad1GetLeftBumperRaw() {
        return gamepad1.left_bumper;
    }

    public boolean gamepad1GetRightStick() {
        return gamepad1.right_stick_button;
    }

    public boolean gamepad1GetLeftStick() {
        return gamepad1.left_stick_button;
    }

    public boolean gamepad1GetRightStickUsed(float deadzone) {
        return deadzone < Math.abs(gamepad1GetRightStickX()) + Math.abs(gamepad1GetRightStickY()) || deadzone == Math.abs(gamepad1GetRightStickX()) + Math.abs(gamepad1GetRightStickY());
    }

    public boolean gamepad1GetRightStickUsed(double deadzone) {
        return deadzone < Math.abs(gamepad1GetRightStickX()) + Math.abs(gamepad1GetRightStickY()) || deadzone == Math.abs(gamepad1GetRightStickX()) + Math.abs(gamepad1GetRightStickY());
    }

    public boolean gamepad1GetLeftStickUsed(double deadzone) {
        return deadzone < Math.abs(gamepad1GetLeftStickX()) + Math.abs(gamepad1GetLeftStickY()) || deadzone == Math.abs(gamepad1GetLeftStickX()) + Math.abs(gamepad1GetLeftStickY());
    }

    public boolean gamepad1GetLeftStickUsed(float deadzone) {
        return deadzone < Math.abs(gamepad1GetLeftStickX()) + Math.abs(gamepad1GetLeftStickY()) || deadzone == Math.abs(gamepad1GetLeftStickX()) + Math.abs(gamepad1GetLeftStickY());
    }


    public double gamepad1GetRightTrigger() {
        return gamepad1.right_trigger;
    }

    public double gamepad1GetLeftTrigger() {
        return gamepad1.left_trigger;
    }

    public double gamepad1GetRightStickX() {
        return gamepad1.right_stick_x;
    }

    public double gamepad1GetRightStickY() {
        return gamepad1.right_stick_y;
    }

    public double gamepad1GetLeftStickX() {
        return gamepad1.left_stick_x;
    }

    public double gamepad1GetLeftStickY() {
        return gamepad1.left_stick_y;
    }

    public boolean gamepad1GetARaw() {
        return gamepad1.a;
    }




    //Gamepad 2

    public boolean gamepad2GetX() {
        if (!gamepad2xPressed && gamepad2.x) {
            gamepad2xPressed = true;
            return true;
        } else if (gamepad2xPressed && !gamepad2.x) {
            gamepad2xPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetY() {
        if (!gamepad2yPressed && gamepad2.y) {
            gamepad2yPressed = true;
            return true;
        } else if (gamepad2yPressed && !gamepad2.y) {
            gamepad2yPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetA() {
        if (!gamepad2aPressed && gamepad2.a) {
            gamepad2aPressed = true;
            return true;
        } else if (gamepad2aPressed && !gamepad2.a) {
            gamepad2aPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetB() {
        if (!gamepad2bPressed && gamepad2.b) {
            gamepad2bPressed = true;
            return true;
        } else if (gamepad2bPressed && !gamepad2.b) {
            gamepad2bPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetDUp() {
        if (!gamepad2dUpPressed && gamepad2.dpad_up) {
            gamepad2dUpPressed = true;
            return true;
        } else if (gamepad2dUpPressed && !gamepad2.dpad_up) {
            gamepad2dUpPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetDDown() {
        if (!gamepad2dDownPressed && gamepad2.dpad_down) {
            gamepad2dDownPressed = true;
            return true;
        } else if (gamepad2dDownPressed && !gamepad2.dpad_down) {
            gamepad2dDownPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetDRight() {
        if (!gamepad2dRightPressed && gamepad2.dpad_right) {
            gamepad2dRightPressed = true;
            return true;
        } else if (gamepad2dRightPressed && !gamepad2.dpad_right) {
            gamepad2dRightPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetDLeft() {
        if (!gamepad2dLeftPressed && gamepad2.dpad_left) {
            gamepad2dLeftPressed = true;
            return true;
        } else if (gamepad2dLeftPressed && !gamepad2.dpad_left) {
            gamepad2dLeftPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetLeftBumperDown() {
        if (!gamepad2leftBumperPressed && gamepad2.left_bumper) {
            gamepad2leftBumperPressed = true;
            return true;
        } else if (gamepad2leftBumperPressed && !gamepad2.left_bumper) {
            gamepad2leftBumperPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetRightBumperDown() {
        if (!gamepad2rightBumperPressed && gamepad2.right_bumper) {
            gamepad2rightBumperPressed = true;
            return true;
        } else if (gamepad2rightBumperPressed && !gamepad2.right_bumper) {
            gamepad2rightBumperPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetRightStickDown() {
        if (!gamepad2rightStickPressed && gamepad2.right_stick_button) {
            gamepad2rightStickPressed = true;
            return true;
        } else if (gamepad2rightStickPressed && !gamepad2.right_stick_button) {
            gamepad2rightStickPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetLeftTriggerDown() {
        if (!gamepad2leftTriggerPressed && gamepad2GetLeftTriggerPressed()) {
            gamepad2leftTriggerPressed = true;
            return true;
        } else if (gamepad2leftTriggerPressed && !gamepad2GetLeftTriggerPressed()) {
            gamepad2leftTriggerPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetRightTriggerDown() {
        if (!gamepad2rightTriggerPressed && gamepad2GetRightTriggerPressed()) {
            gamepad2rightTriggerPressed = true;
            return true;
        } else if (gamepad2rightTriggerPressed && !gamepad2GetRightTriggerPressed()) {
            gamepad2rightTriggerPressed = false;
            return false;
        } else {
            return false;
        }
    }

    public boolean gamepad2GetLeftTriggerPressed() {
        return gamepad2.left_trigger > .5;
    }

    public boolean gamepad2GetRightTriggerPressed() {
        return gamepad2.right_trigger > .5;
    }

    public boolean gamepad2GetDUpRaw() {
        return gamepad2.dpad_up;
    }

    public boolean gamepad2GetDDownRaw() {
        return gamepad2.dpad_down;
    }

    public boolean gamepad2GetDRightRaw() {
        return gamepad2.dpad_right;
    }

    public boolean gamepad2GetDLeftRaw() {
        return gamepad2.dpad_left;
    }

    public boolean gamepad2GetRightBumperRaw() {
        return gamepad2.right_bumper;
    }

    public boolean gamepad2GetLeftBumperRaw() {
        return gamepad2.left_bumper;
    }

    public boolean gamepad2GetRightStick() {
        return gamepad2.right_stick_button;
    }

    public boolean gamepad2GetLeftStickDown() {
        return gamepad2.left_stick_button;
    }

    public boolean gamepad2GetRightStickUsed(float deadzone) {
        return deadzone < Math.abs(gamepad2GetRightStickX()) + Math.abs(gamepad2GetRightStickY()) || deadzone == Math.abs(gamepad2GetRightStickX()) + Math.abs(gamepad2GetRightStickY());
    }

    public boolean gamepad2GetRightStickUsed(double deadzone) {
        return deadzone < Math.abs(gamepad2GetRightStickX()) + Math.abs(gamepad2GetRightStickY()) || deadzone == Math.abs(gamepad2GetRightStickX()) + Math.abs(gamepad2GetRightStickY());
    }

    public boolean gamepad2GetLeftStickUsed(double deadzone) {
        return deadzone < Math.abs(gamepad2GetLeftStickX()) + Math.abs(gamepad2GetLeftStickY()) || deadzone == Math.abs(gamepad2GetLeftStickX()) + Math.abs(gamepad2GetLeftStickY());
    }

    public boolean gamepad2GetLeftStickUsed(float deadzone) {
        return deadzone < Math.abs(gamepad2GetLeftStickX()) + Math.abs(gamepad2GetLeftStickY()) || deadzone == Math.abs(gamepad2GetLeftStickX()) + Math.abs(gamepad2GetLeftStickY());
    }


    public double gamepad2GetRightTrigger() {
        return gamepad2.right_trigger;
    }

    public double gamepad2GetLeftTrigger() {
        return gamepad2.left_trigger;
    }

    public double gamepad2GetRightStickX() {
        return gamepad2.right_stick_x;
    }

    public double gamepad2GetRightStickY() {
        return gamepad2.right_stick_y;
    }

    public double gamepad2GetLeftStickX() {
        return gamepad2.left_stick_x;
    }

    public double gamepad2GetLeftStickY() {
        return gamepad2.left_stick_y;
    }





}