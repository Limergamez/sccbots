package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class rotateHarvester extends RobotComponent {
    private Servo rotateServo;
    private Gamepad gamepad;
    private boolean toggleState = false;
    private boolean previousButtonState = false;

    public rotateHarvester(RobotOpMode robotOpMode, Gamepad gamepad, Servo rotateServo) {
        super(robotOpMode);
        this.gamepad = gamepad;
        this.rotateServo = rotateServo;
    }

    public void update() {
        if (gamepad.right_bumper && !previousButtonState) {
            toggleState = !toggleState;
            if (toggleState) {
                rotateServo.setPosition(1.0);
            } else {
                rotateServo.setPosition(0.0);
            }
        }
        previousButtonState = gamepad.right_bumper;
    }
}
