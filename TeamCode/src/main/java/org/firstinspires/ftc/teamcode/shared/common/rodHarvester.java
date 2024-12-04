package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class rodHarvester extends RobotComponent {
    private Servo rodServo;
    private Gamepad gamepad;

    public rodHarvester(RobotOpMode robotOpMode, Gamepad gamepad, Servo rodServo) {
        super(robotOpMode);
        this.gamepad = gamepad;
        this.rodServo = rodServo;
    }

    public void update() {
        if (gamepad.right_trigger > 0) {
            rodServo.setPosition(1.0);
        } else if (gamepad.left_trigger > 0) {
            rodServo.setPosition(0.0);
        } else {
            rodServo.setPosition(0.5);
        }
    }
}
