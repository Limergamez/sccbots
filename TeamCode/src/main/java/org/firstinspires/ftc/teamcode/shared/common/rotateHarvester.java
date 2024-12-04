/*
package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class rotateHarvester extends RobotComponent {
    private Servo rotateHarvester;
    private Gamepad gamepad;
    private boolean rotatingHarvester = false;
    private boolean previousButtonState = false;

    public rotateHarvester(RobotOpMode robotOpMode, Gamepad gamepad, Servo clawServo) {
        super(robotOpMode);
        this.gamepad = gamepad;
    }

    public void update() {
        if (gamepad.a && !previousButtonState) {
            clawOpen = !clawOpen;
            if (clawOpen) {
                clawServo.setPosition(1.0);
            } else {
                clawServo.setPosition(0.0);
            }
        }
        previousButtonState = gamepad.a;
    }
}
*/
