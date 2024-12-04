/*
package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class spinHarvester extends RobotComponent {
    private Servo clawServo;
    private Gamepad gamepad;
    private boolean clawOpen = false;
    private boolean previousButtonState = false;

    public spinHarvester(RobotOpMode robotOpMode, Gamepad gamepad, Servo clawServo) {
        super(robotOpMode);
        this.gamepad = gamepad;
        this.clawServo = clawServo;
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
