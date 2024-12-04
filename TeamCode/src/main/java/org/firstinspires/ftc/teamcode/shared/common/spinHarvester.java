package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

public class spinHarvester extends RobotComponent {
    private CRServo spinServo;
    private Gamepad gamepad;

    public spinHarvester(RobotOpMode robotOpMode, Gamepad gamepad, CRServo spinServo) {
        super(robotOpMode);
        this.gamepad = gamepad;
        this.spinServo = spinServo;
    }

    public void update() {
        if (gamepad.y) {
            spinServo.setPower(1.0);
        } else {
            spinServo.setPower(0.0);
        }
    }
}
