package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawControl extends RobotComponent{
    Servo clawServo;
    Gamepad gamepad;

    public ClawControl(RobotOpMode robotOpMode, Gamepad gamepad, Servo clawServo) {
        super(robotOpMode);
        this.gamepad = gamepad;
        this.clawServo = clawServo;
    }
    public void update() {
        if(gamepad.a) {
            clawServo.setPosition(+1.0);
        }
        else if(gamepad.b) {
            clawServo.setPosition(+0.0);
        }
    }
}
