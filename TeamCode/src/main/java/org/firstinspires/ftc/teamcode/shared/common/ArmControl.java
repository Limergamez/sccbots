package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ArmControl extends RobotComponent{
    DcMotor armMotor;
    Gamepad gamepad;

    public ArmControl(RobotOpMode robotOpMode, Gamepad gamepad, DcMotor armMotor) {
        super(robotOpMode);
        this.gamepad = gamepad;
        this.armMotor = armMotor;
    }
    public void update() {
        if(gamepad.dpad_left) {
            armMotor.setPower(+1.0);
        }
        else if(gamepad.dpad_right) {
            armMotor.setPower(-1.0);
        } else {
            armMotor.setPower(+0.0);
        }
    }
}
