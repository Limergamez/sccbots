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

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update() {
        if (gamepad.dpad_up) {
            armMotor.setPower(+0.23);
        } else if (gamepad.dpad_down) {
            armMotor.setPower(-0.37);
        }
        else if (Math.abs(gamepad.left_stick_y) > 0.1) {
            double power = -gamepad.left_stick_y * 0.4;
            armMotor.setPower(power);
        }
        else {
            armMotor.setPower(0.0);
        }
    }

}
