package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class LiftControl extends RobotComponent{
    DcMotor liftMotor;
    Gamepad gamepad;

    public LiftControl(RobotOpMode robotOpMode, Gamepad gamepad, DcMotor liftMotor) {
        super(robotOpMode);
        this.gamepad = gamepad;
        this.liftMotor = liftMotor;
    }
    public void update() {
        if(gamepad.right_trigger>0) {
            liftMotor.setPower(gamepad.right_trigger);
        }
        else if(gamepad.left_trigger>0) {
            liftMotor.setPower(-gamepad.left_trigger);
        }
        else {
            liftMotor.setPower(0);
        }
    }
}
