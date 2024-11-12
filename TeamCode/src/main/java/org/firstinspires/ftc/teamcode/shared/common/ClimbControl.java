package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class ClimbControl extends RobotComponent{
    DcMotor climbMotor;
    Gamepad gamepad;

    public ClimbControl(RobotOpMode robotOpMode, Gamepad gamepad, DcMotor climbMotor) {
        super(robotOpMode);
        this.gamepad = gamepad;
        this.climbMotor = climbMotor;
    }
    public void update() {
        if(gamepad.dpad_left) {
            climbMotor.setPower(+1.0);
        }
        else if(gamepad.dpad_right) {
            climbMotor.setPower(-1.0);
        } else {
            climbMotor.setPower(+0.0);
        }
    }
}
