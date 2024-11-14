package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Shaun on 2/07/2017.
 */

public class SuperSlowTurnControl extends RobotComponent {

    private static float[] power_curve =
            {0.00f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.5f, 1.0f};
    private static float[] steer_curve =
            {0.05f, 0.1f, 0.15f, 0.2f, 0.3f, 0.4f, 0.7f, 1.0f};
    final private DcMotor leftBackMotor;
    final private DcMotor rightBackMotor;
    final private DcMotor leftFrontMotor;
    final private DcMotor rightFrontMotor;
    final private Gamepad gamepad;

    public SuperSlowTurnControl(RobotOpMode opMode, Gamepad gamepad, DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor) {
        super(opMode);

        this.gamepad = gamepad;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;

    }

    /*
     * Update the motor power based on the game pad state
     */
    public void update() {
        if (gamepad.left_bumper) {
            leftBackMotor.setPower(+0.25);
            leftFrontMotor.setPower(+0.25);
            rightBackMotor.setPower(-0.25);
            rightFrontMotor.setPower(-0.25);
        }
        if (gamepad.right_bumper) {
            leftBackMotor.setPower(-0.25);
            leftFrontMotor.setPower(-0.25);
            rightBackMotor.setPower(+0.25);
            rightFrontMotor.setPower(+0.25);
        }
    }
}