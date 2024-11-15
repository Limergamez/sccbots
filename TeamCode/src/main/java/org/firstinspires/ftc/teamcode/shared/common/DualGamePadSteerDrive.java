package org.firstinspires.ftc.teamcode.shared.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Shaun on 2/07/2017.
 */

public class DualGamePadSteerDrive extends RobotComponent {

    private static float[] power_curve =
            {0.00f, 0.2f, 0.25f, 0.3f, 0.35f, 0.4f, 0.5f, 1.0f};
    private static float[] steer_curve =
            {0.05f, 0.1f, 0.15f, 0.2f, 0.3f, 0.4f, 0.7f, 1.0f};
    final private DcMotor leftBackMotor;
    final private DcMotor rightBackMotor;
    final private DcMotor leftFrontMotor;
    final private DcMotor rightFrontMotor;
    final private Gamepad gamepad;
    final private Telemetry.Item leftPowerItem;
    final private Telemetry.Item rightPowerItem;
    final private Telemetry.Item steerPowerItem;
    final private Telemetry.Item rawPowerItem;

    public DualGamePadSteerDrive(RobotOpMode opMode, Gamepad gamepad, DcMotor leftBackMotor, DcMotor rightBackMotor, DcMotor leftFrontMotor, DcMotor rightFrontMotor) {
        super(opMode);

        this.gamepad = gamepad;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;

        leftPowerItem = getOpMode().telemetry.addData("Left power", "%.2f", 0.0f);
        leftPowerItem.setRetained(true);
        rightPowerItem = getOpMode().telemetry.addData("Right power", "%.2f", 0.0f);
        rightPowerItem.setRetained(true);
        steerPowerItem = getOpMode().telemetry.addData("steer power", "%.2f", 0.0f);
        steerPowerItem.setRetained(true);
        rawPowerItem = getOpMode().telemetry.addData("raw power", "%.2f", 0.0f);
        rawPowerItem.setRetained(true);
    }


    public void setToFloat() {
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    public void setEncoder(boolean encoder) {
        if (encoder) {
            rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setTargetPosition(double backLeftDistance, double backRightDistance, double frontLeftDistance, double frontRightDistance) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        newFrontLeftTarget = rightBackMotor.getCurrentPosition() + (int) frontLeftDistance;
        newFrontRightTarget = rightFrontMotor.getCurrentPosition() + (int) frontRightDistance;
        newBackLeftTarget = leftBackMotor.getCurrentPosition() + (int) backLeftDistance;
        newBackRightTarget = leftFrontMotor.getCurrentPosition() + (int) backRightDistance;
        leftFrontMotor.setTargetPosition(newFrontLeftTarget);
        rightFrontMotor.setTargetPosition(newFrontRightTarget);
        leftBackMotor.setTargetPosition(newBackLeftTarget);
        rightBackMotor.setTargetPosition(newBackRightTarget);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /*
     * Update the motor power based on the gamepad state
     */
    public void update() {

//        float scalePower = scaleTriggerPower(gamepad.left_trigger - gamepad.right_trigger);
        float scalePower = scaleTriggerPower(-gamepad.left_stick_y);

        float steer = scaleSteerPower(gamepad.right_stick_x);
        float leftPower;
        float rightPower;
        if (scalePower == 0.0f) {
            leftPower = steer;
            rightPower = -steer;
        } else {
            leftPower = scalePower * ((steer < 0) ? 1.0f + steer : 1.0f);
            rightPower = scalePower * ((steer > 0) ? 1.0f - steer : 1.0f);
        }

        if(gamepad.a) {
            leftBackMotor.setPower(-leftPower * 1);
            leftFrontMotor.setPower(-leftPower * 1);
            rightBackMotor.setPower(-rightPower * 1);
            rightFrontMotor.setPower(-rightPower * 1);
        } if(gamepad.left_bumper) {
            leftBackMotor.setPower(-leftPower * 0.45);
            leftFrontMotor.setPower(-leftPower * 0.45);
            rightBackMotor.setPower(-rightPower * 0.45);
            rightFrontMotor.setPower(-rightPower * 0.45);
        } if(gamepad.right_bumper) {
            leftBackMotor.setPower(-leftPower * 0.22);
            leftFrontMotor.setPower(-leftPower * 0.22);
            rightBackMotor.setPower(-rightPower * 0.22);
            rightFrontMotor.setPower(-rightPower * 0.22);
        } else {
            leftBackMotor.setPower(-leftPower * 0.7);
            leftFrontMotor.setPower(-leftPower * 0.7);
            rightBackMotor.setPower(-rightPower * 0.7);
            rightFrontMotor.setPower(-rightPower * 0.7);
        }


        leftPowerItem.setValue("%.2f", leftPower);
        rightPowerItem.setValue("%.2f", rightPower);
        steerPowerItem.setValue("%.2f", steer);
        rawPowerItem.setValue("%.2f", scalePower);
    }

    /**
     * The  DC motors are scaled to make it easier to control them at slower speeds
     * The clip method guarantees the value never exceeds the range 0-1.
     */
    private float scaleTriggerPower(float power) {

        // Ensure the values are legal.
        float clipped_power = Range.clip(power, -1, 1);

        // Remember if this is positive or negative
        float sign = Math.signum(clipped_power);

        // Work only with positive numbers for simplicity
        float abs_power = Math.abs(clipped_power);

        // Map the power value [0..1.0] to a power curve index
        int index = (int) (abs_power * (power_curve.length - 1));

        float scaled_power = sign * power_curve[index];

        return scaled_power;
    }

    private float scaleSteerPower(float p_power) {

        // Ensure the values are legal.
        float clipped_power = Range.clip(p_power, -1, 1);

        // Remember if this is positive or negative
        float sign = Math.signum(clipped_power);

        // Work only with positive numbers for simplicity
        float abs_power = Math.abs(clipped_power);

        // Map the power value [0..1.0] to a power curve index
        int index = (int) (abs_power * (steer_curve.length - 1));

        float scaled_power = sign * steer_curve[index];

        return scaled_power;

    }

}
