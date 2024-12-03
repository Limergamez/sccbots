package org.firstinspires.ftc.teamcode.bessy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shared.common.RobotComponent;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;

public class DualGamePadSteerDriveMecanum extends RobotComponent {

    private final DcMotor leftBackMotor;
    private final DcMotor rightBackMotor;
    private final DcMotor leftFrontMotor;
    private final DcMotor rightFrontMotor;
    private final Gamepad gamepad;
    private final Telemetry.Item leftBackPowerItem;
    private final Telemetry.Item rightBackPowerItem;
    private final Telemetry.Item leftFrontPowerItem;
    private final Telemetry.Item rightFrontPowerItem;

    private static final double SLOW_MODE_MULTIPLIER = 0.5;

    public DualGamePadSteerDriveMecanum(RobotOpMode opMode, Gamepad gamepad,
                                        DcMotor leftBackMotor,
                                        DcMotor rightBackMotor,
                                        DcMotor leftFrontMotor,
                                        DcMotor rightFrontMotor) {
        super(opMode);
        this.gamepad = gamepad;
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBackPowerItem = getOpMode().telemetry.addData("Left Back Power", "%.2f", 0.0);
        rightBackPowerItem = getOpMode().telemetry.addData("Right Back Power", "%.2f", 0.0);
        leftFrontPowerItem = getOpMode().telemetry.addData("Left Front Power", "%.2f", 0.0);
        rightFrontPowerItem = getOpMode().telemetry.addData("Right Front Power", "%.2f", 0.0);
    }

    public void update() {
        double forward = -gamepad.left_stick_y;
        double rotate = gamepad.right_stick_x;
        double strafe = gamepad.left_stick_x;

        double speedMultiplier = gamepad.right_bumper ? SLOW_MODE_MULTIPLIER : 1.0;

        // Set power for forward/backward movement
        double leftFrontPower = forward * speedMultiplier;
        double rightFrontPower = forward * speedMultiplier;
        double leftBackPower = -forward * speedMultiplier;
        double rightBackPower = -forward * speedMultiplier;

        // Strafe: Moves the robot sideways
        leftFrontPower += strafe * speedMultiplier;
        rightFrontPower -= strafe * speedMultiplier;
        leftBackPower -= strafe * speedMultiplier;
        rightBackPower += strafe * speedMultiplier;

        /*/ Rotation: Rotate the robot by adjusting motor powers
       The -= is to ensure that all wheels are moving together for a proper rotation /*/
        leftFrontPower -= rotate * speedMultiplier;
        rightFrontPower -= rotate * speedMultiplier;
        leftBackPower -= rotate * speedMultiplier;
        rightBackPower -= rotate * speedMultiplier;

        double maxPower = Math.max(1.0, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        leftFrontPower /= maxPower;
        rightFrontPower /= maxPower;
        leftBackPower /= maxPower;
        rightBackPower /= maxPower;

        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);

        leftBackPowerItem.setValue("%.2f", leftBackPower);
        rightBackPowerItem.setValue("%.2f", rightBackPower);
        leftFrontPowerItem.setValue("%.2f", leftFrontPower);
        rightFrontPowerItem.setValue("%.2f", rightFrontPower);
    }
}
