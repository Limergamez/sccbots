package org.firstinspires.ftc.teamcode.tessy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Tessy Encoder-Based Odometry Autonomous")
@Disabled
public class TessyAutonomousDriveMecanumOdometry extends LinearOpMode {

    protected DcMotor leftBackMotor;
    protected DcMotor rightBackMotor;
    protected DcMotor leftFrontMotor;
    protected DcMotor rightFrontMotor;

    private static final double TICKS_PER_REV = 537.7; // Verify for your motors
    private static final double WHEEL_DIAMETER_MM = 104; // Verify for your wheels
    private static final double MM_PER_TICK = (Math.PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV;

    private double xPosition = 0;
    private double yPosition = 0;
    private double lastLeftEncoder = 0;
    private double lastRightEncoder = 0;
    private double lastHorizontalEncoder = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");

        // Set motor directions
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behavior
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        // Wait for start
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Example movement: 50 mm left, 1000 mm forward
            moveToPosition(50, 1000, 0.5);
        }
    }

    private void moveToPosition(double targetX, double targetY, double power) {
        // Move along the X-axis (strafe)
        moveAlongAxis(targetX - xPosition, true, power);

        // Move along the Y-axis (forward/backward)
        moveAlongAxis(targetY - yPosition, false, power);

        stopMotors(); // Ensure motors are stopped at the end
    }

    private void moveAlongAxis(double distance, boolean isStrafe, double power) {
        double direction = distance > 0 ? 1 : -1; // Determine movement direction
        double targetDistance = Math.abs(distance);

        while (opModeIsActive() && targetDistance > 5) { // Threshold in mm
            updateOdometry();

            double currentDistance = isStrafe ? Math.abs(xPosition) : Math.abs(yPosition);
            targetDistance = Math.abs(distance) - currentDistance;

            telemetry.addData("Target Distance", targetDistance);
            telemetry.addData("X Position", xPosition);
            telemetry.addData("Y Position", yPosition);
            telemetry.update();

            if (targetDistance < 5) {
                stopMotors();
                break;
            }

            double movementPower = direction * power;

            if (isStrafe) {
                setStrafePower(movementPower);
            } else {
                setForwardPower(movementPower);
            }
        }
    }

    private void setForwardPower(double forwardPower) {
        setMotorPowers(-forwardPower, forwardPower, forwardPower, -forwardPower);
    }

    private void setStrafePower(double strafePower) {
        setMotorPowers(strafePower, strafePower, -strafePower, -strafePower);
    }

    private void setMotorPowers(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        double maxPower = Math.max(1.0, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        leftFrontMotor.setPower(leftFrontPower / maxPower);
        rightFrontMotor.setPower(rightFrontPower / maxPower);
        leftBackMotor.setPower(leftBackPower / maxPower);
        rightBackMotor.setPower(rightBackPower / maxPower);
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    private void resetEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void updateOdometry() {
        double leftEncoder = (leftBackMotor.getCurrentPosition() + leftFrontMotor.getCurrentPosition()) / 2.0;
        double rightEncoder = (rightBackMotor.getCurrentPosition() + rightFrontMotor.getCurrentPosition()) / 2.0;
        double horizontalEncoder = (leftFrontMotor.getCurrentPosition() - rightFrontMotor.getCurrentPosition()) / 2.0;

        double leftDelta = (leftEncoder - lastLeftEncoder) * MM_PER_TICK;
        double rightDelta = (rightEncoder - lastRightEncoder) * MM_PER_TICK;
        double horizontalDelta = (horizontalEncoder - lastHorizontalEncoder) * MM_PER_TICK;

        double deltaX = horizontalDelta;
        double deltaY = (leftDelta + rightDelta) / 2.0;

        xPosition += deltaX;
        yPosition += deltaY;

        lastLeftEncoder = leftEncoder;
        lastRightEncoder = rightEncoder;
        lastHorizontalEncoder = horizontalEncoder;
    }
}
