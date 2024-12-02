package org.firstinspires.ftc.teamcode.tessy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Tessy Odometry Autonomous")
@Disabled
public class TessyAutonomousDriveMecanumOdometry extends LinearOpMode {

    protected DcMotor leftBackMotor;
    protected DcMotor rightBackMotor;
    protected DcMotor leftFrontMotor;
    protected DcMotor rightFrontMotor;

    private static final double TICKS_PER_REV = 8192;
    private static final double WHEEL_DIAMETER_MM = 96;
    private static final double MM_PER_TICK = (Math.PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV;

    private double xPosition = 0;
    private double yPosition = 0;
    private double lastLeftEncoder = 0;
    private double lastRightEncoder = 0;
    private double lastHorizontalEncoder = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        xPosition = 0;
        yPosition = 0;
        /* So this sets our starting position on the field. Basically if we marked middle of
        field as starting then we set positions above to 0 so it knows that's 0.
        To move X strafes the robot, while Y moves forwards and back. If we wanted to move 50 cm
        to the left then we call in moveToPosition: moveToPosition(500, 0, 0.5); */

        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");

        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        waitForStart();

        moveToPosition(500, 1000, 0.5); // This should move 50 mm to left and 1000 mm forward.
    }

    private void moveToPosition(double targetX, double targetY, double power) {
        while (opModeIsActive()) {
            updateOdometry();

            double deltaX = targetX - xPosition;
            double deltaY = targetY - yPosition;

            if (Math.abs(deltaX) < 10 && Math.abs(deltaY) < 10) {
                stopMotors();
                break;
            }

            double strafePower = deltaX > 0 ? power : -power;
            double forwardPower = deltaY > 0 ? power : -power;

            setForwardPower(forwardPower);
            setStrafePower(strafePower);
        }
    }

    private void setForwardPower(double forwardPower) {
        double leftFrontPower = -forwardPower;
        double rightFrontPower = forwardPower;
        double leftBackPower = forwardPower;
        double rightBackPower = -forwardPower;

        setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void setStrafePower(double strafepower) {
        double leftFrontPower = + strafepower;
        double rightFrontPower = + strafepower;
        double leftBackPower = - strafepower;
        double rightBackPower = - strafepower;

        setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

    }

    public void rotateClockwise(double power, long duration) throws InterruptedException {
        setRotationPower(power);
        sleep(duration);
        stopMotors();
    }

    private void setMotorPowers(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
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
    }
    private void setRotationPower(double power) {
        double leftFrontPower = power;
        double rightFrontPower = -power;
        double leftBackPower = power;
        double rightBackPower = -power;

        setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }


    private void stopMotors() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
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

        double deltaX = (leftDelta + rightDelta) / 2.0;
        double deltaY = horizontalDelta;

        xPosition += deltaX;
        yPosition += deltaY;

        lastLeftEncoder = leftEncoder;
        lastRightEncoder = rightEncoder;
        lastHorizontalEncoder = horizontalEncoder;
    }
}
