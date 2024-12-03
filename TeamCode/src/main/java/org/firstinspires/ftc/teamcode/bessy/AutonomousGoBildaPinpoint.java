package org.firstinspires.ftc.teamcode.bessy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Bessy Mecanum Autonomous")

public class AutonomousGoBildaPinpoint extends LinearOpMode {

    // Motor and Odometry setup
    private final DcMotor leftBackMotor;
    private final DcMotor rightBackMotor;
    private final DcMotor leftFrontMotor;
    private final DcMotor rightFrontMotor;
    private GoBildaPinpointDriver odo;

    private static final double MAX_SPEED = 1.0;

    // Telemetry for motor powers
    private Telemetry.Item leftBackPowerItem;
    private Telemetry.Item rightBackPowerItem;
    private Telemetry.Item leftFrontPowerItem;
    private Telemetry.Item rightFrontPowerItem;

    public AutonomousGoBildaPinpoint(RobotOpMode opMode,
                                     DcMotor leftBackMotor,
                                     DcMotor rightBackMotor,
                                     DcMotor leftFrontMotor,
                                     DcMotor rightFrontMotor) {
        this.leftBackMotor = leftBackMotor;
        this.rightBackMotor = rightBackMotor;
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void runOpMode() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        leftBackPowerItem = telemetry.addData("Left Back Power", "%.2f", 0.0);
        rightBackPowerItem = telemetry.addData("Right Back Power", "%.2f", 0.0);
        leftFrontPowerItem = telemetry.addData("Left Front Power", "%.2f", 0.0);
        rightFrontPowerItem = telemetry.addData("Right Front Power", "%.2f", 0.0);

        waitForStart();
        resetRuntime();

        // Autonomous action sequence
        moveForward(300, 0.5);
/*        rotate(90, 0.5);
        strafeRight(200, 0.5);
        moveForward(400, 0.5);*/
    }

    // Move the robot forward for a specified distance
    private void moveForward(double distance, double speed) {
        odo.resetPosAndIMU();
        double targetPosition = odo.getPosition().getX(DistanceUnit.MM) + distance; // Use MM here

        while (opModeIsActive() && odo.getPosition().getX(DistanceUnit.MM) < targetPosition) { // Use MM here
            double leftFrontPower = speed;
            double rightFrontPower = speed;
            double leftBackPower = -speed;
            double rightBackPower = -speed;

            // Normalize motor powers if needed
            normalizeMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.addData("Moving Forward", "Distance: %.2f", odo.getPosition().getX(DistanceUnit.MM)); // Use MM here
            telemetry.update();
        }
        stopMotors();
    }

    // Rotate the robot to a specified angle
    private void rotate(double angle, double speed) {
        double targetHeading = odo.getPosition().getHeading(AngleUnit.DEGREES) + angle;

        while (opModeIsActive() && Math.abs(odo.getPosition().getHeading(AngleUnit.DEGREES) - targetHeading) > 2) {
            double leftFrontPower = -speed;
            double rightFrontPower = speed;
            double leftBackPower = -speed;
            double rightBackPower = speed;

            // Normalize motor powers if needed
            normalizeMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.addData("Rotating", "Angle: %.2f", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        stopMotors();
    }

    // Strafe right for a specified distance
    private void strafeRight(double distance, double speed) {
        odo.resetPosAndIMU();
        double targetPosition = odo.getPosition().getY(DistanceUnit.MM) + distance; // Use MM here

        while (opModeIsActive() && odo.getPosition().getY(DistanceUnit.MM) < targetPosition) { // Use MM here
            double leftFrontPower = speed;
            double rightFrontPower = -speed;
            double leftBackPower = -speed;
            double rightBackPower = speed;

            // Normalize motor powers if needed
            normalizeMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            telemetry.addData("Strafing Right", "Distance: %.2f", odo.getPosition().getY(DistanceUnit.MM)); // Use MM here
            telemetry.update();
        }
        stopMotors();
    }

    // Normalize motor powers to ensure none exceed the maximum power
    private void normalizeMotorPowers(double leftFrontPower, double rightFrontPower,
                                      double leftBackPower, double rightBackPower) {
        double maxPower = Math.max(1.0, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        leftFrontPower /= maxPower;
        rightFrontPower /= maxPower;
        leftBackPower /= maxPower;
        rightBackPower /= maxPower;
    }

    // Set motor powers for Mecanum drive
    private void setMotorPowers(double leftFrontPower, double rightFrontPower,
                                double leftBackPower, double rightBackPower) {
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    // Stop all motors
    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }
}
