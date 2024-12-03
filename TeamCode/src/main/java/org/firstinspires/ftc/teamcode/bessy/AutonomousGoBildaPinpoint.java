package org.firstinspires.ftc.teamcode.bessy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;

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
        moveForward(30, 0.5);  // Move forward 30 inches at 50% speed
        rotate(90, 0.5);       // Rotate 90 degrees at 50% speed
        strafeRight(20, 0.5);  // Strafe right 20 inches at 50% speed
        moveForward(40, 0.5);  // Move forward another 40 inches at 50% speed
    }

    // Move the robot forward for a specified distance
    private void moveForward(double distance, double speed) {
        odo.resetPosAndIMU();
        double targetPosition = odo.getPosition().getX(DistanceUnit.INCH) + distance;

        while (opModeIsActive() && odo.getPosition().getX(DistanceUnit.INCH) < targetPosition) {
            setMotorPowers(speed, speed, -speed, -speed);
            telemetry.addData("Moving Forward", "Distance: %.2f", odo.getPosition().getX(DistanceUnit.INCH));
            telemetry.update();
        }
        stopMotors();
    }

    // Rotate the robot to a specified angle
    private void rotate(double angle, double speed) {
        double targetHeading = odo.getPosition().getHeading(AngleUnit.DEGREES) + angle;

        while (opModeIsActive() && Math.abs(odo.getPosition().getHeading(AngleUnit.DEGREES) - targetHeading) > 2) {
            setMotorPowers(-speed, speed, -speed, speed);  // Rotation logic
            telemetry.addData("Rotating", "Angle: %.2f", odo.getPosition().getHeading(AngleUnit.DEGREES));
            telemetry.update();
        }
        stopMotors();
    }

    // Strafe right for a specified distance
    private void strafeRight(double distance, double speed) {
        odo.resetPosAndIMU();
        double targetPosition = odo.getPosition().getY(DistanceUnit.INCH) + distance;

        while (opModeIsActive() && odo.getPosition().getY(DistanceUnit.INCH) < targetPosition) {
            setMotorPowers(speed, -speed, -speed, speed);
            telemetry.addData("Strafing Right", "Distance: %.2f", odo.getPosition().getY(DistanceUnit.INCH));
            telemetry.update();
        }
        stopMotors();
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
