package org.firstinspires.ftc.teamcode.bessy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Bessy Mecanum Autonomous Rename")
public class AutonomousGoBildaPinpoint extends LinearOpMode {

    // Motor and odometry setup
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private GoBildaPinpointDriver odo;

    private static final double MAX_SPEED = 1.0;

    // Telemetry for motor powers
    private Telemetry.Item leftBackPowerItem;
    private Telemetry.Item rightBackPowerItem;
    private Telemetry.Item leftFrontPowerItem;
    private Telemetry.Item rightFrontPowerItem;

    @Override
    public void runOpMode() {
        // Initialize motors from the hardware map
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");

        // Set motor behaviors
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize odometry
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-84.0, -168.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // Telemetry setup
        leftBackPowerItem = telemetry.addData("Left Back Power", "%.2f", 0.0);
        rightBackPowerItem = telemetry.addData("Right Back Power", "%.2f", 0.0);
        leftFrontPowerItem = telemetry.addData("Left Front Power", "%.2f", 0.0);
        rightFrontPowerItem = telemetry.addData("Right Front Power", "%.2f", 0.0);

        telemetry.addData("Initialization", "Complete");
        telemetry.update();

        waitForStart();
        resetRuntime();

        // Autonomous action sequence
        moveForward(300, 0.25);
    }

    // Move the robot forward for a specified distance
    private void moveForward(double distance, double speed) {
        odo.resetPosAndIMU();  // Reset odometry system to start fresh
        double initialX = odo.getPosition().getX(DistanceUnit.MM);  // Capture initial position for reference
        double targetPosition = initialX + distance;  // Set the target position for movement

        while (opModeIsActive() && odo.getPosition().getX(DistanceUnit.MM) < targetPosition) {
            double leftFrontPower = speed;
            double rightFrontPower = speed;
            double leftBackPower = -speed;
            double rightBackPower = -speed;

            // Normalize motor powers if needed
            normalizeMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            // Set motor powers
            setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

            // Update telemetry for odometry and motor power
            double currentX = odo.getPosition().getX(DistanceUnit.MM);
            telemetry.addData("Current X Position (mm)", currentX);
            telemetry.addData("Target X Position (mm)", targetPosition);
            telemetry.addData("Distance Traveled (mm)", currentX - initialX);
            telemetry.addData("Motor Power", "LeftFront: %.2f, RightFront: %.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Motor Power", "LeftBack: %.2f, RightBack: %.2f", leftBackPower, rightBackPower);
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
