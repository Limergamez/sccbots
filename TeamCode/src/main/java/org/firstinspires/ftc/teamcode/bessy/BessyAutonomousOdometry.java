package org.firstinspires.ftc.teamcode.bessy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Autonomous(name = "Bessy Autonomous with Odometry (goBILDA)", group = "LinearOpMode")
@Disabled
public class BessyAutonomousOdometry extends LinearOpMode {

    GoBildaPinpointDriver odo;

    private DcMotor leftBackMotor, rightBackMotor, leftFrontMotor, rightFrontMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(-90.0, -150.0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        odo.resetPosAndIMU();
        odo.recalibrateIMU();

        double targetX = odo.getPosition().getX(DistanceUnit.MM) + 500;
        double targetY = odo.getPosition().getY(DistanceUnit.MM);

        while (opModeIsActive()) {
            Pose2D pos = odo.getPosition();

            double currentX = pos.getX(DistanceUnit.MM);
            double currentY = pos.getY(DistanceUnit.MM);
            double currentHeading = pos.getHeading(AngleUnit.DEGREES);

            telemetry.addData("Position Information", "X: %.2fmm, Y: %.2fmm\nHeading: %.2f°", currentX, currentY, currentHeading);
            telemetry.update();

            odo.update();

            // Move to target position
            moveToPosition(targetX, targetY);

            // Stop if close to the target position
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double distance = Math.sqrt(errorX * errorX + errorY * errorY);

            if (distance < 10) {
                setMotorPowers(0, 0, 0, 0);  // Stop robot when close to target
                break;
            }
        }
    }

    private void initHardware() {
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    private void moveToPosition(double targetX, double targetY) throws InterruptedException {
        double errorX = targetX - odo.getPosition().getX(DistanceUnit.MM);
        double errorY = targetY - odo.getPosition().getY(DistanceUnit.MM);

        double distance = Math.sqrt(errorX * errorX + errorY * errorY);

        if (distance < 10) {
            setMotorPowers(0, 0, 0, 0);
            return;
        }

        double targetAngle = Math.atan2(errorY, errorX);
        double currentHeading = odo.getPosition().getHeading(AngleUnit.RADIANS);

        // Calculate angle difference and normalize to -π to π
        double angleDifference = targetAngle - currentHeading;
        while (angleDifference > Math.PI) angleDifference -= 2 * Math.PI;
        while (angleDifference < -Math.PI) angleDifference += 2 * Math.PI;

        if (Math.abs(angleDifference) > 0.1) {
            // Rotate towards target angle
            double rotatePower = angleDifference > 0 ? 0.5 : -0.5;
            rotateClockwise(rotatePower, 100);
        } else {
            // Move forward towards the target
            double forwardPower = Math.min(distance * 0.1, 1.0);  // Cap the power to a max value of 1
            moveForward(forwardPower, 100);  // Move forward for a short duration
        }
    }

    public void moveForward(double power, long duration) throws InterruptedException {
        setForwardPower(power);
        sleep(duration);
    }

    public void strafe(double power, long duration) throws InterruptedException {
        setStrafePower(power);
        sleep(duration);
    }

    public void rotateClockwise(double power, long duration) throws InterruptedException {
        setRotationPower(power);
        sleep(duration);
    }

    private void setForwardPower(double power) {
        double leftFrontPower = -power;
        double rightFrontPower = power;
        double leftBackPower = power;
        double rightBackPower = -power;

        setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void setStrafePower(double power) {
        double leftFrontPower = power;
        double rightFrontPower = power;
        double leftBackPower = -power;
        double rightBackPower = -power;

        setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void setRotationPower(double power) {
        double leftFrontPower = power;
        double rightFrontPower = -power;
        double leftBackPower = power;
        double rightBackPower = -power;

        setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void setMotorPowers(double lf, double rf, double lb, double rb) {
        leftFrontMotor.setPower(lf);
        rightFrontMotor.setPower(rf);
        leftBackMotor.setPower(lb);
        rightBackMotor.setPower(rb);
    }
}
