package org.firstinspires.ftc.teamcode.bessy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@Autonomous(name="goBILDA® PinPoint Odometry Example", group="Linear OpMode")
@Disabled
public class SensorGoBildaPinpointExample extends LinearOpMode {

    GoBildaPinpointDriver odo;
    private DcMotor leftBackMotor, rightBackMotor, leftFrontMotor, rightFrontMotor;

    double oldTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(-90.0, -165.0);
        odo.setEncoderResolution(13.26291192);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();


        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        waitForStart();
        resetRuntime();
        odo.resetPosAndIMU();
        odo.recalibrateIMU();
        initHardware();

        double targetX = 0;
        double targetY = 0;

        while (opModeIsActive()) {
            odo.update();
            boolean targetReached = moveToPosition(500, 0);
            sleep(2000);
            moveToPosition(-500, 0);


            if (targetReached) {
                telemetry.addData("Status", "Target Reached!");
                telemetry.update();
                break;
            }

            double newTime = getRuntime();
            double loopTime = newTime-oldTime;
            double frequency = 1/loopTime;
            oldTime = newTime;

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "X: %.3fmm, Y: %.3fmm\n Heading: %.3f°",
                    pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();
            Pose2D vel = odo.getVelocity();
            String velocity = String.format(Locale.US,"XVel: %.3fmm, YVel: %.3fmm\n HVel: %.3f°", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Velocity", velocity);


            /*
            Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
            READY: the device is working as normal
            CALIBRATING: the device is calibrating and outputs are put on hold
            NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
            FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
            FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
            FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
            */
            telemetry.addData("Status", odo.getDeviceStatus());

            telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

            telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
            telemetry.update();
        }

        setMotorPowers(0, 0, 0, 0);
    }

    private boolean moveToPosition(double targetX, double targetY) throws InterruptedException {
        double errorX = targetX - odo.getPosition().getX(DistanceUnit.MM);
        double errorY = targetY - odo.getPosition().getY(DistanceUnit.MM);

        double distance = Math.sqrt(errorX * errorX + errorY * errorY);

        if (distance < 20) {
            setMotorPowers(0, 0, 0, 0);
            return true;
        }

        double targetAngle = Math.atan2(errorY, errorX);
        double currentHeading = odo.getPosition().getHeading(AngleUnit.RADIANS);

        double angleDifference = targetAngle - currentHeading;
        while (angleDifference > Math.PI) angleDifference -= 2 * Math.PI;
        while (angleDifference < -Math.PI) angleDifference += 2 * Math.PI;

        if (Math.abs(angleDifference) > 0.1) {
            double rotatePower = angleDifference > 0 ? 0.5 : -0.5;
            stopMotors(0);
            //rotateClockwise(rotatePower, 100);
        } else {
            double forwardPower = Math.min(distance * 0.1, 1.0);
            moveForward(forwardPower, 100);
        }

        return false;
    }

    public void moveForward(double power, long duration) throws InterruptedException {
        setForwardPower(0.2);
        sleep(duration);
    }

    public void strafe(double power, long duration) throws InterruptedException {
        setStrafePower(power / 2);
        sleep(duration);
    }

    public void rotateClockwise(double power, long duration) throws InterruptedException {
        setRotationPower(power / 2);
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
    private void stopMotors(double power) {
        double leftFrontPower = 0;
        double rightFrontPower = 0;
        double leftBackPower = 0;
        double rightBackPower = 0;

        setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void setMotorPowers(double lf, double rf, double lb, double rb) {
        leftFrontMotor.setPower(lf);
        rightFrontMotor.setPower(rf);
        leftBackMotor.setPower(lb);
        rightBackMotor.setPower(rb);
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
}
