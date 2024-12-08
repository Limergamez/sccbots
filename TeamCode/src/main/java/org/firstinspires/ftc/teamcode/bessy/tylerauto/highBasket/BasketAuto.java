package org.firstinspires.ftc.teamcode.bessy.tylerauto.highBasket;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shared.common.ArmControl;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Bessy High Basket Auto")
public class BasketAuto extends LinearOpMode {

    protected DcMotor leftBackMotor;
    protected DcMotor rightBackMotor;
    protected DcMotor leftFrontMotor;
    protected DcMotor rightFrontMotor;
    protected Servo clawServo;
    protected DcMotor liftMotor;
    protected DcMotor armMotor;
    protected ArmControl armControl;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armControl = new ArmControl(null, null, armMotor);

        waitForStart();

        double desiredVoltage = 12.5;
        //double nominalVoltage = 12.0;
        double currentVoltage = hardwareMap.voltageSensor.get("Expansion Hub 1").getVoltage();
        double compensationFactor = desiredVoltage / currentVoltage;

        double targetPower = 1;
        double compensatedPower = targetPower * compensationFactor;

        //armMotor.setPower(compensatedPower);
        //liftMotor.setPower(compensatedPower);
        leftBackMotor.setPower(compensatedPower);
        leftFrontMotor.setPower(compensatedPower);
        rightBackMotor.setPower(compensatedPower);
        rightFrontMotor.setPower(compensatedPower);

        // Just simply strafes to the left which gives us 1 point :)
        moveForward(0.5, 800);
        rotateClockwise(0.5,300);
        moveArm(0.2, 100);
        useLift(1,600);
    }

    public void moveForward(double power, long duration) throws InterruptedException {
        setForwardPower(power);
        sleep(duration);
        stopMotors();
    }

    public void strafe(double power, long duration) throws InterruptedException {
        setStrafePower(power);
        sleep(duration);
        stopMotors();
    }

    public void rotateClockwise(double power, long duration) throws InterruptedException {
        setRotationPower(power);
        sleep(duration);
        stopMotors();
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

    private void stopMotors() {
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }

    public void openClaw() {
        clawServo.setPosition(1.0);
    }

    public void closeClaw() {
        clawServo.setPosition(0.0);
    }

    public void useLift(double power, int time) throws InterruptedException {
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setPower(power);

        sleep(time);

        liftMotor.setPower(0.2);
    }


    public void moveArm(double power, int time) throws InterruptedException {
        armMotor.setPower(power);
        sleep(time);
        armMotor.setPower(0);
    }
}
