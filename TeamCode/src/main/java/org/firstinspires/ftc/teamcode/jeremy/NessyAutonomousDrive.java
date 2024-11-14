package org.firstinspires.ftc.teamcode.jeremy;

import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.shared.common.ArmControl;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "NessyAutonomousV2")
public class NessyAutonomousDrive extends LinearOpMode {
    // Movement Motors
    protected DcMotor centreRight;
    protected DcMotor backRight;
    protected DcMotor centreLeft;
    protected DcMotor backLeft;

    // Claw Servo, Lift Motor, and Arm Motor
    protected Servo clawServo;
    protected DcMotor liftMotor;
    protected DcMotor armMotor;
    protected ArmControl armControl;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        centreRight = hardwareMap.get(DcMotor.class, "centreRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        centreLeft = hardwareMap.get(DcMotor.class, "centreLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        // Set motor directions
        centreRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        armControl = new ArmControl(null, null, armMotor);

        waitForStart();

        // Autonomous sequence
        sleep(1000);
        closeClaw();
        sleep(500);
        moveForward(-0.30, 2000);
        raiseLift(1.0, 2000);
        openClaw();
        sleep(3000);
        lowerLift(0.5, 2000);
        moveArmDown(0.37, 1000);
    }

    public void moveForward(double power, int time) throws InterruptedException {
        centreRight.setPower(power);
        centreLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
        sleep(time);
        stopMotors();
    }

    public void moveLeft(double power, int time) throws InterruptedException {
        centreRight.setPower(-power);
        backRight.setPower(-power);
        centreLeft.setPower(power);
        backLeft.setPower(power);
        sleep(time);
        stopMotors();
    }

    public void moveRight(double power, int time) throws InterruptedException {
        centreRight.setPower(power);
        backRight.setPower(power);
        centreLeft.setPower(-power);
        backLeft.setPower(-power);
        sleep(time);
        stopMotors();
    }

    public void openClaw() {
        clawServo.setPosition(1.0);
    }
    public void closeClaw() {
        clawServo.setPosition(0.0);
    }

    public void raiseLift(double power, int time) throws InterruptedException {
        liftMotor.setPower(-power);
        sleep(time);
        liftMotor.setPower(0);
    }
    public void lowerLift(double power, int time) throws InterruptedException {
        liftMotor.setPower(power);
        sleep(time);
        liftMotor.setPower(0);
    }

    public void moveArmUp(double power, int time) throws InterruptedException {
        armMotor.setPower(power);
        sleep(time);
        armMotor.setPower(0);
    }
    public void moveArmDown(double power, int time) throws InterruptedException {
        armMotor.setPower(-power);
        sleep(time);
        armMotor.setPower(0);
    }

    public void stopMotors() {
        centreRight.setPower(0);
        centreLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
}
