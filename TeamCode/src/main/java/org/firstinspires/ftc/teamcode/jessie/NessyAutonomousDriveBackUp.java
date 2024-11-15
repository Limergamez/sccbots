package org.firstinspires.ftc.teamcode.jeremy;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shared.common.ArmControl;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Nessy Autonomous Encoder Back Up")
public class NessyAutonomousDriveBackUp extends LinearOpMode {
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

        //Autonomous that grabs specimen and hangs
        closeClaw();
        sleep(500);
        moveForward(-0.35, 1550);
        sleep(500);
        useLift(-0.65, 1300);
        sleep(500);
        moveArm(0.25, 250);
        sleep(500);
        useLift(0.4, 800);
        sleep(500);
        openClaw();
        sleep(500);
        moveArm(-0.25, 400);
        sleep(500);
        moveForward(0.3, 1500);
        useLift(0.5, 2000);
        sleep(500);
        // Comes to parking zone, takes specimen off human and places on blue pole
        moveLeft(0.35, 2000);
        moveForward(0.40, 2000);
        moveLeft(0.35, 850);

    }



    public void moveForward(double power, int time) throws InterruptedException {
        centreRight.setPower(power);
        centreLeft.setPower(power + 0.05);
        backRight.setPower(power + 0.05);
        backLeft.setPower(power);
        sleep(time);
        stopMotors();
    }

    public void setupEncoders(double power, int time, double left, double right) {
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centreRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centreLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveWithEncoders(double power, int time, double left, double right) {
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centreRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        centreLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorPosition(backRight, right);
        setMotorPosition(centreRight, right);
        setMotorPosition(backLeft, left);
        setMotorPosition(centreLeft, left);
        backRight.setPower(power);
        centreRight.setPower(power);
        backLeft.setPower(power);
        centreLeft.setPower(power);


        double startTime = System.currentTimeMillis();
        while(startTime + 3000 > System.currentTimeMillis() && (centreRight.isBusy() || centreLeft.isBusy())) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {

            }
        }



    }

    public void setMotorPosition(DcMotor motor, double position) {
        int  currentPosition = motor.getCurrentPosition();
        motor.setTargetPosition(currentPosition + (int)(position * 6));
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    public void useLift(double power, int time) throws InterruptedException {
        liftMotor.setPower(power);
        sleep(time);
        liftMotor.setPower(0);
    }

    public void moveArm(double power, int time) throws InterruptedException {
        armMotor.setPower(power);
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
