package org.firstinspires.ftc.teamcode.jessie;
// This imports a bunch of pre programmed codes that allows me to more easily work on programming our teams robot
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.shared.common.ArmControl;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Jessie Autonomous V3")
public class JessieAutonomousDrive extends LinearOpMode {
    // Movement Motors -- This basically sets up our main component motors which allows my script to control them
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
        // Initialize motors -- This ensures my code understands what each motor is and that our drivers hub can map
        // them to our robot so it knows what motors to control
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

        waitForStart(); // This basically means the code is ready to be used and is waiting for my further intructions below

        //Autonomous that grabs specimen and hangs -- Below are a set of intructions for what I want the robot to do
        closeClaw(); // This means that the claw will close (self explanatory)
        sleep(500); // This means to wait before executing the next step - in this case its waiting half of a second
        moveForward(-0.35, 1550); // This means our robot will move forwards, the power adjusts the speed and also
        // if we go back or forwards -- The time similar to the wait is telling the code how long to move forward for (1.55 seconds)
        sleep(500);
        useLift(-0.65, 1300); // This means to use the lift again as the forward command it moves up or down
        sleep(500);
        moveArm(0.25, 250); // Similar to the lift command however it controls moving the arm back and forth based on the power set
        sleep(500);
        useLift(0.4, 800);
        sleep(500);
        openClaw(); // Opens the claw
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
        moveArm(-0.25, 350);
        openClaw();
        sleep(300);
        closeClaw();
        moveArm(0.25, 350);

    }



    public void moveForward(double power, int time) throws InterruptedException {
        // This setups what each command is. This one sets up moving forward and backwards for the robot
        // So when we want to move the robot forwards or backwards we can easily just type moveForward then set the (power) and (time)
        centreRight.setPower(power);
        centreLeft.setPower(power + 0.05);
        backRight.setPower(power + 0.05);
        backLeft.setPower(power);
        sleep(time);
        stopMotors();
    }

    public void moveLeft(double power, int time) throws InterruptedException {
        // This is set our the same as move forward. This is applied to every movement in autonomous that we want it to perform
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
