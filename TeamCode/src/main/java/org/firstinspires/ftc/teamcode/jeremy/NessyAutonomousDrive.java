package org.firstinspires.ftc.teamcode.jeremy;

import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "NessyAutonomous")
public class NessyAutonomousDrive extends LinearOpMode {
    // Movement Motors
    protected DcMotor centreRight;
    protected DcMotor backRight;
    protected DcMotor centreLeft;
    protected DcMotor backLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        // Movement motors
        centreRight = hardwareMap.get(DcMotor.class, "centreRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        centreLeft = hardwareMap.get(DcMotor.class, "centreLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        // Setup motor directions
        centreRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Autonomous Code
        moveForward(1, 400);
        sleep(500);
        moveForward(1, 100);
        sleep(500);
        moveLeft(1, 400);
        sleep(500);
        moveRight(1, 400);
    }

    public void moveForward(int power, int time) throws InterruptedException {
        centreRight.setPower(power);
        centreLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);

        // Move forward for the set time
        sleep(time);

        // Stop the motors
        stopMotors();
    }

    public void moveLeft(int power, int time) throws InterruptedException {
        // Setting -Power can help steer each motor
        centreRight.setPower(-power);
        backRight.setPower(-power);
        centreLeft.setPower(power);
        backLeft.setPower(power);

        sleep(time);

        stopMotors();
    }

    public void moveRight(int power, int time) throws InterruptedException {
        centreRight.setPower(power);
        backRight.setPower(power);
        centreLeft.setPower(-power);
        backLeft.setPower(-power);

        sleep(time);

        stopMotors();
    }

    public void stopMotors() {
        centreRight.setPower(0);
        centreLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
}
