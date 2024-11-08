package org.firstinspires.ftc.teamcode.jeremy;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "JeremyAuto")
public class JeremyAutonomousDrive extends LinearOpMode {
    // Movement Motors
    protected DcMotor centreRight;
    protected DcMotor backRight;
    protected DcMotor centreLeft;
    protected DcMotor backLeft;

    @Override
    public void runOpMode() throws InterruptedException {
        // Movement motors
        centreRight = hardwareMap.get(DcMotor.class, "centreRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight"); // Fixed typo
        centreLeft = hardwareMap.get(DcMotor.class, "centreLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        // Set motor directions
        centreRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Autonomous Actions
        moveForward(1, 1000);
        sleep(100);
        moveForward(1, 1000);
    }

    public void moveForward(int power, int time) throws InterruptedException {
        // Autonomous Move Forward Code
        centreRight.setPower(power);
        centreLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);

        // Move forward for set time
        sleep(time);

        // Stop the motors
        centreRight.setPower(0);
        centreLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
}
