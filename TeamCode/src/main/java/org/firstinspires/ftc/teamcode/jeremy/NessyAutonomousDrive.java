package org.firstinspires.ftc.teamcode.jeremy;

import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.shared.common.ArmControl;
import org.firstinspires.ftc.teamcode.shared.common.DualGamePadSteerDrive;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Nessy Autonomous V4")
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

    static final double COUNTS_PER_MOTOR_REV = 537.6; // Counts for a REV motor
    static final double DRIVE_GEAR_REDUCTION = 1.0; // No gear reduction
    static final double WHEEL_DIAMETER_MM = 100.0;  // Wheel diameter in mm (adjust to your wheel size)
    static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);


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

        // Use encoders to move forward a certain distance in mm
        moveWithEncoders(0.5, 500, 500); // Move forward 500 mm
        sleep(500);
    }

    public void moveWithEncoders(double power, double left, double right) {
        // Reset encoders
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centreRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centreLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set target positions for the motors
        int leftTarget = (int) (left * COUNTS_PER_MM);
        int rightTarget = (int) (right * COUNTS_PER_MM);
        backRight.setTargetPosition(rightTarget);
        centreRight.setTargetPosition(rightTarget);
        backLeft.setTargetPosition(leftTarget);
        centreLeft.setTargetPosition(leftTarget);

        // Set to RUN_TO_POSITION mode
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        centreRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        centreLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start moving the motors
        backRight.setPower(power);
        centreRight.setPower(power);
        backLeft.setPower(power);
        centreLeft.setPower(power);

        // Wait for motors to reach their targets
        while (opModeIsActive() && (backRight.isBusy() || centreRight.isBusy() || backLeft.isBusy() || centreLeft.isBusy())) {
            telemetry.addData("Left Position", backLeft.getCurrentPosition());
            telemetry.addData("Right Position", backRight.getCurrentPosition());
            telemetry.update();
            sleep(50);  // Sleep for stability


            // Set motors back to RUN_USING_ENCODER mode
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            centreRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            centreLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}