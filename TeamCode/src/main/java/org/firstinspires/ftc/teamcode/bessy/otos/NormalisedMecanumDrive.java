package org.firstinspires.ftc.teamcode.bessy.otos;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shared.common.RobotComponent;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;

public class NormalisedMecanumDrive extends RobotComponent {

    public enum MecanumDriveMode {
        NORMALIZED, ROTATION_PRIORITY_NORMALIZED
    }

    private MecanumDriveMode driveMode = MecanumDriveMode.NORMALIZED;

    // X - forwards/backwards direction - positive in the forward direction
    // Y - right/right direction - positive in the right direction
    // R - Rotation - positive clockwise
    private double speedX = 0.0;
    private double speedY = 0.0;
    private double speedR = 0.0;

    private RobotOpMode opMode;
    private final DcMotor frontLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor backRightMotor;

    private final Telemetry.Item item;
    private boolean showTelemetry = true;


    public NormalisedMecanumDrive(RobotOpMode opmode,
                                  DcMotor frontLeftMotor,
                                  DcMotor frontRightMotor,
                                  DcMotor backLeftMotor,
                                  DcMotor backRightMotor,
                                  boolean showTelemetry) {
        super(opmode);
        this.opMode = opmode;
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.showTelemetry = showTelemetry;
//
//        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        if (showTelemetry) {
            item = opMode.telemetry.addData("Mecanum", "Forward: %4.2f, Strafe: %4.2f, Rotate: %4.2f", speedX, speedY, speedR);
            item.setRetained(true);
        } else {
            item = null;
        }
    }

    public void setDriveMode(MecanumDriveMode mode) {
        driveMode = mode;
    }

    public void setSpeedXYR(double speedX, double speedY, double speedR) {
        this.speedX = clipMotorPower(speedX);
        this.speedY = clipMotorPower(speedY);
        this.speedR = clipMotorPower(speedR);
    }

    public void setSpeedPolarR(double speed, double direction, double speedR) {
        double radians = Math.toRadians(direction);
        this.speedX = clipMotorPower(speed * Math.cos(radians));
        this.speedY = clipMotorPower(speed * Math.sin(radians));
        this.speedR = clipMotorPower(speedR);
    }

    /**
     * Update motors with latest state
     */
    public void update() {
        switch (driveMode) {
            case NORMALIZED:
                updateNormalized();
                break;
            case ROTATION_PRIORITY_NORMALIZED:
                rotationPriorityNormalized();
                break;
        }
        if (item != null) {
            item.setValue("Forward: %4.2f, Strafe: %4.2f, Rotate: %4.2f", speedX, speedY, speedR);
        }
    }

    private void updateNormalized() {
        //calculate motor powers
        double frontLeftPower = speedX + speedY - speedR;
        double frontRightPower = speedX - speedY + speedR;
        double backLeftPower = speedX - speedY - speedR;
        double backRightPower = speedX + speedY + speedR;

//        double px = speedX;
//        double py = speedY;
//        double pa = speedR;
//
//        double p1 = -px + py - pa;
//        double p2 = px + py + -pa;
//        double p3 = -px + py + pa;
//        double p4 = px + py + pa;

//
//        DcMotor m1 = hardwareMap.dcMotor.get("back_left_motor");
//        DcMotor m2 = hardwareMap.dcMotor.get("front_left_motor");
//        DcMotor m3 = hardwareMap.dcMotor.get("front_right_motor");
//        DcMotor m4 = hardwareMap.dcMotor.get("back_right_motor");
////
//        double max = Math.max(1.0, Math.abs(p1));
//        max = Math.max(max, Math.abs(p2));
//        max = Math.max(max, Math.abs(p3));
//        max = Math.max(max, Math.abs(p4));
//        p1 /= max;
//        p2 /= max;
//        p3 /= max;
//        p4 /= max;
//        m1.setPower(p1);
//        m2.setPower(p2);
//        m3.setPower(p3);
//        m4.setPower(p4);

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        // If the maximum number is greater than 1.0, then normalise by that number
        if (maxPower > 1.0) {
            frontLeftPower = frontLeftPower / maxPower;
            frontRightPower = frontRightPower / maxPower;
            backLeftPower = backLeftPower / maxPower;
            backRightPower = backRightPower / maxPower;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * Calculate rotational speed first, and use remaining headway for translation.
     */
    private void rotationPriorityNormalized() {
        //calculate motor powers
        double translationValues[] = {
                speedX + speedY,
                speedX - speedY,
                speedX - speedY,
                speedX + speedY};

        double rotationValues[] = {
                -speedR,
                speedR,
                -speedR,
                speedR};

        double scaleFactor = 1.0;
        double tmpScale = 1.0;

        // Solve this equation backwards:
        // MotorX = TranslationX * scaleFactor + RotationX
        // to find scaleFactor that ensures -1 <= MotorX <= 1 and 0 < scaleFactor <= 1
        for (int i = 0; i < 4; i++) {
            if (Math.abs(translationValues[i] + rotationValues[i]) > 1) {
                tmpScale = (1 - rotationValues[i]) / translationValues[i];
            } else if (translationValues[i] + rotationValues[i] < -1) {
                tmpScale = (rotationValues[i] - 1) / translationValues[i];
            }
            if (tmpScale < scaleFactor) {
                scaleFactor = tmpScale;
            }
        }

        double frontLeftPower = translationValues[0] * scaleFactor + rotationValues[0];
        double frontRightPower = translationValues[1] * scaleFactor + rotationValues[1];
        double backLeftPower = translationValues[2] * scaleFactor + rotationValues[2];
        double backRightPower = translationValues[3] * scaleFactor + rotationValues[3];

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }


    private double clipMotorPower(double p) {
        return Range.clip(p, -1.0, 1.0);
    }
}

