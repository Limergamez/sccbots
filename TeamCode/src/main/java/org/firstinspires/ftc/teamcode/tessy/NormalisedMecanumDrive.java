package org.firstinspires.ftc.teamcode.tessy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shared.common.RobotComponent;
import org.firstinspires.ftc.teamcode.shared.common.RobotOpMode;
import org.firstinspires.ftc.teamcode.tessy.DualGamePadSteerDriveMecanum;

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
    private DcMotor leftBackMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    private final Telemetry.Item item;
    private boolean showTelemetry = true;


    public NormalisedMecanumDrive(RobotOpMode opmode,
                                  DcMotor leftBackMotor, DcMotor rightBackMotor,
                                  DcMotor leftFrontMotor, DcMotor rightFrontMotor,
                                  boolean showTelemetry) {
        super(opmode);
        this.opMode = opmode;
        this.leftBackMotor = leftBackMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.showTelemetry = showTelemetry;

        if (showTelemetry) {
            item = opMode.telemetry.addData("Mecanum", "Forward: %.2f, Strafe: %0.02f, Rotate: %.2f", speedX, speedY, speedR);
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
            item.setValue("Forward: %.2f, Strafe: %.2f, Rotate: %.2f", speedX, speedY, speedR);
        }
    }

    private void updateNormalized() {
        //calculate motor powers
        double leftBackMotor = speedX + speedY - speedR;
        double frontRightPower = speedX - speedY + speedR;
        double backLeftPower = speedX - speedY - speedR;
        double backRightPower = speedX + speedY + speedR;

        double maxPower = Math.max(Math.abs(leftBackMotor), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        // If the maximum number is greater than 1.0, then normalise by that number
        if (maxPower > 1.0) {
            leftBackMotor = leftBackMotor / maxPower;
            frontRightPower = frontRightPower / maxPower;
            backLeftPower = backLeftPower / maxPower;
            backRightPower = backRightPower / maxPower;
        }

        this.leftBackMotor.setPower(leftBackMotor);
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

        double leftBackMotor = translationValues[0] * scaleFactor + rotationValues[0];
        double frontRightPower = translationValues[1] * scaleFactor + rotationValues[1];
        double backLeftPower = translationValues[2] * scaleFactor + rotationValues[2];
        double backRightPower = translationValues[3] * scaleFactor + rotationValues[3];

        leftBackMotor.setPower(leftBackMotor);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }


    private double clipMotorPower(double p) {
        return Range.clip(p, -1.0, 1.0);
    }
}

