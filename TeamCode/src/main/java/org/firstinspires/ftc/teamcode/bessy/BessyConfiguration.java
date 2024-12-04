package org.firstinspires.ftc.teamcode.bessy;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tessy.TessyConfiguration;

public class BessyConfiguration {
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public SparkFunOTOS odometry;
    public DcMotor liftMotor;
    public DcMotor climbMotor;
    public Servo clawServo;
    public Servo rodHarvesterServo;
    public CRServo spinHarvesterServo;
    public Servo rotateHarvesterServo;
    public DcMotor armMotor;

    public static TessyConfiguration newConfig(HardwareMap hardwareMap, Telemetry telemetry) {
        TessyConfiguration config = new TessyConfiguration();

        config.leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        config.rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        config.leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        config.rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        config.odometry = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        config.liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        config.climbMotor = hardwareMap.get(DcMotor.class, "climbMotor");
        config.clawServo = hardwareMap.get(Servo.class, "clawServo");
        //config.harvesterServo = hardwareMap.get(Servo.class, "harvesterServo");
        config.armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        telemetry.addData("Left Front Motor", config.leftFrontMotor.getConnectionInfo());
        telemetry.addData("Right Front Motor", config.rightFrontMotor.getConnectionInfo());
        telemetry.addData("Left Back Motor", config.leftBackMotor.getConnectionInfo());
        telemetry.addData("Right Back Motor", config.rightBackMotor.getConnectionInfo());
        telemetry.addData("Lift Motor", config.liftMotor.getConnectionInfo());
        telemetry.addData("Climb Motor", config.climbMotor.getConnectionInfo());
        telemetry.addData("Claw Servo", config.clawServo.getConnectionInfo());
        // telemetry.addData("Harvester Servo", config.harvesterServo.getConnectionInfo());
        telemetry.addData("Arm Motor", config.armMotor.getConnectionInfo());

        telemetry.update();

        return config;
    }
}
