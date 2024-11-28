package org.firstinspires.ftc.teamcode.tessy;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TessyConfiguration {
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public SparkFunOTOS odometry;

    public static TessyConfiguration newConfig(HardwareMap hardwareMap, Telemetry telemetry) {
        TessyConfiguration config = new TessyConfiguration();

        config.leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        config.rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        config.leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        config.rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        config.odometry = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");


        telemetry.addLine("TessyConfiguration initialized successfully.");
        telemetry.update();
        return config;
    }
}
