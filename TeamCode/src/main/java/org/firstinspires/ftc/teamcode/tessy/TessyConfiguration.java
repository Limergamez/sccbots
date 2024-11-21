package org.firstinspires.ftc.teamcode.tessy;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TessyConfiguration {
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;

    public static TessyConfiguration newConfig(HardwareMap hardwareMap, Telemetry telemetry) {
        TessyConfiguration config = new TessyConfiguration();

        config.leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
        config.rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
        config.leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        config.rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");

        telemetry.addLine("TessyConfiguration initialized successfully.");
        telemetry.update();
        return config;
    }
}
