package org.firstinspires.ftc.teamcode.jessie;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.shared.common.RobotConfiguration;

/**
 * It is assumed that there is a configuration that is currently activated on the robot controller
 * (run menu / Configure Robot ) with the same name as this class.
 * It is also assumed that the device names in the 'init()' method below are the same as the devices
 * named on the activated configuration on the robot.
 */
public class NessyConfiguration extends RobotConfiguration {
    //Movement motors
    public DcMotor centreRight;
    public DcMotor backRight;
    public DcMotor centreLeft;
    public DcMotor backLeft;

    //Grabby arm motors
    public DcMotor liftMotor;
    public DcMotor climbMotor;
    public Servo clawServo;
    public DcMotor armMotor;

    /**
     * Factory method for this class
     *
     */
    public static NessyConfiguration newConfig(HardwareMap hardwareMap, Telemetry telemetry) {

        NessyConfiguration config = new NessyConfiguration();
        config.init(hardwareMap, telemetry);
        return config;
    }

    /**
     * Assign your class instance variables to the saved device names in the hardware map
     *
     */
    @Override
    protected void init(HardwareMap hardwareMap, Telemetry telemetry) {

        setTelemetry(telemetry);

        centreRight = (DcMotor) getHardwareOn("centreRight", hardwareMap.dcMotor);
        centreLeft = (DcMotor) getHardwareOn("centreLeft", hardwareMap.dcMotor);
        backRight = (DcMotor) getHardwareOn("backRight", hardwareMap.dcMotor);
        backLeft = (DcMotor) getHardwareOn("backLeft", hardwareMap.dcMotor);
        centreRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

//        Telemetry.Item motorsItem = telemetry.addData("Drive Motors: ", new Func<String>() {
//            @SuppressLint("DefaultLocale")
//            @Override
//            public String value() {
//                return String.format("Centre Left: %4.2f  Centre Right: %4.2f  Back Left: %4.2f  Back Right: %4.2f", centreLeft.getPower(), centreRight.getPower(), backLeft.getPower(), backRight.getPower());
//            }
//        });
//        motorsItem.setRetained(true);


        liftMotor = (DcMotor) getHardwareOn("liftMotor", hardwareMap.dcMotor);
        climbMotor = (DcMotor) getHardwareOn("climbMotor", hardwareMap.dcMotor);
        clawServo = (Servo) getHardwareOn("clawServo", hardwareMap.servo);
        armMotor = (DcMotor) getHardwareOn("armMotor", hardwareMap.dcMotor);

//        Telemetry.Item armItem = telemetry.addData("Lift/Arm Motors: ", new Func<String>() {
//            @SuppressLint("DefaultLocale")
//            @Override
//            public String value() {
//                return String.format("Lift Power: %.2f  Climb Power: %.2f", liftMotor.getPower(), climbMotor.getPower());
//            }
//        });
//        armItem.setRetained(true);


    }


}
