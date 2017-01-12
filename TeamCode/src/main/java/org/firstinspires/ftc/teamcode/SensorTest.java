package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Dragonoids on 11/30/2016.
 */

@TeleOp(name="Sensor Test", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class SensorTest extends LinearOpMode{

    ColorSensor colorSensor;
    OpticalDistanceSensor lineSensor;

    ModernRoboticsI2cRangeSensor rangeSensor;

    ModernRoboticsI2cGyro gyro;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;


    public void runOpMode() {
        double range;

        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        lineSensor = hardwareMap.opticalDistanceSensor.get("lineSensor");

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        gyro.resetZAxisIntegrator();
        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
        }
        waitForStart();

        while (opModeIsActive()) {
            //ODS values
            telemetry.addData("ODS Light value", lineSensor.getLightDetected());

            //Color sensor values
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Blue ", colorSensor.blue());

            //range sensor values
            range = rangeSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Range Sensor INCH", range);

            //gyro values
            telemetry.addData("Raw X", gyro.rawX());
            telemetry.addData("Raw Y", gyro.rawY());
            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getIntegratedZValue());

            telemetry.update();

        }
    }
}
