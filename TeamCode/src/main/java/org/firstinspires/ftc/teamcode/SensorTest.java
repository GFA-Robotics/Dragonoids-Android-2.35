package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
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

    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cRangeSensor rangeSensor;
    ModernRoboticsI2cColorSensor colorSensor;

    public void runOpMode() {
        double range;
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
        }

        while (opModeIsActive()) {
            range = rangeSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Range Sensor INCH", range);

            telemetry.addData("Raw X", gyro.rawX());
            telemetry.addData("Raw Y", gyro.rawY());
            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getIntegratedZValue());

            telemetry.update();

        }
    }
}
