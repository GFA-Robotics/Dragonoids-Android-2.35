package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Dragonoids on 11/30/2016.
 */

@TeleOp(name="Gyro Test", group="RED")
public class GyroTest extends LinearOpMode{

    ModernRoboticsI2cGyro gyro;

    public void runOpMode() {

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);
        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
        }

        while (opModeIsActive()) {

            telemetry.addData("Raw X", gyro.rawX());
            telemetry.addData("Raw Y", gyro.rawY());
            telemetry.addData("Raw Z", gyro.rawZ());
            telemetry.addData("Heading", gyro.getIntegratedZValue());

            telemetry.update();
        }
    }
}
