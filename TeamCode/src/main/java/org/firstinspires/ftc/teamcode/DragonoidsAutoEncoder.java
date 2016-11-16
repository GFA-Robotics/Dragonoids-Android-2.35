package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.hardware.SensorEventListener;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

/**
 * Created by Dragonoids on 11/5/2016.
 */
@Autonomous(name="Auto Red Position 1", group="Linear Opmode")
public class DragonoidsAutoEncoder extends LinearOpMode {
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor motorLF;
    DcMotor motorLB;

    ColorSensor colorSensor;

    final static int ENCODER_CPR = 1120;
    final static double WHEEL_CIRC = 4 * Math.PI;
    // go forward 2 tiles
    final static int TILE = 23;

    final static double ROTATE = TILE / WHEEL_CIRC;
    final static double INITDIST = ENCODER_CPR * ROTATE;

    long totTime = 0;

    boolean color;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    public void runOpMode() throws InterruptedException {

        waitForStart();



        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        motorRF = hardwareMap.dcMotor.get("right_drive_front");
        motorRB = hardwareMap.dcMotor.get("right_drive_back");
        motorLF = hardwareMap.dcMotor.get("left_drive_front");
        motorLB = hardwareMap.dcMotor.get("left_drive_back");

        //starts backwards and drives backwards
        motorRF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        forward(.75, .5, 500); // move forward to give us space to rotate


    public void resetEncoders() {
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void forward (double distance, double power, long totTime) {
        resetEncoders();


        distance = ENCODER_CPR * ROTATE * distance;

        totTime = (long) (distance/power) + totTime;

        motorRF.setTargetPosition((int) distance);
        motorRB.setTargetPosition((int) distance);
        motorLF.setTargetPosition((int) distance);
        motorLB.setTargetPosition((int) distance);

        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRF.setPower(power);
        motorRB.setPower(power);
        motorLF.setPower(power);
        motorLB.setPower(power);

        sleep(Math.abs(totTime));

    }

    public void turn (double distance, double power, long totTime) {
        resetEncoders();
        distance = ENCODER_CPR * ROTATE * distance * 1.5 / 180;

        totTime = (long) (distance/power) + totTime;

        motorRF.setTargetPosition((int) (distance));
        motorRB.setTargetPosition((int) (distance));
        motorLF.setTargetPosition((int) (-distance));
        motorLB.setTargetPosition((int) (-distance));

        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRF.setPower(power);
        motorRB.setPower(power);
        motorLF.setPower(power);
        motorLB.setPower(power);

        sleep(Math.abs(totTime));
    }


        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRF.setPower(0.75);
        motorRB.setPower(0.75);
        motorLF.setPower(0.75);
        motorLB.setPower(0.75);

        sleep(Math.abs(totTime));
    }

        telemetry.update();
    }
}
