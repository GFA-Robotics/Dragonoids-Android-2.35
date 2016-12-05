package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.hardware.SensorEventListener;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

/**
 * Created by Dragonoids on 11/5/2016.
 */

public class DragonoidsAuto extends LinearOpMode {
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor motorLF;
    DcMotor motorLB;

    DcMotor motorShootOne;
    DcMotor motorShootTwo;

    Servo loader;

    Servo buttonPresser;

    ColorSensor colorSensor;

    ModernRoboticsI2cGyro gyro;

    private int currentAngle;
    private int targetAngle = 0;



    final static int ENCODER_CPR = 1120;
    final static double WHEEL_CIRC = 4 * Math.PI;
    // 1 tile length is 24 inches
    final static int TILE = 24;

    final static double ROTATE = TILE / WHEEL_CIRC;
    final static double INITDIST = ENCODER_CPR * ROTATE;

    long totTime = 0;

    boolean color;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    public void runOpMode() throws InterruptedException {

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        motorRF = hardwareMap.dcMotor.get("right_drive_front");
        motorRB = hardwareMap.dcMotor.get("right_drive_back");
        motorLF = hardwareMap.dcMotor.get("left_drive_front");
        motorLB = hardwareMap.dcMotor.get("left_drive_back");

        motorShootOne = hardwareMap.dcMotor.get("shooterOne");
        motorShootTwo = hardwareMap.dcMotor.get("shooterTwo");

        motorShootTwo.setDirection(DcMotor.Direction.REVERSE);

        loader = hardwareMap.servo.get("loader");
        buttonPresser = hardwareMap.servo.get("buttonPresser");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        //starts backwards and drives backwards
        motorRF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        gyro.calibrate();
        while (gyro.isCalibrating()) {
            sleep(1);
        }

        currentAngle = gyro.getIntegratedZValue();

        // changed the heading to signed heading [-360,360]
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        buttonPresser.setPosition(.5);
        loader.setPosition(.5);
    }

    public void resetEncoders() {
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void stopMotors() {
        motorRF.setPower(0);
        motorRB.setPower(0);
        motorLF.setPower(0);
        motorLB.setPower(0);
    }

    public void forward (double distance, double power) {
        resetEncoders();

        distance = ENCODER_CPR * ROTATE * distance;



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
        while (Math.abs(motorRB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorRF.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance)) {
        }

        stopMotors();

    }

    public void turn (int angle, double power) {
        resetEncoders();
        currentAngle = gyro.getIntegratedZValue();

        targetAngle = angle;

//        double heading = gyro.getIntegratedZValue();
        double distance = angle * (18 + 2/3);

        motorRF.setTargetPosition((int) distance);
        motorRB.setTargetPosition((int) distance);
        motorLF.setTargetPosition((int) -distance);
        motorLB.setTargetPosition((int) -distance);


        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRF.setPower(power);
        motorRB.setPower(power);
        motorLF.setPower(power);
        motorLB.setPower(power);

        while (Math.abs(motorRB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorRF.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance)) {
        }
        stopMotors();
        resetEncoders();

        telemetry.addData("Angle achieved", gyro.getIntegratedZValue());
        telemetry.update();

    }

    public void strafe (double distance, double power) {
        resetEncoders();
        distance = ENCODER_CPR * ROTATE * distance;

        motorRF.setTargetPosition((int) -distance);
        motorRB.setTargetPosition((int) distance);
        motorLF.setTargetPosition((int) distance);
        motorLB.setTargetPosition((int) -distance);

        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRF.setPower(power);
        motorRB.setPower(power);
        motorLF.setPower(power);
        motorLB.setPower(power);

        while (Math.abs(motorRB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorRF.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance)) {
        }

        stopMotors();

    }

    public void shoot () {

        loader.setPosition(.5);
        sleep(500);

        motorShootOne.setPower(1.0);
        motorShootTwo.setPower(1.0);
        sleep(750);

        loader.setPosition(0);
        sleep(500);

        loader.setPosition(.5);
        sleep(2000);

        loader.setPosition(0);
        sleep(500);
        loader.setPosition(.5);
        sleep(500);

        motorShootOne.setPower(0);
        motorShootTwo.setPower(0);
    }

    public void buttonPress(boolean color) {
        if (color == true) {
            buttonPresser.setPosition(.9);
        }
        else {
            buttonPresser.setPosition(.1);
        }
        telemetry.addData("False is red: Blue is true:", color);
    }

    public boolean detectColor () {

        //false is red
        color = false;
        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Blue ", colorSensor.blue());

        if (colorSensor.red()>colorSensor.blue()) {
            color = false;
        }
        else {
            color = true;
        }

        telemetry.update();
        return color;
    }

    public void adjustHeading() {

        currentAngle = gyro.getIntegratedZValue();


        int adjustedAngle = targetAngle-currentAngle;

        if(!(targetAngle == currentAngle)) {
            turn(adjustedAngle, .25);
        }
    }

}
