package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.hardware.SensorEventListener;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    Servo leftRelease;
    Servo rightRelease;

    ColorSensor colorSensor;
    OpticalDistanceSensor lineSensor;

    ModernRoboticsI2cRangeSensor rangeSensor;

    ModernRoboticsI2cGyro gyro;

    public int targetAngle = 0;
    private int adjustedAngle;

    private double initLight;



    final static int ENCODER_CPR = 1120;
    final static double WHEEL_CIRC = 4 * Math.PI;
    // 1 tile length is 24 inches
    final static int TILE = 24;

    final static double ROTATE = TILE / WHEEL_CIRC;

    int color;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    //variables for bangBang
    double fVelocity;
    long fVelocityTime;
    long fLastVelocityTime;
    int fEncoder;
    int fLastEncoder;

    public void runOpMode() throws InterruptedException {

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        lineSensor = hardwareMap.opticalDistanceSensor.get("lineSensor");

        motorRF = hardwareMap.dcMotor.get("right_drive_front");
        motorRB = hardwareMap.dcMotor.get("right_drive_back");
        motorLF = hardwareMap.dcMotor.get("left_drive_front");
        motorLB = hardwareMap.dcMotor.get("left_drive_back");

        motorShootOne = hardwareMap.dcMotor.get("shooterOne");
        motorShootTwo = hardwareMap.dcMotor.get("shooterTwo");

        motorShootTwo.setDirection(DcMotor.Direction.REVERSE);

        loader = hardwareMap.servo.get("loader");

        leftRelease = hardwareMap.servo.get("leftLift");
        rightRelease = hardwareMap.servo.get("rightLift");

        leftRelease.setDirection(Servo.Direction.REVERSE);

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

        //starts backwards and drives backwards
        motorRF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        gyro.calibrate();
        while (!isStopRequested() && gyro.isCalibrating()) {
            sleep(1);
            telemetry.addData("Measurement mode", gyro.getMeasurementMode());
            telemetry.update();
        }

        gyro.resetZAxisIntegrator();

        targetAngle = 0;

        // changed the heading to signed heading [-360,360]
        gyro.setHeadingMode(ModernRoboticsI2cGyro.HeadingMode.HEADING_CARTESIAN);

        loader.setPosition(.515);

        leftRelease.setPosition(.25);
        rightRelease.setPosition(.25);

        motorShootOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShootTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initLight = lineSensor.getLightDetected();

        colorSensor.enableLed(false);

        telemetry.addData("Calibrated", targetAngle);
        telemetry.update();
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
        while (opModeIsActive() && (Math.abs(motorRB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorRF.getCurrentPosition())<=Math.abs(distance) ||
                Math.abs(motorLB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance))) {
        }

        stopMotors();

    }

    public void turn (int angle) {
        resetEncoders();

        double power;

        targetAngle = angle;

        int currentAngle = gyro.getIntegratedZValue();

        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if ((targetAngle-currentAngle)>0) {
            while (opModeIsActive() && (targetAngle > gyro.getIntegratedZValue())) {

                if(((targetAngle - gyro.getIntegratedZValue()) * .009)> .4) {
                    power = .4;
                } else {
                    power = (targetAngle - gyro.getIntegratedZValue()) * .009;
                }

                if (power < .018) {
                    break;
                }

                motorRF.setPower(power);
                motorRB.setPower(power);
                motorLF.setPower(-power);
                motorLB.setPower(-power);

                telemetry.addData("power" , power);
                telemetry.addData("Current angle" , gyro.getIntegratedZValue());
                telemetry.addData("Target angle", targetAngle);
                telemetry.update();
            }
            stopMotors();
        }
        else {
            while (opModeIsActive() && (targetAngle < gyro.getIntegratedZValue())) {
                if(((targetAngle - gyro.getIntegratedZValue()) * .009)< -.4) {
                    power = -.4;
                } else {
                power = (targetAngle - gyro.getIntegratedZValue()) * .009;
                }

                if (power > -.018) {
                    break;
                }

                motorRF.setPower(power);
                motorRB.setPower(power);
                motorLF.setPower(-power);
                motorLB.setPower(-power);

                telemetry.addData("power" , power);
                telemetry.addData("Current angle" , gyro.getIntegratedZValue());
                telemetry.addData("Target angle", targetAngle);
                telemetry.update();
            }
            stopMotors();
        }
        stopMotors();

        telemetry.addData("Angle reached", gyro.getIntegratedZValue());
        telemetry.addData("Angle wanted", targetAngle);
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

        while (opModeIsActive() && (Math.abs(motorRB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorRF.getCurrentPosition())<=Math.abs(distance) ||
                Math.abs(motorLB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance))) {
        }

        stopMotors();

    }

    public void rightDiagonal (double distance, double power) {
        resetEncoders();

        distance = ENCODER_CPR * ROTATE * distance * 2;


            motorRF.setTargetPosition((int) distance);
            motorLB.setTargetPosition((int) distance);


        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRF.setPower(power);
        motorLB.setPower(power);

        while (opModeIsActive() && (Math.abs(motorRF.getCurrentPosition())<=Math.abs(distance) ||
                Math.abs(motorLB.getCurrentPosition())<=Math.abs(distance))) {
        }

        stopMotors();
    }

    public void leftDiagonal (double distance, double power) {
        resetEncoders();

        distance = ENCODER_CPR * ROTATE * distance * 2;

        motorRB.setTargetPosition((int) distance);
        motorLF.setTargetPosition((int) distance);

        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRB.setPower(power);
        motorLF.setPower(power);

        while (opModeIsActive()&&(Math.abs(motorRB.getCurrentPosition())<=Math.abs(distance) || Math.abs(motorLF.getCurrentPosition())<=Math.abs(distance))) {
        }

        stopMotors();
    }

    public void shoot () {
        sleep(250);
        motorShootOne.setPower(.75);
        motorShootTwo.setPower(.75);
        sleep(250);
        loader.setPosition(.3);
        sleep(500);

        loader.setPosition(.508);
        sleep(1250);

        loader.setPosition(.3);
        sleep(500);

        motorShootOne.setPower(0);
        motorShootTwo.setPower(0);

        loader.setPosition(.515);
    }

    public void bangBang () {
        fVelocityTime = System.nanoTime();

        fEncoder = motorShootOne.getCurrentPosition();

        fVelocity = (double) (fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);

        if (fVelocity >= .82) {
            motorShootOne.setPower(.78);
            motorShootTwo.setPower(.78);
        } else if (fVelocity < .82) {
            motorShootOne.setPower(.82);
            motorShootTwo.setPower(.82);
        }
        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;
    }

    public int detectColor () {

        //0 is no color
        color = 0;
        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Blue ", colorSensor.blue());

        if (colorSensor.red()>colorSensor.blue()) {
            //red is 1
            color = 1;
        } else if (colorSensor.blue() > colorSensor.red()){
            //blue is 2
            color = 2;
        }

        return color;
    }

    //target angle should return back to its target angle before the adjust
    public void adjustHeading() {

       int currentAngle = gyro.getIntegratedZValue();

        int prevTargetAngle = targetAngle;

        adjustedAngle = (targetAngle);

        if (!(currentAngle > -8 && currentAngle < 8)) {
            turn(adjustedAngle);
        }
        targetAngle = prevTargetAngle;
    }

    public void adjustHeadingSensor() {

    }

    public boolean detectLine(){

        boolean foundLine = false;

            //light from darker block should be below certain threshold (0.189 is temporary value)
            if (lineSensor.getLightDetected() <= initLight+.3){
                foundLine = false;
            }

            //light from white tape should be larger than grey, return on line
            else if (lineSensor.getLightDetected() >= initLight+.3){
                foundLine = true;
                stopMotors();
                resetEncoders();
            }

        return foundLine;
    }

    public void alignLine(boolean value) {

        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if(value) {
            while(!detectLine() && opModeIsActive()){


                motorRF.setPower(-.15);
                motorRB.setPower(-.15);
                motorLF.setPower(-.15);
                motorLB.setPower(-.15); }
        } else {
            while(!detectLine() && opModeIsActive()){

                motorRF.setPower(.15);
                motorRB.setPower(.15);
                motorLF.setPower(.15);
                motorLB.setPower(.15); }
            }
        stopMotors();
    }


    public void adjustRange () {
        double range = getRange();
        motorRF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (range>7) {
            while ((opModeIsActive()&&getRange()>7.75)) {
                motorRF.setPower(-.35);
                motorRB.setPower(.35);
                motorLF.setPower(.35);
                motorLB.setPower(-.35);
            }
        }
        else {
            while (opModeIsActive()&&(getRange()<7.75)){
                motorRF.setPower(.35);
                motorRB.setPower(-.35);
                motorLF.setPower(-.35);
                motorLB.setPower(.35);
            }
        }
        stopMotors();
        }
    public double getRange () {
        double range = rangeSensor.getDistance(DistanceUnit.INCH);
        return range;
    }

}
