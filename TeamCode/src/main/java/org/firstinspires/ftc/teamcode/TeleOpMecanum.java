/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp Mecanum", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class TeleOpMecanum extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor motorLF;
    DcMotor motorLB;

    DcMotor motorDisp;
    DcMotor motorLift;

    DcMotor motorShootOne;
    DcMotor motorShootTwo;

    Servo loader;

    Servo leftLift;
    Servo rightLift;

    ColorSensor colorSensor;
    boolean color;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F,0F,0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    double drive;
    double strafe;
    double rotate;

    final static int ENCODER_CPR = 1120;
    final static double WHEEL_CIRC = 4 * Math.PI;

    boolean lifted = false;

    double fVelocity;

    long fVelocityTime;
    long fLastVelocityTime;

    int fEncoder;
    int fLastEncoder;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        motorRF = hardwareMap.dcMotor.get("right_drive_front");
        motorRB = hardwareMap.dcMotor.get("right_drive_back");
        motorLF = hardwareMap.dcMotor.get("left_drive_front");
        motorLB = hardwareMap.dcMotor.get("left_drive_back");

        motorDisp = hardwareMap.dcMotor.get("collector");
        motorLift = hardwareMap.dcMotor.get("lift");

        motorShootOne = hardwareMap.dcMotor.get("shooterOne");
        motorShootTwo = hardwareMap.dcMotor.get("shooterTwo");

        motorShootOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorShootTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        loader = hardwareMap.servo.get("loader");
        loader.setPosition(.5);

        leftLift = hardwareMap.servo.get("leftLift");
        rightLift = hardwareMap.servo.get("rightLift");

        colorSensor = hardwareMap.colorSensor.get("sensor_color");

        leftLift.setDirection(Servo.Direction.REVERSE);

        leftLift.setPosition(1);
        rightLift.setPosition(1);

        motorRF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


        motorShootTwo.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //output telemetry of motors
            telemetry.addData("rightFront", + motorRF.getPower());
            telemetry.addData("leftFront", + motorLF.getPower());
            telemetry.addData("rightBack", + motorRB.getPower());
            telemetry.addData("leftBack", + motorLB.getPower());
            detectColor();

            //Run the collector
            if (gamepad2.a) {
                motorDisp.setPower(1.0);
            }
            else if(gamepad2.b) {
                motorDisp.setPower(-1.0);
            } else {
                motorDisp.setPower(0);
            }

            //Fire up the shoot motors
            if (gamepad2.left_bumper){
                bangBang();
            } else {
                motorShootOne.setPower(0);
                motorShootTwo.setPower(0);
            }

            //load a ball into the shooter
            if (gamepad2.right_bumper) {
                loader.setPosition(0.3);
            } else {
                loader.setPosition(.98);
            }

            // Cap ball is a WIP
            if (gamepad2.right_trigger > 0.1) {
                lifted = true;
                motorLift.setPower(gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0){
                motorLift.setPower(-gamepad2.left_trigger);
            } else {
                motorLift.setPower(0);
                motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            
            if(gamepad2.y || gamepad1.y) {
                lifted = false;
            }

            if (!lifted) {
                drive = scaleInput(-gamepad1.left_stick_y);
                strafe = scaleInput(gamepad1.left_stick_x);
                rotate = scaleInput(gamepad1.right_stick_x);
            } else {
                drive = scaleInput(-gamepad1.left_stick_y)*.2;
                strafe = scaleInput(gamepad1.left_stick_x)*.2;
                rotate = scaleInput(gamepad1.right_stick_x)*.2;
            }

            motorLF.setPower(Range.clip(drive - strafe + rotate, -1.0, 1.0));
            motorLB.setPower(Range.clip(drive + strafe + rotate, -1.0, 1.0));
            motorRF.setPower(Range.clip(drive + strafe - rotate, -1.0, 1.0));
            motorRB.setPower(Range.clip(drive - strafe - rotate, -1.0, 1.0));

            telemetry.addData("Distance Traveled: ", motorLF.getCurrentPosition() * (WHEEL_CIRC / ENCODER_CPR));
            telemetry.update();

        }

    }

    private float scaleInputOriginal(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.45, 0.52, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return (float)dScale;
    }
    private float scaleInput (double value) {
        // Return the value (from -1 to 1) squared to scale it quadratically
        float magnitude = (float) Math.pow(value, 2);
        if (value < 0) {
            return -1 * magnitude;
        }
        else {
            return magnitude;
        }
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


        return color;
    }
    public void bangBang () {
        fVelocityTime = System.nanoTime();

        fEncoder = motorShootOne.getCurrentPosition();

        fVelocity = (double) (fEncoder - fLastEncoder) / (fVelocityTime - fLastVelocityTime);

        if (fVelocity >= .92) {
            motorShootOne.setPower(.88);
            motorShootTwo.setPower(.88);
        } else if (fVelocity < .92) {
            motorShootOne.setPower(.92);
            motorShootTwo.setPower(.92);
        }
        fLastEncoder = fEncoder;
        fLastVelocityTime = fVelocityTime;
    }
}
