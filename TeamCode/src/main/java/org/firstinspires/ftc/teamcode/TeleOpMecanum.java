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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    DcMotor motorShootOne;
    DcMotor motorShootTwo;

    Servo loader;
    double drive;
    double strafe;
    double rotate;

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

        motorShootOne = hardwareMap.dcMotor.get("shooterOne");
        motorShootTwo = hardwareMap.dcMotor.get("shooterTwo");

        loader = hardwareMap.servo.get("loader");

        motorRF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        motorShootOne.setDirection(DcMotor.Direction.REVERSE);
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



            if (gamepad2.a) {
                motorDisp.setPower(-.65);
            }
            else if(gamepad2.b) {
                motorDisp.setPower(.45);
            } else {
                motorDisp.setPower(0);
            }

            if (gamepad2.left_bumper){
                motorShootOne.setPower(1.0);
                motorShootTwo.setPower(1.0);
            } else {
                motorShootOne.setPower(0);
                motorShootTwo.setPower(0);
            }

            if (gamepad2.right_bumper) {
                loader.setPosition(.45);
            } else {
                loader.setPosition(0);
            }

            float turningAmount = gamepad1.left_stick_x;

            // we're going to convert to polar, add pi/4 to theta, and convert back to cartesian.
            double r = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(-gamepad1.right_stick_y, 2));
            double t = 0;
            if(gamepad1.right_stick_x != 0) {
                t = Math.atan(-gamepad1.right_stick_y / gamepad1.right_stick_x); // over to polar
            }
            //double newt = t + (Math.PI / 4); // adjust theta
            double processedX = Math.cos(t)*r;
            double processedY = Math.sin(t)*r; // back to cartesian
//                                                                                                          Check how input is coming from controller; re implement newt



            /*
            drive	= -gamepad1.left_stick_y;
            strafe	= gamepad1.left_stick_x;
            rotate	= gamepad1.right_stick_x;
            */
            telemetry.addData("processedX", + processedX);
            telemetry.addData("processedY", + processedY);
            telemetry.update();

            motorRF.setPower(Range.clip(-processedX - turningAmount, -1, 1));
            motorLF.setPower(Range.clip(processedY + turningAmount, -1, 1));
            motorRB.setPower(Range.clip(processedY - turningAmount, -1, 1));
            motorLB.setPower(Range.clip(-processedX + turningAmount, -1, 1));

        //do u even hack
        }

    }
    private float scaleInputOriginal(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

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
}
