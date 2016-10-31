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


        motorRF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorRB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //output telemetry of motors
            telemetry.addData("rightFront", + );
            telemetry.addData("leftFront", + motorLF.getPower());
            telemetry.addData("rightBack", + motorRB.getPower());
            telemetry.addData("leftBack", + motorLB.getPower());
            telemetry.update();

            float turningAmount = gamepad1.left_stick_x;
            // we're going to convert to polar, add pi/4 to theta, and convert back to cartesian.
            double r = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(-gamepad1.right_stick_y, 2));
            double t = Math.atan(-gamepad1.right_stick_y/gamepad1.right_stick_x); // over to polar
            double newt = t + (Math.PI / 4); // adjust theta
            double processedX = Math.cos(newt)*r;
            double processedY = Math.sin(newt)*r; // back to cartesian

            //manually set the power of RF
            double RF = -processedX - turningAmount;
            double RB = processedY - turningAmount;
            double LF = processedY + turningAmount;
            double LB = -processedX + turningAmount;

            System.out.println(RF + ", " + RB + ", " + LF + ", " + LB);
            /*
            if ((-processedX-turningAmount)>1) {
                motorRF.setPower(1);
            } else if ((-processedX-turningAmount)<-1) {
                motorRF.setPower(-1);
            } else {
                motorRF.setPower=Range.clip(-processedX - turningAmount, -1.0, 1.0);
            }

            //manually set the power of RB
            if ((processedY - turningAmount)>1) {
                motorRB.setPower(1);
            } else if ((processedY - turningAmount)<-1) {
                motorRB.setPower(-1);
            } else {
                motorRB.setPower=Range.clip(processedY - turningAmount, -1.0, 1.0);
            }

            //manually set the power of LF

            if ((processedY + turningAmount)>1) {
                motorLF.setPower(1);
            } else if ((processedY + turningAmount)<-1) {
                motorLF.setPower(-1);
            } else {
                motorLF.setPower=Range.clip(processedY + turningAmount, -1.0, 1.0);
            }
            //manually set the power of LB
            if ((-processedX + turningAmount)>1) {
                motorLB.setPower(1);
            } else if ((-processedX + turningAmount)<-1) {
                motorLB.setPower(-1);
            } else {
                motorLB.setPower=Range.clip(-processedX + turningAmount, -1.0, 1.0);
            }
            */

        }
    }
}
