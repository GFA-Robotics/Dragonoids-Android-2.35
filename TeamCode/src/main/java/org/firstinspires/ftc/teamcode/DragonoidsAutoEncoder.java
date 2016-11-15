package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;

/**
 * Created by Dragonoids on 11/5/2016.
 */
@Autonomous(name="Encoder Drive", group="Linear Opmode")
public class DragonoidsAutoEncoder extends OpMode {
    DcMotor motorRF;
    DcMotor motorRB;
    DcMotor motorLF;
    DcMotor motorLB;

    final static int ENCODER_CPR = 1120;
    final static double WHEEL_CIRC = 4 * Math.PI;
    // go forward 2 tiles
    final static int TILE = 23;

    final static double TILEROTATE = TILE / WHEEL_CIRC;
    final static double INITDIST = ENCODER_CPR * TILEROTATE;




    public void init() {
        motorRF = hardwareMap.dcMotor.get("right_drive_front");
        motorRB = hardwareMap.dcMotor.get("right_drive_back");
        motorLF = hardwareMap.dcMotor.get("left_drive_front");
        motorLB = hardwareMap.dcMotor.get("left_drive_back");

        //starts backwards and drives backwards
        motorLF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorLB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void start() {

        //runs 1 tile forward
        motorRF.setTargetPosition((int) INITDIST);
        motorRB.setTargetPosition((int) INITDIST);
        motorLF.setTargetPosition((int) INITDIST);
        motorLB.setTargetPosition((int) INITDIST);

        motorRF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRF.setPower(0.75);
        motorRB.setPower(0.75);
        motorLF.setPower(0.75);
        motorLB.setPower(0.75);
/*
        //strafes to the right
        motorRF.setTargetPosition((int) STRAFDIST);
        motorRB.setTargetPosition((int) STRAFDIST);
        motorLF.setTargetPosition((int) STRAFDIST);
        motorLB.setTargetPosition((int) STRAFDIST);
        */
    }

    public void loop() {
        telemetry.addData("Distance Traveled: ", motorLF.getCurrentPosition() * (WHEEL_CIRC / ENCODER_CPR));
        telemetry.update();
    }
}
