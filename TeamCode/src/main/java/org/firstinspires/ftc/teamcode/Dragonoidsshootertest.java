package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Dragonoids on 12/7/2016.
 */
@Disabled
public class Dragonoidsshootertest extends LinearOpMode {


    DcMotor motorShootOne;
    DcMotor motorShootTwo;

    Servo loader;
    @Override
    public void runOpMode() {

        motorShootOne = hardwareMap.dcMotor.get("shooterOne");
        motorShootTwo = hardwareMap.dcMotor.get("shooterTwo");

        loader = hardwareMap.servo.get("loader");
        motorShootTwo.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

        loader.setPosition(.5);
        sleep(500);

        motorShootOne.setPower(1);
        motorShootTwo.setPower(1);

        while (opModeIsActive()) {

            loader.setPosition(.5);
            sleep(1000);

            loader.setPosition(0);
            sleep(2000);
        }
        loader.setPosition(.5);
        sleep(500);

        motorShootOne.setPower(0);
        motorShootTwo.setPower(0);
    }
}
