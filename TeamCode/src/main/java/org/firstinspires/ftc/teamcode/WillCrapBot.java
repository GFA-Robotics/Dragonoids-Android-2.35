package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp (name = "Crapbot Driving Code", group = "Linear Opmode")
@Disabled
public class WillCrapBot extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;

    double drive;
    double rotate;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeft = hardwareMap.dcMotor.get ("motorLeft");
        motorRight = hardwareMap.dcMotor.get ("motorRight");

        motorLeft.setDirection (DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive())
        {

            drive	= -(gamepad1.left_stick_y);
            rotate	= (gamepad1.left_stick_x);


            motorLeft.setPower((Range.clip(drive - rotate, -.5, .5)));
            motorRight.setPower((Range.clip(drive + rotate, -.5, .5)));

            idle ();
        }
    }
}