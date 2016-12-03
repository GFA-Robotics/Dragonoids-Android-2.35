package org.firstinspires.ftc.teamcode;

/**
 * Created by Dragonoids on 11/18/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Blue Autonomous 1", group="Blue")
public class DragonoidsAutoBlue1 extends DragonoidsAuto {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        //set presser in neutral pos
        buttonPresser.setPosition(0);

        //get in position to shoot and shoot twice
        forward(-.5,.75);
        shoot();

        //move in pos to detect left light of first beacon
        strafe(1,.75);
        forward(-1.25,.75);
        strafe(.66, .75);

        //detect color true is blue
        if(detectColor() == true){
            buttonPresser.setPosition(30);
            sleep(250);
            buttonPresser.setPosition(0);
        }
        else{
            buttonPresser.setPosition(-30);
            sleep(250);
            buttonPresser.setPosition(0);
        }
        strafe(.33,.75);

        //move in pos to detect left light of second beacon
        strafe(-1,.75);
        forward(-2,75);
        strafe(.66,.75);

        //detect color true is blue
        if(detectColor() == true){
            buttonPresser.setPosition(30);
            sleep(250);
            buttonPresser.setPosition(0);
        }
        else{
            buttonPresser.setPosition(-30);
            sleep(250);
            buttonPresser.setPosition(0);
        }
        strafe(.33,.75);
        strafe(-1,.75);

        telemetry.addData("Distance Traveled: ", motorLF.getCurrentPosition() * (WHEEL_CIRC / ENCODER_CPR));
        telemetry.update();


    }
}
