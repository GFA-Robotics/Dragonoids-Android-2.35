package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Dragonoids on 11/16/2016.
 */

@Autonomous(name="Red Autonomous 1", group="RED")
public class DragonoidsAutoRed1 extends DragonoidsAuto{


    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        waitForStart();

        forward(1, 1);
        turn(45, .75); // turn towards the beacon
        forward(2.1, 1); // move towards the beacon
        turn(-135, .75); // turn so the back of the robot faces the beacon

        forward(-.25, .5);

        if(detectColor()==true) { // if left light is wrong, adjust to hit other button

            strafe(.37, .65); // move to second light
            forward(-.1, .15); // bump into button
        }
        else { // if light is correct color, no need to adjust

            forward(-.1, .15); // bump into button
        }

        forward(1, .5); //move to second beacon from current position
        turn(90, .5);
        forward(2, 1);
        turn(-90, .5);
        forward(-.75, .5);
        if(detectColor()==true) { // if left light is wrong, adjust to hit other button

            strafe(.37, .65); // move to second light
            forward(-.1, .15); // bump into button
        }
        else { // if light is correct color, no need to adjust

            forward(-.1, .15); // bump into button
        }

        //call to forward
//        forward(1, .75);


        //call to turn between [-90, 90]
//        turn(-90, .75);

        //call to strafe
//        strafe(1, .75);


        telemetry.addData("Distance Traveled: ", motorLF.getCurrentPosition() * (WHEEL_CIRC / ENCODER_CPR));
        telemetry.update();



    }
}
