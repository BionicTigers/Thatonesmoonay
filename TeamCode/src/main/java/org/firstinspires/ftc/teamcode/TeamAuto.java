package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Vuforia Distance Tracker", group ="summerProjects")
public class TeamAuto extends LinearOpMode {

    @Override public void runOpMode() {
        //initialization
        Navigation nav = new Navigation(this);

        waitForStart();
        while (opModeIsActive()) {


        }
    }
}
