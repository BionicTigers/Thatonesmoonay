package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Team Autonomous")
public class TeamAuto extends LinearOpMode {

    @Override public void runOpMode() {
        //initialization
        Navigation nav = new Navigation(this,telemetry,true);

        //try determine vision stuff
        nav.updatePos();
        nav.updateTeam();
        nav.updateCubePos();

        waitForStart();

        //try determine vision stuff
        nav.updatePos();
        nav.updateTeam();
        nav.updateCubePos();


        //landing motor commands

        //try determine vision stuff
        nav.updatePos();
        nav.updateTeam();
        nav.updateCubePos();

        //goto cube
        //collect/nudge
        //goto depot
        //deploy team marker thing
        //go to appropriate crater
        //begin collection
        //parking
    }
}
