package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Team Autonomous")
public class TeamAuto extends LinearOpMode {

    private boolean teamAndLocationDetermined = false;

    @Override public void runOpMode() {
        //initialization
        Navigation nav = new Navigation(this,telemetry);


        waitForStart();

        if(nav.updatePos()) {
            nav.updateTeam();
            teamAndLocationDetermined = true;
        }

        //landing motor

        if(nav.updatePos() && !teamAndLocationDetermined) {
            nav.updateTeam();
            teamAndLocationDetermined = true;
        }

        //locate cube

        while(nav.updatePos() && !teamAndLocationDetermined) {
            nav.updateTeam();
            while (nav.updateCubePos() == null){

            teamAndLocationDetermined = true;
            }
        }

        //goto cube

        if(nav.updatePos() && !teamAndLocationDetermined) {
            nav.updateTeam();
            teamAndLocationDetermined = true;
        }

        //if location is undetermined, drive forward. If gyro detects crater, goto depot

        //deploy team marker thing
        //go to appropriate crater
        //begin collection

        //auto end?
    }
}
