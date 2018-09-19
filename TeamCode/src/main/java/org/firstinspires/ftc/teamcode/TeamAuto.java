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
            nav.determineTeam();
            teamAndLocationDetermined = true;
        }

        //landing motor

        if(nav.updatePos() && !teamAndLocationDetermined) {
            nav.determineTeam();
            teamAndLocationDetermined = true;
        }

        //locate cube

        if(nav.updatePos() && !teamAndLocationDetermined) {
            nav.determineTeam();
            teamAndLocationDetermined = true;
        }

        //goto cube

        if(nav.updatePos() && !teamAndLocationDetermined) {
            nav.determineTeam();
            teamAndLocationDetermined = true;
        }

        //if location is undetermined, drive forward. If gyro detects crater, goto depot

        //deploy team marker thing
        //go to appropriate crater
        //begin collection

        //auto end?
    }
}
