package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Team Autonomous")
public class TeamAuto extends LinearOpMode {

    @Override public void runOpMode() {
        //initialization
        Navigation nav = new Navigation(this,telemetry);

        waitForStart();
        while (opModeIsActive()) {

            //landing

            //identify cube

            //collect/knock cube while avoiding other items

            //nav to marker deploy

            //deploy marker

            //enter crater

        }
    }
}
