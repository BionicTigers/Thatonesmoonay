package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Test Open CV", group="Test")
public class TestOpenCV extends LinearOpMode {

    @Override public void runOpMode() {
        Navigation nav = new Navigation(this,telemetry,false,true);

        waitForStart();

        while(opModeIsActive()) {
            nav.updateCubePos();
            nav.telemetryMethod();
        }
    }

}
