package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Debug Autonomous", group ="test")
public class DebugAuto extends LinearOpMode {

    @Override public void runOpMode() {
        //initialization
        Navigation nav = new Navigation(this,telemetry);

        waitForStart();

        while(opModeIsActive()) {
            nav.updatePos();
            telemetry.addData("pos",nav.pos);
            telemetry.addData("debug",nav.telemetryString);
            telemetry.update();

        }
    }
}
