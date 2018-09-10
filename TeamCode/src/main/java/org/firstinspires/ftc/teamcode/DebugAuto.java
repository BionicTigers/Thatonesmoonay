package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Debug Autonomous", group ="test")
public class DebugAuto extends LinearOpMode {

    @Override public void runOpMode() {
        //initialization
        Navigation nav = new Navigation(this);

        waitForStart();
        while (opModeIsActive()) {
            nav.updatePos();
            telemetry.addData("x:",""+nav.pos.getLocation(0));
            telemetry.addData("y:",""+nav.pos.getLocation(1));
            telemetry.addData("z:",""+nav.pos.getLocation(2));
            telemetry.addData("rot:",""+nav.pos.getLocation(3));
            telemetry.addData("Misc Debug",nav.tele);
            telemetry.update();
        }
    }
}
