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
            telemetry.addData("ra",nav.motorRightA.getCurrentPosition());
            telemetry.addData("rb",nav.motorRightB.getCurrentPosition());
            telemetry.addData("la",nav.motorLeftA.getCurrentPosition());
            telemetry.addData("lb",nav.motorLeftB.getCurrentPosition());
            telemetry.update();
        }
    }
}
