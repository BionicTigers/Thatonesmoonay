package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto Crater", group="Auto")
public class AutoCrater extends LinearOpMode {
    public void runOpMode() {
        AutoGeneric autoGeneric = new AutoGeneric(AutoGeneric.StartPos.CRATER, this, telemetry);
        boolean boolsam = false;
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        int i = 1;
        while (!opModeIsActive()){
            while (runtime.seconds() < 5 || !boolsam)
                boolsam = autoGeneric.sampling();
            telemetry.addData("Test", i);
            i++;
            telemetry.update();
            runtime.reset();
        }
        waitForStart();

        autoGeneric.runOpMode();
    }
}
