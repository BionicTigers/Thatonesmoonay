package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Test Chris Autonomous", group="Test")
public class TestChrisNav extends LinearOpMode{
    private GoldAlignDetector detector;
    @Override public void runOpMode() {

        Navigation nav = new Navigation(this, telemetry, false);

        waitForStart();
        while (!isStopRequested()) {
            nav.updateCubePos();
            telemetry.addData("",nav.getCubePos());

        }
        nav.stopVuforia();
    }
    }
