package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name="Test Chris Autonomous", group="Test")

public class TestChrisNav extends LinearOpMode {
    private Navigation nav;
    @Override
    public void runOpMode() {
        nav = new Navigation(this, telemetry, true);
        waitForStart();
        while(opModeIsActive()){
        nav.updateCubePos();
        telemetry.addData("", nav.getCubePos());
        telemetry.update();}

    }
}