package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name="Test Chris Autonomous", group="Test")
public class TestChrisNav extends OpMode {
    private GoldAlignDetector detector;
    private Navigation nav;
    @Override
    public void init() {
        nav = new Navigation(this, telemetry, false);
    }

    @Override
    public void loop() {
            nav.updateCubePos();
            telemetry.addData("", nav.getCubePos());
            telemetry.update();
    }

    @Override
    public void stop() {
        nav.stopVuforia();
    }
}
