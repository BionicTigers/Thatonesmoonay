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
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 2, false);
        detector.useDefaults();

        // Tuning
        detector.alignSize = 200; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
        //initialization
        //liftrawrh = hardwareMap.servo.get("liftrawrh");
        //flickyWrist = hardwareMap.servo.get("flicky");

        //liftrawrh.setPosition(1.0);
        //flickyWrist.setPosition(0.5);
        Navigation nav = new Navigation(this,telemetry, false);

        waitForStart();

        //try determine vision stuff
        // nav.updatePos();
        //nav.updateTeam();

        nav.updateCubePos();


        nav.getCubePos();

        //try determine vision stuff
        //nav.updatePos();
        //nav.updateTeam();
        nav.updateCubePos();
        nav.driveEncoderReset();}

    }
