/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//EXIST
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoDrivingtoDepot", group="Auto")
public class AutoDrivingtoDepot extends LinearOpMode {

    private DcMotor backLeft; //left motor back
    //private DcMotor frontLeft; //left motor front (Ladies first)ot
    private DcMotor backRight; //right motor back
    //private DcMotor frontRight; //right motor front (Ladies first)
    private DcMotor collector;
    private DcMotor evangelino;
    private Servo teamMarker;
    private Servo flickyWrist;

    //Variables//
    private double speed, xValue, yValue, leftPower, rightPower;
    public int target, calibToggle;
    private Servo liftrawrh;
    private ElapsedTime runtime = new ElapsedTime();
    private GoldAlignDetector detector;
    private boolean otherstuff = false;



    @Override
    public void runOpMode() {
        //Motors//
        backLeft = hardwareMap.dcMotor.get("backLeft"); //Left Back
        //frontLeft = hardwareMap.dcMotor.get("frontLeft"); //Left Front
        backRight = hardwareMap.dcMotor.get("backRight"); //Right Back
        //frontRight = hardwareMap.dcMotor.get("frontRight"); //Right Front
        evangelino = hardwareMap.dcMotor.get("lift");
        //collector = hardwareMap.dcMotor.get("collector");
        //teamMarker = hardwareMap.servo.get("teamMarker");
        backRight.setDirection(DcMotor.Direction.REVERSE);
        liftrawrh = hardwareMap.servo.get("liftrawrh");
        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        //flickyWrist = hardwareMap.servo.get("flicky");
        // collector = hardwareMap.dcMotor.get("collector");

        liftrawrh.setPosition(1.0);
        // flickyWrist.setPosition(0.5);
        //Variables//
        calibToggle = 0;
        int target = 0;
        double speed = 0;
        telemetry.addData("Status", "DogeCV 2018.0 - Gold Align Example");

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1, false);
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 150; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        waitForStart();
        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  should get you unhooked
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            liftrawrh.setPosition(0);
            backLeft.setPower(.4);
              backRight.setPower(-.4);
              evangelino.setPower(.25);

        }


//        while (opModeIsActive() && runtime.seconds() < .75) {
//            liftrawrh.setPosition(0);
//            backLeft.setPower(.4);
//            backRight.setPower(-.4);
//        }





        sleep(1000);
        runtime.reset();
            while (opModeIsActive() && !detector.getAligned()) {
                telemetry.addData("Path", "we samplin bois", runtime.seconds());
                // this code should rotate the bot until it is aligned with the mineral
                backLeft.setPower(0);
                backRight.setPower(0);

                evangelino.setPower(0);
                backLeft.setPower(-.35);
                backRight.setPower(.35);
            }
            // this code should drive forward
            while (opModeIsActive()) {
                telemetry.addData("Path", "drive at the gold boi", runtime.seconds());
                backLeft.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(.55);
                backRight.setPower(.55);
            }



    }

}