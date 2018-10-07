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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="DrivingtoDepot", group="Pushbot")
public class DrivingtoDepot extends LinearOpMode {

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

    @Override
    public void runOpMode() {
        //Motors//
        backLeft = hardwareMap.dcMotor.get("backLeft"); //Left Back
        //frontLeft = hardwareMap.dcMotor.get("frontLeft"); //Left Front
        backRight = hardwareMap.dcMotor.get("backRight"); //Right Back
        //frontRight = hardwareMap.dcMotor.get("frontRight"); //Right Front
        evangelino = hardwareMap.dcMotor.get("lift");
        //collector = hardwareMap.dcMotor.get("collector");
        teamMarker = hardwareMap.servo.get("teamMarker");
        backRight.setDirection(DcMotor.Direction.REVERSE);
        liftrawrh = hardwareMap.servo.get("liftrawrh");
        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        flickyWrist = hardwareMap.servo.get("flicky");
        collector = hardwareMap.dcMotor.get("collector");

        liftrawrh.setPosition(1.0);
        flickyWrist.setPosition(0.5);
        //Variables//
        calibToggle = 0;
        int target = 0;
        double speed = 0;


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        evangelino.setPower(0.1);
        liftrawrh.setPosition(0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2)) {
            telemetry.addData("Path", "Forward", runtime.seconds());
            telemetry.update();
        }
        backLeft.setPower(-.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Forward", runtime.seconds());
            telemetry.update();

            backLeft.setPower(-.5);
            //frontLeft.setPower(-.5);
            backRight.setPower(-.5);
            //frontRight.setPower(-.5);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 3)) {
                telemetry.addData("Path", "Forward", runtime.seconds());
                telemetry.update();
            }
            //teamMarker.setPosition(0.5);

            backLeft.setPower(0);
            backRight.setPower(0);
            // frontRight.setPower(0);
            // frontLeft.setPower(0.0);

            backLeft.setPower(.25);
            backRight.setPower(-.25);
            // frontRight.setPower(-.25);
            //frontLeft.setPower(.25);
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1)) {
                telemetry.addData("Path", "Backward", runtime.seconds());
                telemetry.update();
            }
            backLeft.setPower(0);
            backRight.setPower(0);

            sleep(1000);
        }
    }}
