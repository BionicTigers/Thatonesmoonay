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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

@Autonomous(name="DrivingtoDepot", group="Pushbot")
public class DrivingtoDepot extends LinearOpMode {

    //Drivetrain Motors//
    private DcMotor motorLeone; //left motor back
    private DcMotor motorLannay; //left motor front (Ladies first)ot
    private DcMotor motorRigetony; //right motor back
    private DcMotor motorRachella; //right motor front (Ladies first)

    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        motorLeone = hardwareMap.dcMotor.get("motorLeone"); //Left Back
        motorLannay = hardwareMap.dcMotor.get("motorLannay"); //Left Front
        motorRigetony = hardwareMap.dcMotor.get("motorRigetony"); //Right Back
        motorRachella = hardwareMap.dcMotor.get("motorRachella"); //Right Front
        motorRigetony.setDirection(DcMotor.Direction.REVERSE);
        motorRachella.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        motorLeone.setPower(.5);
        motorLannay.setPower(.5);
        motorRigetony.setPower(.5);
        motorRachella.setPower(.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Forward", runtime.seconds());
            telemetry.update();
        }
        motorLeone.setPower(0);
        motorRigetony.setPower(0);
        motorRachella.setPower(0);
        motorLannay.setPower(0.0);

        motorLeone.setPower(-.25);
        motorRigetony.setPower(-.25);
        motorRachella.setPower(-.25);
        motorLannay.setPower(-.25);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Backward", runtime.seconds());
            telemetry.update();
        }
        motorLeone.setPower(0);
        motorRigetony.setPower(0);
        motorRachella.setPower(0);
        motorLannay.setPower(0);

        motorLeone.setPower(.5);
        motorRigetony.setPower(-.5);
        motorRachella.setPower(.5);
        motorLannay.setPower(-.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.5)) {
            telemetry.addData("Path", "Turn", runtime.seconds());
            telemetry.update();
        }
        motorLeone.setPower(0);
        motorRigetony.setPower(0);
        motorRachella.setPower(0);
        motorLannay.setPower(0.0);

        motorLeone.setPower(.75);
        motorRigetony.setPower(.75);
        motorRachella.setPower(.75);
        motorLannay.setPower(.75);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 5.0)) {
            telemetry.addData("Path", "Forward", runtime.seconds());
            telemetry.update();
        }
        motorLeone.setPower(0);
        motorRigetony.setPower(0);
        motorRachella.setPower(0);
        motorLannay.setPower(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
