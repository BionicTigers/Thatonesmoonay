package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Navigation;

@Autonomous(name="autoencoders")
public class autoencoders extends LinearOpMode {
    private Servo liftrawrh;
    private Servo flickyWrist;
    @Override public void runOpMode() {
        //initialization
        //liftrawrh = hardwareMap.servo.get("liftrawrh");
        //flickyWrist = hardwareMap.servo.get("flicky");

        //liftrawrh.setPosition(1.0);
        //flickyWrist.setPosition(0.5);
        Navigation nav = new Navigation(this,telemetry, true, false, false, true);

        waitForStart();
        nav.updateCubePos();


        nav.getCubePos();

        //try determine vision stuff
        //nav.updatePos();
        //nav.updateTeam();
        nav.updateCubePos();
        nav.driveEncoderReset();

        nav.driveMethodSimple(10f,10f,0.5f,0.5f);
        sleep(1000);
        nav.driveMethodSimple(-10f,-10f,0.5f,0.5f);

    }
}
