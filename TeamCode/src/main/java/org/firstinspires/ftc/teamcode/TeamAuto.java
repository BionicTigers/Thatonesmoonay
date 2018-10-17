package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Team Autonomous")
public class TeamAuto extends LinearOpMode {
    private Servo liftrawrh;
    private Servo flickyWrist;
    @Override public void runOpMode() {
        //initialization
        //liftrawrh = hardwareMap.servo.get("liftrawrh");
        //flickyWrist = hardwareMap.servo.get("flicky");

        //liftrawrh.setPosition(1.0);
        //flickyWrist.setPosition(0.5);
        Navigation nav = new Navigation(this,telemetry, true, false, true, true);

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
        nav.driveEncoderReset();

        //landing motor commands
//        liftrawrh.setPosition(0.3);
        nav.rotateTo(100f, 200f, 40f);
        sleep(100);
//        //Start lift down motors(2) go down, need method (note to self), reverse one motor
        nav.rotateTo(0f, 200f, 40f);
        nav.goDistance(30f,50f,2f);


        //try determine vision stuff
        //nav.updatePos();

//        nav.updateTeam();
//        nav.updateCubePos();

        //go to cube
       // nav.setLift(100);
        //collect/nudge

        //goto depot

        //deploy team marker thing

        //go to appropriate crater

        //begin collection

        //parking
    }
}
