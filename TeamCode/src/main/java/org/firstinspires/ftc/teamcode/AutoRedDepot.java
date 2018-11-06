package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name="AutoRedDepot", group="Auto")
public class AutoRedDepot extends LinearOpMode {
    private Servo liftrawrh;
    private Servo flickyWrist;
    int ralph;
    @Override public void runOpMode() {
        //initialization
        //liftrawrh = hardwareMap.servo.get("liftrawrh");
        //flickyWrist = hardwareMap.servo.get("flicky");

        //liftrawrh.setPosition(1.0);
        //flickyWrist.setPosition(0.5);
        Navigation nav = new Navigation(this,telemetry, true);

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
//          Start lift down motors(2) go down, need method (note to self), reverse one motor


//        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//            telemetry.addData("VuMark", "%s visible", vuMark);
//            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
//
//            if (vuMark == RelicRecoveryVuMark.RIGHT) {
//                ralph = 470;
//            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
//                ralph = 540;
//            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
//                ralph = 620;
//            } else {
//                ralph = 540;
//            }

// faces back towards crater// goes to middle postition //red crater
//        nav.rotateTo(110f, 200f, 40f); //get off hook by going to the front
//        sleep(100);
//        nav.rotateTo(0f, 200f, 40f); //turn back to straighten self out
//        sleep(100);
//        nav.goDistance(25f,50f); //go to the middle spot at sampling
//        sleep(100);
//        nav.goDistance(-10f,150); //back up, after hitting the sampling in the middle
//        nav.rotateTo(240f,300f,40f); //turn right to get to wall
//        sleep(100);
//        nav.goDistance(40f,150f); // drive towards wall and get close enough to it
//        sleep(100);
//        nav.rotateTo(185f,400f,40f); //turn left to face depot and square up with wall
//        sleep(100);
//        nav.goDistance(40f,75f); //drive to depot to drop off teamMarker
//        sleep(100);
//        //some type of servo to drop off the teamMarker, notetoJOJO, ask build about teamMarker
//        nav.goDistance(-65f,75f); //go the backwards toward the crater and park in front of it

//In order to face front towards crater// Goes to middle position//red crater
//        nav.rotateTo(110f, 200f, 40f); //get off hook by going to the front
//        sleep(100);
//        nav.rotateTo(0f, 200f, 40f); //turn back to straighten self out
//        sleep(100);
//        nav.goDistance(25f,50f); //go to the middle spot at sampling
//        sleep(100);
//        nav.goDistance(-10f,150); //back up, after hitting the sampling in the middle
//        sleep(100);
//        nav.rotateTo(115f,300f,40f); //turn left to face wall
//        sleep(100);
//        nav.goDistance(-43f,150f); // drive towards wall and get close enough to it
//        sleep(100);
//        nav.rotateTo(65f,400f,40f); //turn right to face depot and square up with wall
//        sleep(100);
//        nav.goDistance(-37f,150f); //drive to depot to drop off teamMarker
//        sleep(100);
//        //some type of servo to drop off the teamMarker, notetoJOJO, ask build about teamMarker
//        nav.goDistance(65f,100f); //go the forwards toward the crater and park in front of it


//In order to face front towards crater//goes to right mineral//red crater

        nav.pointTurn(100f,200,40f);
        sleep(100);
        nav.pointTurn(350f,200f,40f);

//        //nav.rotateTo(f,200,40f);
//        sleep(100);
//        nav.goDistance(25f,50f); //go to the left spot at sampling
//        sleep(100);
//        nav.goDistance(-10f,150); //back up, after hitting the sampling in the middle
//        sleep(100);
//        nav.rotateTo(355f,300f,40f); //turn left to face wall //subject to change
//        sleep(100);
//        nav.goDistance(-48f,150f); // drive towards wall and get close enough to it
//        sleep(100);

//Right Mineral// red corner
//        nav.rotateTo(105f,200,40f);
//        sleep(100);
//        nav.rotateTo(25f,200f,40f);




/* lower, turn out of hook, turn to recenter, read CV: get which one is the yellow guy.
    divide to four different options,
        turn to where you need to go
        go forward to the square mineral, knock it off the square
        go backwards
        turn towards wall
        run certain distance to wall
    turn using same/similar angle to turn towards depot
    run forwards to crater
    place collector in?
*/



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

