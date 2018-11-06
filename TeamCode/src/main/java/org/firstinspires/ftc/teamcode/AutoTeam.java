//package org.firstinspires.ftc.teamcode;
////EXIST
//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.DogeCV;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//
//@Autonomous(name="Team Autonomous")
//public class TeamAuto extends LinearOpMode {
//    private Servo liftrawrh;
//    private Servo flickyWrist;
//    public float ralph;
//    public float driveForward1;
//    public float turnLeft1;
//    public float driveBackward1;
//    public int setter;
//    public float rotate;
//    public float depotDrive;
//    public float craterDrive;
//    private GoldAlignDetector detector;
//
//    @Override public void runOpMode() {
//        detector = new GoldAlignDetector();
//        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 1, false);
//        detector.useDefaults();
//
//        // Optional Tuning
//        detector.alignSize = 200; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
//        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
//        detector.downscale = 0.4; // How much to downscale the input frames
//
//        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
//        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
//        detector.maxAreaScorer.weight = 0.005;
//
//        detector.ratioScorer.weight = 5;
//        detector.ratioScorer.perfectRatio = 1.0;
//
//        detector.enable();
//        //initialization
//        //liftrawrh = hardwareMap.servo.get("liftrawrh");
//        //flickyWrist = hardwareMap.servo.get("flicky");
//
//        //liftrawrh.setPosition(1.0);
//        //flickyWrist.setPosition(0.5);
//        Navigation nav = new Navigation(this,telemetry, true, false, false, true);
//        setter = 2;
//        waitForStart();
//
//        //try determine vision stuff
//       // nav.updatePos();
//        //nav.updateTeam();
//
//            nav.updateCubePos();
//
//
//            nav.getCubePos();
//
//        //try determine vision stuff
//        //nav.updatePos();
//        //nav.updateTeam();
//        nav.updateCubePos();
//        nav.driveEncoderReset();
//
//        //landing motor commands
////        liftrawrh.setPosition(0.3);
////          Start lift down motors(2) go down, need method (note to self), reverse one motor
//
//
////        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
////            telemetry.addData("VuMark", "%s visible", vuMark);
////            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
////
////            if (vuMark == RelicRecoveryVuMark.RIGHT) {
////                ralph = 470;
////            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
////                ralph = 540;
////            } else if (vuMark == RelicRecoveryVuMark.LEFT) {
////                ralph = 620;
////            } else {
////                ralph = 540;
////            }
//
//// faces back towards crater// goes to middle postition //red crater
////        nav.rotateTo(110f, 200f, 40f); //get off hook by going to the front
////        sleep(100);
////        nav.rotateTo(0f, 200f, 40f); //turn back to straighten self out
////        sleep(100);
////        nav.goDistance(25f,50f); //go to the middle spot at sampling
////        sleep(100);
////        nav.goDistance(-10f,150); //back up, after hitting the sampling in the middle
////        nav.rotateTo(240f,300f,40f); //turn right to get to wall
////        sleep(100);
////        nav.goDistance(40f,150f); // drive towards wall and get close enough to it
////        sleep(100);
////        nav.rotateTo(185f,400f,40f); //turn left to face depot and square up with wall
////        sleep(100);
////        nav.goDistance(40f,75f); //drive to depot to drop off teamMarker
////        sleep(100);
////        //some type of servo to drop off the teamMarker, notetoJOJO, ask build about teamMarker
////        nav.goDistance(-65f,75f); //go the backwards toward the crater and park in front of it
//
////In order to face front towards crater// Goes to middle position//red crater
////        nav.rotateTo(110f, 200f, 40f); //get off hook by going to the front
////        sleep(100);
////        nav.rotateTo(0f, 200f, 40f); //turn back to straighten self out
////        sleep(100);
////        nav.goDistance(25f,50f); //go to the middle spot at sampling
////        sleep(100);
////        nav.goDistance(-10f,150); //back up, after hitting the sampling in the middle
////        sleep(100);
////        nav.rotateTo(115f,300f,40f); //turn left to face wall
////        sleep(100);
////        nav.goDistance(-43f,150f); // drive towards wall and get close enough to it
////        sleep(100);
////        nav.rotateTo(65f,400f,40f); //turn right to face depot and square up with wall
////        sleep(100);
////        nav.goDistance(-37f,150f); //drive to depot to drop off teamMarker
////        sleep(100);
////        //some type of servo to drop off the teamMarker, notetoJOJO, ask build about teamMarker
////        nav.goDistance(65f,100f); //go the forwards toward the crater and park in front of it
//
////In order to face front towards crater//goes to right mineral//red crater//left
////        nav.rotateTo(100f,200f,40f);
////        sleep(100);
////        nav.rotateTo(0f,200f,40f);
////        sleep(100);
////        nav.rotateTo(325f,200f,40f);
////        sleep(100);
////        nav.goDistance(28f,50f); //go to the middle spot at sampling
////        sleep(100);
////        nav.goDistance(-10f,50f);
////        sleep(100);
////        nav.rotateTo(30f,250f,40f); //turn left to face wall (115) 245
////        sleep(100);
////        nav.goDistance(-35,100);
//
////right
////        nav.rotateTo(100f,200f,40f);
////        sleep(100);
////        nav.rotateTo(0f,200f,40f);
////        sleep(100);
////        nav.rotateTo(33f,200f,40f);
////        sleep(100);
////        nav.goDistance(28f,50f); //go to the middle spot at sampling
////        sleep(100);
////        nav.goDistance(-10f,50f);
////        sleep(100);
////        nav.rotateTo(110f,250f,40f); //turn left to face wall (115) 245
////        sleep(100);
////        nav.goDistance(-49,100);
//
//        //FULL CODE//
//
//
//
//
//        //all//
//        nav.driveEncoderReset();
//        if (!detector.getAligned()){
//            nav.rotateTo(55f, 200f, 40f); //get off hook by going to the front
//            sleep(100);
//            telemetry.addData("right","left");
//            if (detector.getAligned()){setter=0;telemetry.addData("center","left");}
//            else{setter =1;telemetry.addData("left","left"); }}
//            else {setter = 1;}
//
//       nav.rotateTo(55f,200f,40f);
//        telemetry.update();
//        sleep(500);
//
//        if (setter == 0) { //right
//            ralph= 33f; //One stick
//            driveForward1 = 28f;
//            turnLeft1 = 110f;
//            driveBackward1 = -49f;
//            rotate = 50f;
//            depotDrive = -42f;
//            craterDrive = 65f;
//        } else if (setter == 1) { //middle
//            ralph = 0f;
//            driveForward1 = 25f;
//            turnLeft1 = 115f;
//            driveBackward1 = -35f;
//            rotate = 50f;
//            depotDrive = -42f;
//            craterDrive = 80f;
//        }
//        else if (setter == 2){ //left
//            ralph = 325f;
//            driveForward1 = 28f;
//            turnLeft1 = 40f;
//            driveBackward1 = -32;
//            rotate = 340f;
//            depotDrive = -35f;
//            craterDrive = 65f;
//        }
//        telemetry.addData("","outofsampel");
//        telemetry.update();
//
//        nav.rotateTo(55f, 200f, 40f); //get off hook by going to the front
//        sleep(100);
//        nav.rotateTo(0f, 200f, 40f); //turn back to straighten self out
//        sleep(100);
//        nav.rotateTo(ralph,200f,40f);
//        nav.goDistance(driveForward1,50f); //go to the middle spot at sampling
//        sleep(100);
//        nav.goDistance(-10f,150); //back up, after hitting the sampling in the middle
//        sleep(100);
//        nav.rotateTo(turnLeft1,300f,40f); //turn left to face wall
//        sleep(100);
//        nav.goDistance(-43f,150f); // drive towards wall and get close enough to it
//        sleep(100);
//        nav.rotateTo(rotate,400f,40f); //turn right to face depot and square up with wall
//        sleep(100);
//        nav.goDistance(depotDrive,150f); //drive to depot to drop off teamMarker
//        sleep(100);
//        //some type of servo to drop off the teamMarker, notetoJOJO, ask build about teamMarker
//        nav.goDistance(craterDrive,100f); //go the forwards toward the crater and park in front of it
//
//
////        nav.goDistance(-43f,150f); // drive towards wall and get close enough to it
////        sleep(100);
//
////In order to face front towards crater//goes to the left mineral//red crater
////        nav.rotateTo(100f,200f,40f);
////        sleep(100);
////        nav.rotateTo(0f,200f,40f);
////        sleep(100);
////        nav.rotateTo(25f,200f,40f);
////
//
//        //forward to mineral left and right
//        //nav.goDistance(26f,50f); //go to the middle spot at sampling
//       //sleep(100);
//
////turn to wall
////        nav.rotateTo(90f,300f,40f); //turn left to face wall (115)
////        sleep(100);
////        nav.goDistance(-43f,150f); // drive towards wall and get close enough to it
////        sleep(100);
//
//
////        //nav.rotateTo(f,200,40f);
////        sleep(100);
////        nav.goDistance(25f,50f); //go to the left spot at sampling
////        sleep(100);
////        nav.goDistance(-10f,150); //back up, after hitting the sampling in the middle
////        sleep(100);
////        nav.rotateTo(355f,300f,40f); //turn left to face wall //subject to change
////        sleep(100);
////        nav.goDistance(-48f,150f); // drive towards wall and get close enough to it
////        sleep(100);
//
////Right Mineral// red corner
////        nav.rotateTo(105f,200,40f);
////        sleep(100);
////        nav.rotateTo(25f,200f,40f);
//
//
//
//
///* lower, turn out of hook, turn to recenter, read CV: get which one is the yellow guy.
//        divide to four different options,
//            turn to where you need to go
//            go forward to the square mineral, knock it off the square
//            go backwards
//            turn towards wall
//            run certain distance to wall
//        turn using same/similar angle to turn towards depot
//        run forwards to crater
//        place collector in?
// */
//
//
//
//        //try determine vision stuff
//        //nav.updatePos();
//
////        nav.updateTeam();
////        nav.updateCubePos();
//
//        //go to cube
//       // nav.setLift(100);
//        //collect/nudge
//
//        //goto depot
//
//        //deploy team marker thing
//
//        //go to appropriate crater
//
//        //begin collection
//
//        //parking
//    }
//}
package org.firstinspires.ftc.teamcode;
//EXIST
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name="Auto Team", group="Auto")
public class AutoTeam extends LinearOpMode {
    private Servo liftrawrh;
    private Servo flickyWrist;
    public float ralph;
    public float driveForward1;
    public float turnLeft1;
    public float driveBackward1;
    public int setter;
    public float rotate;
    public float depotDrive;
    public float craterDrive;
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
        Navigation nav = new Navigation(this,telemetry, true);
        setter = 2;
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

//In order to face front towards crater//goes to right mineral//red crater//left
//        nav.rotateTo(100f,200f,40f);
//        sleep(100);
//        nav.rotateTo(0f,200f,40f);
//        sleep(100);
//        nav.rotateTo(325f,200f,40f);
//        sleep(100);
//        nav.goDistance(28f,50f); //go to the middle spot at sampling
//        sleep(100);
//        nav.goDistance(-10f,50f);
//        sleep(100);
//        nav.rotateTo(30f,250f,40f); //turn left to face wall (115) 245
//        sleep(100);
//        nav.goDistance(-35,100);

//right
//        nav.rotateTo(100f,200f,40f);
//        sleep(100);
//        nav.rotateTo(0f,200f,40f);
//        sleep(100);
//        nav.rotateTo(33f,200f,40f);
//        sleep(100);
//        nav.goDistance(28f,50f); //go to the middle spot at sampling
//        sleep(100);
//        nav.goDistance(-10f,50f);
//        sleep(100);
//        nav.rotateTo(110f,250f,40f); //turn left to face wall (115) 245
//        sleep(100);
//        nav.goDistance(-49,100);

        //FULL CODE//
        for (int i=0; i++ <2;) {
            if (detector.getAligned()) {
                setter = i;
                i+=5000;
            } else {
                turnLeft1 = 110f;
                sleep(500);
            }
        }



        //all//
        if (detector.getAligned()){setter=1;}
        nav.pointTurn(55f, 200f, 40f);
        if (detector.getAligned()){setter=2;}else {setter=0;}//get off hook by going to the front
        sleep(100);
        nav.pointTurn(55f,200f,40f);
        sleep(100);
        if (setter == 0) { //right
            ralph= 33f; //One stick
            driveForward1 = 28f;
            turnLeft1 = 110f;
            driveBackward1 = -49f;
            rotate = 50f;
            depotDrive = -42f;
            craterDrive = 65f;
        } else if (setter == 1) { //middle
            ralph = 0f;
            driveForward1 = 25f;
            turnLeft1 = 115f;
            driveBackward1 = -35f;
            rotate = 50f;
            depotDrive = -42f;
            craterDrive = 80f;
        }
        else if (setter == 2){ //left
            ralph = 325f;
            driveForward1 = 28f;
            turnLeft1 = 40f;
            driveBackward1 = -32;
            rotate = 340f;
            depotDrive = -35f;
            craterDrive = 65f;
        }
        nav.pointTurn(0f, 200f, 40f); //turn back to straighten self out
        sleep(100);
        nav.pointTurn(ralph,200f,40f);
        nav.goDistance(driveForward1,50f); //go to the middle spot at sampling
        sleep(100);
        nav.goDistance(-10f,150); //back up, after hitting the sampling in the middle
        sleep(100);
        nav.pointTurn(turnLeft1,300f,40f); //turn left to face wall
        sleep(100);
        nav.goDistance(-43f,150f); // drive towards wall and get close enough to it
        sleep(100);
        nav.pointTurn(rotate,400f,40f); //turn right to face depot and square up with wall
        sleep(100);
        nav.goDistance(depotDrive,150f); //drive to depot to drop off teamMarker
        sleep(100);
        //some type of servo to drop off the teamMarker, notetoJOJO, ask build about teamMarker
        nav.goDistance(craterDrive,100f); //go the forwards toward the crater and park in front of it


//        nav.goDistance(-43f,150f); // drive towards wall and get close enough to it
//        sleep(100);

//In order to face front towards crater//goes to the left mineral//red crater
//        nav.rotateTo(100f,200f,40f);
//        sleep(100);
//        nav.rotateTo(0f,200f,40f);
//        sleep(100);
//        nav.rotateTo(25f,200f,40f);
//

        //forward to mineral left and right
        //nav.goDistance(26f,50f); //go to the middle spot at sampling
        //sleep(100);

//turn to wall
//        nav.rotateTo(90f,300f,40f); //turn left to face wall (115)
//        sleep(100);
//        nav.goDistance(-43f,150f); // drive towards wall and get close enough to it
//        sleep(100);


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