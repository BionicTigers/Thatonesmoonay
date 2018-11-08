package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

    @Autonomous(name="BlueDepot") //Goes through the depot and sampling
    public class AutoDepotDrive extends LinearOpMode {
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

        @Override public void runOpMode() {
            //initialization
            //liftrawrh = hardwareMap.servo.get("liftrawrh");
            //flickyWrist = hardwareMap.servo.get("flicky");

            //liftrawrh.setPosition(1.0);
            //flickyWrist.setPosition(0.5);
            Navigation nav = new Navigation(this, telemetry, true);
            setter = 1;
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


            //FULL CODE//

            if (setter == 0) { //right
                ralph = 33f; //One stick
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
            } else if (setter == 2) { //left
                ralph = 325f;
                driveForward1 = 28f;
                turnLeft1 = 40f;
                driveBackward1 = -32;
                rotate = 340f;
                depotDrive = -35f;
                craterDrive = 65f;
            }


            //all//
            nav.pointTurn(110f, 200f, 40f); //get off hook by going to the front
            sleep(100);
            nav.pointTurn(0f, 200f, 40f); //turn back to straighten self out
            sleep(100);
            nav.pointTurn(ralph, 200f, 40f);
            nav.goDistance(driveForward1, 50f); //go to the middle spot at sampling
            sleep(100);
            nav.goDistance(-10f, 150); //back up, after hitting the sampling in the middle
            sleep(100);
            nav.pointTurn(turnLeft1, 300f, 40f); //turn left to face wall
            sleep(100);
            nav.goDistance(-43f, 150f); // drive towards wall and get close enough to it
            sleep(100);
            nav.pointTurn(rotate, 400f, 40f); //turn right to face depot and square up with wall
            sleep(100);
            nav.goDistance(depotDrive, 150f); //drive to depot to drop off teamMarker
            sleep(100);
            //some type of servo to drop off the teamMarker, notetoJOJO, ask build about teamMarker
            nav.goDistance(craterDrive, 100f); //go the forwards toward the crater and park in front of it

            //        nav.pointTurn(110f, 200f, 40f); //get off hook by going to the front
//        sleep(100);
//        nav.pointTurn(0f, 200f, 40f); //turn back to straighten self out
//        sleep(100);
//        nav.pointTurn(33f,200f,40f);
//        nav.goDistance(28f,50f); //go to the middle spot at sampling
//        sleep(100);
//        nav.goDistance(-10f,100);    //back up, after hitting the sampling in the middle
//        sleep(100);
//
//        nav.pointTurn(220f,100,40f);
//        sleep(200);
//        nav.goDistance(-30,50f); //go to the middle spot at sampling
//        sleep(100);
//        nav.pointTurn(125f,300f,40f); //turn left to face wall
//        sleep(100);
//        nav.goDistance(-15f,150f); // drive towards depot and get close enough to it
//        sleep(100);
//        nav.pointTurn(80f,400f,40f); //turn right to face depot and square up with wall
//        sleep(100);
//        nav.goDistance(-5,150f); //drive to depot to drop off teamMarker
//        sleep(100);
//        nav.pointTurn(240f,100f,40f);
            //some type of servo to drop off the teamMarker, note to JOJO, ask build about teamMarker
//        nav.goDistance(craterDrive,100f); //go the forwards toward the crater and park in front of it
        }}

