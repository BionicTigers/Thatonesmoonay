package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name="doublesampling") //goes around the samples and lines up wall, very similar to red crater
public class AutoDoubleSampling extends LinearOpMode {
    public float ralph;
    public float driveForward1;
    public float turnLeft1;
    public float driveBackward1;
    public int setter;
    public float rotate;
    public float depotDrive;
    public float craterDrive;
    public float sampleTurn;
    public float sampleDistance;

    @Override
    public void runOpMode() {
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
        nav.driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (setter == 0) { //right
            ralph = 33f; //One stick
            driveForward1 = 28f;
            turnLeft1 = 110f;
            driveBackward1 = -49f;
            rotate = 50f;
            depotDrive = 42f;

            sampleTurn = 20;
            sampleDistance = 10;

            craterDrive = 65f;
        } else if (setter == 1) { //middle
            ralph = 0f;
            driveForward1 = 25f;
            turnLeft1 = 115f;
            driveBackward1 = -35f;
            rotate = 50f;
            depotDrive = 42f;

            sampleTurn = 70;
            sampleDistance = 10;

            craterDrive = -80f;
        } else if (setter == 2) { //left
            ralph = 325f;
            driveForward1 = 28f;
            turnLeft1 = 40f;
            driveBackward1 = -32;
            rotate = 340f;
            depotDrive = 35f;

            sampleTurn = 0;
            sampleDistance = 10;

            craterDrive = -65f;
        }

        nav.pointTurn(110f, 200f, 40f); //get off hook by going to the right, we are hooked by the butt of the robot
        sleep(100);
        nav.pointTurn(0f, 200f, 40f); //turn back to straighten self out, left
        sleep(100);
        //read CV sampling
        nav.pointTurn(ralph, 200f, 40f); //turn right or left to go to the sample
        nav.goDistance(driveForward1, 50f); //go to the middle spot at sampling
        sleep(100);
        nav.goDistance(-10f, 150); //back up, after hitting the sampling in the middle
        sleep(100);
        nav.pointTurn(turnLeft1, 300f, 40f); //turn right to face wall with butt towards depot
        sleep(100);
        nav.goDistance(-43f, 150f); // drive towards wall and get close enough to it
        sleep(100);
        nav.pointTurn(rotate, 400f, 40f); //turn right to face depot and square up with wall with butt towards depot
        sleep(100);
        nav.goDistance(depotDrive, 150f); //drive to depot to drop off teamMarker
        sleep(100);

        nav.pointTurn(sampleTurn,200f,40f);
        sleep(100);
        nav.goDistance(sampleDistance,50f);
        sleep(50);
        nav.goDistance(-sampleDistance,50f);
        sleep(50);
        nav.pointTurn((360f - sampleTurn),200,40f);
        sleep(50);
        nav.goDistance(60f,50f);
        sleep(50);
        //put code for double sampling
        /* turn to certain degree (to the right to face
            move forward certain distance
            move backsame distance
            turn the reverse direction
         */
        //some type of servo to drop off the teamMarker, notetoJOJO, ask build about teamMarker
        nav.goDistance(craterDrive, 100f); //go the forwards toward the crater and park in front of it
    }
}