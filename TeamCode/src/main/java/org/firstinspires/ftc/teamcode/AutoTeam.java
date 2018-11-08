package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

@Autonomous(name="RedCrater",group="BTBT")
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

    private DcMotor extendy;
    private DcMotor lifty;
    private DcMotor liftyJr;

    //Servos//
    private Servo flicky;
    private Servo liftyLock;
    private CRServo collecty;
    private Servo droppy;
    private Servo droppyJr;

    //Servo Position Storage//
    private double colPos;
    private double dropPos;

    @Override public void runOpMode() {
        //initialization
        //liftrawrh = hardwareMap.servo.get("liftrawrh");
        //flickyWrist = hardwareMap.servo.get("flicky");

        //liftrawrh.setPosition(1.0);
        //flickyWrist.setPosition(0.5)
        Navigation nav = new Navigation(this,telemetry, false);
        setter = 1;
        waitForStart();

        //try determine vision stuff
        // nav.updatePos();
        //nav.updateTeam();

        extendy = hardwareMap.dcMotor.get("extendy");
        lifty = hardwareMap.dcMotor.get("lifty");
        liftyJr = hardwareMap.dcMotor.get("liftyJr");
        liftyJr.setDirection(DcMotor.Direction.REVERSE);

        //Servos//
        //flicky = hardwareMap.servo.get("flicky");
        liftyLock = hardwareMap.servo.get("liftyLock");
        collecty = hardwareMap.crservo.get("collecty");
        droppy = hardwareMap.servo.get("droppy");
        droppyJr = hardwareMap.servo.get("droppyJr");
        nav.updateCubePos();


        nav.getCubePos();

        //try determine vision stuff
        //nav.updatePos();
        //nav.updateTeam();
        nav.updateCubePos();
        nav.driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //landing motor commands
        liftyLock.setPosition(0.5);
        nav.setLiftHeight(300);

        waitForStart();
//          Start lift down motors(2) go down, need method (note to self), reverse one motor

        // liftyLock.setPosition(0.3);
        // droppy.setPosition(0.3);
        // droppyJr.setPosition(0.7);


        while (opModeIsActive()) {
            //FULL CODE//
            nav.driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            //nav.setLiftHeight(300);

            //all//
            nav.pointTurn(110f, 200f, 40f); //get off hook by going to the front
            sleep(100);
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

        }
    }}
