package org.firstinspires.ftc.teamcode;
//EXIST
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

/**
 * A class for all movement methods for Rover Ruckus.
 */
public class Navigation{
    public static final String TAG = "Vuforia Navigation Sample";

    private float maximumMotorPower = 1f;           //when executing a goToLocation function, robot will never travel faster than this value (percentage 0=0%, 1=100%)
    private float minimumMotorPower = 0.2f;
    private float liftPower = 0.3f;                 //power the lift will run at
    private float encoderCountsPerRev = 537.6f;     //encoder ticks per one revolution
    private boolean useTelemetry;

    //------game element locations-----//
    public static final Location cargoBlueGold = new Location(-8.31f,27f,-8.31f,0f);
    public static final Location cargoBlueSilver = new Location(-8.31f,27f,8.31f,0f);
    public static final Location cargoRedGold = new Location(8.31f,27f,8.31f,0f);
    public static final Location cargoRedSilver = new Location(8.31f,27f,-8.31f,0f);

    //-----enums-----//
    public enum CubePosition {UNKNOWN, LEFT, MIDDLE, RIGHT}
    private CubePosition cubePos = CubePosition.UNKNOWN;
    public enum CollectorHeight {LOWER, DUMP, PARK}
    public enum LiftHeight {LOWER, PARK, SCORE}
    public enum CollectorExtension {PARK, DUMP, OUT}
    public enum LiftLock {LOCK,UNLOCK}
    public enum CollectorSweeper {INTAKE,OUTTAKE}

    //-----robot hardware, position, and dimensions-----//
    private com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter;
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    private float wheelDistance = 6;                //distance from center of robot to center of wheel (inches)
    private float wheelDiameter = 4;                //diameter of wheel (inches)
    private Location pos = new Location();           //location of robot as [x,y,z,rot] (inches / degrees)

    //Drivetrain Motors//
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //Other Motors//
    private DcMotor extendy; //collector extension
    private DcMotor lifty;  //collector lift a
    private DcMotor liftyJr; //collector lift b

    //Servos//
    private Servo liftyLock;
    private CRServo collecty;  //collection sweeper
    private CRServo trappy;  //collector trapdoor
    private Servo droppy;  //lift motor a
    private Servo droppyJr; //lift motor b
    // Setup variables
    private ElapsedTime runtime = new ElapsedTime();
    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private Location camLocation = new Location(0f,6f,6f,0f);
    //private Location pos = new Location();


    // Vuforia variables
    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;
    private Dogeforia vuforia;
    private WebcamName webcamName;
    private SamplingOrderDetector detector;

    //    private VuforiaLocalizer vuforia;
    private VuforiaTrackables vumarks;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private Location[] vumarkLocations = new Location[4];

    public Navigation(com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter, org.firstinspires.ftc.robotcore.external.Telemetry telemetry, boolean useTelemetry) {
        this.hardwareGetter = hardwareGetter;
        this.telemetry = telemetry;
        this.useTelemetry = useTelemetry;
        // VUFORIA INIT FOR DOGE//
        webcamName = hardwareGetter.hardwareMap.get(WebcamName.class, "Webcam 1");

        // Set up parameters for Vuforia
        int cameraMonitorViewId = hardwareGetter.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareGetter.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Vuforia licence key
        parameters.vuforiaLicenseKey = " AYSaZfX/////AAABGZyGj0QLiEYhuyrGuO59xV2Jyg9I+WGlfjyEbBxExILR4A183M1WUKucNHp5CnSpDGX5nQ9OD3w5WCfsJuudFyJIJSKZghM+dOlhTWWcEEGk/YB0aOLEJXKK712HpyZqrvwpXOyKDUwIZc1mjWyLT3ZfCmNHQ+ouLKNzOp2U4hRqjbdWf1ZkSlTieiR76IbF6x7MX5ZtRjkWeLR5hWocakIaH/ZPDnqo2A2mIzAzCUa8GCjr80FJzgS9dD77lyoHkJZ/5rNe0k/3HfUZXA+BFSthRrtai1W2/3oRCFmTJekrueYBjM4wuuB5CRqCs4MG/64AzyKOdqmI05YhC1tVa2Vd6Bye1PaMBHmWNfD+5Leq ";
        parameters.fillCameraMonitorViewParent = true;

        // Set camera name for Vuforia config
        parameters.cameraName = webcamName;

        // Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();


        //Setup trackables
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);


        final int CAMERA_FORWARD_DISPLACEMENT  = 110;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 0;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES,
                        CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        // Activate the targets
        targetsRoverRuckus.activate();


        //Drivetrain Motors//
        frontLeft = hardwareGetter.hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareGetter.hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareGetter.hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareGetter.hardwareMap.dcMotor.get("backRight");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Other Motors//
        extendy = hardwareGetter.hardwareMap.dcMotor.get("extendy");
        extendy.setDirection(DcMotorSimple.Direction.REVERSE);
        extendy.setPower(0.5f);
        extendy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendy.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifty = hardwareGetter.hardwareMap.dcMotor.get("lifty");
        lifty.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftyJr = hardwareGetter.hardwareMap.dcMotor.get("liftyJr");
        liftyJr.setDirection(DcMotor.Direction.REVERSE);
        liftyJr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servos//
        //flicky = hardwareGetter.hardwareMap.servo.get("flicky");
        //liftyLock = hardwareGetter.hardwareMap.servo.get("liftyLock");
        collecty = hardwareGetter.hardwareMap.crservo.get("collecty");
        trappy = hardwareGetter.hardwareMap.crservo.get("trappy");
        droppy = hardwareGetter.hardwareMap.servo.get("droppy");
        droppyJr = hardwareGetter.hardwareMap.servo.get("droppyJr");
        droppyJr.setDirection(Servo.Direction.REVERSE);

        //DOGE CV
        detector = new SamplingOrderDetector();
        detector.init(hardwareGetter.hardwareMap.appContext,CameraViewDisplay.getInstance(),0,false);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;
        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();
    }

    /**
     * Updates the cube location enumerator using OpenCV. Access using [nav].cubePos.
     * @return boolean, true if updated, false if not updated.
     */
    public boolean updateCubePos() {

        SamplingOrderDetector.GoldLocation order = detector.getCurrentOrder();
        if(order == SamplingOrderDetector.GoldLocation.UNKNOWN) return false;

        switch(order) {
            case LEFT:
                cubePos = CubePosition.LEFT;
                break;
            case CENTER:
                cubePos = CubePosition.MIDDLE;
                break;
            case RIGHT:
                cubePos = CubePosition.RIGHT;
                break;
        }
        return true;
    }

    public CubePosition getCubePos() {
        return cubePos;
    }

    /**
     * Sets drive motor powers.
     * @param left power of left two motors as percentage (0-1).
     * @param right power of right two motors as percentage (0-1).
     */
    public void drivePower(float left, float right) {
        frontLeft.setPower(left);
        frontRight.setPower(right);
        backLeft.setPower(left);
        backRight.setPower(right);
    }

    /**
     * Sets drive motor target encoder to given values.
     * @param left encoder set for left motors.
     * @param right encoder set for right motors.
     */
    public void drivePosition(int left, int right) {
        frontLeft.setTargetPosition(left);
        frontRight.setTargetPosition(right);
        backLeft.setTargetPosition(left);
        backRight.setTargetPosition(right);
    }
//a
    /**
     * Sets all drive motor run modes to given mode.
     * @param r DcMotor mode to given value.
     */
    public void driveMode(DcMotor.RunMode r) {
        frontLeft.setMode(r);
        frontRight.setMode(r);
        backLeft.setMode(r);
        backRight.setMode(r);
    }


    /**
     * Stops all drive motors and resets encoders.
     */
    public void stopAllMotors() {
        drivePower(0f,0f);
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Pseudo PID to drive the given distance. Will slow down from maximumMotorPower to minimumMotorPower starting at [slowdown] behind target.
     * @param distance Distance to drive forward in inches.
     * @param slowdown Distance to start linearly slowing down before target position.
     */
    public void goDistance(float distance, float slowdown) {
        driveMethodComplex(distance, slowdown, 0f, frontLeft, 1f, 1f, false, minimumMotorPower, maximumMotorPower);
        pos.translateLocal(distance);
    }

    /**
     * Pseudo PID to rotate the given rotation. Will slow down from maximumMotorPower to minimumMotorPower starting at [slowdown] behind target azimuth.
     * Precision will stop making adjustments once it is within given degrees of target azimuth.
     * @param rot Target azimuth in degrees
     * @param slowdown Degrees behind target rotation to slow down
     * @param precision Degrees of imprecision in rotation value
     */
    public void pointTurn(float rot, float slowdown, float precision) {
        float rota = (rot - pos.getLocation(3)) % 360f;
        float rotb = -(360f - rota);
        float optimalRotation = (Math.abs(rota) < Math.abs(rotb) ? rota : rotb); //selects shorter rotation
        float distance = (float)(Math.toRadians(optimalRotation) * wheelDistance); //arc length of turn (radians * radius)
        slowdown = (float)(Math.toRadians(slowdown) * wheelDistance);

        driveMethodComplex(distance, slowdown, precision, frontLeft, 1f, -1f, true, 0.05f, maximumMotorPower);

        pos.setRotation(rot);
    }

    /**
     * Pseudo PID to rotate to face the given Location. Will slow down from maximumMotorPower to minimumMotorPower starting at [slowdown] behind target azimuth.
     * Precision will stop making adjustments once it is within given degrees of target azimuth.
     * @param loc Target Location object
     * @param slowdown Degrees behind target rotation to slow down
     * @param precision Degrees of imprecision in rotation value
     */
    public void pointTurn(Location loc, float slowdown, float precision) {
        pointTurn((float) Math.toDegrees(Math.atan2(loc.getLocation(2) - pos.getLocation(2), loc.getLocation(0) - pos.getLocation(0))), slowdown, precision);
    }

    public void pointTurnRelative(float rot, float slowdown, float precision) {
        pointTurn(pos.getLocation(3)+rot,slowdown,precision);
    }

    /**
     * Sets lift motor to given encoder position
     * @param position Encoder ticks for lift motor
     */
    public void setLiftHeight(int position) {
        lifty.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftyJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifty.setTargetPosition(position);
        liftyJr.setTargetPosition(position);
        lifty.setPower(liftPower);
        liftyJr.setPower(liftPower);
    }

    public void setLiftHeight(LiftHeight position) {
        switch(position) {
            case LOWER:
                setLiftHeight(0);
                break;
            case PARK:
                setLiftHeight(0);
                break;
            case SCORE:
                setLiftHeight(-9000);
                break;
        }
    }

    public void setCollectionSweeper(float power) {
        collecty.setPower(power);
    }

    public void setCollectionSweeper(CollectorSweeper power) {
        switch(power) {
            case INTAKE:
                setCollectionSweeper(0.5f);
                break;
            case OUTTAKE:
                setCollectionSweeper(-0.5f);
                break;
        }
    }

    public void setCollectorHeight(float position) {
        droppy.setPosition(position);
        droppyJr.setPosition(position);
    }

    public void setCollectorHeight(CollectorHeight position) {
        switch(position) {
            case LOWER:
                setCollectorHeight(0f);
                break;
            case DUMP:
                setCollectorHeight(0.8f);
                break;
            case PARK:
                setCollectorHeight(0.9f);
                break;
        }
    }

    public void setCollectorExtension(int position) {
        extendy.setTargetPosition(position);
    }

    public void setCollectorExtension(CollectorExtension position) {
        switch (position){
            case PARK:
                setCollectorExtension(0);
                break;
            case DUMP:
                setCollectorExtension(-500);
                break;
            case OUT:
                setCollectorExtension(-1600);
                break;
        }
    }

    public void setLiftLock(float position) {
        liftyLock.setPosition(position);
    }

    public void setLiftLock(LiftLock position) {
        switch(position) {
            case LOCK:
                setLiftLock(0.7f);
                break;
            case UNLOCK:
                setLiftLock(0.2f);
                break;
        }
    }

    private void driveMethodComplex(float distance, float slowdown, float precision, DcMotor encoderMotor, float lModifier, float rModifier, boolean doubleBack, float minPower, float maxPower) {
        distance *= lModifier;

        int initEncoder = encoderMotor.getCurrentPosition();
        int targetEncoder = (int)(distance / (wheelDiameter * Math.PI) * encoderCountsPerRev) + initEncoder;
        int slowdownEncoder = (int)(slowdown / (wheelDiameter * Math.PI) * encoderCountsPerRev);
        int premodifier = (targetEncoder > 0) ? 1 : -1;
        driveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while ((!doubleBack && (targetEncoder - encoderMotor.getCurrentPosition())*premodifier > precision) || (doubleBack && Math.abs(targetEncoder - encoderMotor.getCurrentPosition()) > precision)) {
            float uncappedPower = (targetEncoder - encoderMotor.getCurrentPosition()) / (float)slowdownEncoder;
            float power = (uncappedPower < 0 ? -1:1) * Math.min(maxPower, Math.max(minPower, Math.abs(uncappedPower)));
            drivePower(power*lModifier, power*rModifier);
            if(useTelemetry) telemetryMethod();
        }
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveMethodSimple(float distanceL, float distanceR, float LPower, float RPower) {
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int l = (int)(distanceL / (wheelDiameter * Math.PI) * encoderCountsPerRev);
        int r = (int)(distanceR / (wheelDiameter * Math.PI) * encoderCountsPerRev);
        drivePosition(l,r);
        drivePower(LPower,RPower);
        while(frontLeft.isBusy()) {
            if(useTelemetry) telemetryMethod();
        }
    }

    /**
     * A simple method to output the status of all motors and other variables to telemetry.
     */
    public void telemetryMethod() {
        String motorString = "FL-" + frontLeft.getCurrentPosition() + " BL-" + backLeft.getCurrentPosition() + " FR-" + frontRight.getCurrentPosition() + " BR-" + backRight.getCurrentPosition();
        telemetry.addData("Drive", motorString);
        telemetry.addData("Lift",lifty.getCurrentPosition()+" " +liftyJr.getCurrentPosition());
        telemetry.addData("Collector L/E/Sw/T",lifty.getCurrentPosition()+" "+extendy.getCurrentPosition()+" "+collecty.getPower()+" "+trappy.getPower());
        telemetry.addData("Pos",pos);
        telemetry.addData("CubePos",cubePos);
        telemetry.update();
    }
    public void stopVuforia(){
        vuforia.stop();
    }
}