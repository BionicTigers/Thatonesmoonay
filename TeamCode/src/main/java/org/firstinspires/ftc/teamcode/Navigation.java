package org.firstinspires.ftc.teamcode;
//EXIST
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.Timer;

/**
 * A class for all movement methods for Rover Ruckus!
 */
public class Navigation{
    public static final String TAG = "Vuforia Navigation Sample";

    //-----tweak values-----//
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
    public enum CollectorHeight {UPPER, LOWER}
    public enum LiftHeight {LOWER, PARK, SCORE}
    public enum CollectorExtension {IN, OUT}

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
    //private Servo flicky;   //
    //private Servo liftyLock;
    private CRServo collecty;  //collection sweeper
    private CRServo trappy;  //collector trapdoor
    private Servo droppy;  //lift motor a
    private Servo droppyJr; //lift motor b

    //-----internal values-----//
    /*
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables vumarks;
    private Location[] vumarkLocations = new Location[4];
    private boolean useAnyCV;
    private WebcamName webcamName;
    private int captureCounter = 0;
    private File captureDirectory= AppUtil.ROBOT_DATA_DIR;
    private Location camLocation = new Location(0f,6f,6f,0f);
    */

    SamplingOrderDetector detector;

    public Navigation(com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter, org.firstinspires.ftc.robotcore.external.Telemetry telemetry, boolean useTelemetry) {
        this.hardwareGetter = hardwareGetter;
        this.telemetry = telemetry;
        this.useTelemetry = useTelemetry;


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
        extendy.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifty = hardwareGetter.hardwareMap.dcMotor.get("lifty");
        liftyJr = hardwareGetter.hardwareMap.dcMotor.get("liftyJr");
        liftyJr.setDirection(DcMotor.Direction.REVERSE);

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
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;
        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;
        detector.enable();
    }

    /**
     * Updates the cube location enumerator using OpenCV. Will not overwrite old data. Access using [nav].cubePos.
     * @return boolean, true if updated, false if not updated or was updated in past.
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
     * Calls DcMotor.RunMode.STOP_AND_RESET_ENCODER for drive motors.
     */
    public void driveEncoderReset() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
    public void rotateTo(float rot, float slowdown, float precision) {
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
    public void rotateTo(Location loc, float slowdown, float precision) {
        rotateTo((float) Math.toDegrees(Math.atan2(loc.getLocation(2) - pos.getLocation(2), loc.getLocation(0) - pos.getLocation(0))), slowdown, precision);
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

    //TODO test values for accurate height
    public void setLiftHeight(LiftHeight position) {
        switch(position) {
            case LOWER:
                setLiftHeight(0);
                break;
            case PARK:
                setLiftHeight(1);
                break;
            case SCORE:
                setLiftHeight(2);
                break;
        }
    }

    public void setCollectionSweeper(float power) {
        collecty.setPower(power);
    }

    public void setCollectorHeight(float position) {
        droppy.setPosition(position);
        droppyJr.setPosition(position);
    }

    //TODO test values for accurate height
    public void setCollectorHeight(CollectorHeight position) {
        switch(position) {
            case LOWER:
                setCollectorHeight(0f);
                break;
            case UPPER:
                setCollectorHeight(10f);
        }
    }

    public void setCollectorExtension(int position) {
        extendy.setTargetPosition(position);
    }

    //TODO test values for accurate extension
    public void setCollectorExtension(CollectorExtension position) {
        switch (position){
            case IN:
                setCollectorExtension(0);
                break;
            case OUT:
                setCollectorExtension(100);
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
        driveEncoderReset();
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
        telemetry.addData("Collector L/E/S/T",lifty.getCurrentPosition()+" "+extendy.getCurrentPosition()+" "+collecty.getPower()+" "+trappy.getPower());
        telemetry.addData("CubePos",cubePos);
        telemetry.update();
    }
}