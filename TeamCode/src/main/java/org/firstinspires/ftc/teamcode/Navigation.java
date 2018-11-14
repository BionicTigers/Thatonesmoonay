package org.firstinspires.ftc.teamcode;
//EXIST
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
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

import java.io.File;

/**
 * A class for all movement methods for Rover Ruckus.
 */
public class Navigation{

    //-----tweak values-----//
    private float maximumMotorPower = 0.5f;           //when executing a goToLocation function, robot will never travel faster than this value (percentage 0=0%, 1=100%)
    private float minimumMotorPower = 0.2f;
    private float encoderCountsPerRev = 537.6f;     //encoder ticks per one revolution
    private boolean useTelemetry;

    //-----enums-----//
    public enum CubePosition {UNKNOWN, LEFT, MIDDLE, RIGHT}
    private CubePosition cubePos = CubePosition.UNKNOWN;
    public enum CollectorHeight {COLLECT, DUMP}
    public enum LiftHeight {LOWER, HOOK, SCORE}
    public enum CollectorExtension {PARK, DUMP, OUT}
    public enum LiftLock {LOCK,UNLOCK}
    public enum CollectorSweeper {INTAKE,OUTTAKE}

    //-----robot hardware, position, and dimensions-----//
    private com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter;
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    private float wheelDistance = 6;                //distance from center of robot to center of wheel (inches)
    private float wheelDiameter = 4;                //diameter of wheel (inches)
    private Location pos = new Location();           //location of robot as [x,y,z,rot] (inches / degrees)

    //-----motors-----//
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor extendy; //collector extension
    private DcMotor lifty;  //lift motor a
    private DcMotor liftyJr; //lift motor b

    //-----servos-----//
    private Servo droppy;  //collection lift a
    private Servo droppyJr; //collection lift b
    private CRServo collecty;  //collection sweeper
    private Servo liftyLock; //lift lock

    //-----internal values-----//
    SamplingOrderDetector detector;

    public Navigation(com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter, org.firstinspires.ftc.robotcore.external.Telemetry telemetry, boolean useTelemetry) {
        this.hardwareGetter = hardwareGetter;
        this.telemetry = telemetry;
        this.useTelemetry = useTelemetry;

        //-----motors-----//
        frontLeft = hardwareGetter.hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareGetter.hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareGetter.hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareGetter.hardwareMap.dcMotor.get("backRight");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendy = hardwareGetter.hardwareMap.dcMotor.get("extendy");
        extendy.setDirection(DcMotorSimple.Direction.REVERSE);
        extendy.setPower(1f);
        extendy.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendy.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lifty = hardwareGetter.hardwareMap.dcMotor.get("lifty");
        lifty.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifty.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lifty.setPower(1);

        liftyJr = hardwareGetter.hardwareMap.dcMotor.get("liftyJr");
        liftyJr.setDirection(DcMotor.Direction.REVERSE);
        liftyJr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftyJr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftyJr.setPower(1);

        //-----servos-----//
        liftyLock = hardwareGetter.hardwareMap.servo.get("liftyLock");
        collecty = hardwareGetter.hardwareMap.crservo.get("collecty");
        droppy = hardwareGetter.hardwareMap.servo.get("droppy");
        droppyJr = hardwareGetter.hardwareMap.servo.get("droppyJr");
        droppyJr.setDirection(Servo.Direction.REVERSE);

        //-----doge cv-----//
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
        driveMethodComplex(-distance, slowdown, 0f, frontLeft, 1f, 1f, false, minimumMotorPower, maximumMotorPower);
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

        driveMethodComplex(distance, slowdown, precision, frontLeft, 1f, -1f, true, 0.05f, 0.25f);
        //driveMethodSimple(distance, -distance, 0.25f, 0.25f);


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
        lifty.setTargetPosition(position);
        liftyJr.setTargetPosition(position);
        while(lifty.isBusy()) {
            if(useTelemetry) telemetryMethod();
        }
    }

    public void setLiftHeight(LiftHeight position) {
        switch(position) {
            case LOWER:
                setLiftHeight(0);
                break;
            case HOOK:
                setLiftHeight(2300);
                break;
            case SCORE:
                setLiftHeight(8200);
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
            case COLLECT:
                setCollectorHeight(0f);
                break;
            case DUMP:
                setCollectorHeight(0.8f);
                break;
        }
    }

    public void setCollectorExtension(int position) {
        extendy.setTargetPosition(position);
        while(extendy.isBusy()) {
            if(useTelemetry) telemetryMethod();
        }
    }

    public void setCollectorExtension(CollectorExtension position) {
        switch (position){
            case PARK:
                setCollectorExtension(0);
                break;
            case DUMP:
                setCollectorExtension(500);
                break;
            case OUT:
                setCollectorExtension(1600);
                break;
        }
    }

    public void setLiftLock(float position) {
        liftyLock.setPosition(position);
        try
        {
            Thread.sleep(1000);
        }
        catch(InterruptedException ex)
        {
            Thread.currentThread().interrupt();
        }
    }

    public void setLiftLock(LiftLock position) {
        switch(position) {
            case LOCK:
                setLiftLock(0.9f);
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
        telemetry.addData("Collector L/E/C",lifty.getCurrentPosition()+" "+extendy.getCurrentPosition()+" "+collecty.getPower());
        telemetry.addData("Pos",pos);
        telemetry.addData("CubePos",cubePos);
        telemetry.update();
    }
}