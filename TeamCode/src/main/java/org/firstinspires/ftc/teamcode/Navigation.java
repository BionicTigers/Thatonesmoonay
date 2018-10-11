package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * A class for all movement methods for Rover Ruckus!
 */
public class Navigation{

    //------game element locations-----//
    public static final Location cargoBlueGold = new Location(-8.31f,27f,-8.31f,0f);
    public static final Location cargoBlueSilver = new Location(-8.31f,27f,8.31f,0f);
    public static final Location cargoRedGold = new Location(8.31f,27f,8.31f,0f);
    public static final Location cargoRedSilver = new Location(8.31f,27f,-8.31f,0f);

    //-----enums-----//
    public enum Team {UNKNOWN, REDNORTH, REDSOUTH, BLUENORTH, BLUESOUTH}
    public Team team = Team.UNKNOWN;
    public enum CubePosition {UNKNOWN, LEFT, MIDDLE, RIGHT}
    public CubePosition cubePos = CubePosition.UNKNOWN;

    //-----robot hardware elements-----//
    //if you guys rename these I will cry - Quinn
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor lift;

    //-----robot position and dimensions-----//
    public Location pos = new Location();           //location of robot as [x,y,z,rot] (inches)
    public boolean posHasBeenUpdated = false;
    private float wheelDistance = 6;                //distance from center of robot to center of wheel (inches)
    private float wheelDiameter = 4;                //diameter of wheel (inches)
    private Location camLocation = new Location(0f,6f,6f,0f);

    //-----telemetry-----//
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;


    //-----vuforia------//
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables vumarks;
    private Location[] vumarkLocations = new Location[4];

    //-----tweak values-----//
    private float minimumSlowdownDistance = 10f;    //when executing a goToLocation function, robot will begin slowing this far from destination (inches)
    private float maximumMotorPower = 1f;           //when executing a goToLocation function, robot will never travel faster than this value (percentage 0=0%, 1=100%)
    private float minimumMotorPower = 0.2f;
    private float liftPower = 0.3f;                 //power the lift will run at
    private float killDistance = 0;                 //kills program if robot farther than distance in x or z from origin (inches) (0 means no kill)
    private float encoderCountsPerRev = 537.6f;     //encoder ticks per one revolution

    public Navigation(com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter, org.firstinspires.ftc.robotcore.external.Telemetry tele) {
        telemetry = tele;

        frontLeft = hardwareGetter.hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareGetter.hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareGetter.hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareGetter.hardwareMap.dcMotor.get("backRight");
        lift = hardwareGetter.hardwareMap.dcMotor.get("lift");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("current",1);
        telemetry.update();
        int cameraMonitorViewId = hardwareGetter.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareGetter.hardwareMap.appContext.getPackageName());
        telemetry.addData("current",2);
        telemetry.update();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        telemetry.addData("current",3);
        telemetry.update();
        parameters.vuforiaLicenseKey = " AYSaZfX/////AAABGZyGj0QLiEYhuyrGuO59xV2Jyg9I+WGlfjyEbBxExILR4A183M1WUKucNHp5CnSpDGX5nQ9OD3w5WCfsJuudFyJIJSKZghM+dOlhTWWcEEGk/YB0aOLEJXKK712HpyZqrvwpXOyKDUwIZc1mjWyLT3ZfCmNHQ+ouLKNzOp2U4hRqjbdWf1ZkSlTieiR76IbF6x7MX5ZtRjkWeLR5hWocakIaH/ZPDnqo2A2mIzAzCUa8GCjr80FJzgS9dD77lyoHkJZ/5rNe0k/3HfUZXA+BFSthRrtai1W2/3oRCFmTJekrueYBjM4wuuB5CRqCs4MG/64AzyKOdqmI05YhC1tVa2Vd6Bye1PaMBHmWNfD+5Leq ";
        telemetry.addData("current",4);
        telemetry.update();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        telemetry.addData("current",5);
        telemetry.update();
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        telemetry.addData("current",6);
        telemetry.update();
        vumarks = vuforia.loadTrackablesFromAsset("18-19_rover_ruckus");
        telemetry.addData("current",7);
        telemetry.update();
        vumarkLocations[0] = new Location(0f,5.75f,71.5f,180f); //east
        vumarkLocations[1] = new Location(-71.5f,5.75f,0f,270f); //north
        vumarkLocations[2] = new Location(0f,5.75f,-71.5f,0f); //west
        vumarkLocations[3] = new Location(71.5f,5.75f,0f,90f); //south
        vumarks.activate();
    }

    /**
     * Updates the Robot's position using Vuforia. Value is in inches from center of map (see ccoordinate_diagram.png). Access using [nav].pos.
     * @return boolean, true if updated, false otherwise
     */
    public boolean updatePos() {
        for (int i = 0; i < vumarks.size(); i++) {
            OpenGLMatrix testLocation = ((VuforiaTrackableDefaultListener) vumarks.get(i).getListener()).getPose();
            if (testLocation != null) {
                Location markLocation = new Location(vumarkLocations[i].getLocation(0), vumarkLocations[i].getLocation(1), vumarkLocations[i].getLocation(2), vumarkLocations[i].getLocation(3) - (float)Math.toDegrees(testLocation.get(1,2)));
                markLocation.translateLocal(testLocation.getTranslation().get(1), -testLocation.getTranslation().get(0), testLocation.getTranslation().get(2));
                markLocation.translateLocal(camLocation.getLocation(0),camLocation.getLocation(1),camLocation.getLocation(2));
                markLocation.setRotation(markLocation.getLocation(3) + 180f);
                pos = markLocation;
                posHasBeenUpdated = true;
                if( killDistance!= 0 && (Math.abs(pos.getLocation(0)) >  killDistance || Math.abs(pos.getLocation(2)) >  killDistance)) throw new IllegalStateException("Robot outside of killDistance at pos: " + pos);
                return true;
            }
        }
        return false;
    }

    /**
     * Updates the robot team enumerator using the current position. Access using [nav].team.
     * @return boolean, true if updated, false otherwise.
     */
    public boolean updateTeam() {
        updatePos();
        if(!posHasBeenUpdated) return false;
        float x = pos.getLocation(0);
        float z = pos.getLocation(2);
        if(x <= 0) {
            if(z <= 0) team = Team.BLUESOUTH;
            else team = Team.BLUENORTH;
        }
        else {
            if(z <= 0) team = Team.REDSOUTH;
            else team = Team.REDNORTH;
        }
        return true;
    }

    /**
     * Updates the cube location enumerator using OpenCV. Access using [nav].cubePos.
     * @return boolean, true if updated, false otherwise.
     */
    public boolean updateCubePos() {

        //chirs put code here

        return false;
    }

    /**
     * Sets drive motor powers.
     * @param left power of left two motors as percentage (0-1).
     * @param right power of right two motors as percentage (0-1).
     */
    public void drivePower(float left, float right) {
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }

    /**
     * Sets drive motor target encoder to given values.
     * @param left encoder set for left motors.
     * @param right encoder set for right motors.
     */
    public void drivePosition(int left, int right) {
        frontLeft.setTargetPosition(left);
        backLeft.setTargetPosition(left);
        frontRight.setTargetPosition(right);
        backRight.setTargetPosition(right);
    }

    /**
     * Sets all drive motor run modes to given mode.
     * @param r DcMotor mode to given value.
     */
    public void driveMode(DcMotor.RunMode r) {
        frontLeft.setMode(r);
        backLeft.setMode(r);
        frontRight.setMode(r);
        backRight.setMode(r);
    }

    /**
     * Calls DcMotor.RunMode.STOP_AND_RESET_ENCODER for drive motors.
     */
    public void driveEncoderReset() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        updatePos();
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

        driveMethodComplex(distance, slowdown, precision, frontLeft, 1f, -1f, true, 0.2f, maximumMotorPower);

        pos.setRotation(rot);
        updatePos();
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
     * @param pos Encoder ticks for lift motor
     */
    public void setLift(int pos) {
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(pos);
        lift.setPower(liftPower);
    }

    /**
     * Good luck.
     * @param distance
     * @param slowdown
     * @param precision
     * @param encoderMotor
     * @param lModifier
     * @param rModifier
     * @param doubleBack
     * @param minPower
     * @param maxPower
     */
    private void driveMethodComplex(float distance, float slowdown, float precision, DcMotor encoderMotor, float lModifier, float rModifier, boolean doubleBack, float minPower, float maxPower) {
        distance *= lModifier;

        int initEncoder = encoderMotor.getCurrentPosition();
        int targetEncoder = (int)(distance / (wheelDiameter * Math.PI) * encoderCountsPerRev) + initEncoder;
        int slowdownEncoder = (int)(slowdown / (wheelDiameter * Math.PI) * encoderCountsPerRev);

        driveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while ((targetEncoder - encoderMotor.getCurrentPosition()) > precision || (doubleBack && (targetEncoder - encoderMotor.getCurrentPosition()) < precision)) {
            float uncappedPower = (targetEncoder - encoderMotor.getCurrentPosition()) / (float)slowdownEncoder;
            float power = (uncappedPower < 0 ? -1:1) * Math.min(maxPower, Math.max(minPower, Math.abs(uncappedPower)));
            drivePower(power*lModifier, power*rModifier);
        }
        driveEncoderReset();
    }
}