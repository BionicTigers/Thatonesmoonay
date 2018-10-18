package org.firstinspires.ftc.teamcode;
//EXIST
import android.graphics.Bitmap;
import android.graphics.PixelFormat;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import com.vuforia.Image;

/**
 * A class for all movement and calculation methods for Rover Ruckus!
 */
public class Navigation{

    //-----tweak values-----//
    private float liftPower = 0.3f;                 //power at which the lift will run
    private float encoderCountsPerRev = 537.6f;     //encoder ticks per one revolution (NevRest 20)
    private boolean useTelemetry = false;           //display motor values when running etc
    private boolean nothingButDrive = false;        //use only drive motors (for testing)
    private boolean twoDriveWheels = false;         //use two drive motors instead of four (for "prototype" bot)
    private boolean useAnyCV;                       //use any openCV or Vuforia methods (for when Chris is dying but we need to run code)
    private DcMotor encoderMotor;                   //motor to use in encoder-based methods

    //------game element locations-----//
    public static final Location cargoBlueGold = new Location(-8.31f,27f,-8.31f,0f);
    public static final Location cargoBlueSilver = new Location(-8.31f,27f,8.31f,0f);
    public static final Location cargoRedGold = new Location(8.31f,27f,8.31f,0f);
    public static final Location cargoRedSilver = new Location(8.31f,27f,-8.31f,0f);

    //-----enums-----//
    public enum Team {UNKNOWN, REDNORTH, REDSOUTH, BLUENORTH, BLUESOUTH}
    private Team team = Team.UNKNOWN;
    public enum CubePosition {UNKNOWN, LEFT, MIDDLE, RIGHT}
    private CubePosition cubePos = CubePosition.UNKNOWN;

    //-----robot hardware, position, and dimensions-----//
    private com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor lift;
    private Location pos = new Location();           //location of robot as [x,y,z,rot] (inches / degrees)
    private boolean posHasBeenUpdated = false;       //used with methods that require a vuforia input as not to produce inaccurate results
    private float wheelDistance = 6;                //distance from center of robot to center of wheel (inches)
    private float wheelDiameter = 4;                //diameter of wheel (inches)
    private Location camLocation = new Location(0f,6f,6f,0f);

    //-----internal values-----//
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    private VuforiaLocalizer vuforia;
    private Dogeforia dogeforia;
    private VuforiaTrackables vumarks;
    private Location[] vumarkLocations = new Location[4];
    private SamplingOrderDetector detector;

    /**
     * Constructs and initializes a Navigation class
     * @param hardwareGetter used to calls for motors, sensors, cameras, etc.
     * @param telemetry used to output motor values to telemetry
     * @param nothingButDrive if true, init only drive motors
     * @param twoDriveWheels if true, init frontLeft and frontRight only, does not ever call rear motors
     * @param useAnyCV if true, init Vuforia and OpenCV
     * @param useTelemetry if true, will output motor values to telemetry
     */
    public Navigation(com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter, org.firstinspires.ftc.robotcore.external.Telemetry telemetry, boolean nothingButDrive, boolean twoDriveWheels, boolean useAnyCV, boolean useTelemetry) {
        this.hardwareGetter = hardwareGetter;
        this.telemetry = telemetry;
        this.nothingButDrive = nothingButDrive;
        this.twoDriveWheels = twoDriveWheels;
        this.useAnyCV = useAnyCV;
        this.useTelemetry = useTelemetry;


        frontLeft = hardwareGetter.hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareGetter.hardwareMap.dcMotor.get("frontRight");
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        if(!twoDriveWheels) {
            backLeft = hardwareGetter.hardwareMap.dcMotor.get("backLeft");
            backRight = hardwareGetter.hardwareMap.dcMotor.get("backRight");
            backRight.setDirection(DcMotor.Direction.REVERSE);
        }

        if(!nothingButDrive) {
            lift = hardwareGetter.hardwareMap.dcMotor.get("lift");
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        this.encoderMotor = frontLeft;
        driveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if(useAnyCV) {
            int cameraMonitorViewId = hardwareGetter.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareGetter.hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
            parameters.vuforiaLicenseKey = " AYSaZfX/////AAABGZyGj0QLiEYhuyrGuO59xV2Jyg9I+WGlfjyEbBxExILR4A183M1WUKucNHp5CnSpDGX5nQ9OD3w5WCfsJuudFyJIJSKZghM+dOlhTWWcEEGk/YB0aOLEJXKK712HpyZqrvwpXOyKDUwIZc1mjWyLT3ZfCmNHQ+ouLKNzOp2U4hRqjbdWf1ZkSlTieiR76IbF6x7MX5ZtRjkWeLR5hWocakIaH/ZPDnqo2A2mIzAzCUa8GCjr80FJzgS9dD77lyoHkJZ/5rNe0k/3HfUZXA+BFSthRrtai1W2/3oRCFmTJekrueYBjM4wuuB5CRqCs4MG/64AzyKOdqmI05YhC1tVa2Vd6Bye1PaMBHmWNfD+5Leq ";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            vuforia = ClassFactory.createVuforiaLocalizer(parameters);
            vumarks = vuforia.loadTrackablesFromAsset("18-19_rover_ruckus");
            vumarkLocations[0] = new Location(0f, 5.75f, 71.5f, 180f); //east
            vumarkLocations[1] = new Location(-71.5f, 5.75f, 0f, 270f); //north
            vumarkLocations[2] = new Location(0f, 5.75f, -71.5f, 0f); //west
            vumarkLocations[3] = new Location(71.5f, 5.75f, 0f, 90f); //south
            vumarks.activate();
        }
    }

    /**
     * Updates the Robot's position using Vuforia. Value is in inches from center of map (see ccoordinate_diagram.png).
     * @return boolean, true if updated, false otherwise
     */
    public boolean updatePos() {
        //will never run method given useVuforia is false
        if(!useAnyCV) return false;

        for (int i = 0; i < vumarks.size(); i++) {
            OpenGLMatrix testLocation = ((VuforiaTrackableDefaultListener) vumarks.get(i).getListener()).getPose();
            if (testLocation != null) {
                Location markLocation = new Location(vumarkLocations[i].getLocation(0), vumarkLocations[i].getLocation(1), vumarkLocations[i].getLocation(2), vumarkLocations[i].getLocation(3) - (float) Math.toDegrees(testLocation.get(1, 2)));
                markLocation.translateLocal(testLocation.getTranslation().get(1), -testLocation.getTranslation().get(0), testLocation.getTranslation().get(2));
                markLocation.translateLocal(camLocation.getLocation(0), camLocation.getLocation(1), camLocation.getLocation(2));
                markLocation.setRotation(markLocation.getLocation(3) + 180f);
                pos = markLocation;
                posHasBeenUpdated = true;
                return true;
            }
        }
        return false;
    }

    /**
     * Gets the position variable of the robot
     * @return Location, robot's position in inches from the origin. Rotation in degrees.
     */
    public Location getPos() {
        return pos;
    }

    /**
     * Updates the robot team enumerator using the current position. Will not overwrite old data.
     * @return boolean, true if updated, false if not updated or was updated in past.
     */
    public boolean updateTeam() {
        if(team != Team.UNKNOWN || !useAnyCV) return false;
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
     * Gets the robot team enumerator.
     * @return Team, the robot's current team.
     */
    public Team getTeam() {
        return team;
    }

    /**
     * Updates the cube location enumerator using OpenCV. Will not overwrite old data.
     * @return boolean, true if updated, false if not updated or was updated in past.
     */
    public boolean updateCubePos() {
        if(cubePos != CubePosition.UNKNOWN || !useAnyCV) return false;

        //completed using these tutorials:
        //
        // Init and syntax --- https://github.com/bchay/ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/VuMarkReader.java
        // Yellow identification --- http://aishack.in/tutorials/tracking-colored-objects-opencv/

        //tweaks
        Scalar minHSV = new Scalar(20, 100, 100);
        Scalar maxHSV = new Scalar(30, 255, 255);
        int scaledWidth = 300;
        int scaledHeight = 100;

        //inside a try-catch because Vuforia is so unconfident in their image extraction method that it is required
        try {
            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();     //returns "list" of all export images
            Image image = frame.getImage(0);

            for(int i = 1; image.getFormat() != PixelFormat.RGB_565; i++) {             //finds RGB_565 output image in "list"
                image = frame.getImage(i);
            }

            Bitmap bitmapImage = Bitmap.createBitmap(image.getWidth(),image.getHeight(),Bitmap.Config.RGB_565); //convert to bitmap

            Mat cvMat = new Mat();
            Utils.bitmapToMat(bitmapImage,cvMat);   //convert bitmap to mat for OpenCV

            Mat resizeMat = new Mat(scaledWidth, scaledHeight, cvMat.type());
            Imgproc.resize(cvMat,resizeMat,resizeMat.size(),0,0,Imgproc.INTER_NEAREST); //resize with nearest neighbor interpolation (doesn't lose any color data)

            Imgproc.cvtColor(resizeMat,resizeMat,Imgproc.COLOR_RGB2HSV);    //converting to HSV

            Mat thresholdMat = new Mat();   //info - HSV (hue, saturation, value). OpenCV uses hues from 0-179, so any 255 hue system needs to be *180/240.
            Core.inRange(resizeMat, minHSV, maxHSV, thresholdMat);  //outputs yellow(20-30) objects to b/w binary mat

            cvMat.release();
            resizeMat.release();

            Moments moments = Imgproc.moments(thresholdMat,true);
            double moment10 = moments.m10;
            double moment01 = moments.m01;
            double area = moments.m00;

            int posX = (int)(moment10/area); //moment10/area gives camera x coordinate
            int posY = (int)(moment01/area); //moment01/area gives camera y coordinate

            //This may need some work
            if(posX < scaledWidth/3) cubePos = CubePosition.LEFT;
            else if(posX > (scaledWidth/3) * 2) cubePos=CubePosition.RIGHT;
            else cubePos = CubePosition.MIDDLE;

            if(useTelemetry) telemetryMethod();

            return true;
        }
        catch (InterruptedException e) {
            cubePos = CubePosition.UNKNOWN;
            return false;
        }

        /*

        detector = new SamplingOrderDetector();
        dogeforia.setDogeCVDetector(detector);
        detector.init(hardwareGetter.hardwareMap.appContext, CameraViewDisplay.getInstance(),1,useAnyCV);
        detector.useDefaults();

        detector.downscale = 0.4; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.001;

        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
        dogeforia.start();

        SamplingOrderDetector.GoldLocation position = detector.getCurrentOrder();


        if(position == SamplingOrderDetector.GoldLocation.UNKNOWN) return false;
        switch (position) {
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
        */
    }

    /**
     * Gets the enumerator for the cube position.
     * @return CubePosition, the position of the cube.
     */
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
        if(!twoDriveWheels) {
            backLeft.setPower(left);
            backRight.setPower(right);
        }
    }

    /**
     * Sets drive motor target encoder to given values.
     * @param left encoder set for left motors.
     * @param right encoder set for right motors.
     */
    public void drivePosition(int left, int right) {
        frontLeft.setTargetPosition(left);
        frontRight.setTargetPosition(right);
        if(!twoDriveWheels) {
            backLeft.setTargetPosition(left);
            backRight.setTargetPosition(right);
        }
    }

    /**
     * Sets all drive motor run modes to given mode.
     * @param r DcMotor mode to given value.
     */
    public void driveMode(DcMotor.RunMode r) {
        frontLeft.setMode(r);
        frontRight.setMode(r);
        if(!twoDriveWheels) {
            backLeft.setMode(r);
            backRight.setMode(r);
        }
    }

    /**
     * Calls DcMotor.RunMode.STOP_AND_RESET_ENCODER for drive motors.
     */
    public void driveEncoderReset() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if(!twoDriveWheels) {
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
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
     * @param linearSlowdownDistance Distance to start linearly slowing down before target position.
     * @param precision Distance from target with which the program can stop execution.
     */
    public void goDistance(float distance, float linearSlowdownDistance, float precision) {
        float minimumPower = 0.2f;
        float maximumPower = 1.0f;

        int initEncoder = encoderMotor.getCurrentPosition();
        int finalEncoder = (int)(distance / (wheelDiameter * Math.PI) * encoderCountsPerRev) + initEncoder;
        int slowdownEncoder = (int)(linearSlowdownDistance / (wheelDiameter * Math.PI) * encoderCountsPerRev);
        int precisionEncoder = (int)(precision / (wheelDiameter * Math.PI) * encoderCountsPerRev);

        driveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(Math.abs(finalEncoder-encoderMotor.getCurrentPosition()) > precisionEncoder) {
            float uncappedPower = (slowdownEncoder != 0) ? (finalEncoder - encoderMotor.getCurrentPosition()) / slowdownEncoder : 1f;
            float cappedPower = (Math.min(maximumPower, Math.max(minimumPower, Math.abs(uncappedPower)))) * ((uncappedPower < 0) ? -1f : 1f);
            drivePower(cappedPower, cappedPower);
            if(useTelemetry) {
                telemetryMethod();
            }
        }

        pos.translateLocal(distance);
        updatePos();
    }

    /**
     * Pseudo PID to drive the distance to the given point. Will slow down from maximumMotorPower to minimumMotorPower starting at [slowdown] behind target.
     * ---Will drive distance in straight line, not guaranteed to have proper rotation---
     * @param targetPosition Location to drive distance to.
     * @param linearSlowdownDistance Distance to start linearly slowing down before target position.
     * @param precision Distance from target with which the program can stop execution.
     */
    public void goDistance(Location targetPosition, float linearSlowdownDistance, float precision) {
        float distance = (float)Math.sqrt(Math.pow(pos.getLocation(0)-targetPosition.getLocation(0),2) + Math.pow(pos.getLocation(2)-targetPosition.getLocation(2),2));
        goDistance(distance,linearSlowdownDistance,precision);
    }

    /**
     * Pseudo PID to rotate the given rotation. Will slow down from maximumMotorPower to minimumMotorPower starting at [slowdown] behind target azimuth.
     * Precision will stop making adjustments once it is within given degrees of target azimuth.
     * @param rot Target azimuth in degrees
     * @param linearSlowdownDistance Degrees behind target rotation to slow down
     * @param precision Degrees of imprecision in rotation value
     */
    public void rotateTo(float rot, float linearSlowdownDistance, float precision) {
        float minimumPower = 0.05f;
        float maximumPower = 0.5f;

        float rota = (rot - pos.getLocation(3)) % 360f;
        float rotb = -(360f - rota);
        float optimalRotation = (Math.abs(rota) < Math.abs(rotb) ? rota : rotb); //selects shorter rotation

        int initEncoder = encoderMotor.getCurrentPosition();
        int finalEncoder = (int)(Math.toRadians(optimalRotation) * wheelDistance / (wheelDiameter * Math.PI) * encoderCountsPerRev) + initEncoder;
        int slowdownEncoder = (int)(Math.toRadians(linearSlowdownDistance) * wheelDistance / (wheelDiameter * Math.PI) * encoderCountsPerRev);
        int precisionEncoder = (int)(precision / (wheelDiameter * Math.PI) * encoderCountsPerRev);

        driveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(Math.abs(finalEncoder-encoderMotor.getCurrentPosition()) > precisionEncoder) {
            float uncappedPower = (slowdownEncoder != 0) ? (finalEncoder - encoderMotor.getCurrentPosition()) / slowdownEncoder : 1f;
            float cappedPower = (Math.min(maximumPower, Math.max(minimumPower, Math.abs(uncappedPower)))) * ((uncappedPower < 0) ? -1f : 1f);
            drivePower(cappedPower, -cappedPower);
            if(useTelemetry) {
                telemetryMethod();
            }
        }

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
        if(!nothingButDrive) {
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(pos);
            lift.setPower(liftPower);
        }
    }

    /**
     * Method to use built-in PID to drive given distance, using Matt's system. JoJo wanted this. Untested.
     */
    private void driveMethodSimple(float distanceL, float distanceR, float LPower, float RPower) {
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
    private void telemetryMethod() {
        String motorString = "FL-" + frontLeft.getCurrentPosition() + " FR-" + frontRight.getCurrentPosition();
        if(!twoDriveWheels) motorString += " BL-" + backLeft.getCurrentPosition() + " BR-" + backRight.getCurrentPosition();
        telemetry.addData("Drive", motorString);

        if(!nothingButDrive) telemetry.addData("Lift",lift.getCurrentPosition());

        telemetry.addData("Pos",pos);
        telemetry.addData("Team",team);
        telemetry.addData("CubePos",cubePos);
        telemetry.update();
    }
}