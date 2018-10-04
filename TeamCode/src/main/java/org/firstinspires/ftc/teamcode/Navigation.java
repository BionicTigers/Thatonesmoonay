package org.firstinspires.ftc.teamcode;
//EXIST
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class Navigation{
    /*
    //-----REFERENCE------//
        x+ is back (south)(audience side)
        z+ is right (east)(red side)
        x- is back (north)(back side)
        z- is left (west)(blue side)
        rotation is in degrees, going from 0 @ z+ to 360 @ z+
     */

    //------game element locations-----//
    public static final Location cargoBlueGold = new Location(-8.31f,27f,-8.31f,0f);
    public static final Location cargoBlueSilver = new Location(-8.31f,27f,8.31f,0f);
    public static final Location cargoRedGold = new Location(8.31f,27f,8.31f,0f);
    public static final Location cargoRedSilver = new Location(8.31f,27f,-8.31f,0f);

    //-----team enum-----//
    public enum Team {UNKNOWN, REDNORTH, REDSOUTH, BLUENORTH, BLUESOUTH}
    public Team team = Team.UNKNOWN;

    //-----public robot elements-----//
    //if you guys rename these I will cry - Quinn
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;

    //-----robot position and dimensions-----//
    public Location pos = new Location(); //location of robot as [x,y,z,rot] (inches)
    public boolean posHasBeenUpdated = false;
    private float wheelDistance = 6; //distance from center of robot to center of wheel (inches)
    private float wheelDiameter = 4; //diameter of wheel (inches)
    private Location camLocation = new Location(0f,6f,6f,0f);


    //-----vuforia init------//
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables vumarks;
    private Location[] vumarkLocations = new Location[4];

    //-----tweak values-----//
    private float minimumSlowdownDistance = 10f; //when executing a goToLocation function, robot will begin slowing this far from destination (inches)
    private float maximumMotorPower = 1f; //when executing a goToLocation function, robot will never travel faster than this value (percentage 0=0%, 1=100%)
    private float minimumMotorPower = 0.2f;
    private float killDistance = 0; //kills program if robot farther than distance in x or z from origin (inches) (0 means no kill)
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
    private float encoderCountsPerRev = 537.6f;
    private float precisionRatio = 0.2f; //percentage of maximum motor power to use in precision ops

    /** Constructor class for hardware init. Requires local LinearOpMode for phone cameras in Vuforia.
     *
     * @param hardwareGetter LinearOpMode class that has direct access to Hardware components. Call from LinearOpMode as 'new Navigation(this)'
     */
    public Navigation(com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter, org.firstinspires.ftc.robotcore.external.Telemetry tele) {
        frontLeft = hardwareGetter.hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareGetter.hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareGetter.hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareGetter.hardwareMap.dcMotor.get("backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        int cameraMonitorViewId = hardwareGetter.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareGetter.hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = " AYSaZfX/////AAABGZyGj0QLiEYhuyrGuO59xV2Jyg9I+WGlfjyEbBxExILR4A183M1WUKucNHp5CnSpDGX5nQ9OD3w5WCfsJuudFyJIJSKZghM+dOlhTWWcEEGk/YB0aOLEJXKK712HpyZqrvwpXOyKDUwIZc1mjWyLT3ZfCmNHQ+ouLKNzOp2U4hRqjbdWf1ZkSlTieiR76IbF6x7MX5ZtRjkWeLR5hWocakIaH/ZPDnqo2A2mIzAzCUa8GCjr80FJzgS9dD77lyoHkJZ/5rNe0k/3HfUZXA+BFSthRrtai1W2/3oRCFmTJekrueYBjM4wuuB5CRqCs4MG/64AzyKOdqmI05YhC1tVa2Vd6Bye1PaMBHmWNfD+5Leq ";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        vumarks = vuforia.loadTrackablesFromAsset("18-19_rover_ruckus");
        vumarkLocations[0] = new Location(0f,5.75f,71.5f,180f); //east
        vumarkLocations[1] = new Location(-71.5f,5.75f,0f,270f); //north
        vumarkLocations[2] = new Location(0f,5.75f,-71.5f,0f); //west
        vumarkLocations[3] = new Location(71.5f,5.75f,0f,90f); //south

        vumarks.activate();

        telemetry = tele;
    }

    /** Updates position using vuforia.
     *
     * @return True if position was changed, false otherwise.
     */
    public boolean updatePos() {
        ArrayList<Location> validPositions = new ArrayList<>();
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

    /** Sets team value to correct quadrant based on Vuforia position.
     *
     * @return Returns false if unsuccessful, true otherwise.
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

    public void drivePower(float left, float right) {
        driveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }

    public void drivePosition(int left, int right) {
        frontLeft.setTargetPosition(left);
        backLeft.setTargetPosition(left);
        frontRight.setTargetPosition(right);
        backRight.setTargetPosition(right);
    }

    public void driveMode(DcMotor.RunMode r) {
        frontLeft.setMode(r);
        backLeft.setMode(r);
        frontRight.setMode(r);
        backRight.setMode(r);
    }

    public void driveEncoderReset() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /** Traverse given distance in a straight line, slows down approaching point.
     *
     * @param distance distance to travel (inches). Use negatives for backwards.
     * @param precision distance behind goal to reduce power (inches)
     */
    public void goDistance(float distance, float precision) {
        int initEncoder = frontRight.getCurrentPosition();
        int targetEncoder = (int)(distance / (wheelDiameter * Math.PI) * encoderCountsPerRev) + initEncoder;
        int precisionEncoder = (int)(precision / (wheelDiameter * Math.PI) * encoderCountsPerRev);

        driveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while ((targetEncoder - backLeft.getCurrentPosition()) / (float)precisionEncoder > 0) {
            float uncappedPower = (targetEncoder - backLeft.getCurrentPosition()) / (float)precisionEncoder;
            float power = (uncappedPower < 0 ? -1:1) * Math.min(maximumMotorPower, Math.max(minimumMotorPower, Math.abs(uncappedPower)));
            drivePower(power, power);
            telemetry.addData("Left Front", frontLeft.getCurrentPosition());
            telemetry.addData("Left Back", backLeft.getCurrentPosition());
            telemetry.addData("Right Front", frontRight.getCurrentPosition());
            telemetry.addData("Right Back", backRight.getCurrentPosition());
            telemetry.addData("PowerUncapped", uncappedPower);
            telemetry.addData("Power", power);
            telemetry.update();
        }

        driveEncoderReset();
        pos.translateLocal(distance);
        updatePos();
    }

    /** Rotates front of robot to rotation rot, either positive or negative, and sets new rotation in position.
     *
     * @param rot Heading to rotate robot towards.
     * @param precision distance behind goal to cut power (inches). 0 will stop bot at point, but may take much longer due to slowdown.
     */
    public void rotateTo(float rot, float precision) {
        float rota = rot - pos.getLocation(3);
        float rotb = (rot-360) - pos.getLocation(3);
        float optimalRotation = Math.abs(rota)<Math.abs(rotb) ? rota : rotb; //selects shorter rotation
        float distance = 360 * (float)(Math.toRadians(optimalRotation) * wheelDistance / (wheelDiameter*Math.PI)); //arc length of turn / circumference of wheel * 360
        float initDistance = frontRight.getCurrentPosition() * (wheelDiameter / 2f);
        float elapsedDistance = initDistance;
        if(distance > 0) {
            while(distance-elapsedDistance > (0+precision)) {
                float motorPower = Math.min(maximumMotorPower, maximumMotorPower*(elapsedDistance-distance/distance));
                drivePower(-motorPower,motorPower);
                elapsedDistance = frontRight.getCurrentPosition() * (wheelDiameter / 2f) - initDistance;
                telemetry.addData("distance",elapsedDistance);
                telemetry.update();
            }
        }
        else {
            while(distance-elapsedDistance < (0-precision)) {
                float motorPower = Math.min(maximumMotorPower, maximumMotorPower*(elapsedDistance-distance/distance));
                drivePower(motorPower,-motorPower);
                elapsedDistance = frontRight.getCurrentPosition() * (wheelDiameter / 2f) + initDistance;
            }
        }
        drivePower(0f,0f);
        pos.setRotation(rot);
        updatePos();
    }

    /** Rotates front of robot to face point, either positive or negative, and sets new rotation in position.
     *
     * @param loc Point to rotate robot towards.
     * @param precision distance behind goal to stop (inches). 0 will stop bot at point, but may take much longer due to slowdown.
     */
    public void rotateToFace(Location loc, float precision) {
        rotateTo((float) Math.toDegrees(Math.atan2(loc.getLocation(2) - pos.getLocation(2), loc.getLocation(0) - pos.getLocation(0))), precision);
    }
}