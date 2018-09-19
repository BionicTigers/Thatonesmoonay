package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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
    public DcMotor motorLeftA;
    public DcMotor motorLeftB;
    public DcMotor motorRightA;
    public DcMotor motorRightB;

    //-----robot position and dimensions-----//
    public Location pos = new Location(); //location of robot as [x,y,z,rot] (inches)
    public boolean posHasBeenUpdated = false;
    private float wheelDistance = 6; //distance from center of robot to center of wheel (inches)
    private float wheelDiameter = 4; //diameter of wheel (inches)
    private Location[] camLocations = new Location[1];


    //-----vuforia init------//
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables vumarks;
    private Location[] vumarkLocations = new Location[4];

    //-----tweak values-----//
    private float minimumSlowdownDistance = 10f; //when executing a goToLocation function, robot will begin slowing this far from destination (inches)
    private float maximumMotorPower = 0.9f; //when executing a goToLocation function, robot will never travel faster than this value (percentage 0=0%, 1=100%)
    private float killDistance = 0; //kills program if robot farther than distance in x or z from origin (inches) (0 means no kill)
    private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;

    /** Constructor class for hardware init. Requires local LinearOpMode for phone cameras in Vuforia.
     *
     * @param hardwareGetter LinearOpMode class that has direct access to Hardware components. Call from LinearOpMode as 'new Navigation(this)'
     */
    public Navigation(com.qualcomm.robotcore.eventloop.opmode.LinearOpMode hardwareGetter, org.firstinspires.ftc.robotcore.external.Telemetry tele) {
        motorLeftA = hardwareGetter.hardwareMap.dcMotor.get("motorLeftA");
        motorLeftB = hardwareGetter.hardwareMap.dcMotor.get("motorLeftB");
        motorRightA = hardwareGetter.hardwareMap.dcMotor.get("motorRightA");
        motorRightB = hardwareGetter.hardwareMap.dcMotor.get("motorRightB");

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

        //private static final Location camRight = new Location();
        camLocations[0] = new Location(0f,6f,6f,0f);
        //private static final Location camLeft = new Location();
        //private static final Location camBack = new Location();

        vumarks.activate();

        telemetry = tele;
    }

    /** Set power value of drive motors to given percentage.
     *
     * @param power Power percentage to supply motor. 0 = 0%, 1 = 100%
     */
    public void setMotors(float power) {
        setMotors(power,power);
    }

    /** Set power values of left and right drive motors to given percentage.
     *
     * @param powerL Power percentage to supply left motors. 0 = 0%, 1 = 100%
     * @param powerR Power percentage to supply right motors. 0 = 0%, 1 = 100%
     */
    public void setMotors(float powerL, float powerR) {
        motorLeftA.setPower(powerL);
        motorLeftB.setPower(powerL);
        motorRightA.setPower(powerR);
        motorRightB.setPower(powerR);
    }

    /** Stops drive motors.
     */
    public void stopMotors() {
        setMotors(0f,0f);
    }

    /** Updates position using vuforia.
     *
     * @return True if position was changed, false otherwise.
     */
    public boolean updatePos() {
        return updatePos(true);
    }

    /** Updates position using vuforia.
     *
     * @param averageResults If true averages all viewed vumarks, if false, bases off of reading one (slightly faster).
     * @return True if position was changed, false otherwise.
     */
    public boolean updatePos(boolean averageResults) {

        ArrayList<Location> validPositions = new ArrayList<>();

        for (int i = 0; i < vumarks.size(); i++) {
            OpenGLMatrix testLocation = ((VuforiaTrackableDefaultListener) vumarks.get(i).getListener()).getPose();
            if (testLocation != null) {
                Location markLocation = new Location(vumarkLocations[i].getLocation(0), vumarkLocations[i].getLocation(1), vumarkLocations[i].getLocation(2), vumarkLocations[i].getLocation(3) - (float)Math.toDegrees(testLocation.get(1,2)));
                markLocation.translateLocal(testLocation.getTranslation().get(1), -testLocation.getTranslation().get(0), testLocation.getTranslation().get(2));
                markLocation.setRotation(markLocation.getLocation(3) + 180f);
                validPositions.add(markLocation);
            }
        }

        if(validPositions.size() == 0) return false;
        Location result = validPositions.get(0);
        if(averageResults) {
            float x = 0f;
            float y = 0f;
            float z = 0f;
            float rot = 0f;
            for(Location l : validPositions) {
                x += l.getLocation(0);
                y += l.getLocation(1);
                z += l.getLocation(2);
                rot += l.getLocation(3);
                rot %= 360f;
            }
            x /= validPositions.size();
            y /= validPositions.size();
            z /= validPositions.size();
            rot /= validPositions.size();
            result = new Location(x,y,z,rot);
        }
        pos = result;
        posHasBeenUpdated = true;
        if( killDistance!= 0 && (Math.abs(pos.getLocation(0)) >  killDistance || Math.abs(pos.getLocation(2)) >  killDistance)) throw new IllegalStateException("Robot outside of killDistance at pos: " + pos);
        return true;
    }

    /** Sets team value to correct quadrant based on Vuforia position.
     *
     * @return Returns false if unsuccessful, true otherwise.
     */
    public boolean determineTeam() {
        if(!posHasBeenUpdated) {
            updatePos();
            if(!posHasBeenUpdated) return false;
        }
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

    /* REQUIRES OPEN CV TO LOCATE GOLD CUBE :(
        SOUNDS LIKE A CHRIS PROBLEM
    /** Using Open CV, locates gold mineral in front of the robot.
      *
      * @return Returns location of gold cube in world coordinates.
      */
    /*public Location goldMineralLocation() {

    }
    */

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
        float initDistance = motorRightA.getCurrentPosition() * (wheelDiameter / 2f);
        float elapsedDistance = initDistance;
        if(distance > 0) {
            while(distance-elapsedDistance > (0+precision)) {
                float motorPower = Math.min(maximumMotorPower, maximumMotorPower*(elapsedDistance-distance/distance));
                setMotors(-motorPower,motorPower);
                elapsedDistance = motorRightA.getCurrentPosition() * (wheelDiameter / 2f) - initDistance;
                telemetry.addData("distance",elapsedDistance);
                telemetry.update();
            }
        }
        else {
            while(distance-elapsedDistance < (0-precision)) {
                float motorPower = Math.min(maximumMotorPower, maximumMotorPower*(elapsedDistance-distance/distance));
                setMotors(motorPower,-motorPower);
                elapsedDistance = motorRightA.getCurrentPosition() * (wheelDiameter / 2f) + initDistance;
            }
        }
        stopMotors();
        pos.setRotation(rot);
        updatePos();
    }

    /** Rotates front of robot to face point, either positive or negative, and sets new rotation in position.
     *
     * @param loc Point to rotate robot towards.
     * @param precision distance behind goal to stop (inches). 0 will stop bot at point, but may take much longer due to slowdown.
     */
    public void rotateToFace(Location loc, float precision) {
        rotateTo((float)Math.toDegrees(Math.atan2(loc.getLocation(2)-pos.getLocation(2),loc.getLocation(0)-pos.getLocation(0))),precision);
    }

    /** Traverse given distance in a straight line, slows down approaching point.
     *
     * @param distance distance to travel (inches). Use negatives for backwards.
     * @param precision distance behind goal to cut power (inches). 0 will stop bot at point, but may take much longer due to slowdown.
     */
    public void goDistance(float distance, float precision) {
        float initDistance = motorRightA.getCurrentPosition() * (wheelDiameter / 2f);
        float elapsedDistance = initDistance;
        if(distance > 0) {
            while (distance - elapsedDistance > (0 + precision)) {
                float motorPower = Math.min(maximumMotorPower, maximumMotorPower * (elapsedDistance - distance / distance));
                setMotors(motorPower);
                elapsedDistance = motorRightA.getCurrentPosition() * (wheelDiameter / 2f) - initDistance;
            }
        }
        else {
            while (distance - elapsedDistance < (0 - precision)) {
                float motorPower = Math.min(maximumMotorPower, maximumMotorPower * (elapsedDistance - distance / distance));
                setMotors(-motorPower);
                elapsedDistance = motorRightA.getCurrentPosition() * (wheelDiameter / 2f) + initDistance;
            }
        }
        stopMotors();
        pos.translateLocal(distance);
        updatePos();
    }

    /** Rotates front of robot to face point, either positive or negative, and sets new rotation in position,
     *  then travels in straight line to reach destination.
     *
     * @param loc Destination point in world space
     * @param precision distance behind goal to cut power (inches). 0 will stop bot at point, but may take much longer due to slowdown.
     */
    public void goToLocation(Location loc, float precision) {
        rotateToFace(loc, precision);
        goDistance((float)Math.sqrt(Math.pow(loc.getLocation(0)-pos.getLocation(0),2)+Math.pow(loc.getLocation(2)-pos.getLocation(2),2)), precision);
    }
}