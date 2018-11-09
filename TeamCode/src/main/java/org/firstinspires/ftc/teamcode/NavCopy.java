//package org.firstinspires.ftc.teamcode;
//
//
////EXIST
//import android.graphics.Bitmap;
//import android.graphics.PixelFormat;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.function.Consumer;
//import org.firstinspires.ftc.robotcore.external.function.Continuation;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//import org.opencv.android.Utils;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.imgproc.Moments;
//
//import com.qualcomm.robotcore.util.RobotLog;
//import com.qualcomm.robotcore.util.ThreadPool;
//import com.vuforia.Frame;
//import com.vuforia.Image;
//
//import java.io.File;
//import java.io.FileOutputStream;
//import java.io.IOException;
//import java.util.Locale;
//
//
//public class NavCopy {
//
//    /**
//     * A class for all movement methods for Rover Ruckus!
//     */
//        public static final String TAG = "Vuforia Navigation Sample";
//
//        //-----tweak values-----//
//        private float maximumMotorPower = 1f;           //when executing a goToLocation function, robot will never travel faster than this value (percentage 0=0%, 1=100%)
//        private float minimumMotorPower = 0.2f;
//        private float liftPower = 0.3f;                 //power the lift will run at
//        private float encoderCountsPerRev = 537.6f;     //encoder ticks per one revolution
//        private boolean useTelemetry = false;           //display motor values when running etc
//        private boolean nothingButDrive = false;
//        private boolean twoWheels = false;
//
//        //------game element locations-----//
//        public static final Location cargoBlueGold = new Location(-8.31f,27f,-8.31f,0f);
//        public static final Location cargoBlueSilver = new Location(-8.31f,27f,8.31f,0f);
//        public static final Location cargoRedGold = new Location(8.31f,27f,8.31f,0f);
//        public static final Location cargoRedSilver = new Location(8.31f,27f,-8.31f,0f);
//
//        //-----enums-----//
//        public enum Team {UNKNOWN, REDNORTH, REDSOUTH, BLUENORTH, BLUESOUTH}
//        private org.firstinspires.ftc.teamcode.Navigation.Team team = org.firstinspires.ftc.teamcode.Navigation.Team.UNKNOWN;
//        public enum CubePosition {UNKNOWN, LEFT, MIDDLE, RIGHT}
//        private org.firstinspires.ftc.teamcode.Navigation.CubePosition cubePos = org.firstinspires.ftc.teamcode.Navigation.CubePosition.UNKNOWN;
//
//        //-----robot hardware, position, and dimensions-----//
//        private com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter;
//        private DcMotor lift;
//      //location of robot as [x,y,z,rot] (inches / degrees)
//        private boolean posHasBeenUpdated = false;       //used with methods that require a vuforia input as not to produce inaccurate results
//        private float wheelDistance = 6;                //distance from center of robot to center of wheel (inches)
//        private float wheelDiameter = 4;                //diameter of wheel (inches)
//        private Location camLocation = new Location(0f,6f,6f,0f);
//        private Location pos = new Location();
//
//    //-----internal values-----//
//        private org.firstinspires.ftc.robotcore.external.Telemetry telemetry;
//        private VuforiaLocalizer vuforia;
//        private VuforiaTrackables vumarks;
//
//        private Location[] vumarkLocations = new Location[4];
//        private boolean useAnyCV;
//        private WebcamName webcamName;
//        private int captureCounter = 0;
//        private File captureDirectory= AppUtil.ROBOT_DATA_DIR;
//
//
//        public NavCopy(com.qualcomm.robotcore.eventloop.opmode.OpMode hardwareGetter, org.firstinspires.ftc.robotcore.external.Telemetry telemetry, boolean nothingButDrive, boolean twoWheels, boolean useAnyCV, boolean useTelemetry, boolean useWebcam) {
//            this.hardwareGetter = hardwareGetter;
//            this.telemetry = telemetry;
//            this.nothingButDrive = nothingButDrive;
//            this.twoWheels = twoWheels;
//            this.useAnyCV = useAnyCV;
//            this.useTelemetry = useTelemetry;
//
//                if (useAnyCV && useWebcam) {
//
//                    int cameraMonitorViewId = hardwareGetter.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareGetter.hardwareMap.appContext.getPackageName());
//                    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//                    parameters.vuforiaLicenseKey = " AYSaZfX/////AAABGZyGj0QLiEYhuyrGuO59xV2Jyg9I+WGlfjyEbBxExILR4A183M1WUKucNHp5CnSpDGX5nQ9OD3w5WCfsJuudFyJIJSKZghM+dOlhTWWcEEGk/YB0aOLEJXKK712HpyZqrvwpXOyKDUwIZc1mjWyLT3ZfCmNHQ+ouLKNzOp2U4hRqjbdWf1ZkSlTieiR76IbF6x7MX5ZtRjkWeLR5hWocakIaH/ZPDnqo2A2mIzAzCUa8GCjr80FJzgS9dD77lyoHkJZ/5rNe0k/3HfUZXA+BFSthRrtai1W2/3oRCFmTJekrueYBjM4wuuB5CRqCs4MG/64AzyKOdqmI05YhC1tVa2Vd6Bye1PaMBHmWNfD+5Leq ";
//                   // parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
//                    //WebcamName webcamName;
//                    webcamName = hardwareGetter.hardwareMap.get(WebcamName.class, "Webcam 1");
//                    parameters.cameraName = webcamName;
//
//                    vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//                    vumarks = vuforia.loadTrackablesFromAsset("18-19_rover_ruckus");
//                    vumarkLocations[0] = new Location(0f, 5.75f, 71.5f, 180f); //east
//                    vumarkLocations[1] = new Location(-71.5f, 5.75f, 0f, 270f); //north
//                    vumarkLocations[2] = new Location(0f, 5.75f, -71.5f, 0f); //west
//                    vumarkLocations[3] = new Location(71.5f, 5.75f, 0f, 90f); //south
//                    vumarks.activate();
//
//                }
//                if (useAnyCV && !useWebcam) {
//                    int cameraMonitorViewId = hardwareGetter.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareGetter.hardwareMap.appContext.getPackageName());
//                    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//                    parameters.vuforiaLicenseKey = " AYSaZfX/////AAABGZyGj0QLiEYhuyrGuO59xV2Jyg9I+WGlfjyEbBxExILR4A183M1WUKucNHp5CnSpDGX5nQ9OD3w5WCfsJuudFyJIJSKZghM+dOlhTWWcEEGk/YB0aOLEJXKK712HpyZqrvwpXOyKDUwIZc1mjWyLT3ZfCmNHQ+ouLKNzOp2U4hRqjbdWf1ZkSlTieiR76IbF6x7MX5ZtRjkWeLR5hWocakIaH/ZPDnqo2A2mIzAzCUa8GCjr80FJzgS9dD77lyoHkJZ/5rNe0k/3HfUZXA+BFSthRrtai1W2/3oRCFmTJekrueYBjM4wuuB5CRqCs4MG/64AzyKOdqmI05YhC1tVa2Vd6Bye1PaMBHmWNfD+5Leq ";
//                    parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
//                    vuforia = ClassFactory.createVuforiaLocalizer(parameters);
//                    vumarks = vuforia.loadTrackablesFromAsset("18-19_rover_ruckus");
//                    vumarkLocations[0] = new Location(0f, 5.75f, 71.5f, 180f); //east
//                    vumarkLocations[1] = new Location(-71.5f, 5.75f, 0f, 270f); //north
//                    vumarkLocations[2] = new Location(0f, 5.75f, -71.5f, 0f); //west
//                    vumarkLocations[3] = new Location(71.5f, 5.75f, 0f, 90f); //south
//                    vumarks.activate();
//
//
//                }
//
//        }
//
//        /**
//         * Updates the Robot's position using Vuforia. Value is in inches from center of map (see ccoordinate_diagram.png). Access using [nav].pos.
//         * @return boolean, true if updated, false otherwise
//         */
//        public boolean updatePos() {
//            //will never run method given useVuforia is false
//            if(!useAnyCV) return false;
//
//
//            for (int i = 0; i < vumarks.size(); i++) {
//                OpenGLMatrix testLocation = ((VuforiaTrackableDefaultListener)  vumarks.get(i).getListener()).getPose();
//                if (testLocation != null) {
//                    Location markLocation = new Location(vumarkLocations[i].getLocation(0), vumarkLocations[i].getLocation(1), vumarkLocations[i].getLocation(2), vumarkLocations[i].getLocation(3) - (float) Math.toDegrees(testLocation.get(1, 2)));
//                    markLocation.translateLocal(testLocation.getTranslation().get(1), -testLocation.getTranslation().get(0), testLocation.getTranslation().get(2));
//                    markLocation.translateLocal(camLocation.getLocation(0), camLocation.getLocation(1), camLocation.getLocation(2));
//                    markLocation.setRotation(markLocation.getLocation(3) + 180f);
//                    pos = markLocation;
//                    posHasBeenUpdated = true;
//                    return true;
//                }
//            }
//            return false;
//        }
//
//        public Location getPos() {
//            return pos;
//        }
//
//        /**
//         * Updates the robot team enumerator using the current position. Will not overwrite old data. Access using [nav].team.
//         * @return boolean, true if updated, false if not updated or was updated in past.
//         */
//        public boolean updateTeam() {
//            if(team != org.firstinspires.ftc.teamcode.Navigation.Team.UNKNOWN || !useAnyCV) return false;
//            updatePos();
//            if(!posHasBeenUpdated) return false;
//            float x = pos.getLocation(0);
//            float z = pos.getLocation(2);
//            if(x <= 0) {
//                if(z <= 0) team = org.firstinspires.ftc.teamcode.Navigation.Team.BLUESOUTH;
//                else team = org.firstinspires.ftc.teamcode.Navigation.Team.BLUENORTH;
//            }
//            else {
//                if(z <= 0) team = org.firstinspires.ftc.teamcode.Navigation.Team.REDSOUTH;
//                else team = org.firstinspires.ftc.teamcode.Navigation.Team.REDNORTH;
//            }
//            return true;
//        }
//
//        public org.firstinspires.ftc.teamcode.Navigation.Team getTeam() {
//            return team;
//        }
//
//        /**
//         * Updates the cube location enumerator using OpenCV. Will not overwrite old data. Access using [nav].cubePos.
//         * @return boolean, true if updated, false if not updated or was updated in past.
//         */
//        public boolean updateCubePos() {
//            if(cubePos != org.firstinspires.ftc.teamcode.Navigation.CubePosition.UNKNOWN || !useAnyCV) return false;
//
//            //completed using these tutorials:
//            //
//            // Init and syntax --- https://github.com/bchay/ftc_app/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/VuMarkReader.java
//            // Yellow identification --- http://aishack.in/tutorials/tracking-colored-objects-opencv/
//            // Centroid locator --- https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
//
//            //tweaks
//            Scalar minHSV = new Scalar(20, 100, 100);
//            Scalar maxHSV = new Scalar(30, 255, 255);
//            int scaledWidth = 300;
//            int scaledHeight = 100;
//
//            //inside a try-catch because Vuforia is so unconfident in their image extraction method that it is required
//            try {
//                VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();     //returns "list" of all export images
//                Image image = frame.getImage(0);
//
//                for(int i = 1; image.getFormat() != PixelFormat.RGB_565; i++) {             //finds RGB_565 output image in "list"
//                    image = frame.getImage(i);
//                }
//
//                Bitmap bitmapImage = Bitmap.createBitmap(image.getWidth(),image.getHeight(),Bitmap.Config.RGB_565); //convert to bitmap
//
//                Mat cvMat = new Mat();
//                Utils.bitmapToMat(bitmapImage,cvMat);   //convert bitmap to mat for OpenCV
//
//                Mat resizeMat = new Mat(scaledWidth, scaledHeight, cvMat.type());
//                Imgproc.resize(cvMat,resizeMat,resizeMat.size(),0,0,Imgproc.INTER_NEAREST); //resize with nearest neighbor interpolation (doesn't lose any color data)
//
//                Imgproc.cvtColor(resizeMat,resizeMat,Imgproc.COLOR_RGB2HSV);    //converting to HSV
//
//                Mat thresholdMat = new Mat();   //info - HSV (hue, saturation, value). OpenCV uses hues from 0-179, so any 255 hue system needs to be *180/240.
//                Core.inRange(resizeMat, minHSV, maxHSV, thresholdMat);  //outputs yellow(20-30) objects to b/w binary mat
//
//                cvMat.release();
//                resizeMat.release();
//
//                Moments moments = Imgproc.moments(thresholdMat,true);
//                double moment10 = moments.m10;
//                double moment01 = moments.m01;
//                double area = moments.m00;
//
//                //accounting for not seeing any cubes
//                if(area == 0) {
//                    cubePos = org.firstinspires.ftc.teamcode.Navigation.CubePosition.UNKNOWN;
//                    return false;
//                }
//
//                int posX = (int) (moment10 / area); //moment10/area gives camera x coordinate
//                int posY = (int) (moment01 / area); //moment01/area gives camera y coordinate
//
//                //This may need some work
//                if(posX < scaledWidth/3) cubePos = org.firstinspires.ftc.teamcode.Navigation.CubePosition.LEFT;
//                else if(posX > (scaledWidth/3) * 2) cubePos= org.firstinspires.ftc.teamcode.Navigation.CubePosition.RIGHT;
//                else cubePos = org.firstinspires.ftc.teamcode.Navigation.CubePosition.MIDDLE;
//
//                //if(useTelemetry) telemetryMethod();
//
//                return true;
//            }
//            catch (InterruptedException e) {
//                cubePos = org.firstinspires.ftc.teamcode.Navigation.CubePosition.UNKNOWN;
//                return false;
//            }
//        }
//
//        public org.firstinspires.ftc.teamcode.Navigation.CubePosition getCubePos() {
//            return cubePos;
//        }
//
//        /**
//         * Sets drive motor powers.
//         * @param left power of left two motors as percentage (0-1).
//         * @param right power of right two motors as percentage (0-1).
//         */
//
//    }
