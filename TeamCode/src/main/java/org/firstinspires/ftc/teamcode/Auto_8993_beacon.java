package org.firstinspires.ftc.teamcode;

import android.widget.ImageView;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by haizhang on 12/8/16.
 */

@Autonomous(name = "Vuforia")
public class Auto_8993_beacon extends LinearOpMode {

    ImageView display;
    HardwareTest robot           = new HardwareTest();
    public static final String TAG = "Vuforia Sample";
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     COUNTS_PER_MOTOR_REV    = 720 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    OpenGLMatrix phoneLocation;
    OpenGLMatrix lastKnownLocation;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;
    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;
    private VuforiaLocalizer locale;
    private VuforiaLocalizer.Parameters params;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        setupVuforia();
        lastKnownLocation=createMatrix(0,0,0,0,0,0);
        waitForStart();

        while(opModeIsActive())
        {
            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            // Send information about whether the target is visible, and where the robot is
            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));

            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();

        }

        // Drive 18 inch
//        encoderDrive(DRIVE_SPEED,  18,  18, 5.0);
//
//        // Analyze beacon
//        getBeaconConfig(VortexUtils.getImageFromFrame(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565), tools, locale.getCameraCalibration());
    }

//    public int getBeaconConfig(Image img, VuforiaTrackableDefaultListener beacon, CameraCalibration camCal) {
//
//        OpenGLMatrix pose = beacon.getRawPose();
//
//        if (pose != null && img != null && img.getPixels() != null) {
//
//            Matrix34F rawPose = new Matrix34F();
//            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
//
//            rawPose.setData(poseData);
//
//            //calculating pixel coordinates of beacon corners
//            float[][] corners = new float[4][2];
//
//            corners[0] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, 276, 0)).getData(); //upper left of beacon
//            corners[1] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, 276, 0)).getData(); //upper right of beacon
//            corners[2] = Tool.projectPoint(camCal, rawPose, new Vec3F(127, -92, 0)).getData(); //lower right of beacon
//            corners[3] = Tool.projectPoint(camCal, rawPose, new Vec3F(-127, -92, 0)).getData(); //lower left of beacon
//
//            //getting camera image...
//            Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
//            bm.copyPixelsFromBuffer(img.getPixels());
//
//            //turning the corner pixel coordinates into a proper bounding box
//            Mat crop = OCVUtils.bitmapToMat(bm, CvType.CV_8UC3);
//            float x = Math.min(Math.min(corners[1][0], corners[3][0]), Math.min(corners[0][0], corners[2][0]));
//            float y = Math.min(Math.min(corners[1][1], corners[3][1]), Math.min(corners[0][1], corners[2][1]));
//            float width = Math.max(Math.abs(corners[0][0] - corners[2][0]), Math.abs(corners[1][0] - corners[3][0]));
//            float height = Math.max(Math.abs(corners[0][1] - corners[2][1]), Math.abs(corners[1][1] - corners[3][1]));
//
//
//            //make sure our bounding box doesn't go outside of the image
//            //OpenCV doesn't like that...
//            x = Math.max(x, 0);
//            y = Math.max(y, 0);
//            width = (x + width > crop.cols())? crop.cols() - x : width;
//            height = (y + height > crop.rows())? crop.rows() - y : height;
//
//
//            //cropping bounding box out of camera image
//            final Mat cropped = new Mat(crop, new Rect((int) x, (int) y, (int) width, (int) height));
//
//            //filtering out non-beacon-blue colours in HSV colour space
//            Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGB2HSV_FULL);
//
//            //get filtered mask
//            //if pixel is within acceptable blue-beacon-colour range, it's changed to white.
//            //Otherwise, it's turned to black
//            Mat mask = new Mat();
//            Core.inRange(cropped, VortexUtils.blueLow, VortexUtils.blueHigh, mask);
//            Moments mmnts = Imgproc.moments(mask, true);
//
//            //calculating centroid of the resulting binary mask via image moments
//            Log.i("CentroidX", "" + ((mmnts.get_m10() / mmnts.get_m00())));
//            Log.i("CentroidY", "" + ((mmnts.get_m01() / mmnts.get_m00())));
//
//            //checking if blue either takes up the majority of the image (which means the beacon is all blue)
//            //or if there's barely any blue in the image (which means the beacon is all red or off)
//            if (mmnts.get_m00() / mask.total() > 0.8) {
//                return VortexUtils.BEACON_ALL_BLUE;
//            } else if (mmnts.get_m00() / mask.total() < 0.1) {
//                return VortexUtils.BEACON_NO_BLUE;
//            }//elseif
//
//            //Note: for some reason, we end up with a image that is rotated 90 degrees
//            //if centroid is in the bottom half of the image, the blue beacon is on the left
//            //if the centroid is in the top half, the blue beacon is on the right
//            if ((mmnts.get_m01() / mmnts.get_m00()) < cropped.rows() / 2) {
//                return VortexUtils.BEACON_RED_BLUE;
//            } else {
//                return VortexUtils.BEACON_BLUE_RED;
//            }//else
//        }//if
//
//        return VortexUtils.BEACON_NOT_VISIBLE;
//    }//getBeaconConfig


    public void setupVuforia() {
        params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        locale = ClassFactory.createVuforiaLocalizer(params);
 //       Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        //       locale.setFrameQueueCapacity(1);

        VuforiaTrackables beacons = locale.loadTrackablesFromAsset("FTC_2016-17");
//        VuforiaTrackableDefaultListener tools = (VuforiaTrackableDefaultListener) beacons.get(1).getListener();
//        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();
//        VuforiaTrackableDefaultListener legos = (VuforiaTrackableDefaultListener) beacons.get(2).getListener();
//        VuforiaTrackableDefaultListener gears = (VuforiaTrackableDefaultListener) beacons.get(3).getListener();

        target = beacons.get(0);
        target.setName("wheels");

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        OpenGLMatrix targetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        target.setLocation(targetLocationOnField);
        RobotLog.ii(TAG, "Red Target=%s", new Object[]{targetLocationOnField.getData().toString()});

//        target.setLocation(createMatrix(0,0,0,0,0,0));

        phoneLocation = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii(TAG, "phone=%s", phoneLocation.getData().toString());
//
        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, params.cameraDirection);
    }

    public OpenGLMatrix createMatrix(float x,float y,float z,float u,float v,float w)
    {
        return OpenGLMatrix.translation(x,y,z).multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES,u,v,w));
    }

    /*
*  Method to perfmorm a relative move, based on encoder counts.
*  Encoders are not reset as the move is based on the current position.
*  Move will stop if any of three conditions occur:
*  1) Move gets to the desired position
*  2) Move runs out of time
*  3) Driver stops the opmode running.
*/
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(Math.abs(speed));
            robot.rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftMotor.isBusy() && robot.rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftMotor.getCurrentPosition(),
                        robot.rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall)
    {
        return new VectorF((float) (trans.get(0)
                - offWall.get(0) * Math.sin(Math.toRadians(robotAngle))
                - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))),
                trans.get(1),
                (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle))
                        - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image)
    {
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]},
                {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1]
                + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);
    }

    // Formats a matrix into a readable string
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}
