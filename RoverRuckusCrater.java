/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


@Autonomous(name="Rover Ruckus Crater")

public class RoverRuckusCrater extends LinearOpMode {
    private DcMotor fl, fr, bl, br, robotlift, armrotate;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, correction;

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    private String lastVumark = null;
    private List<VuforiaTrackable> allTrackables = new ArrayList<>();
    private Dogeforia vuforia;
    private GoldAlignDetector detector;

    private static final double STRAFE = 44.1, MOVE = 35, TURN = 8;

    // moves forward by rot rotations of the wheel
    // (backwards if pow and rot are negative)
    private void moveTo(double inches, double pow, double angle) {
        fl.setTargetPosition( (int) Math.round(inches * MOVE) + fl.getCurrentPosition() );

        if (fl.getCurrentPosition() < fl.getTargetPosition())
            while ( opModeIsActive() && fl.getCurrentPosition() < fl.getTargetPosition() ) {
                correction = checkDirection(angle, 0.03);

                fl.setPower(-pow + correction);
                fr.setPower(-pow);
                bl.setPower(-pow + correction);
                br.setPower(-pow);
                idle();
            }
        else
            while ( opModeIsActive() && fl.getCurrentPosition() > fl.getTargetPosition() ) {
                correction = checkDirection(angle, 0.03);

                fl.setPower(pow + correction);
                fr.setPower(pow);
                bl.setPower(pow + correction);
                br.setPower(pow);
                idle();
            }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    // turns right by rot rotations of the wheel
    // (left if pow and rot are negative)
    private void turnTo(double degrees, double pow) {
        fl.setTargetPosition( (int) Math.round(degrees * TURN) + fl.getCurrentPosition() );
        double target = -getAngle() + degrees;

        if (degrees > 0) {   // turn right.
            fl.setPower(-pow);
            fr.setPower(pow);
            bl.setPower(-pow);
            br.setPower(pow);
        } else if (degrees < 0) {   // turn left.
            fl.setPower(pow);
            fr.setPower(-pow);
            bl.setPower(pow);
            br.setPower(-pow);
        } else
            return;

        int i = 1;
        if (fl.getCurrentPosition() < fl.getTargetPosition())
            while ( opModeIsActive() && fl.getCurrentPosition() < fl.getTargetPosition() ) { if(i++ % 15 == 0) getAngle(); else idle(); }
        else
            while ( opModeIsActive() && fl.getCurrentPosition() > fl.getTargetPosition() ) { if(i++ % 15 == 0) getAngle(); else idle(); }

        // turn the motors off.
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);

        double error = target + getAngle();

        if(Math.abs(error) > 2)
            turnTo(error, pow);
    }

    // strafes right by rot rotations of the wheel
    // (left if pow and rot are negative)
    private void strafeTo(double inches, double pow, double angle) {
        fl.setTargetPosition( (int) Math.round(inches * STRAFE) + fl.getCurrentPosition() );

        if (fl.getCurrentPosition() < fl.getTargetPosition())
            while ( opModeIsActive() && fl.getCurrentPosition() < fl.getTargetPosition() ) {
                correction = checkDirection(angle, 0.02);

                fl.setPower(-pow + correction);
                fr.setPower(pow - correction);
                bl.setPower(pow);
                br.setPower(-pow);
            }
        else
            while ( opModeIsActive() && fl.getCurrentPosition() > fl.getTargetPosition() ) {
                correction = checkDirection(angle, 0.02);

                fl.setPower(pow + correction);
                fr.setPower(-pow - correction);
                bl.setPower(-pow);
                br.setPower(pow);
            }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    // detects vumarks in order to determine position in the arena
    // times out after a certain number of checks
    // returns whether a vumark has been detected or not
    private boolean detect(int timeout) {
        int t = 0;

        do {
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                    lastVumark = trackable.getName();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    return true;
                }
            }
            t++;

            wait(10);
        } while( opModeIsActive() && t < timeout );

        return false;
    }

    private boolean reposition(double dx, double dy){
        if( detect(50) ){
            VectorF translation = lastLocation.getTranslation();
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);

            double x = translation.get(0) / mmPerInch, y = translation.get(1) / mmPerInch, h = rotation.thirdAngle;

            if( lastVumark.equals("Back-Space") ) {
                double cx = dx - x, cy = dy + y;
                turnTo(h, 0.3);
                resetAngle();
                wait(300);
                strafeTo(cy, 0.3, 0);
                moveTo(cx, 0.3, 0);
            } else if ( lastVumark.equals("Front-Craters") ) {
                double cx = dx + x, cy = dy - y, ch = (h < -90 ? h + 180 : -h);
                turnTo(ch, 0.3);
                resetAngle();
                wait(300);
                strafeTo(cy, 0.3, 0);
                moveTo(cx, 0.3, 0);
            }else if( lastVumark.equals("Red-Footprint") ) {
                double cx = x + dy, cy = y + dx, ch = h + 90;
                turnTo(ch, 0.3);
                resetAngle();
                wait(300);
                strafeTo(cx, 0.3, 0);
                moveTo(cy, 0.3, 0);
            } else if ( lastVumark.equals("Blue-Rover") ) {
                double cx = dy - x, cy = dx - y, ch = h - 90;
                turnTo(ch, 0.3);
                resetAngle();
                wait(300);
                strafeTo(cx, 0.3, 0);
                moveTo(cy, 0.3, 0);
            }

            return true;
        }else{
            return false;
        }
    }

    //waits for millis milliseconds
    private void wait(int millis) {
        long ellapsed = System.nanoTime() + (long) millis * 1000000L;

        while( System.nanoTime() < ellapsed && opModeIsActive() )
            idle();
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection(double a, double gain) {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle;

        angle = getAngle() - a;

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle * gain;        // reverse sign of angle for correction.

        return correction;
    }

    public void runOpMode() {

        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "AfjKWRL/////AAABmfOEbMH3xUPLtlD2h7I4sNdQnVvF1uqK/Rye/iqJN0YFnthrt0LwwqpR6HnB5dY35Zgbdrn6BLpSjLWVrUPqQfsRh9BdK7rUVtf9yPhfSefN3OsepEiUEXxzB2rMxgUjs47BLgc3V7RCUwdgKGcWvY2k9xfWj+ampWjN1KaHLsB25KlgrF2IkTZyC0QuTk+Mbl0mu12iKhKv0EQsLS3WMya1qDD4KzwyH8mqEyqMg50WVYVT7rGdBk29nhgbe5TWhrqpVzSZdXdiP2Zqgg0C670HDC5LfOtkyatHetKYOSXq6n1r9xm4B9HjX97ZKy+vJMvZLp5EFvLLpz54O64+c1QJ8q4eRkHRb5e2NOXKjIgd";
        parameters.fillCameraMonitorViewParent = true;

        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();

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

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix.translation(0, mmFTCFieldWidth, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));blueRover.setLocation(blueRoverLocationOnField);
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix.translation(0, -mmFTCFieldWidth, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));redFootprint.setLocation(redFootprintLocationOnField);
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix.translation(-mmFTCFieldWidth, 0, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));frontCraters.setLocation(frontCratersLocationOnField);
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix.translation(mmFTCFieldWidth, 0, mmTargetHeight).multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));backSpace.setLocation(backSpaceLocationOnField);

        final int CAMERA_FORWARD_DISPLACEMENT  = 150;
        final int CAMERA_VERTICAL_DISPLACEMENT = 120;
        final int CAMERA_LEFT_DISPLACEMENT     = -65;

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, CAMERA_CHOICE == FRONT ? 90 : -90, 0, 0));

        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        }

        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);

        detector.yellowFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW, 100);
        detector.alignSize = 300; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = -150; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames
        detector.maxError = 80;
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;
        detector.perfectAreaScorer.perfectArea = 17000;
        detector.perfectAreaScorer.weight = 0.005;
        detector.maxAreaScorer.weight = 0.005;
        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment
        detector.useDefaults();
        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();

        //settings for the IMU
        BNO055IMU.Parameters param = new BNO055IMU.Parameters();

        param.mode                = BNO055IMU.SensorMode.IMU;
        param.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        param.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        param.loggingEnabled      = false;

        //map the motors and IMU
        fl = hardwareMap.get(DcMotor.class, "FLmotor");
        fr = hardwareMap.get(DcMotor.class, "FRmotor");
        bl = hardwareMap.get(DcMotor.class, "BLmotor");
        br = hardwareMap.get(DcMotor.class, "BRmotor");
        robotlift = hardwareMap.get(DcMotor.class, "RoboLift");
        armrotate = hardwareMap.get(DcMotor.class, "ArmRotate");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(param);

        //make sure everything brakes
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armrotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverses the motors to make the logic easier
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        //reset and set encoders
        robotlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armrotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robotlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armrotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // make sure the imu gyro is calibrated before continuing.
        while ( !isStopRequested() && !imu.isGyroCalibrated() )
            wait(50);

        //update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //lowers the robot
        robotlift.setPower(-1);

        while( robotlift.getCurrentPosition() < 14500 && opModeIsActive() )
            idle();

        robotlift.setPower(0);

        resetAngle();

        //move out from the lander
        moveTo(0.5, 0.2, 0);
        strafeTo(-4, 0.25, 0);
        moveTo(8, 0.3, 0);

        //lower the lift
        robotlift.setPower(1);

        //move to see vumark
        turnTo(-70, 0.3);
        moveTo(18, 0.3, 70);

        //read vumark and position self
        wait(700);
        if( !reposition(41, 20) ) {
            turnTo(20, 0.3);
            resetAngle();
            wait(300);
            strafeTo(2, 0.5, 0);
            moveTo(-2, 0.3, 0);
        }

        //finish lowering the lift
        while( robotlift.getCurrentPosition() > 200 && opModeIsActive() )
            idle();
        robotlift.setPower(0);

        //move to gold vantagepoint
        turnTo(40, 0.3);
        moveTo(-2.25, 0.3, -40);

        //detect and knock off gold
        wait(700);
        boolean hasDetected = false;

        if ( detector.getAligned() ) {
            moveTo(11, 0.3, -40);
            wait(300);
            moveTo(-9, 0.3, -40);
            strafeTo(-2, 0.5, -45);
            hasDetected = true;
        }

        if ( !hasDetected ) {
            strafeTo(13, 0.5, -40);
            moveTo(1, 0.3, -40);
            wait(700);
            if( detector.getAligned() ) {
                strafeTo(0.5, 0.5, -40);
                moveTo(9, 0.3, -40);
                wait(300);
                moveTo(-8, 0.3, -40);
                strafeTo(-21, 0.5, -45);
                hasDetected = true;
            }
        }

        if ( !hasDetected ) {
            strafeTo(14, 0.5, -40);
            moveTo(11, 0.3, -40);
            wait(300);
            moveTo(-7, 0.3, -40);
            strafeTo(-39, 0.5, -45);
        }

        //disable the gold detector
        detector.disable();

        //reposition self
        turnTo(-60, 0.3);
        wait(700);
        if( !reposition(43, 19.3) ) {
            //TODO default correction values
            return;
        }

        //move to depot
        moveTo(17, 0.3, 0);
        turnTo(-90, 0.3);
        moveTo(55, 0.4, 90);

        //deposit marker in depot
        armrotate.setPower(-0.5);

        while( armrotate.getCurrentPosition() < 1000 && opModeIsActive() )
            idle();

        armrotate.setPower(0.5);

        while( armrotate.getCurrentPosition() > 300 && opModeIsActive() )
            idle();

        armrotate.setPower(0);

        //move to crater
        turnTo(180, 0.3);
        moveTo(50, 0.5, -90);

        //lower arm over crater
        armrotate.setPower(-0.5);

        while( armrotate.getCurrentPosition() < 1500 && opModeIsActive() )
            idle();

        armrotate.setPower(0);
    }
}
