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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Rover Ruckus Simple")

public class RoverRuckusSimple extends LinearOpMode {
    private DcMotor fl, fr, bl, br, robotlift, armrotate;

    // moves forward by rot rotations of the wheel
    // (backwards if pow and rot are negative)
    private void moveTo(double inches, double pow) {
        fl.setTargetPosition( (int) Math.round(inches * 33.6) + fl.getCurrentPosition() );

        fl.setPower(-pow);
        fr.setPower(-pow);
        bl.setPower(-pow);
        br.setPower(-pow);

        if (fl.getCurrentPosition() < fl.getTargetPosition())
            while ( opModeIsActive() && fl.getCurrentPosition() < fl.getTargetPosition() ) { idle(); }
        else
            while ( opModeIsActive() && fl.getCurrentPosition() > fl.getTargetPosition() ) { idle(); }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    // turns right by rot rotations of the wheel
    // (left if pow and rot are negative)
    private void turnTo(double degrees, double pow) {
        fl.setTargetPosition( (int) Math.round(degrees * 9.447) + fl.getCurrentPosition() );

        fl.setPower(-pow);
        fr.setPower(pow);
        bl.setPower(-pow);
        br.setPower(pow);

        if (fl.getCurrentPosition() < fl.getTargetPosition())
            while ( opModeIsActive() && fl.getCurrentPosition() < fl.getTargetPosition() ) { idle(); }
        else
            while ( opModeIsActive() && fl.getCurrentPosition() > fl.getTargetPosition() ) { idle(); }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    // strafes right by rot rotations of the wheel
    // (left if pow and rot are negative)
    private void strafeTo(double inches, double pow) {
        fl.setTargetPosition( (int) Math.round(inches * 46.7) + fl.getCurrentPosition() );

        fl.setPower(-pow);
        fr.setPower(pow);
        bl.setPower(pow);
        br.setPower(-pow);

        if (fl.getCurrentPosition() < fl.getTargetPosition())
            while ( opModeIsActive() && fl.getCurrentPosition() < fl.getTargetPosition() ) { idle(); }
        else
            while ( opModeIsActive() && fl.getCurrentPosition() > fl.getTargetPosition() ) { idle(); }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }

    public void runOpMode() {
        //map the motors
        fl = hardwareMap.get(DcMotor.class, "FLmotor");
        fr = hardwareMap.get(DcMotor.class, "FRmotor");
        bl = hardwareMap.get(DcMotor.class, "BLmotor");
        br = hardwareMap.get(DcMotor.class, "BRmotor");
        robotlift = hardwareMap.get(DcMotor.class, "RoboLift");
        armrotate = hardwareMap.get(DcMotor.class, "ArmRotate");

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

        //update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //lowers the robot
        robotlift.setPower(-1);

        while( robotlift.getCurrentPosition() < 14000 && opModeIsActive() )
            idle();

        robotlift.setPower(0);

        //move out from the lander
        strafeTo(-4, -0.2);
        moveTo(6, 0.3);

        //lower the lift
        robotlift.setPower(1);

        while( robotlift.getCurrentPosition() > 100 && opModeIsActive() )
            idle();

        robotlift.setPower(0);
    }
}
