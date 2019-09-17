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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="Rover Ruckus Drive")

public class RoverRuckusDrive extends LinearOpMode {
    private DcMotor fl, fr, bl, br, robotlift, armrotate, armextend;
    private CRServo intake;

    public void runOpMode() {

        //map the motors
        fl = hardwareMap.get(DcMotor.class, "FLmotor");
        fr = hardwareMap.get(DcMotor.class, "FRmotor");
        bl = hardwareMap.get(DcMotor.class, "BLmotor");
        br = hardwareMap.get(DcMotor.class, "BRmotor");

        robotlift = hardwareMap.get(DcMotor.class, "RoboLift");
        armrotate = hardwareMap.get(DcMotor.class, "ArmRotate");
        armextend = hardwareMap.get(DcMotor.class, "ArmExtend");

        intake = hardwareMap.get(CRServo.class, "IntakeServo");

        //make sure everything brakes
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robotlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armrotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armextend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverses the motors to make the logic easier
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        //encoder settings
        robotlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armextend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armextend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armrotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robotlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //update status
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        boolean d_pad = true;
        double speed = 0.3;

        while (opModeIsActive()) {

            /* gamepad2 controls */

            //runs the intake servo
            if(this.gamepad2.x)
                intake.setPower(1);
            else if(this.gamepad2.a)
                intake.setPower(0);
            else if(this.gamepad2.b)
                intake.setPower(-1);

            //controls for the robot lift
            robotlift.setPower( (this.gamepad2.left_bumper && (robotlift.getCurrentPosition() > 0 || this.gamepad2.y) ? 1 : 0) - (this.gamepad2.right_bumper ? 1 : 0) );

            //controls for the arm
            armrotate.setPower( (this.gamepad2.left_trigger - this.gamepad2.right_trigger) * 0.4 ); //positive is up/counterclockwise
            armextend.setPower( ( (this.gamepad2.dpad_down ? 1 : 0) - ( this.gamepad2.dpad_up ? 1 : 0) ) * 0.8 );

            /* gamepad1 controls */

            //sets the speed modifier
            if(gamepad1.dpad_up){
                if(d_pad && speed < 1.0)
                    speed += 0.1;
                d_pad = false;
            }else if(gamepad1.dpad_down){
                if(d_pad && speed > 0.1)
                    speed = speed - 0.1;
                d_pad = false;
            }else
                d_pad = true;

            //calculate wheel power
            double move0 = this.gamepad1.left_stick_y + this.gamepad1.left_stick_x;
            double move1 = this.gamepad1.left_stick_y - this.gamepad1.left_stick_x;

            //run the wheels
            fl.setPower( (move1 - this.gamepad1.right_stick_x) * speed );
            fr.setPower( (move0 + this.gamepad1.right_stick_x) * speed );
            bl.setPower( (move0 - this.gamepad1.right_stick_x) * speed );
            br.setPower( (move1 + this.gamepad1.right_stick_x) * speed );

            telemetry.addData("% of Speed: ", speed);
            telemetry.addData("pos: ", armextend.getCurrentPosition() );
            telemetry.update();
        }
    }
}
