/*
Copyright 2024 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class AGirlsTeleop extends LinearOpMode {
    
    DcMotorEx intake, Lextend, Rextend, arm, fl, fr, bl, br;
    Servo wrist;
    PIDFController controller = new PIDFController(50, 0, 2, 0);
    
    double strafemulti;
    double forwardmulti;
    double turnmulti;
    
    int armTarget = 0;
    int extendTarget = 0;
    double intakeSpin = 0.0;
    double intakePos = 0.0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        Lextend = hardwareMap.get(DcMotorEx.class, "EL");
        Rextend = hardwareMap.get(DcMotorEx.class, "ER");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        wrist = hardwareMap.servo.get("wrist");
        
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        fr.setDirection(DcMotorEx.Direction.REVERSE);
        br.setDirection(DcMotorEx.Direction.REVERSE);
        
        Lextend.setDirection(DcMotorEx.Direction.REVERSE);
        
        Rextend.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Lextend.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        Rextend.setTargetPosition(0);
        Lextend.setTargetPosition(0);
        
        wrist.setPosition(0.01);
        
        Rextend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Lextend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
            
            if(gamepad1.a || gamepad2.a) { //HOME
                armTarget(0);
                intakeSpin = 0;
                extendTarget(0);
                wrist.setPosition(0.25);
            }
            
            if(gamepad1.y || gamepad2.y) { //HIGHBASKET
                extendTarget(-3300);
                wrist.setPosition(0.15);
                armTarget(1200);
            }
            
            /*if(gamepad1.x || gamepad2.x) {
                wrist.setPosition(0.3);
            }*/
            
            if(gamepad1.b || gamepad2.b) { //LOWBASKET
                extendTarget(-1200);
                wrist.setPosition(0.9);
            }
            
            if(gamepad1.right_bumper || gamepad2.right_bumper) { //PRE-INTAKE
                armTarget(7700);
                wrist.setPosition(0.25);
            }
            
            if((gamepad1.left_trigger > 0.05) || (gamepad2.left_trigger > 0.05)){ //OUTTAKE
                intake.setPower(-0.6);
            }
            
            else if((gamepad1.right_trigger > 0.05) || (gamepad2.right_trigger > 0.05)){ //INTAKE   
                intake();
                intakePos();
                //wrist.setPosition(0.6);
               // intake.setPower(0.5);         
                
            }
            if (!(gamepad1.left_trigger > 0.05) || (gamepad2.left_trigger > 0.05) && !(gamepad1.right_trigger > 0.05) || gamepad2.right_trigger > 0.05) {
                intake.setPower(0);
            }
            // intake.setPower(intakeSpin);
            
            double strafemulti = 1;
            double forwardmulti = 1;
            double turnmulti = 1;
            
            double leftx = -strafemulti*(gamepad1.left_stick_x); //strafe left and right
            double lefty = -forwardmulti*(gamepad1.left_stick_y + gamepad1.right_stick_y); //forward and backward
            double rightx = -turnmulti*(gamepad1.right_stick_x); //turn left and right
                        
            fl.setPower(lefty - leftx - rightx);
            fr.setPower(lefty + leftx + rightx);
            bl.setPower(lefty + leftx - rightx);
            br.setPower(lefty - leftx + rightx);
            
            telemetry.addData("armpos", arm.getCurrentPosition());
            telemetry.update();
            
           
        }
    }
    
    /*public void rotate(double target, double position){
        arm.setPower(controller.calculate(position, target));
    }
    
    public void armTarget(int encoders){
        rotate(rotate, Rextend.getCurrentPosition());
        rotate(rotate, Lextend.getCurrentPosition());
    }*/
    
    public void armTarget(int encoders){
        arm.setTargetPosition(encoders);
        arm.setPower(1);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    
    public void scorePos(){
        armTarget(0);
    }
    
    public void extendTarget(int encoders){
        Rextend.setTargetPosition(encoders);
        Rextend.setPower(1);
        Rextend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Lextend.setTargetPosition(encoders);
        Lextend.setPower(1);
        Rextend.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    
    public void intake(){
        wrist.setPosition(0.3);
        intake.setPower(1);
    }
    
    public void intakePos(){
        armTarget(9000);
    }
    
    // public void intakeSpin(double power){
    //     intake.setPower(power);
    // }
    
    
    public void home(){
        armTarget(0);
        extendTarget(0);
    }
    
    public void armMotionLimits() {
        
        if (Math.abs(armTarget-arm.getCurrentPosition()) < 5) {
            arm.setPower(1.0);
        } else {
            if (armTarget >= 400){
                    
                if (arm.getCurrentPosition() <= 400) {
                    arm.setPower(Math.max(-1, Math.min(1, controller.calculate(arm.getCurrentPosition(), armTarget))));
                } else {
                    arm.setPower(Math.max(-1, Math.min(1, controller.calculate(arm.getCurrentPosition(), armTarget))));
                }
            
            }
            if (armTarget < 400) {
                if (arm.getCurrentPosition() >= 450) {
                    arm.setPower(Math.max(-1, Math.min(1, controller.calculate(arm.getCurrentPosition(), armTarget))));
                } else {
                    arm.setPower(Math.max(-1, Math.min(1, controller.calculate(arm.getCurrentPosition(), armTarget))));
                }
            }
        }
    }
}
