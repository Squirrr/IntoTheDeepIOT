//ASpecimenAuton
/*
Copyright 2026 FIRST Tech Challenge Team 21330

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
package org;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous()

public class ASpecimenAuton extends LinearOpMode {
    
    ElapsedTime timer;
    public DcMotorEx lslide;
    public DcMotorEx rslide;
    PIDFController controller;
    DcMotorEx lf, rf, lb, rb;


    @Override
    public void runOpMode() {

        int lpos, rpos;
        int extendTarget = 0;
        timer = new ElapsedTime();


        // Motor and servo declarations
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
      //  Servo wrist = hardwareMap.servo.get("wrist");
        Servo ls = hardwareMap.servo.get("ls");
        Servo rs = hardwareMap.servo.get("rs");
        Servo lservo = hardwareMap.servo.get("lservo");
        Servo rservo = hardwareMap.servo.get("rservo");
        Servo claw = hardwareMap.servo.get("claw");
        lslide = hardwareMap.get(DcMotorEx.class, "lslide");
        rslide = hardwareMap.get(DcMotorEx.class, "rslide");
     

        // Motor direction configurations
        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        lslide.setDirection(DcMotorEx.Direction.REVERSE);
        rslide.setDirection(DcMotorEx.Direction.FORWARD);

        // Set zero power behavior for all motors
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // lslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // rslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        lslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        lslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        
        controller = new PIDFController(0.0075,0,0,0);
        
        // PID constants for the arm (adjust these based on experimentation)
      

        double power = 1;
        double lefty1, leftx1, rightx1, armpos, sliderpos;
        rs.setPosition(0.99);
        ls.setPosition(0.01);
        lservo.setPosition(0.01);
        rservo.setPosition(0.99);
        

        waitForStart();
        claw.setPosition(0.01);
        
        lslide.setPower(0);
        rslide.setPower(0);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            timer.reset();
            // lservo.setPosition(0.73);
            // rservo.setPosition(0.27);
            extendTarget = 250;
            
            while (timer.seconds() < 2) {
                lslide.setPower(-controller.calculate(extendTarget, lslide.getCurrentPosition()));
                rslide.setPower(-controller.calculate(extendTarget, lslide.getCurrentPosition()+10));
                lf.setPower(-0.25);
                rf.setPower(-0.25);
                lb.setPower(-0.25);
                rb.setPower(-0.25);
            }
            while (timer.seconds() < 3) {
                lslide.setPower(-controller.calculate(extendTarget, lslide.getCurrentPosition()));
                rslide.setPower(-controller.calculate(extendTarget, lslide.getCurrentPosition()+10));
                lservo.setPosition(0.73);
                rservo.setPosition(0.27);
                lf.setPower(0.0);
                rf.setPower(0.0);
                lb.setPower(0.0);
                rb.setPower(0.0);
            }
            extendTarget=0;
            while (timer.seconds() < 4.0) {
                lslide.setPower(-controller.calculate(extendTarget, lslide.getCurrentPosition()));
                rslide.setPower(-controller.calculate(extendTarget, lslide.getCurrentPosition()+10));
                claw.setPosition(0.37);
                lservo.setPosition(0.01);
                rservo.setPosition(0.99);
                lf.setPower(0.25);
                rf.setPower(0.25);
                lb.setPower(0.25);
                rb.setPower(0.25);
            }
            while (timer.seconds() < 6) {
                lslide.setPower(-controller.calculate(extendTarget, lslide.getCurrentPosition()));
                rslide.setPower(-controller.calculate(extendTarget, lslide.getCurrentPosition()+10));
                lservo.setPosition(0.01);
                rservo.setPosition(0.99);
                lf.setPower(0.5);
                rf.setPower(-0.5);
                lb.setPower(-0.5);
                rb.setPower(0.5);
            }
        }
    }
}
