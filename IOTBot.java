package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.PIDFController;

@TeleOp
public class IOTBot extends LinearOpMode {
    public DcMotorEx lslide;
    public DcMotorEx rslide;
    PIDFController controller;
    
    @Override
    public void runOpMode() throws InterruptedException {
        int lpos, rpos;
        int extendTarget = 0;


        // Motor and servo declarations
        DcMotor lf = hardwareMap.dcMotor.get("lf");
        DcMotor lb = hardwareMap.dcMotor.get("lb");
        DcMotor rf = hardwareMap.dcMotor.get("rf");
        DcMotor rb = hardwareMap.dcMotor.get("rb");
      //  Servo wrist = hardwareMap.servo.get("wrist");
        Servo ls = hardwareMap.servo.get("ls");
        Servo rs = hardwareMap.servo.get("rs");
        Servo lservo = hardwareMap.servo.get("lservo");
        Servo rservo = hardwareMap.servo.get("rservo");
        Servo claw = hardwareMap.servo.get("claw");
        lslide = hardwareMap.get(DcMotorEx.class, "lslide");
        rslide = hardwareMap.get(DcMotorEx.class, "rslide");
     

        // Motor direction configurations
        lf.setDirection(DcMotor.Direction.FORWARD);
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
        rs.setPosition(1);
        ls.setPosition(0);
        lservo.setPosition(0);
        rservo.setPosition(1);
        

        waitForStart();
        
        lslide.setPower(0);
        rslide.setPower(0);

        while (opModeIsActive()) {
            lpos = lslide.getCurrentPosition();
            rpos = rslide.getCurrentPosition();
            // Gamepad control inputs
            lefty1 = -gamepad1.left_stick_y;
            leftx1 = -gamepad1.left_stick_x;
            rightx1 = -gamepad1.right_stick_x;
          

            // Wrist control using gamepad 2 left joystick x
        
            if (gamepad1.dpad_right || gamepad2.dpad_right) {
                rs.setPosition(1);
                ls.setPosition(0);
            }
            if (gamepad1.x || gamepad2.x) {
                lservo.setPosition(0.73);
                rservo.setPosition(0.27);
                telemetry.addLine("x");
            }
            if (gamepad1.a || gamepad2.a) {
                lservo.setPosition(0);
                rservo.setPosition(1);
                extendTarget = 0;
            }
             if (gamepad1.right_bumper || gamepad2.right_bumper) {
               claw.setPosition(0);
               
            }
             if (gamepad2.left_bumper) {
                claw.setPosition(0.37);
            }
            if (gamepad2.b){
                lservo.setPosition(1);
                rservo.setPosition(0);
            }
            if(gamepad2.dpad_left){
                extendTarget = 850;
            }
            // Base movement logic
            if (gamepad1.left_bumper || gamepad2.guide) {
                power = 0.3; // Low speed mode
            } else {
                power = 1.0; // Full power driving
            }
            
            lf.setPower((power * -lefty1) + (power * leftx1) + power * rightx1);
            rb.setPower((power * lefty1) - (power * leftx1) + power * rightx1);
            rf.setPower((power * lefty1) + (power * leftx1) + power * rightx1);
            lb.setPower((power * lefty1) - (power * -leftx1) + power * -rightx1);
            
            // lslide.setPower(-0.5 * armpos);
            // rslide.setPower(-0.5 * armpos);
           
            if (gamepad1.dpad_up || gamepad2.dpad_up) { 
                // // Move arm up
                // lslide.setTargetPosition(lpos+20);
                // rslide.setTargetPosition(rpos+20);
                // lslide.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                // rslide.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                extendTarget = 250;
                // rslide.setPower((250-lslide.getCurrentPosition()) * 1);
                //lservo.setPosition(0.6);
            //   rservo.setPosition(0.4);
                telemetry.addLine("dpad up");
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                // Move arm down
                // lslide.setTargetPosition(lpos-20);
                // rslide.setTargetPosition(rpos-20);
                // lslide.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                // rslide.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                // lslide.setPower((5-lslide.getCurrentPosition()) * 0.003);
                // rslide.setPower((5-lslide.getCurrentPosition()) * 0.003);
                lservo.setPosition(0);
                rservo.setPosition(1);
                extendTarget = 0;
                claw.setPosition(0.37);
                // rservo.setPosition(1);
                telemetry.addLine("dpad down");
            }
            if(gamepad1.right_trigger > 0.075){
                ls.setPosition(1);
                rs.setPosition(0);
            } else {
                ls.setPosition(0);
                rs.setPosition(1);
            }
            
            // lslide.setPower(1);
            lslide.setPower(-controller.calculate(extendTarget, lslide.getCurrentPosition()));
            rslide.setPower(-controller.calculate(extendTarget, lslide.getCurrentPosition()+10));
          
            // Telemetry data
            telemetry.addData("power:", power);
            telemetry.addData("low power mode:", gamepad1.right_bumper);
            telemetry.addData("lslide pos", lslide.getCurrentPosition());
            telemetry.addData("rslide pos", rslide.getCurrentPosition());
            telemetry.addData("l outtake servo pos", lservo.getPosition());
            telemetry.addData("extendtarget", extendTarget);
            telemetry.addData("extendPIDF", controller.calculate(lslide.getCurrentPosition(), extendTarget));
            telemetry.addData("Version", "2");
            telemetry.update();
            }
        }
}
