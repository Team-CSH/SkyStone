package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;
import java.util.List;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Thread_TestOPMODE")
public class OpMode_Thread extends OpMode {


    public DcMotor scissor_lift_left_motor;
    public DcMotor scissor_lift_right_motor;
    public CRServo suction_servo1;
    public CRServo suction_servo2;
    public CRServo expansion_servo;
    public Servo pliers1_servo;
    public Servo pliers2_servo;
    final double PLIERS_ON = 1.0;
    final double PLIERS_OFF = 0.0;

    public boolean resetHardware() { //chestie fara sens doar ca sa dea true
        scissor_lift_right_motor.setPower(0);
        scissor_lift_left_motor.setPower(0);
        suction_servo1.setPower(0);
        suction_servo2.setPower(0);
        expansion_servo.setPower(0);
        pliers1_servo.setPosition(PLIERS_OFF);
        pliers2_servo.setPosition(PLIERS_OFF);
        return true;
    }

    public void scissor_lift_motors(double power) {
        scissor_lift_right_motor.setPower(power);
        scissor_lift_left_motor.setPower(-power);
    }

    public void suction_servos(double power) {
        suction_servo1.setPower(power);
        suction_servo2.setPower(-power);
    }

    public void Arm_Servos(double power) {
        expansion_servo.setPower(power);
    }

    public void pliers(double pos) {
        pliers1_servo.setPosition(pos);
        pliers2_servo.setPosition(-pos);
    }

    @Override
    public void init() {
        //MOTORS AND HARDWARE MAPS
        scissor_lift_left_motor = hardwareMap.dcMotor.get("scissor_lift_left_motor");
        scissor_lift_right_motor = hardwareMap.dcMotor.get("scissor_lift_right_motor");
        scissor_lift_right_motor.setDirection(DcMotor.Direction.REVERSE);
        //SERVOS
        suction_servo1 = hardwareMap.crservo.get("suction_servo1");
        suction_servo2 = hardwareMap.crservo.get("suction_servo2");
        expansion_servo = hardwareMap.crservo.get("expansion_servo");
        pliers1_servo = hardwareMap.servo.get("pliers1_servo");
        pliers2_servo = hardwareMap.servo.get("pliers2_servo");
        resetHardware();
    }



    @Override
    public void loop() {
        Thread lift_up_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                if(gamepad1.left_stick_y > 0) {
                    scissor_lift_motors(1);
                } else {
                    scissor_lift_motors(0);
                }
            }
        });

        Thread lift_down_thread = new Thread(new Runnable() {
            @Override
            public void run() {
                if(gamepad1.left_stick_y < 0) {
                    scissor_lift_motors(-1);
                } else {
                    scissor_lift_motors(0);
                }
            }
        });

        Thread suction_intake = new Thread(new Runnable() {
            @Override
            public void run() {
                if(gamepad1.a) {
                    suction_servos(1);
                } else {
                    suction_servos(0);
                }
            }
        });

        Thread suction_outtake = new Thread(new Runnable() {
            @Override
            public void run() {
                if(gamepad1.y) {
                    suction_servos(-1.0);
                } else {
                    suction_servos(0.0);
                }
            }
        });

        Thread arm_extended = new Thread(new Runnable() {
            @Override
            public void run() {
                if(gamepad1.right_bumper) {
                    Arm_Servos(1);
                } else {
                    Arm_Servos(0);
                }
            }
        });

        Thread arm_retracted = new Thread(new Runnable() {
            @Override
            public void run() {
                if(gamepad1.left_bumper) {
                    Arm_Servos(-1);
                } else {
                    Arm_Servos(0);
                }
            }
        });

        Thread pliers_on = new Thread(new Runnable() {
            @Override
            public void run() {
                if(gamepad1.x) {
                    pliers(PLIERS_ON);
                }
            }
        });

        Thread pliers_off = new Thread(new Runnable() {
            @Override
            public void run() {
                if(gamepad1.b) {
                    pliers(PLIERS_OFF);
                }
            }
        });

        //starting the threads
        lift_up_thread.start();
        lift_down_thread.start();
        suction_intake.start();
        suction_outtake.start();
        arm_extended.start();
        arm_retracted.start();
        pliers_on.start();
        pliers_off.start();

        /* INTREBARE: Oare ar trebui sa folosesc si .join() ca sa astepte a un thread sa moara , si sa fie un fel de memory cleaner? sau act like one */
    }
}
