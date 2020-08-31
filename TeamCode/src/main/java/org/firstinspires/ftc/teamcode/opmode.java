package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Opmode multithreading")
public class OpMode_Thread extends OpMode {

    private enum States {
        LIFT_UP_STATE,
        LIFT_DOWN_STATE,
        SUCTION_INTAKE_STATE,
        SUCTION_OUTTAKE_STATE,
        ARM_EXTENDED_STATE,
        ARM_RETRACTED_STATE,
        PLIERS_ON_STATE,
        PLIERS_OFF_STATE,
        STOP_STATE,
    }

    public DcMotor scissor_lift_left_motor;
    public DcMotor scissor_lift_right_motor;
    public CRServo suction_servo1;
    public CRServo suction_servo2;
    public CRServo expansion_servo;
    public Servo pliers1_servo;
    public Servo pliers2_servo;
    public States LiftStates;
    public States SuctionStates;
    public States ArmStates;
    public States PliersStates;
    final double PLIERS_ON = 1.0;
    final double PLIERS_OFF = 0.0;

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

    public void changestate(States nextstate, States actualstate) {
        actualstate = nextstate;
    }

    @Override
    public void init() {
        scissor_lift_left_motor = hardwareMap.dcMotor.get("scissor_lift_left_motor");
        scissor_lift_right_motor = hardwareMap.dcMotor.get("scissor_lift_right_motor");
        scissor_lift_right_motor.setDirection(DcMotor.Direction.REVERSE);
        suction_servo1 = hardwareMap.crservo.get("suction_servo1");
        suction_servo2 = hardwareMap.crservo.get("suction_servo2");
        expansion_servo = hardwareMap.crservo.get("expansion_servo");
        pliers1_servo = hardwareMap.servo.get("pliers1_servo");
        pliers2_servo = hardwareMap.servo.get("pliers2_servo");
    }

    @Override
    public void loop() {
        Thread suction = new Thread(new Runnable() {
            @Override
            public void run() {
                changestate(SuctionStates, States.STOP_STATE);
                switch (SuctionStates) {
                    case SUCTION_INTAKE_STATE:
                        if (gamepad1.a) { suction_servos(1); } 
                        else { changestate(SuctionStates, States.SUCTION_OUTTAKE_STATE); }
                        break;
                    case SUCTION_OUTTAKE_STATE:
                        if (gamepad1.y) { suction_servos(-1.0); } 
                        else { changestate(SuctionStates, States.STOP_STATE); }
                        break;
                    case STOP_STATE:
                        suction_servos(0); break;
                }
            }
        });
        Thread lift = new Thread(new Runnable() {
            @Override
            public void run() {
                changestate(LiftStates, States.STOP_STATE);
                switch (LiftStates) {
                    case LIFT_UP_STATE:
                        if (gamepad1.left_stick_y > 0) { scissor_lift_motors(1); } 
                        else { changestate(LiftStates, States.LIFT_DOWN_STATE); }
                        break;
                    case LIFT_DOWN_STATE:
                        if (gamepad1.left_stick_y < 0) { scissor_lift_motors(-1); } 
                        else { changestate(LiftStates, States.STOP_STATE); }
                        break;
                    case STOP_STATE:
                        scissor_lift_motors(0); break;
                }
            }
        });
        Thread arm = new Thread(new Runnable() {
            @Override
            public void run() {
                changestate(ArmStates, States.STOP_STATE);
                switch (ArmStates) {
                    case ARM_EXTENDED_STATE:
                        if (gamepad1.right_bumper) { Arm_Servos(1); } 
                        else { changestate(ArmStates, States.ARM_RETRACTED_STATE); }
                        break;
                    case ARM_RETRACTED_STATE:
                        if (gamepad1.left_bumper) { Arm_Servos(-1); } 
                        else { changestate(ArmStates, States.STOP_STATE); }
                        break;
                    case STOP_STATE:
                        Arm_Servos(0); break;
                }
            }
        });
        Thread pliers = new Thread(new Runnable() {
            @Override
            public void run() {
                changestate(PliersStates, States.PLIERS_OFF_STATE);
                switch (PliersStates) {
                    case PLIERS_ON_STATE:
                        if (gamepad1.x) { pliers(PLIERS_ON); } 
                        else { changestate(PliersStates, States.PLIERS_OFF_STATE); }
                        break;
                    case PLIERS_OFF_STATE:
                        if (gamepad1.b) { pliers(PLIERS_OFF); } break;
                }
            }
        });
        suction.start(); lift.start(); arm.start(); pliers.start();
    }
}
