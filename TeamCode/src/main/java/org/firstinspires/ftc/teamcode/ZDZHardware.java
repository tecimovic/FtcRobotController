package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class ZDZHardware {
    // Declare OpMode members.
    private final DcMotor leftDrive;
    private final DcMotor rightDrive;
    private final DcMotorEx launcher;
    private final CRServo leftFeeder;
    private final CRServo rightFeeder;
    private final Servo light;

    private final DistanceSensor frontDistance;
    private final NormalizedColorSensor colorSensor;
    private ElapsedTime timer;

    private LED ledLeftRed;
    private LED ledLeftGreen;
    private LED ledRightRed;
    private LED ledRightGreen;

    private int tickCount=0;


    ZDZHardware(HardwareMap hardwareMap) {
        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         */
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        light = hardwareMap.get(Servo.class, "light");
        frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        ledLeftRed = hardwareMap.get(LED.class, "led_left_red");
        ledLeftGreen = hardwareMap.get(LED.class, "led_left_green");
        ledRightRed = hardwareMap.get(LED.class, "led_right_red");
        ledRightGreen = hardwareMap.get(LED.class, "led_right_green");

        lightWhite();
        timer = new ElapsedTime();
        timer.reset();
    }

    public CRServo rightFeeder() { return rightFeeder; }
    public CRServo leftFeeder() { return leftFeeder; }
    public DcMotor leftDrive() { return leftDrive; }
    public DcMotor rightDrive() { return rightDrive; }
    public DcMotorEx launcher() { return launcher; }
    public NormalizedRGBA colorSensor() { return colorSensor.getNormalizedColors(); }
    public double frontDistanceInCentimeters() { return frontDistance.getDistance(DistanceUnit.CM);}

    public Servo getLight() {
        return light;
    }
 public void lightwhee() {
        for(double x=0.280;x >=0.723;x+=0.01 ){
            light.setPosition(x);
            halfSecondTick(1);

        }
     for(double x=0.723;x <=0.280;x-=0.01 ){
         light.setPosition(x);
         halfSecondTick(1);

     }

 }
    public void lightWhite() { light.setPosition(1.0);}
    public void lightGreen() { light.setPosition(0.5); }
    public void lightOff() { light.setPosition(0.0); }

    public void lightRed() { light.setPosition(0.280);}

    public void lightBlue() { light.setPosition(0.63); }

    public void ledsOff() {
        this.ledRightRed.off();
        this.ledLeftRed.off();
        this.ledRightGreen.off();
        this.ledLeftGreen.off();
    }
    public void ledsRed() {
        this.ledRightRed.on();
        this.ledLeftRed.on();
        this.ledRightGreen.off();
        this.ledLeftGreen.off();
    }

    public void ledsGreen() {
        this.ledRightRed.off();
        this.ledLeftRed.off();
        this.ledRightGreen.on();
        this.ledLeftGreen.on();
    }
    public void init_loop() {
        if (timer.milliseconds() > 500 ) {
            timer.reset();
            halfSecondTick(tickCount++);
        }
    }

    private void halfSecondTick(int count) {
        switch(count%3) {
            case 0:
                lightRed();
                ledsRed();
                break;
            case 1:
                lightGreen();
                ledsGreen();
                break;
            case 2:
                lightBlue();
                ledsOff();
                break;
        }
    }
}