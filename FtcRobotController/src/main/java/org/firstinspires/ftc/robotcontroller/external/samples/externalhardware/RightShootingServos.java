package org.firstinspires.ftc.robotcontroller.external.samples.externalhardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RightShootingServos {
    public Servo RightShooting;
    public double SetPos = 0;
 //   public void init(HardwareMap hwmap) {
        public void init() {
            RightShooting = hardwareMap.get(Servo.class, "Door");


    }


}
