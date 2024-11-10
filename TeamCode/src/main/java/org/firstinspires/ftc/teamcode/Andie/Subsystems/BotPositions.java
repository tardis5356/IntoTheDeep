package org.firstinspires.ftc.teamcode.Andie.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BotPositions {
    public static double GRIPPER_OPEN = .3, GRIPPER_CLOSED = 0, GRIPPER_INTAKE = .15;
    public static double WRIST_TRANSIT = 0, WRIST_WALL = .2, WRIST_INTAKE = .3, WRIST_BASKET = .7;
    public static double ARM_BASKET = 1, ARM_INTAKE = .4, ARM_SPECIMEN = .7, ARM_TRANSIT = .5;
    public static double INTAKE_NEUTRAL = 0, INTAKE_DOWN = 0.3, INTAKE_IN = 1, INTAKE_OUT = -1;
    public static int LIFT_WALL = 10, LIFT_SPECIMEN_HIGH = 40, LIFT_SPECIMEN_LOW = 20, LIFT_BASKET_HIGH = 50, LIFT_BASKET_LOW = 20;
}
