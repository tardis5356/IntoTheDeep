package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BotPositions {
    public static double GRIPPER_OPEN = .6, GRIPPER_CLOSED = 0.373, GRIPPER_INTAKE = .28;

    public static double WRIST_TRANSIT = 0.87, WRIST_WALL = .8, WRIST_INTAKE = .668, WRIST_BASKET = .5, WRIST_SPECIMEN = .58;
    //TODO: WRIST_INTAKE, WRIST WALL
    public static double ARM_BASKET = .85, ARM_INTAKE = .29, ARM_WALL = .08, ARM_TRANSIT = .5, ARM_SPECIMEN =.57;
    //TODO: ARM_WALL, ARM_INTAKE, ARM_BASKET (probably)
    public static double INTAKE_ARM_DOWN = 0.069, INTAKE_ARM_NEUTRAL = 0.35, INTAKE_ARM_TRANSFER = .4, INTAKE_ARM_OUTAKE = .6,
            INTAKE_WRIST_DOWN = .2, INTAKE_WRIST_NEUTRAL = .9, INTAKE_WRIST_TRANSFER = .4, INTAKE_WRIST_OUTAKE = .6,
            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_STOP = 0;

    public static double EXTENDO_IN = 0.78, EXTENDO_OUT = .4, EXTENDO_SPECLEFT =.65, EXTENDO_SPECMID =.53, EXTENDO_SPECRIGHT = .4;
    //TODO: EXTENDO_OUT, add EXTENDO_TRANSFER if EXTENDO_IN doesn't work

    public static int LIFT_WALL = -235, LIFT_SPECIMEN_HIGH = -720, LIFT_SPECIMEN_LOW = 0, LIFT_BASKET_HIGH = -2990,
            LIFT_BASKET_LOW = -280, LIFT_TRANSIT = -700, LIFT_LIMIT = -3000, LIFT_TOLERANCE = 10, LIFT_INTAKE = 0;
    //TODO: LIFT_INTAKE, LIFT_TRANSIT, LIFT_BASKET_LOW, LIFT_BASKET_HIGH, LIFT_WALL

    public static double RED_MAX = 750, RED_MIN = 300, BLUE_MAX = 750, BLUE_MIN = 300, YELLOW_MAX = 650, YELLOW_MIN = 620;
    //TODO: all the color limits, with different limits for the intake and gripper should it be necessary
    public static double LIFT_FF = 0.1, WHINCH_FF = 0;

    public static double LIFT_P = 0.009, LIFT_D = 0.0003;
}
