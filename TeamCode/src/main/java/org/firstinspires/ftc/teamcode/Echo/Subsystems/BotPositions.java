package org.firstinspires.ftc.teamcode.Echo.Subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class BotPositions {
    public static double GRIPPER_OPEN = .475, GRIPPER_CLOSED = 0.373, GRIPPER_INTAKE = .475;
    public static double WRIST_TRANSIT = 0.87, WRIST_WALL = .92, WRIST_INTAKE = .42, WRIST_BASKET = .5, WRIST_SPECIMEN = .4;
    public static double ARM_BASKET = .85, ARM_INTAKE = .38, ARM_WALL = .2, ARM_TRANSIT = .5, ARM_SPECIMEN =.43;
    public static double INTAKE_ARM_DOWN = 0.05, INTAKE_ARM_TRANSFER = 0.58, INTAKE_ARM_UP = .19, INTAKE_ARM_OUTAKE = .8,
            INTAKE_WRIST_DOWN = .0, INTAKE_WRIST_TRANSFER = .8, INTAKE_WRIST_UP = .0, INTAKE_WRIST_OUTAKE = .6,
            INTAKE_IN = 1, INTAKE_OUT = -1, INTAKE_STOP = 0, INTAKE_TRANSFER = .2;
    public static double EXTENDO_IN = 0.82, EXTENDO_OUT = .5, EXTENDO_SPECLEFT =.65, EXTENDO_SPECMID =.53, EXTENDO_SPECRIGHT = .4;
    public static int LIFT_WALL = -235, LIFT_SPECIMEN_HIGH = -720, LIFT_SPECIMEN_LOW = 0, LIFT_BASKET_HIGH = -2990,
            LIFT_BASKET_LOW = -280, LIFT_TRANSIT = -700, LIFT_LIMIT = -3000, LIFT_TOLERANCE = 10, LIFT_INTAKE = 0;
    public static double RED_MAX = 750, RED_MIN = 300, BLUE_MAX = 750, BLUE_MIN = 300, YELLOW_MAX = 650, YELLOW_MIN = 620;
    public static double ANTI_GRAV = 0.1, WHINCH_FF = 0;
    public static double LIFT_P = 0.009, LIFT_D = 0.0003;
}
