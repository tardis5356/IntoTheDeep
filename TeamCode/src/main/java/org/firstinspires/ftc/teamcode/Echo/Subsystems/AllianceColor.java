package org.firstinspires.ftc.teamcode.Echo.Subsystems;

//this is a public variable. it is kept as a separate file such that all files can read and write to it,
//but additionally it being a separate file means that whatever value is fed to it will be maintained between programs being started.
//this lets us assign it a value in auto, specifically the color of the sample, so that logic in teleop can be based off of things in auto
public class AllianceColor {
    public static String aColor = new String();
}
