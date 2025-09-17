// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what
 * you are doing, do not modify this file except to change the parameter class
 * to the startRobot
 * call.
 */
public final class Main {
  private Main() {
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>
   * If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    // :(){:|:&};:
    // Put that into your linux terminal for a fun time ;) - Ian
    // Or do `sudo rm -rf --no-preserve-root /` for a perminate fun time - Brogan
    
    for (String arg : args) {
      System.out.println("AAAAAAAAAAARRRRRRRRGGGGGGGUUUUUUUMMMMMMMMEEEEEEENNNNNNTTTTT");
      System.out.println(arg);
    }
    RobotBase.startRobot(Robot::new);
  }
}
