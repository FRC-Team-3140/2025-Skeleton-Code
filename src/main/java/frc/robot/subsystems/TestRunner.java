// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.tests.TestAlgaeIntake;
import frc.robot.tests.TestAlgaeReef;
import frc.robot.libs.NetworkTables;
import frc.robot.tests.Test;
import frc.robot.tests.TestElevator;
import frc.robot.tests.TestEndEffector;
import frc.robot.tests.TestSwerve;

public class TestRunner extends SubsystemBase {
  private static TestRunner instance = null;

  public enum TestType {
    SWERVE,
    ALGAE_INTAKE,
    END_EFFECTOR,
    ELEVATOR,
    SOURCE_HANDOFF,
    ALGAE_REEF,
    ALGAE_GROUND,
  };

  private final HashMap<TestType, Test> tests = new HashMap<TestType, Test>();

  public static TestRunner getInstance() {
    if (instance == null) {
      instance = new TestRunner();
    }
    return instance;
  }

  private TestRunner() {
    // Subsystems
    tests.put(TestType.SWERVE, new TestSwerve(NetworkTables.swerveButton_b, TestType.SWERVE));
    tests.put(TestType.ALGAE_INTAKE, new TestAlgaeIntake(NetworkTables.algaeButton_b, TestType.ALGAE_INTAKE));
    tests.put(TestType.END_EFFECTOR, new TestEndEffector(NetworkTables.effectorButton_b, TestType.END_EFFECTOR));
    tests.put(TestType.ELEVATOR, new TestElevator(NetworkTables.elevatorButton_b, TestType.ELEVATOR));

    // Tests
    tests.put(TestType.ALGAE_GROUND, new TestAlgaeIntake(NetworkTables.algaeGroundButton_b, TestType.ALGAE_GROUND));
    tests.put(TestType.SOURCE_HANDOFF, new TestAlgaeIntake(NetworkTables.sourceButton_b, TestType.SOURCE_HANDOFF));
    tests.put(TestType.ALGAE_REEF, new TestAlgaeReef(NetworkTables.reefButton_b, TestType.ALGAE_REEF));
  }

  @Override
  public void periodic() {
    for (TestType type : tests.keySet()) {
      tests.get(type).QueryNetworkTable();

      if (!tests.get(type).running)
        continue;

      tests.get(type).Periodic();
    }
  }

  public boolean isRunning(TestType type) {
    return tests.get(type).running;
  }

  public void setState(TestType type, boolean run) {
    if (tests.get(type).running == run)
      return;

    if (run) {
      tests.get(type).Start();
    } else {
      tests.get(type).Stop();
    }
  }

  public void updateStates() {
    for (TestType type : tests.keySet()) {
      setState(type, tests.get(type).ntEntry.getBoolean(false));
    }
  }

  public void stopAll() {
    for (TestType type : tests.keySet()) {
      setState(type, false);
    }
  }
}
