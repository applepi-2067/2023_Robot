  // Monkey test selection
  // 0: turning drive controls are inverted
  // 1: claw belt broken
  // x2: claw pneumatics broken
  // x3: claw pnematics reversed
  // x4: high scoring position too high
  // x5: medium scoring position too high
  // x6: no lights
  // x7: waist doesn't spin
  // x8: arm doesn't extend

package frc.robot;

import java.util.concurrent.ThreadLocalRandom;

public class MonkeyTest {
    public static int m_monkeySelection = ThreadLocalRandom.current().nextInt(0, 8 + 1);  // max is 8
    private static MonkeyTest instance = null;

    public static int get() {
        return m_monkeySelection;
    }
}