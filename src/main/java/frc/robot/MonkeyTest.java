  // Monkey test selection
  // 0: turning drive controls are inverted
  // 1: claw belt broken
  // 2: claw pneumatics broken
  // 3: claw pnematics reversed
  // 4: shoulder position is +20 degrees off zero
  // 5: no lights
  // 6: waist doesn't spin
  // 7: arm doesn't extend

package frc.robot;

import java.util.concurrent.ThreadLocalRandom;

public class MonkeyTest {
    public static int m_monkeySelection = ThreadLocalRandom.current().nextInt(0, 8 + 1);  // max is 8
    private static MonkeyTest instance = null;

    public static int get() {
        return m_monkeySelection;
    }
}