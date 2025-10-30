package frc.robot;

import java.util.Optional;

/** 
<p>
<strong>Example Code for Systerface</strong>
<br>
<p>
<code>public class Intake extends SubsystemBase</code> <strong>implements Systerface</strong> <code>{</code>
<br>
*/
public interface Systerface {
	// This method should return
	// your subsystem's state.
	default Optional<Object> getCurrentState() {
		return Optional.empty();
	}
}

