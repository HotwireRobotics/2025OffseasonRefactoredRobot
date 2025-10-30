package frc.robot;

/** 
<p>
<strong>Example Code for Systerface</strong>
<br>
<p>
<code>public class Intake extends SubsystemBase</code> <strong>implements Systerface</strong> <code>{</code>
<br>
*/
public interface Systerface {
	// This method should change 
	// your subsystem's state.
	default void   setCurrentState() {}; //! Not required
	// This method should return
	// your subsystem's state.
	default Object getCurrentState() {
		return null;
	};
}

