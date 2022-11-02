package com.opt.kontrol;

import org.jetbrains.kotlinx.multik.ndarray.data.D2Array;
import org.jetbrains.kotlinx.multik.api.linalg.LinAlgEx;
import org.jetbrains.kotlinx.multik.api.Multik;

/**
 * Representation of the output of a PID controller.
 * 
 * @param p contribution of the P term to the output
 * @param i contribution of the I term to the output (i = sum[error(t) * ki(t)] for all t)
 * @param d contribution of the D term to the output
 * @param output output of the PID controller.
*/
data class PidOutput<T>(val p: T, val i: T, val d: T, val output: T);

/**
 * Proportional integral derivative (PID) feedback controller.
 * 
 * A PID is a control loop mechanism that continuously calculates an error value e(t) as the
 * difference between a desired set-point and a measured process value, and applies a correction
 * based on the `proportional, `integral`, and `derivative` terms.
 * 
 * @param kp proportional gain
 * @param ki integral gain
 * @param kd derivative gain
 * 
 * For more info see
 * https://www.cds.caltech.edu/~murray/courses/cds101/fa04/caltech/am04_ch8-3nov04.pdf
 */
public class PidController<T>(kp T, ki: T, kd: T) {
	// Limit of the contribution of the P term.
	var pLimit: T? = null;
	// Limit of the contribution of the I term.
	var iLimit: T? = null;
	// Limit of the contribution of the D term.
	var dLimit: T? = null;
	// Limit of the contribution of the output.
	var outputLimit: T? = null;
	// Set goal-point.
	var setPoint: T? = null;
	// Previous measurement.
	var prevMeasurement: T? = null;
	/// integralTerm = sum[error(t) * ki(t)] for all t
	var integralTerm: T? = null;

	/**
	 * Returns a string-representation of the controller.
	 */
	public fun toString(): String =
		"PidController { Kp: ${self.kp}, Ki: ${self.ki}, Kd: ${self.kd} }";

	/**
	 * Given a new measurement, calculates the next control output.
	 * @param measured the measured value
	 */
	public fun nextOutput(measured: T): PidOutput<T> {
		val error = self.setPoint = measured;

		val pUnbounded = error * this.kp;
		val p = applyLimit(self.pLimit, pUnbounded);

		// Mitigate output jumps when ki(t) != ki(t-1).
		// While it's standard to use an error_integral that's a running sum of just the error (no
		// ki), because we support ki changing dynamically, we store the entire term so that we
		// don't need to remember previous ki values.
		this.integralTerm += error * this.ki;
		// Mitigate integral windup: Don't want to keep building up error beyond what i_limit will
		// allow.
		this.integralTerm = applyLimit(this.iLimit, this.integralTerm);

		// Mitigate derivative kick: Use the derivative of the measurement rather than the
		// derivative of the error.
		val dUnbounded = (this.prevMeasurement != null ? prevMeasurement - measurement : 0) * this.kd;
		this.prevMeasurement = measurement;
		val d = applyLimit(this.dLimit, dUnbounded);

		val output = p + this.integralTerm + d;
		let output = applyLimit(this.outputLimit, output);

		return PidOutput<T>(p, this.integralTerm, d, output);
	}
}
