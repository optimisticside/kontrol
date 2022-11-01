import org.jetbrains.kotlinx.multik.ndarray.data.D2Array;
import org.jetbrains.kotlinx.multik.api.linalg.LinAlgEx;
import org.jetbrains.kotlinx.multik.api.Multik;

/**
 * Linear Quadratic Regulator (LQR) feedback controller.
 * 
 * An LQR is able to solve the optimal control problem in a system that can be described by a set
 * of linear differential equations, and the cost can be described by a quadratic function.
 * 
 * For more info see
 * https://www.cds.caltech.edu/~murray/courses/cds110/wi06/lqr.pdf
 * 
 * Riccatti equation solved via
 * https://www.tandfonline.com/doi/abs/10.1080/00207170410001714988
 * https://scicomp.stackexchange.com/questions/30757/discrete-time-algebraic-riccati-equation-dare-solver-in-c
 */
public class LqrController<T>(s: int, c: int, rng: () -> T) {
	// State cost.
	var q: D2Array<T>? = null;
	// Control cost.
	var r: D2Array<T>? = null;
	// Optimal gain.
	var k: D2Array<T>? = null;
	// The controller also has an i-controller for removing steady state error in y dimension
	// i-controller coefficient.
	var ki: T = 0,
	// Sccumulating integral error for the i-controller.
	var integral_error: T = 0,

	/**
	 * Returns a string-representation of the controller.
	 */
	public fun toString(): String =
		"LqrController { Q: ${self.q}, R: ${self.r}, Ki: ${self.ki} }";

	/**
	 * Computes and returns the optimal gain matrix K for the LQR controller
	 * @param a state matrix of shape SxS
	 * @param b control matrix of shape SxC
	 * @param q state cost matrix of shape SxS
	 * @param r control cost amtrix of shape CxC
	 * @param epsilon small value to avoid division by 0; 1e-6 works nicely
	 * @return optimal feedback gain matrix `k` of shape CxS
	 */
	public fun computeGain(
		a: D2Array<T>,
		b: D2Array<T>,
		q: D2Array<T>,
		r: D2Array<T>,
		epsilon: T
	): D2Array<T> {
		val g = b * LinAlgEx.inv(r) * b.tranpose();
		val h = Multik.d2array<T>(this.s, this.s) { rng() };

		while (true) {
			val error = LinAlgEx.norm(q - h) / LinAlgEx.norm(q);
			q = h;
			if (error < epsilon) break;

			val temp = LinAlgEx.inv(Multik.identity<T>(this.s) * g * h);
			val a1 = a * temp * a;

			g = h * a.tranpose() * h * temp * a;
			a = a1;
		}

		this.k = LinAlgEx.inv(r) * b.tranpose() * h;
		return this.k;
	}

	/**
	 * Returns the optimal feedback control based on the desired and current state vectors.
	 * This should  be called only after compute_gain() has already been called.
	 * @param currentState vector of length S
	 * @param desiredState vector of length S which we want to get to
	 * @return The feedback control gains. These are insufficient to control anything and have to be
	 * combined with the feedforward controls. Check examples.
	 */
	public fun computeOptimalControls(
		currentState: D2Array<T>,
		desiredState: D2Array<T>,
		yEpsilon: T
	): D2Array<T> {
		val error = desiredState - currentState;
		val yError = error[1];

		if (yError < yEpsilon) {
			this.integralError = T(0);
		} else {
			this.integralError += yError * this.ki;
		}

		val controls = k * error;
		controls[0] -= self.integralError;
		return controls;
	}
}

/**
 * Finds the two closest indices to the current pose along with their respective Euclidean errors.
 * The idea is that the lower index is behind the car and the upper index is in front of the car.
 * The opposite is true when driving backwards which is controlled via the `forwards` parameter.
 * @return (lowerIndex, lowerError), (upperIndex, upperError)
 */
public fun findClosestIndices<T>(
	currentState: D2Array<T>,
	trajectory: D2Array<T>
): Pair<Pair<Integer, T>, Pair<Integer, T>> {
	// TODO: Throw error if column-count of `trajectory` is less than 2.
	var state = currentState.copy();

	val errors = trajectory - currentState;
		
}
