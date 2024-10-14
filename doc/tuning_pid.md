The safe limits for the PID controller's constants—Kp, Ki, and Kd—can vary depending on your drone's dynamics, sensor noise, and specific tuning needs. There are no fixed values that are universally safe, but you can follow these general guidelines to prevent instability during flight:

1. Proportional Gain (Kp):
 * Effect: Controls how aggressively the system reacts to error. Increasing Kp makes the system correct errors more rapidly.
 * Safe Range: Start small, typically between 0.1 to 1.
   * Too high: The drone may become unstable, oscillate, or even crash as it overcompensates.
   * Too low: The drone will respond sluggishly, and it may not track the desired setpoint effectively.

2. Integral Gain (Ki):
 * Effect: Corrects accumulated past errors (e.g., steady-state error). It helps the drone reach the target value over time by addressing persistent small errors.
 * Safe Range: Typically between 0.001 to 0.1.
   * Too high: It can cause the system to wind up, overcorrect, and overshoot, leading to oscillations.
   * Too low: It won’t effectively eliminate steady-state errors, so the drone may fail to maintain a stable hover or course.

3. Derivative Gain (Kd):
 * Effect: Reacts to the rate of change of the error, smoothing out the response and reducing overshoot. It dampens oscillations caused by Kp.
 * Safe Range: Typically between 0.01 to 0.5.
   * Too high: It can make the drone overly sensitive to noise, leading to erratic corrections.
   * Too low: The drone may oscillate as it overshoots the target value.

General PID Tuning Process:
 * Start with Kp: Increase Kp until the system oscillates, then reduce it to about 50-70% of the oscillatory value.
 * Tune Ki: Slowly increase Ki until the steady-state error is corrected without significant overshoot.
 * Add Kd: Finally, add Kd to dampen oscillations and smooth the response.

Practical Considerations:
 * Start Small: Begin with low values for all three parameters and gradually increase them.
 * Iterate Slowly: Make small incremental changes and test the response of the drone after each adjustment.
 * Simulation First: If possible, use simulation tools to test different values before applying them on the actual hardware.

