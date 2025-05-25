import numpy as np
import math
from pid import PidController
class PidAutoTuner:
    """
    PidAutoTuner class using the Ziegler-Nichols second method (closed-loop oscillation)
    to tune PID parameters for a system controlled by a PID whose output determines
    the ON-duration of a fixed-power heater.
    """
    def __init__(self, model_factory, initial_temp, setpoint,
                 kp_start=0.1, kp_increment=0.1, kp_max=50.0,
                 sim_duration_per_kp=400.0,
                 analysis_window_ratio=0.6, 
                 min_oscillations_for_detection=3,
                 amplitude_stability_ratio_threshold=0.3, # Max relative std dev of amplitudes
                 overshoot_margin_factor=2.0,
                 undershoot_margin_factor=1.0,
                 heater_power=200.0, # The fixed power of the heater when ON
                 simulation_dt=1.0, # Time step for PID computation and model advancement
                 verbose=False):
        """
        Initializes the PidAutoTuner.

        Args:
            model_factory (callable): A function that takes 'initial_temp' as an argument
                                      and returns a fresh instance of the HeatingModel.
                                      e.g., lambda temp: HeatingModel(initial_temp=temp, ...)
            initial_temp (float): The starting temperature of the system for tuning runs.
            setpoint (float): The target temperature for the PID controller.
            kp_start (float): Initial Kp value to start the search for Ku.
            kp_increment (float): Step size to increment Kp during the search.
            kp_max (float): Maximum Kp value to try.
            sim_duration_per_kp (float): Duration (in seconds) to simulate the system for each Kp trial.
            analysis_window_ratio (float): Fraction of the end of simulation data to use for oscillation analysis (0.0 to 1.0).
            min_oscillations_for_detection (int): Minimum number of full oscillations required in the analysis window.
            amplitude_stability_ratio_threshold (float): Max relative standard deviation for peak/trough amplitudes
                                                        (e.g., 0.3 means std_dev/mean < 0.3). Lower is stricter.
            overshoot_margin_factor (float): Factor to determine max allowed overshoot.
                                             max_temp_allowed = setpoint + (setpoint - initial_temp) * factor.
            undershoot_margin_factor (float): Factor to determine max allowed undershoot from initial_temp.
                                              min_temp_allowed = initial_temp - (setpoint - initial_temp) * factor.
            heater_power (float): The power level applied by model.set_heating().
            simulation_dt (float): The time step (dt) used for PID computations and model advancement.
            verbose (bool): If True, prints detailed progress and diagnostics.
        """
        self.model_factory = model_factory
        self.initial_temp = initial_temp
        self.setpoint = setpoint
        self.kp_start = kp_start
        self.kp_increment = kp_increment
        self.kp_max = kp_max
        self.sim_duration_per_kp = sim_duration_per_kp
        self.analysis_window_ratio = analysis_window_ratio
        self.min_oscillations_for_detection = min_oscillations_for_detection
        self.amplitude_stability_ratio_threshold = amplitude_stability_ratio_threshold
        self.overshoot_margin_factor = overshoot_margin_factor
        self.undershoot_margin_factor = undershoot_margin_factor
        self.heater_power = heater_power
        self.dt = simulation_dt
        self.verbose = verbose

    def _run_simulation_for_kp_trial(self, kp_test_value):
        """
        Runs a simulation with a P-only controller using the given kp_test_value.
        This method simulates the specific control logic where PID output sets heater ON-duration.
        """
        model = self.model_factory(initial_temp=self.initial_temp)
        # Ensure the provided PidController class is used for P-only control
        # This assumes PidController can be instantiated for P-only.
        # If your PidController class needs a reset method, call it.
        pid_p_only = PidController(kp=kp_test_value, ki=0, kd=0, setpoint=self.setpoint)
        if hasattr(pid_p_only, 'reset'):
            pid_p_only.reset()

        time_points = []
        temp_points = []
        
        num_steps = int(self.sim_duration_per_kp / self.dt)
        current_waiting_time_steps = 0 # In terms of dt steps

        for step_num in range(num_steps):
            current_temp = model.get_current_temp()
            current_time = model.get_current_time() # Or step_num * self.dt if model doesn't track time

            time_points.append(current_time)
            temp_points.append(current_temp)

            if current_waiting_time_steps <= 0:
                current_waiting_time_steps = 0
                model.stop_heating()

            # PID computation occurs at each dt interval
            # The PidController's compute method uses kp_test_value
            pid_output_value = pid_p_only.compute(current_temp, self.dt)

            if pid_output_value > 0:
                model.set_heating(self.heater_power) # Use configured heater_power
                # The PID output is interpreted as a duration in seconds.
                # Convert this duration to number of dt steps.
                current_waiting_time_steps = math.ceil(pid_output_value / self.dt)
            else:
                model.stop_heating()
                current_waiting_time_steps = 0 
            
            model.advance_time(self.dt) # Advance model time by dt
            if current_waiting_time_steps > 0:
                current_waiting_time_steps -= 1 # Decrement by 1 step (since it's in terms of dt steps)
        
        return time_points, temp_points

    def _analyze_oscillation_characteristics(self, time_points, temp_points, current_kp):
        """
        Analyzes temperature data to detect sustained oscillations and estimate Pu.
        Returns (status, Pu_candidate)
        status can be "oscillating", "diverged", "settled", "inconclusive"
        """
        if len(temp_points) < 2 / self.dt * self.min_oscillations_for_detection : # Need enough data for min oscillations
             if self.verbose: print(f"KP={current_kp:.3f}: Not enough data points for analysis ({len(temp_points)}).")
             return "inconclusive", None

        analysis_start_index = int(len(temp_points) * (1.0 - self.analysis_window_ratio))
        analysis_temps = np.array(temp_points[analysis_start_index:])
        analysis_times = np.array(time_points[analysis_start_index:])

        if len(analysis_temps) < 20: # Arbitrary minimum for robust analysis
            if self.verbose: print(f"KP={current_kp:.3f}: Analysis window too small ({len(analysis_temps)} points).")
            return "inconclusive", None

        # 1. Check for divergence (runaway temperatures)
        max_temp = np.max(analysis_temps)
        min_temp = np.min(analysis_temps)
        
        # Define dynamic bounds based on initial temperature and setpoint
        temp_range = abs(self.setpoint - self.initial_temp)
        if temp_range < 1e-3: temp_range = 1.0 # Avoid zero range if initial is at setpoint

        upper_bound = self.setpoint + temp_range * self.overshoot_margin_factor
        # For undershoot, prevent it from going too far below initial_temp or an absolute minimum (e.g., 0 C)
        lower_bound_relative = self.initial_temp - temp_range * self.undershoot_margin_factor
        lower_bound = max(0.0, lower_bound_relative) # Assuming temperatures shouldn't be negative

        if max_temp > upper_bound or min_temp < lower_bound:
            if self.verbose: print(f"KP={current_kp:.3f}: System diverged or exceeded safety margins. MaxT={max_temp:.2f} (Limit: {upper_bound:.2f}), MinT={min_temp:.2f} (Limit: {lower_bound:.2f})")
            return "diverged", None

        # 2. Check for settling (temperature became too stable)
        if np.std(analysis_temps) < 0.1 * temp_range * 0.1: # If std dev is very small (e.g. <1% of range)
            if self.verbose: print(f"KP={current_kp:.3f}: System appears settled (std dev: {np.std(analysis_temps):.3f}).")
            return "settled", None

        # 3. Find peaks and troughs (simple method, consider scipy.signal.find_peaks if available)
        # This requires a more robust peak detection for real-world signals.
        # For this simulation, a simpler one might work.
        # We are looking for oscillations around the setpoint.
        try:
            from scipy.signal import find_peaks
            # Peaks above setpoint, troughs below (by looking for inverted signal peaks)
            # Prominence can help filter noise. Distance ensures peaks are separated.
            # Heuristic prominence: 5-10% of (setpoint - initial_temp)
            prominence_threshold = 0.05 * temp_range 
            min_peak_dist_samples = int((0.5 * (self.sim_duration_per_kp / self.min_oscillations_for_detection)) / self.dt if self.min_oscillations_for_detection >0 else 5/self.dt)


            peak_indices, _ = find_peaks(analysis_temps, height=self.setpoint, prominence=prominence_threshold, distance=min_peak_dist_samples)
            trough_indices, _ = find_peaks(-analysis_temps + 2*self.setpoint, height=self.setpoint, prominence=prominence_threshold, distance=min_peak_dist_samples) # Inverted signal
        except ImportError:
            if self.verbose: print("SciPy not found, using basic peak/trough detection (less robust).")
            peak_indices = []
            trough_indices = []
            # Basic detection (less robust):
            for i in range(1, len(analysis_temps) - 1):
                # Peak if significantly above setpoint
                if analysis_temps[i-1] < analysis_temps[i] > analysis_temps[i+1] and analysis_temps[i] > self.setpoint + prominence_threshold:
                    peak_indices.append(i)
                # Trough if significantly below setpoint
                if analysis_temps[i-1] > analysis_temps[i] < analysis_temps[i+1] and analysis_temps[i] < self.setpoint - prominence_threshold:
                    trough_indices.append(i)
            peak_indices = np.array(peak_indices) # Ensure numpy array for consistency
            trough_indices = np.array(trough_indices)


        if len(peak_indices) < self.min_oscillations_for_detection or len(trough_indices) < self.min_oscillations_for_detection:
            if self.verbose: print(f"KP={current_kp:.3f}: Not enough significant oscillations detected. Peaks: {len(peak_indices)}, Troughs: {len(trough_indices)} (min_req: {self.min_oscillations_for_detection}).")
            return "inconclusive", None
        
        # 4. Check amplitude stability of the last few oscillations
        # Consider amplitudes relative to setpoint
        last_n_peaks = analysis_temps[peak_indices[-self.min_oscillations_for_detection:]]
        last_n_troughs = analysis_temps[trough_indices[-self.min_oscillations_for_detection:]]

        peak_amplitudes = last_n_peaks - self.setpoint
        trough_depths = self.setpoint - last_n_troughs
        
        if np.any(peak_amplitudes <= 1e-3) or np.any(trough_depths <= 1e-3): # Amplitudes too small
             if self.verbose: print(f"KP={current_kp:.3f}: Oscillation amplitudes too small to be reliable.")
             return "inconclusive", None

        rel_std_peaks = np.std(peak_amplitudes) / np.mean(peak_amplitudes) if np.mean(peak_amplitudes) > 1e-3 else float('inf')
        rel_std_troughs = np.std(trough_depths) / np.mean(trough_depths) if np.mean(trough_depths) > 1e-3 else float('inf')

        if rel_std_peaks > self.amplitude_stability_ratio_threshold or rel_std_troughs > self.amplitude_stability_ratio_threshold:
            if self.verbose: print(f"KP={current_kp:.3f}: Oscillation amplitudes not stable enough. RelStdPeaks: {rel_std_peaks:.3f}, RelStdTroughs: {rel_std_troughs:.3f} (Threshold: {self.amplitude_stability_ratio_threshold:.3f})")
            return "inconclusive", None

        # 5. Calculate Ultimate Period (Pu)
        # Average time difference between consecutive peaks and troughs
        peak_times = analysis_times[peak_indices[-self.min_oscillations_for_detection:]]
        trough_times = analysis_times[trough_indices[-self.min_oscillations_for_detection:]]
        
        periods = []
        if len(peak_times) >= 2:
            periods.extend(np.diff(peak_times))
        if len(trough_times) >= 2:
            periods.extend(np.diff(trough_times))
        
        if not periods or np.mean(periods) < self.dt * 2: # Period must be at least 2*dt
            if self.verbose: print(f"KP={current_kp:.3f}: Could not reliably determine period or period too short. Periods found: {periods}")
            return "inconclusive", None
            
        pu_candidate = np.mean(periods)
        if self.verbose: print(f"KP={current_kp:.3f}: Sustained oscillations detected. Pu candidate: {pu_candidate:.3f}s")
        return "oscillating", pu_candidate


    def tune(self):
        """
        Performs the Ziegler-Nichols tuning process.
        Returns a dictionary with tuned Kp, Ki, Kd parameters, or an error message.
        """
        kp_current = self.kp_start
        ku = None
        pu = None
        
        if self.verbose:
            print("--- Starting PID Auto-Tuning (Ziegler-Nichols Method) ---")
            print(f"Setpoint: {self.setpoint}°C, Initial Temp: {self.initial_temp}°C")
            print(f"Kp search range: {self.kp_start} to {self.kp_max}, increment: {self.kp_increment}")
            print(f"Simulation per Kp: {self.sim_duration_per_kp}s, Analysis window: last {self.analysis_window_ratio*100:.0f}%")

        last_kp_before_divergence = None
        last_pu_before_divergence = None

        while kp_current <= self.kp_max:
            if self.verbose: print(f"\nTesting Kp = {kp_current:.4f}")
            
            time_points, temp_points = self._run_simulation_for_kp_trial(kp_current)
            status, pu_candidate = self._analyze_oscillation_characteristics(time_points, temp_points, kp_current)

            if status == "oscillating":
                if self.verbose: print(f"Sustained oscillation found at Kp={kp_current:.4f} with Pu={pu_candidate:.3f}s. This is Ku and Pu.")
                ku = kp_current
                pu = pu_candidate
                break # Found Ku and Pu, exit loop
            
            elif status == "diverged":
                if self.verbose: print(f"Kp={kp_current:.4f} caused divergence.")
                # If we had a previous Kp that was oscillating or inconclusive but not settled,
                # that might be closer to Ku. This logic is tricky.
                # For now, if it diverges, we stop and use the last "good" oscillation if any.
                if last_kp_before_divergence and last_pu_before_divergence:
                    ku = last_kp_before_divergence
                    pu = last_pu_before_divergence
                    if self.verbose: print(f"Using last Kp before divergence: Ku={ku:.4f}, Pu={pu:.3f}")
                else:
                     if self.verbose: print("System diverged before any stable oscillation was found. Tuning failed.")
                     return {"error": "System diverged before stable oscillation", "kp_tried": kp_current}
                break 
            
            elif status == "settled":
                if self.verbose: print(f"Kp={kp_current:.4f} resulted in a settled system. Increasing Kp.")
                last_kp_before_divergence = None # Reset as this Kp is too low
                last_pu_before_divergence = None
            
            elif status == "inconclusive":
                if self.verbose: print(f"Kp={kp_current:.4f} was inconclusive. Increasing Kp.")
                # Store this as a potential candidate if next Kp diverges
                # This is a heuristic: if next step diverges, current Kp might be Ku
                # However, _analyze_oscillation_characteristics should ideally find Ku directly
                # For now, let's not rely on this too much, but keep it as a weak fallback.
                # last_kp_before_divergence = kp_current 
                # last_pu_before_divergence = None # We don't have a Pu for inconclusive
            
            kp_current += self.kp_increment
            if kp_current > self.kp_max and ku is None: # Check if loop will exceed max_kp
                 if self.verbose: print(f"Reached Kp_max ({self.kp_max}) without definitive sustained oscillations.")
                 if last_kp_before_divergence and last_pu_before_divergence: # Fallback if loop ended after some oscillations
                     ku = last_kp_before_divergence
                     pu = last_pu_before_divergence
                     if self.verbose: print(f"Using last Kp before Kp_max: Ku={ku:.4f}, Pu={pu:.3f}")
                 else:
                     return {"error": "Reached Kp_max without finding Ku and Pu."}
                 break # Exit loop if Kp_max reached

        if ku is None or pu is None or pu <= 1e-3 : # Check for valid Pu
            if self.verbose: print("Tuning failed to find valid Ku and Pu.")
            return {"error": "Failed to find valid Ku and Pu after search."}

        # Calculate PID parameters using Ziegler-Nichols rules for a "Classic PID"
        # Kp_pid = 0.6 * Ku
        # Ti = Pu / 2.0  => Ki_pid = Kp_pid / Ti = Kp_pid / (0.5 * Pu)
        # Td = Pu / 8.0  => Kd_pid = Kp_pid * Td = Kp_pid * (0.125 * Pu)
        
        tuned_kp = 0.6 * ku
        ti = 0.5 * pu
        td = 0.125 * pu

        tuned_ki = tuned_kp / ti if ti > 1e-6 else 0 # Avoid division by zero
        tuned_kd = tuned_kp * td

        results = {
            "kp": tuned_kp, "ki": tuned_ki, "kd": tuned_kd,
            "ku": ku, "pu": pu,
            "ti": ti, "td": td,
            "rule": "Ziegler-Nichols Classic PID"
        }

        if self.verbose:
            print("\n--- Ziegler-Nichols Tuning Successful ---")
            print(f"Ultimate Gain (Ku): {ku:.4f}")
            print(f"Ultimate Period (Pu): {pu:.3f} s")
            print("Calculated PID Parameters (Classic ZN for PID):")
            print(f"  Kp: {results['kp']:.4f}")
            print(f"  Ki: {results['ki']:.4f} (Ti: {results['ti']:.3f} s)")
            print(f"  Kd: {results['kd']:.4f} (Td: {results['td']:.3f} s)")

        return results

