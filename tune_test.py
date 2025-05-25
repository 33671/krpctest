from heater_model import HeatingModel
from pid import PidController
import numpy as np
import matplotlib.pyplot as plt
from auto_tune import PidAutoTuner

def model_factory_for_tuner(initial_temp=25.0):
    # Create and configure a new model instance
    model = HeatingModel(initial_temp=initial_temp)
    model.enable_disturbance()  # Or disable for cleaner tuning if preferred
    model.set_heat_capacity(2000.0) # Use your model's parameters
    model.set_heat_transfer_coeff(2)
    # Any other setup your model needs
    return model


if __name__ == "__main__":
    INITIAL_SYSTEM_TEMP = 25.0
    TARGET_SETPOINT = 70.0
    HEATER_POWER_LEVEL = 500.0 # The power level your model.set_heating() uses
    SIMULATION_DT_VALUE = 15.0   # The dt used in your simulation loop

    # Instantiate the tuner
    tuner = PidAutoTuner(
        model_factory=model_factory_for_tuner,
        initial_temp=INITIAL_SYSTEM_TEMP,
        setpoint=TARGET_SETPOINT,
        kp_start=1.0,       # Adjust Kp search parameters as needed
        kp_increment=0.5,  # Smaller increments give finer results but take longer
        kp_max=30.0,        # Max Kp to prevent excessive search time / instability
        sim_duration_per_kp=600.0, # Longer duration helps stabilize oscillations
        analysis_window_ratio=0.7, # Analyze last 70% of data
        min_oscillations_for_detection=3,
        amplitude_stability_ratio_threshold=0.25, # Stricter: e.g. 25% relative std dev
        overshoot_margin_factor=1.5, # How much overshoot is "divergence"
        undershoot_margin_factor=0.75,
        heater_power=HEATER_POWER_LEVEL,
        simulation_dt=SIMULATION_DT_VALUE,
        verbose=True  # Set to True to see detailed output
    )

    print("Starting auto-tuning process...")
    tuned_params = tuner.tune()

    print("\n--- Auto-Tuning Complete ---")
    if "error" in tuned_params:
        print(f"Tuning failed: {tuned_params['error']}")
        if "kp_tried" in tuned_params:
            print(f"Last Kp tried: {tuned_params['kp_tried']}")
    else:
        print("Successfully tuned PID parameters:")
        print(f"  Ku: {tuned_params['ku']:.4f}, Pu: {tuned_params['pu']:.3f}s")
        print(f"  Kp: {tuned_params['kp']:.4f}")
        print(f"  Ki: {tuned_params['ki']:.4f} (Ti: {tuned_params['ti']:.3f}s)")
        print(f"  Kd: {tuned_params['kd']:.4f} (Td: {tuned_params['td']:.3f}s)")

        # You can now use these parameters in your main simulation:
        # pid_controller_tuned = PidController(
        #     kp=tuned_params['kp'],
        #     ki=tuned_params['ki'],
        #     kd=tuned_params['kd'],
        #     setpoint=TARGET_SETPOINT
        # )
        # ... then run your simulation with pid_controller_tuned ...