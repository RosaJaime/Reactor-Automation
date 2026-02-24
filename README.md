# Reactor Automation HMI (ODrive Agitator)

A touch-friendly desktop app for running a reactor agitator motor with an ODrive controller.

It provides:
- Manual velocity + timer runs
- Multi-step recipe runs (speed + duration per step)
- Previous recipe run history with run IDs
- ODrive configuration screen for common settings
- Data logger with live plots and CSV export
- PDF report export for recipe runs (ReportLab)
- Auto reconnect when the ODrive disconnects/reboots
- Motor calibration workflow (required before motor controls are enabled)

## What This Program Does

The main app (`odrive_hmi.pyw`) is a PySide6 GUI that controls `axis0` on an ODrive.

Current motor control mode in the HMI:
- Closed-loop velocity control
- The app sends velocity commands using `axis.controller.input_vel`
- The app requests `CLOSED_LOOP_CONTROL` when running
- The app returns the axis to `IDLE` when stopped

Before starting a run, the app checks whether the motor/encoder are calibrated. If not:
- Start/Stop and motor velocity controls are disabled
- A `Calibrate Motor` button is shown on the Home screen
- Trying to start shows a popup telling the user to calibrate first

## Main Screens

- `Home`
  - Set manual velocity (RPM)
  - Set timer for a single run
  - Start/Stop
  - Select and run recipes
  - Calibrate motor
- `Recipe Builder`
  - Create/edit named recipes with multiple steps
  - Each step has velocity (RPM) and duration
- `ODrive Configuration`
  - Read/apply a curated list of common ODrive settings
  - Export `get_json()` to file
- `Data Logger`
  - Live trend plots (voltage/current/power/commanded RPM/estimated RPM)
  - CSV export

## Screenshots

<p>
  <img src="Images/Main%20Screen.png" alt="Main screen" width="49%" />
  <img src="Images/Recipe%20Builder.png" alt="Recipe builder screen" width="49%" />
</p>
<p>
  <img src="Images/Recipe%20Run%20View.png" alt="Recipe run monitor view" width="100%" />
</p>

## Requirements

- Python 3.10+ (recommended)
- ODrive Python package
- PySide6
- ReportLab (for PDF reports)

Install dependencies:

```powershell
pip install -r requirements.txt
```

`requirements.txt` currently includes:
- `PySide6`
- `odrive`
- `reportlab`

Note:
- If `PySide6.QtCharts` is available, the app uses QtCharts for plotting.
- If not, it falls back to a built-in simple plot widget.

## Quick Start

1. Connect the ODrive and motor hardware safely.
1. Install Python dependencies.
1. Run the app:

```powershell
python odrive_hmi.pyw
```

1. Wait for the app to connect (it auto-retries).
1. On first use (or after a reset/reconfig), click `Calibrate Motor`.
1. After calibration completes, use manual mode or a recipe and press `Start`.

## Simulation Mode (No Hardware)

The app supports a simulated backend for UI testing.

In `odrive_hmi.pyw`, change:

```python
BACKEND = "real"  # "sim" or "real"
```

to:

```python
BACKEND = "sim"
```

Then run the app normally.

## Run Modes

### Single Run (Manual)

- Set `Velocity` in RPM
- Set `Timer`
- Press `Start`

The app will:
- Run at the requested velocity
- Count down remaining time
- Stop automatically when time expires

### Recipe Run

- Build or select a recipe
- Press `Start`

A recipe is a sequence of steps:
- `velocity_rpm`
- `duration_s`

The app transitions through each step automatically and stops at the end.

## Calibration Workflow (Important)

The HMI now expects a calibrated motor before enabling motor controls.

What the app does when you press `Calibrate Motor`:
- Sends the axis to `IDLE`
- Requests ODrive `FULL_CALIBRATION_SEQUENCE`
- Waits for the axis to return to `IDLE`
- Checks calibration state

If calibration succeeds:
- Motor controls are enabled
- A success popup is shown

If calibration fails or times out:
- Motor controls stay disabled
- A popup explains the failure

## Files and What They Are For

- `odrive_hmi.pyw`
  - Main application (GUI, run engine, worker thread, ODrive I/O)
- `odrive_config.py`
  - Helper script/snippet for configuring ODrive velocity-control settings (typically used in `odrivetool`)
- `manually_run_odrive_no_encoder.py`
  - Legacy/manual open-loop `LOCKIN_SPIN` test script (separate from the HMI)

## Saved Data / Output Locations

The app stores local state and logs in standard OS locations.

App config directory (recipes, UI state, history DB):
- `QStandardPaths.AppConfigLocation/PolyJouleOdriveHMI`

Typical contents:
- `recipes.json`
- `ui_state.json`
- `history.sqlite`

Exports are written to:
- Downloads folder (or `~/Output` fallback)

Examples:
- ODrive JSON export: `odrive_get_json_YYYYMMDD_HHMMSS.json`
- Data logger CSV export: `datalogger_<minutes>min_YYYYMMDD_HHMMSS.csv`
- Recipe run PDF report: `recipe_run_report_<runid>_YYYYMMDD_HHMMSS.pdf`

## Safety Notes

- Start with the motor unloaded or mechanically safe.
- Verify current limits and velocity limits in ODrive config before running.
- Keep an emergency stop / power disconnect available during testing.
- Do not rely on software-only stop behavior for personnel safety.

## Troubleshooting

### App says ODrive is not connected

- Check USB/power to ODrive
- Confirm `odrive` Python package is installed
- Wait a moment: the app auto-reconnects periodically

### Start button is disabled

Common causes:
- ODrive is disconnected
- Calibration is in progress
- Motor is not calibrated yet (use `Calibrate Motor`)

### Calibration fails

- Verify motor and encoder wiring
- Confirm axis configuration matches your hardware
- Check ODrive errors in `odrivetool`
- Try applying configuration (`odrive_config.py` / HMI config screen), reboot ODrive, reconnect, and calibrate again

## Development Notes

- All ODrive reads/writes run in a worker thread to keep the UI responsive.
- The run engine (timing/recipes) is separate from hardware I/O.
- The app auto-stops the run engine if the ODrive disconnects during a run.
