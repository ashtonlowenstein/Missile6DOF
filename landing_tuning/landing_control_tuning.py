import pandas as pd
import numpy as np
from pathlib import Path

def score_run(path):
    df = pd.read_csv(path)

    idx = df["z"].abs().idxmin()
    row = df.loc[idx]

    xy_err = np.hypot(row["x"], row["y"])
    speed_err = np.linalg.norm([row["vx"], row["vy"], row["vz"]])

    min_z = df["z"].min()
    max_tilt = df.get("tilt", pd.Series([0])).max()
    max_gimbal = max(
        df["gimbal_pitch_cmd"].abs().max(),
        df["gimbal_yaw_cmd"].abs().max()
    )

    crash_penalty = 1000.0 if min_z < -1.0 else 0.0

    score = (
            100.0 * abs(row["z"]) +
            50.0  * abs(row["vz"]) +
            20.0  * xy_err +
            5.0   * max_gimbal +
            crash_penalty
    )

    return {
        "file": path.name,
        "t_best": row["t"],
        "z_best": row["z"],
        "vz_best": row["vz"],
        "xy_err": xy_err,
        "speed_err": speed_err,
        "max_tilt": max_tilt,
        "max_gimbal": max_gimbal,
        "score": score,
    }

RUN_DIR = Path.cwd()
csv_files = sorted(RUN_DIR.rglob("*.csv"))
rows = [score_run(p) for p in csv_files]
summary = pd.DataFrame(rows).sort_values("score")

summary.to_csv(Path(str(RUN_DIR) + "/landing_gain_summary_1.csv"), index=False)
print(summary.head(10))