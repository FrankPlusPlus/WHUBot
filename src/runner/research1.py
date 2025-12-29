"""Generate a Cartesian trajectory, solve inverse kinematics, and export CSV.

This module provides the `TrajectoryGenerator` class which reads a YAML configuration,
computes a trajectory, solves for joint angles, and writes a CSV file.
run: python src/runner/research1.py --config configs/research1_config.yaml

"""

from __future__ import annotations
import argparse
import yaml
import numpy as np
import pandas as pd
from scipy.optimize import least_squares
import math
import os
import logging
import sys

# plotting (optional)
try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None
    logging.debug('matplotlib not available; plot saving will be disabled')
# allow importing from src/utils when running as a script
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils.robot_utils import Robot, parse_robot_xml

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')


def tpoly_segment(p0: np.ndarray, pf: np.ndarray, N: int) -> np.ndarray:
    """Compute quintic interpolation between p0 and pf.

    Parameters
    ----------
    p0 : array_like
        Start point.
    pf : array_like
        End point.
    N : int
        Number of samples (including endpoints).

    Returns
    -------
    np.ndarray
        Interpolated points with shape (N, len(p0)).
    """
    t = np.linspace(0, 1, N)
    s = 6 * t**5 - 15 * t**4 + 10 * t**3  # quintic blending
    return (p0[np.newaxis, :] + (pf - p0)[np.newaxis, :] * s[:, np.newaxis])


def mtraj_tpoly(via: np.ndarray, samples_per_segment: int) -> np.ndarray:
    """Build a closed trajectory by concatenating tpoly segments through via points.

    Parameters
    ----------
    via : ndarray
        Via points array of shape (n, 2).
    samples_per_segment : int
        Number of samples per segment.

    Returns
    -------
    ndarray
        Trajectory points stacked vertically.
    """
    assert via.shape[0] >= 2
    segments = []
    segments.append(tpoly_segment(via[4], via[0], samples_per_segment))
    for i in range(1, 5):
        segments.append(tpoly_segment(via[i-1], via[i], samples_per_segment))
    return np.vstack(segments)  # Robot implementation moved to `src/utils/robot_utils.py` and imported from there
# Import example (module is on sys.path):
#   from utils.robot_utils import Robot, parse_robot_xml


class TrajectoryGenerator:
    """Generate Cartesian trajectories and compute inverse kinematics.

    Parameters
    ----------
    cfg_path : str
        Path to the YAML configuration file.

    Methods
    -------
    generate_trajectory()
        Compute the trajectory, solve IK for each sample, and write a CSV file.
    """
    def __init__(self, cfg_path: str):
        with open(cfg_path, 'r') as f:
            cfg = yaml.safe_load(f)
        self.via = np.array(cfg['via_points'], dtype=float)
        self.dt = float(cfg.get('dt', 0.05))
        self.samples_per_segment = int(cfg.get('samples_per_segment', 40))

        # Required config keys; raise an error if missing so the user can fix the configuration
        if 'q0_deg' not in cfg:
            raise KeyError("Missing required config key: 'q0_deg'")
        self.q0_deg = np.array(cfg['q0_deg'], dtype=float)

        if 'end_effector_z' not in cfg:
            raise KeyError("Missing required config key: 'end_effector_z'")
        self.end_effector_z = float(cfg['end_effector_z'])

        if 'output_path' not in cfg:
            raise KeyError("Missing required config key: 'output_path'")
        self.output_path = cfg['output_path']

        ik_cfg = cfg.get('ik', {})
        # IK parameters: prefer values from config; otherwise use sensible defaults
        self.ik_params = {
            'max_nfev': int(ik_cfg.get('max_nfev', 200)),
            'ftol': float(ik_cfg.get('ftol', 1e-8)),
            'xtol': float(ik_cfg.get('xtol', 1e-8)),
            'reg_weight': float(ik_cfg.get('reg_weight', 0.02)),
        }
        # init robot from config or (optionally) from a robot XML file
        robot_cfg = cfg.get('robot', {})
        # optional: allow reading robot DH parameters from an exported robot XML
        if bool(cfg.get('robot_from_xml', False)):
            xml_path = cfg.get('robot_xml_path', None)
            if xml_path is None:
                raise KeyError("robot_from_xml=True requires 'robot_xml_path' in config")
            rd, ra, ralpha, jv_initial = parse_robot_xml(xml_path)
            logging.info(f"Loaded robot DH parameters and JVInitial from XML {xml_path}: d={rd}, a={ra}, alpha={ralpha}, jv_initial={jv_initial}")
            # Create robot instance and store JVInitial as its attribute
            self.robot = Robot(d=rd, a=ra, alpha=ralpha, jv_initial=jv_initial)
            # If JVInitial is available, use it as q0_deg
            if jv_initial is not None and jv_initial.shape == (6,):
                logging.info(f"Using JVInitial from XML as q0_deg: {jv_initial}")
                self.q0_deg = jv_initial
            else:
                logging.warning('JVInitial from XML missing or not length 6; falling back to config q0_deg')
        else:
            # Configuration key renaming: prefer 'b' (link offsets) but fall back to 'd' for compatibility
            rb = robot_cfg.get('b', robot_cfg.get('d', None))
            ra = robot_cfg.get('a', None)
            ralpha = robot_cfg.get('alpha', None)
            if 'd' in robot_cfg and 'b' not in robot_cfg:
                logging.warning("Using deprecated config key 'd'; prefer 'b' for link offsets (will map to Robot.d)")
            self.robot = Robot(d=rb, a=ra, alpha=ralpha)
        self.random_seed = cfg.get('random_seed', None)
        if self.random_seed is not None:
            np.random.seed(self.random_seed)

        # plotting configuration: optional mapping 'plots' with keys 'displacement','speed','acceleration'
        plots_cfg = cfg.get('plots', {}) or {}
        default_plot_dir = os.path.dirname(self.output_path) or os.path.join('data','output')
        # ensure default dir exists
        if default_plot_dir and not os.path.exists(default_plot_dir):
            os.makedirs(default_plot_dir, exist_ok=True)
        self.plot_paths = {
            'displacement': plots_cfg.get('displacement', os.path.join(default_plot_dir, 'displacement.png')),
            'speed': plots_cfg.get('speed', os.path.join(default_plot_dir, 'speed.png')),
            'acceleration': plots_cfg.get('acceleration', os.path.join(default_plot_dir, 'acceleration.png')),
        }
        self.save_plots = bool(cfg.get('save_plots', True))

        # DH table export configuration
        # 'dh_table_path' : string path to save PNG table (optional)
        # 'save_dh_table' : bool to control saving (default True)
        self.dh_table_path = cfg.get('dh_table_path', os.path.join(default_plot_dir, 'dh_table.png'))
        self.save_dh_table = bool(cfg.get('save_dh_table', True))

        # If requested, save the DH parameter table (b, a, alpha, theta) as PNG
        def _save_dh_table(path: str):
            if not self.save_dh_table:
                logging.debug('DH table saving disabled by config')
                return
            if plt is None:
                logging.warning('matplotlib not available; cannot save DH table %s', path)
                return
            try:
                headers = ['b (m)', 'a (m)', 'alpha (rad)']
                b_vals = [f'{v:.6f}' for v in self.robot.d]
                a_vals = [f'{v:.6f}' for v in self.robot.a]
                alpha_vals = [f'{v:.6f}' for v in self.robot.alpha]
                cell_text = list(zip(b_vals, a_vals, alpha_vals))
                fig, ax = plt.subplots(figsize=(8, 2.6))
                ax.axis('off')
                # Build table with centered cells and styled header
                table = ax.table(cellText=cell_text, colLabels=headers, loc='center', cellLoc='center')
                # ensure reasonable column widths
                try:
                    table.auto_set_column_width(col=list(range(len(headers))))
                except Exception:
                    pass
                # style header row
                ncols = len(headers)
                nrows = len(cell_text)
                for col in range(ncols):
                    cell = table[0, col]
                    cell.set_text_props(weight='bold', color='white', ha='center', va='center')
                    cell.set_facecolor('#4f81bd')
                    cell.set_edgecolor('black')
                # style data cells
                for row in range(1, nrows+1):
                    for col in range(ncols):
                        c = table[row, col]
                        c.set_text_props(ha='center', va='center')
                        c.set_edgecolor('#dddddd')
                table.auto_set_font_size(False)
                table.set_fontsize(10)
                table.scale(1, 1.4)
                table.auto_set_font_size(False)
                table.set_fontsize(10)
                table.scale(1, 1.4)
                plt.tight_layout()
                out_dir = os.path.dirname(path)
                if out_dir and not os.path.exists(out_dir):
                    os.makedirs(out_dir, exist_ok=True)
                plt.savefig(path, dpi=150, bbox_inches='tight')
                plt.close()
                logging.info('Saved DH parameter table to %s', path)
            except Exception as e:
                logging.warning('Failed to save DH table %s: %s', path, e)

        # attempt save now (robot and q0 are available)
        try:
            _save_dh_table(self.dh_table_path)
        except Exception as e:
            logging.warning('Error while attempting to save DH table: %s', e)

        # Saved IK support removed: script now always computes IK from the configured via points
        # (previous config keys like use_saved_ik/save_ik/ik_save_path are no longer used)

    def generate_trajectory(self) -> dict:
        """Compute trajectory points, solve inverse kinematics, and export CSV.

        Returns
        -------
        dict
            Dictionary containing computed arrays and metadata (time, s, displacement,
            speed, acceleration, q_all_rad, q_all_deg, output_path).
        """
        s = mtraj_tpoly(self.via, self.samples_per_segment)  # shape (200,2)
        total_samples = s.shape[0]

        # Time vector aligned with samples (0:dt:10); final time_out aligns with trajectory samples
        time_full = np.arange(0, total_samples * self.dt + self.dt, self.dt)  # length samples+1
        time_out = time_full[1:total_samples+1]

        # Compute displacement, speed, and acceleration using finite differences
        s_x = s[:,0]
        s_y = s[:,1]
        ds = np.sqrt(np.diff(s_x)**2 + np.diff(s_y)**2)
        displacement = np.concatenate(([0.0], np.cumsum(ds)))
        speed = np.diff(displacement) / np.diff(time_out)
        acceleration = np.diff(speed) / np.diff(time_out[1:])

        # Inverse kinematics
        q_all = np.zeros((total_samples, 6), dtype=float)
        q_all[0,:] = np.deg2rad(self.q0_deg)

        # Helper: save a simple line plot if matplotlib available
        def _save_plot(x, y, path, title, xlabel, ylabel):
            if not self.save_plots:
                logging.debug('plot saving disabled by config')
                return
            if plt is None:
                logging.warning('matplotlib not available; cannot save plot %s', path)
                return
            try:
                plt.figure(figsize=(8,4))
                plt.plot(x, y, '-b')
                plt.grid(True)
                plt.xlabel(xlabel)
                plt.ylabel(ylabel)
                plt.title(title)
                plt.tight_layout()
                out_dir = os.path.dirname(path)
                if out_dir and not os.path.exists(out_dir):
                    os.makedirs(out_dir, exist_ok=True)
                plt.savefig(path, dpi=150)
                plt.close()
                logging.info('Saved plot to %s', path)
            except Exception as e:
                logging.warning('Failed to save plot %s: %s', path, e)

        # Compute IK for every sample (historical IK loading has been removed)
        for i in range(1, total_samples):
            target = np.array([s[i,0], s[i,1], self.end_effector_z])
            def rotation_matrix_to_angle_axis(R: np.ndarray) -> np.ndarray:
                    # R is 3x3 rotation matrix. returns vector = angle * axis (3,)
                    trace = np.trace(R)
                    cos_theta = (trace - 1.0) / 2.0
                    cos_theta = np.clip(cos_theta, -1.0, 1.0)
                    theta = math.acos(cos_theta)
                    if abs(theta) < 1e-12:
                        return np.zeros(3)
                    # axis
                    rx = R[2,1] - R[1,2]
                    ry = R[0,2] - R[2,0]
                    rz = R[1,0] - R[0,1]
                    axis = np.array([rx, ry, rz]) / (2.0 * math.sin(theta))
                    return axis * theta

            R_target = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, -1.0]])

            def residual_cartesian(q_vec):
                """Cartesian residual: position error + orientation (angle-axis of inv(R_cur)*R_target) + small continuity regularizer"""
                T = self.robot.fk(q_vec)
                pos = T[:3,3]
                R_current = T[:3,:3]
                R_err = R_current.T @ R_target
                ang_axis = rotation_matrix_to_angle_axis(R_err)
                w_reg = float(self.ik_params.get('reg_weight', 0.02))
                reg = w_reg * (q_vec - q_all[i-1,:])
                return np.hstack((pos - target, ang_axis, reg))

            try:
                res = least_squares(
                    residual_cartesian,
                    q_all[i-1,:],
                    method='lm',
                    max_nfev=int(self.ik_params.get('max_nfev', 2000)),
                    ftol=self.ik_params.get('ftol', 1e-12),
                    xtol=self.ik_params.get('xtol', 1e-12),
                )
                if res.success or res.status > 0:
                    q_all[i,:] = res.x
                    if i < 6:
                        logging.debug(f"IK step {i}: success=True residual_norm={np.linalg.norm(res.fun):.6g}")
                else:
                    logging.warning(f"IK did not converge at step {i}, using previous q (status={res.status})")
                    q_all[i,:] = q_all[i-1,:]
            except Exception as e:
                logging.warning(f"IK solver exception at step {i}: {e}; using previous q")
                q_all[i,:] = q_all[i-1,:]

        # convert to degrees 
        q_deg = np.rad2deg(q_all)
        q_deg = ((q_deg + 180.0) % 360.0) - 180.0

        # --- DEBUG: show first rows of computed joint angles (radian) ---
        logging.debug(f"q_all[:8,:] (rad) = {q_all[:8,:]}")

        # Build output matrix: time, then for each joint [deg, 0, 0]
        time_col = time_out.reshape(-1,1)
        out_cols = [time_col]
        # Convert angles to degrees and wrap to (-180, 180]
        q_deg = np.rad2deg(q_all)
        q_deg = ((q_deg + 180.0) % 360.0) - 180.0
        zero_cols = np.zeros((total_samples, 2))
        for j in range(6):
            out_cols.append(q_deg[:, j:j+1])
            out_cols.append(zero_cols)
        output_matrix = np.hstack(out_cols)

        # Create output directory
        out_dir = os.path.dirname(self.output_path)
        if out_dir and not os.path.exists(out_dir):
            os.makedirs(out_dir, exist_ok=True)



        # For exact parity with the validated "improved" CSV we write rows using the same
        # formatting used when the improved file was created:
        # - time: round to 2 decimals
        # - each joint angle: rounded to 6 decimals via round(..., 6) (stringified by CSV writer)
        # - the two following columns after each joint are integer zeros
        import csv
        with open(self.output_path, 'w', newline='') as of:
            w = csv.writer(of)
            for i in range(total_samples):
                row = [round((i+1)*self.dt, 2)]
                for j in range(6):
                    # round to 6 decimals (keeps few decimals when possible)
                    row.append(round(q_deg[i, j], 6))
                    row.append(0)
                    row.append(0)
                w.writerow(row)
        logging.info(f"Wrote CSV to {self.output_path} using improved formatting")

        # --- Save time-series plots (displacement, speed, acceleration) if requested ---
        # time vectors for plotting:
        # displacement -> time_out (len N)
        # speed -> time_out[1:] (len N-1)
        # acceleration -> time_out[2:] (len N-2)
        try:
            if self.save_plots:
                _save_plot(time_out, displacement, self.plot_paths.get('displacement'),
                           'Displacement vs Time', 'Time (s)', 'Displacement (m)')
                _save_plot(time_out[1:], speed, self.plot_paths.get('speed'),
                           'Speed vs Time', 'Time (s)', 'Speed (m/s)')
                _save_plot(time_out[2:], acceleration, self.plot_paths.get('acceleration'),
                           'Acceleration vs Time', 'Time (s)', 'Acceleration (m/s^2)')
        except Exception as e:
            logging.warning('Exception while saving plots: %s', e)

        return {
            'time': time_out,
            's': s,
            'displacement': displacement,
            'speed': speed,
            'acceleration': acceleration,
            'q_all_rad': q_all,
            'q_all_deg': q_deg,
            'output_path': self.output_path,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', '-c', type=str, default='configs/research1_config.yaml', help='path to config yaml')

    args = parser.parse_args()

    tg = TrajectoryGenerator(args.config)
    tg.generate_trajectory()


if __name__ == '__main__':
    main()
