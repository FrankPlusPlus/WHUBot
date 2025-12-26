"""Robot utilities.

Provides the Robot class (DH forward kinematics and storage of initial joint angles)
and the `parse_robot_xml` helper to extract robot parameters from an XML file.
"""
from __future__ import annotations
import math
import numpy as np
import xml.etree.ElementTree as ET
from typing import Tuple


class Robot:
    """Robot model storing DH parameters and optional initial joint values.

    Parameters
    ----------
    d : sequence, optional
        Link offsets d (6 values). If omitted, defaults are used.
    a : sequence, optional
        Link lengths a (6 values). If omitted, defaults are used.
    alpha : sequence, optional
        Link twist angles in radians (6 values).
    jv_initial : sequence, optional
        Initial joint values in degrees (6 values).

    Attributes
    ----------
    d, a, alpha : np.ndarray
        Arrays of DH parameters (length 6).
    jv_initial : np.ndarray or None
        Initial joint angles in degrees if provided.
    """
    def __init__(self, d=None, a=None, alpha=None, jv_initial=None):
        if d is None:
            self.d = np.array([0.660, 0.0, 0.150, -0.432, 0.0, -0.056])
        else:
            self.d = np.array(d, dtype=float)
        if a is None:
            self.a = np.array([0.01, 0.432, 0.02, 0.0, 0.0, 0.0])
        else:
            self.a = np.array(a, dtype=float)
        if alpha is None:
            self.alpha = np.array([math.pi/2, math.pi, math.pi/2, math.pi/2, math.pi/2, -math.pi])
        else:
            self.alpha = np.array(alpha, dtype=float)
        # store JVInitial (degrees) as attribute for use as q0
        if jv_initial is not None:
            jv_arr = np.array(jv_initial, dtype=float)
            if jv_arr.shape == (6,):
                self.jv_initial = jv_arr
            else:
                # pad/trim
                padded = np.zeros(6, dtype=float)
                padded[:min(6, jv_arr.size)] = jv_arr[:6]
                self.jv_initial = padded
        else:
            self.jv_initial = None

    @staticmethod
    def _rotz(theta: float):
        c = math.cos(theta); s = math.sin(theta)
        return np.array([[c, -s, 0, 0],[s, c, 0, 0],[0,0,1,0],[0,0,0,1]])
    @staticmethod
    def _rotx(alpha: float):
        c = math.cos(alpha); s = math.sin(alpha)
        return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])

    @staticmethod
    def _transz(d: float):
        return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])

    @staticmethod
    def _transx(a: float):
        return np.array([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    def fk(self, q):
        """Compute forward kinematics for the robot.

        Parameters
        ----------
        q : array_like
            Joint angles in radians (length 6).

        Returns
        -------
        np.ndarray
            4x4 homogeneous transform of the end-effector.
        """
        T = np.eye(4)
        for i in range(6):
            T = T @ self._rotz(q[i]) @ self._transz(self.d[i]) @ self._transx(self.a[i]) @ self._rotx(self.alpha[i])
        return T


def parse_robot_xml(xml_path: str) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Parse a robot XML and extract DH parameters and JVInitial values.

    Parameters
    ----------
    xml_path : str
        Path to the XML file to parse.

    Returns
    -------
    tuple of np.ndarray
        (d, a, alpha, jv_initial) each as length-6 arrays. alpha is in radians; jv_initial in degrees.

    Notes
    -----
    Heuristics: LinkLength -> a, TwistAngle -> alpha (deg->rad), JointOffset -> d. Values that appear
    to be in millimeters are converted to meters.
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Parse links
    links = root.findall('.//RALink')
    a_list = []
    alpha_list = []
    d_list = []
    for link in links:
        ll = link.findtext('LinkLength')
        ta = link.findtext('TwistAngle')
        jo = link.findtext('JointOffset')
        ll_val = float(ll) if ll is not None else 0.0
        ta_val = float(ta) if ta is not None else 0.0
        jo_val = float(jo) if jo is not None else 0.0
        a_list.append(ll_val)
        alpha_list.append(np.deg2rad(ta_val))
        d_list.append(jo_val)

    # Parse joint initial velocities/positions (JVInitial)
    joints = root.findall('.//RAJoint')
    jv_list = []
    for j in joints:
        jv = j.findtext('JVInitial')
        if jv is not None:
            jv_list.append(float(jv))

    def pad_or_trim(arr, length=6):
        arr = arr[:length]
        if len(arr) < length:
            arr = arr + [0.0] * (length - len(arr))
        return arr

    a_list = pad_or_trim(a_list)
    alpha_list = pad_or_trim(alpha_list)
    d_list = pad_or_trim(d_list)
    jv_list = pad_or_trim(jv_list)

    # Unit heuristics: if values look like mm (>10), convert to meters
    if max(abs(x) for x in a_list) > 10:
        a_list = [x / 1000.0 for x in a_list]
    if max(abs(x) for x in d_list) > 10:
        d_list = [x / 1000.0 for x in d_list]

    return (np.array(d_list, dtype=float), np.array(a_list, dtype=float), np.array(alpha_list, dtype=float), np.array(jv_list, dtype=float))