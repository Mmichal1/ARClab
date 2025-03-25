from abc import ABCMeta, abstractmethod
from functools import partial

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from scipy.integrate import solve_ivp


class Model:
    __metaclass__ = ABCMeta

    def __init__(self, state: np.array, dt: float) -> None:
        self._state = state  # vehicle state
        self._dt = dt  # time step

    @abstractmethod
    def step(self, u: np.array):
        pass

    @property
    def n(self):
        return len(self.state)

    @property
    @abstractmethod
    def m(self):
        pass

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, state):
        self._state = state


class UnicycleModel(Model):
    def __init__(
        self, state: np.array, dt: float, Iw=0.000125, If=0.01125, mw=0.1, mf=1.0, Rw=0.05, L=0.15, e=0.1, delta=0.05
    ) -> None:
        # state is vehicle state: x, y, theta
        super().__init__(state, dt)
        self._Iw = Iw
        self._If = If
        self._Ip = self._If + 2 * self._Iw

        self._mw = mw
        self._mf = mf
        self._mp = self._mf + 2 * self._mw

        self._Rw = Rw
        self._L = L

        self._e = e
        self._delta = delta

    def step(self, u: np.array):
        K = np.array([[np.cos(self.state[2]), 0], [np.sin(self.state[2]), 0], [0, 1]])
        dx = np.matmul(K, u)
        self._state = self._state + dx * self._dt

        return self._state

    @property
    def m(self):
        return 2

    @property
    def M(self):
        mp = self._mp
        Ip = self._Ip
        Iw = self._Iw
        M = np.array(
            [
                [mp, 0, 0, 0, 0],
                [0, mp, 0, 0, 0],
                [0, 0, Ip, 0, 0],
                [0, 0, 0, Iw, 0],
                [0, 0, 0, 0, Iw],
            ]
        )
        return M

    @property
    def B(self):
        B = np.array(
            [
                [0, 0],
                [0, 0],
                [0, 0],
                [1, 0],
                [0, 1],
            ]
        )
        return B

    @property
    def G(self):
        q = self._state
        L = self._L
        Rw = self._Rw
        G = np.array(
            [
                [np.cos(q[2]), np.cos(q[2])],
                [np.sin(q[2]), np.sin(q[2])],
                [1 / L, -1 / L],
                [2 / Rw, 0],
                [0, 2 / Rw],
            ]
        )
        return G

    def G_d1(self, state_d1):
        q = self._state
        q_d1 = state_d1
        G_d1 = np.array(
            [
                [-np.sin(q[2]) * q_d1[2], -np.sin(q[2]) * q_d1[2]],
                [np.cos(q[2]) * q_d1[2], np.cos(q[2]) * q_d1[2]],
                [0, 0],
                [0, 0],
                [0, 0],
            ]
        )
        return G_d1

    @property
    def e(self):
        return self._e

    @property
    def delta(self):
        return self._delta

    @property
    def h(self):
        e = self._e
        delta = self._delta
        x = self._state[0]
        y = self._state[1]
        theta = self._state[2]
        h = np.array(
            [
                x + e * np.cos(theta + delta),
                y + e * np.sin(theta + delta),
            ]
        )
        return h


def trajectory_generator_square(t, dt=1):
    if type(t) != np.ndarray:
        # TODO: generate square trajectory
        # with e.g. piece wise approach
        h = np.array([0, 0])
        h_d1 = np.array([0, 0])
        h_d2 = np.array([0, 0])
    else:
        # do not change below code of this function
        h = np.zeros((2, len(t)))
        h_d1 = np.zeros((2, len(t)))
        h_d2 = np.zeros((2, len(t)))
        for i in range(len(t)):
            h[:, i], h_d1[:, i], h_d2[:, i] = trajectory_generator_square(t[i])

    return h, h_d1, h_d2


def trajectory_generator_circle(t, w=np.pi * 0.4, offset=0.2, A=1.0):
    h = np.array([A * np.cos(t * w + offset), A * np.sin(t * w + offset)])
    # TODO: calculate first and second derivative
    h_d1 = np.array([t, t])
    h_d2 = np.array([t, t])
    return h, h_d1, h_d2


class Simulator:
    __metaclass__ = ABCMeta

    def __init__(self, model, dt=0.01) -> None:
        self._model = model
        self._stats = {}
        self._dt = dt

    @abstractmethod
    def step(self, t, state):
        pass

    def run(self, start: np.array, T: float, dt: float, trajectory):
        self.T = T
        self.dt = dt
        self._trajectory = trajectory

        startTime = 0
        self._stats = {"t": [startTime], "next": 0}

        solver = solve_ivp(
            self.step,
            [0, T],
            start,
            method="RK45",
            rtol=1e-3,
            atol=1e-6,
            t_eval=list(np.arange(startTime, T + self._dt, self._dt)),
        )

        return self._stats, solver


class SimulatorDynamics(Simulator):
    def __init__(self, model, dt=0.01) -> None:
        self._model = model
        self._stats = {}
        self._dt = dt

    def step(self, t, state):
        # state = [ h, h_d1, k ] = [ h1, h2, h1', h2', x, y, theta, phi_r, phi_l ]
        #   - h      = state[0:2]
        #   - h_d1   = state[2:4]
        #   - k      = state[4:9] (which is q = [x, y, theta, phi_r, phi_l])

        h = state[0:2]
        h_d1 = state[2:4]
        k = state[4:9]  # unicycle "full" state
        q = k.reshape((-1,))  # same as k, just cleaner symbol
        self._model.state = q  # update the model’s internal state

        # Unicycle parameters
        e = self._model.e
        delta = self._model.delta

        # Matrices from the model
        M = self._model.M
        B = self._model.B
        G = self._model.G

        # -----------------------------
        # 1) Compute Jacobian dh/dq
        #    h(q) = [ x + e cos(theta + delta),
        #             y + e sin(theta + delta) ]
        # -----------------------------
        x, y, theta = q[0], q[1], q[2]
        dh_dq = np.array([[1.0, 0.0, -e * np.sin(theta + delta)], [0.0, 1.0, e * np.cos(theta + delta)]])  # shape 2x3

        # Rinv, R, etc. (we won't use them for pure constant-driving,
        # but we’ll fill them to keep the structure)
        Rinv = dh_dq @ G[0:3, :]  # 2x2
        detRinv = np.linalg.det(Rinv)
        R = np.linalg.inv(Rinv)  # 2x2
        detR = np.linalg.det(R)
        RT = R.T

        # For demonstration we skip advanced dynamics
        # and set Ms, Cs, Bs, etc. to identity-like.
        Ms = np.eye(2)
        Cs = np.zeros((2, 2))
        Bs = np.eye(2)

        Mh = np.eye(2)
        Ch = np.zeros((2, 2))
        Bh = np.eye(2)

        # 2) Constant wheel velocities for Task 1
        u = np.array([0.1, 0.1])  # Right wheel = 0.1, Left wheel = 0.1

        # 3) We get q' = G @ u for the dynamic model’s states
        k_d1 = G @ u  # shape (5,)

        # 4) The linearized velocities h_d1 = d/dt [h(q)] = dh/dq * q'
        #    but note h depends only on (x,y,theta), so we only use the first 3 entries of q'.
        q_d1_xyz = k_d1[0:3]
        new_h_d1 = dh_dq @ q_d1_xyz

        # 5) The second derivative of h, h_d2, is zero if we apply constant inputs
        new_h_d2 = np.zeros(2)  # we’re not controlling or accelerating in tasks 1

        # 6) Form the new ODE state derivative
        #    We are integrating [h, h_d1, k].
        #    So derivative is [h_d1, h_d2, k_d1].
        new_state = np.concatenate([new_h_d1, new_h_d2, k_d1])

        # For logging/printing
        if t >= self._stats.get("next", 0.0):
            self._stats["next"] = t + self._dt
            print(
                f"t: {t:.2f}, "
                f"h: {h},  x={q[0]:.2f}, y={q[1]:.2f}, theta={q[2]:.2f}, "
                f"detRinv={detRinv:.6f}, detR={detR:.6f}"
            )

        return new_state


class SimulatorKinematics(Simulator):
    def __init__(self, model, dt=0.01) -> None:
        self._model = model
        self._stats = {}
        self._dt = dt

    def step(self, t, state):
        h = state[0:2]
        _ = state[2:4]
        k = state[4:9]
        q = k.reshape((-1,))

        e = self._model.e
        delta = self._model.delta

        self._model.state = q
        G = self._model.G

        dh_dq = np.array([[1, 2, 3], [4, 5, 6]])
        Rinv = dh_dq @ G[0:3, :]
        detRinv = np.linalg.det(Rinv)

        R = np.linalg.inv(Rinv)
        detR = np.linalg.det(R)

        hd, hd_d1, _ = self._trajectory(t)
        eh = np.zeros((2))

        # TODO: some calculations

        h_d1 = np.zeros((2))
        k_d1 = np.zeros((5))

        h_d2 = np.array([0, 0])
        new_state = np.concatenate([h_d1, h_d2, k_d1])

        if t >= self._stats["next"]:
            self._stats["next"] = t + self._dt
            print(
                f"t: {t:.2f}, "
                f"e_h: {eh}, h: {h}, hd: {hd}, "
                f"x: {q[0]:.2f}, y: {q[1]:.2f}, theta: {q[2]:.2f}, "
                f"detRinv: {detRinv:.6f}, detR: {detR:.6f}"
            )

        return np.array(new_state)
