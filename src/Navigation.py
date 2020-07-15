#!/usr/bin/env python

import numpy as np


def DEG_TO_RAD(x):
    return ((x * np.pi) / 180.0)


def RAD_TO_DEG(x):
    return ((x * 180.0) / np.pi)


class NAVIGATION:
    def __init__(self):
        self._name = 'Navigation lib'
        self.em = EarthModel('WGS84')  ## EarthModel or WGS84

    def ned2lla(self, ned, LLA0):
        LLA = np.array([0.0, 0.0, 0.0])
        LLA0Rad = np.array([0.0, 0.0])

        for i in range(2):
            LLA0Rad[i] = DEG_TO_RAD(LLA0[i])

        a = self.em.getEquatorialRadius
        f = self.em.getFlattening

        Rn = a / np.sqrt(1 - (2 * f - f * f) * np.sin(LLA0Rad[0]) * np.sin(LLA0Rad[0]))
        Rm = Rn * (1 - (2 * f - f * f)) / (1 - (2 * f - f * f) * np.sin(LLA0Rad[0]) * np.sin(LLA0Rad[0]))

        LLA[0] = RAD_TO_DEG((LLA0Rad[0] + np.arctan2(1, Rm) * ned[0]))
        LLA[1] = RAD_TO_DEG((LLA0Rad[1] + np.arctan2(1, Rn * np.cos(LLA0Rad[0])) * ned[1]))
        LLA[2] = -ned[2] + LLA0[2]
        return LLA

    def lla2ned(self, LLA, LLA0):
        ned = np.zeros((3, 1))
        LLARad = np.zeros((2, 1))
        LLA0Rad = np.zeros((2, 1))

        for i in range(2):
            LLA0Rad[i] = DEG_TO_RAD(LLA0[i])
            LLARad[i] = DEG_TO_RAD(LLA[i])

        a = self.em.getEquatorialRadius
        f = self.em.getFlattening

        dLat = (LLARad[0] - LLA0Rad[0])
        dLon = (LLARad[1] - LLA0Rad[1])

        Rn = a / np.sqrt(1 - (2 * f - f * f) * np.sin(LLA0Rad[0]) * np.sin(LLA0Rad[0]))
        Rm = Rn * (1 - (2 * f - f * f)) / (1 - (2 * f - f * f) * np.sin(LLA0Rad[0]) * np.sin(LLA0Rad[0]))

        ned[0] = dLat / np.arctan2(1, Rm)
        ned[1] = dLon / np.arctan2(1, Rn * np.cos(LLA0Rad[0]))
        ned[2] = -LLA[2] + LLA0[2]

        return ned


class EarthModel:
    def __init__(self, model):
        if model == 'EarthModel':
            self.getEquatorialRadius = 0.0
            self.getEccentricity = 0.0
            self.getFlattening = 0.0
        elif model == 'WGS84':
            self.getEquatorialRadius = 6378137.0
            self.getEccentricity = 8.1819190842622e-2
            self.getFlattening = 1.0 / 298.257223563
