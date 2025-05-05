from dronekit import LocationGlobalRelative
import pyproj
from typing import Tuple
from constants import DEFAULT_ELLPS


class Geodesy:

    def __init__(self, ellps: str = DEFAULT_ELLPS):
        self._geod = pyproj.Geod(ellps=ellps)

    def inverse(
        self,
        coord1: LocationGlobalRelative,
        coord2: LocationGlobalRelative
    ) -> Tuple[float, float]:

        fwd_az, _, dist = self._geod.inv(
            coord1.lon, coord1.lat,
            coord2.lon, coord2.lat
        )
        return dist, (fwd_az + 360) % 360
