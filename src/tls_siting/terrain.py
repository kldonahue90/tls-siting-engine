"""Terrain auditing and DEM verification module for TLS Siting Engine."""

import math
from typing import List, Tuple, Union, Optional
import numpy as np
import rasterio

from tls_siting.exceptions import PhysicalInvariantError


class TerrainAuditor:
    """
    Audits digital elevation models (DEM) for physical terrain clearances.
    """

    def __init__(self, dem_source: Union[str, rasterio.DatasetReader, rasterio.io.MemoryFile]):
        if isinstance(dem_source, rasterio.io.MemoryFile):
            self.dataset = dem_source.open()
            self._should_close = True
        elif isinstance(dem_source, str):
            self.dataset = rasterio.open(dem_source)
            self._should_close = True
        elif isinstance(dem_source, rasterio.DatasetReader):
            self.dataset = dem_source
            self._should_close = False
        else:
            raise ValueError(f"Unsupported dem_source type: {type(dem_source)}")

    def __del__(self):
        if getattr(self, "_should_close", False) and hasattr(self, "dataset"):
            try:
                self.dataset.close()
            except Exception:
                pass

    def sample_elevation(self, coord: Tuple[float, ...]) -> float:
        """
        Samples orthometric elevation from DEM at coordinate.

        coord can be (x, y) / (x, y, z) or (lat, lon) / (lat, lon, z) or (x, y) in dataset CRS.
        """
        x_val, y_val = float(coord[0]), float(coord[1])
        # If 3 elements provided and dataset fails or for fallback:
        try:
            # rasterio sample takes list of (x, y) or (lon, lat) tuples depending on CRS
            vals = list(self.dataset.sample([(x_val, y_val)]))
            return float(vals[0][0])
        except Exception:
            if len(coord) >= 3:
                return float(coord[2])
            raise

    def verify_ground_drop(self, crown_coord: Tuple[float, ...], asa_coord: Tuple[float, ...]) -> float:
        """
        Computes delta_z = Z_crown - Z_asa.
        Raises PhysicalInvariantError if delta_z > 4.0 ft (1.2192 m).

        Returns delta_z in meters.
        """
        z_crown = crown_coord[2] if len(crown_coord) >= 3 else self.sample_elevation(crown_coord)
        z_asa = asa_coord[2] if len(asa_coord) >= 3 else self.sample_elevation(asa_coord)

        # Also sample DEM directly if z was passed but DEM is available
        try:
            dem_z_crown = self.sample_elevation(crown_coord)
            z_crown = dem_z_crown
        except Exception:
            pass

        try:
            dem_z_asa = self.sample_elevation(asa_coord)
            z_asa = dem_z_asa
        except Exception:
            pass

        delta_z = z_crown - z_asa
        max_drop_m = 4.0 * 0.3048  # 1.2192 m

        if delta_z > max_drop_m + 1e-6:
            raise PhysicalInvariantError(
                f"ASA ground drop delta_z = {delta_z:.3f}m exceeds max allowable 4.0 ft (1.2m) relative to runway crown."
            )
        return float(delta_z)

    def verify_calbit_sightline(
        self,
        calbit_coord: Tuple[float, ...],
        esa_coord: Tuple[float, ...],
        samples: int = 50
    ) -> float:
        """
        Traces 3D optical chord between Cal/BIT (1.8m AGL) and ESA midpoint (4.6m AGL).
        Asserts that intervening DEM terrain does not breach the chord by > 1.8m (6.0 ft).

        Returns max terrain breach relative to chord in meters.
        """
        x1, y1 = float(calbit_coord[0]), float(calbit_coord[1])
        z1 = (float(calbit_coord[2]) if len(calbit_coord) >= 3 else self.sample_elevation(calbit_coord)) + 1.8

        x2, y2 = float(esa_coord[0]), float(esa_coord[1])
        z2 = (float(esa_coord[2]) if len(esa_coord) >= 3 else self.sample_elevation(esa_coord)) + 4.6

        max_breach = -float("inf")

        for i in range(samples):
            t = i / float(samples - 1) if samples > 1 else 0.0
            x_t = x1 + t * (x2 - x1)
            y_t = y1 + t * (y2 - y1)
            z_chord = z1 + t * (z2 - z1)

            z_terrain = self.sample_elevation((x_t, y_t))
            breach = z_terrain - z_chord
            if breach > max_breach:
                max_breach = breach

        max_breach_limit_m = 1.8  # 6.0 ft / 1.8m

        if max_breach > max_breach_limit_m + 1e-6:
            raise PhysicalInvariantError(
                f"Cal/BIT to ESA optical sightline breached by terrain by {max_breach:.3f}m (> 1.8m / 6.0 ft limit)."
            )

        return float(max_breach)

    def verify_esa_guys_slope(
        self,
        esa_coord: Tuple[float, ...],
        guy_coords: Optional[List[Tuple[float, ...]]] = None
    ) -> float:
        """
        Samples elevations across guy wire footprint (21.0 ft / 6.4 m radius) and
        asserts maximum slope <= 10.0% to protect +-0.11 deg tilt limit.

        Returns maximum slope percentage (e.g. 8.5 for 8.5%).
        """
        esa_x, esa_y = float(esa_coord[0]), float(esa_coord[1])
        esa_z = esa_coord[2] if len(esa_coord) >= 3 else self.sample_elevation(esa_coord)
        try:
            esa_z = self.sample_elevation(esa_coord)
        except Exception:
            pass

        radius_m = 21.0 * 0.3048  # 6.4008 m

        if guy_coords is None:
            guy_coords = []
            for angle_deg in [0.0, 120.0, 240.0]:
                rad = math.radians(angle_deg)
                gx = esa_x + radius_m * math.cos(rad)
                gy = esa_y + radius_m * math.sin(rad)
                guy_coords.append((gx, gy))

        max_slope_pct = 0.0

        for gx, gy in [(esa_x, esa_y)] + [g[:2] for g in guy_coords]:
            z_g = self.sample_elevation((gx, gy))
            dist = math.hypot(gx - esa_x, gy - esa_y)
            if dist > 1e-6:
                slope_pct = (abs(z_g - esa_z) / dist) * 100.0
                if slope_pct > max_slope_pct:
                    max_slope_pct = slope_pct

        # Also check slope between guy anchor pairs
        for i in range(len(guy_coords)):
            for j in range(i + 1, len(guy_coords)):
                g1, g2 = guy_coords[i], guy_coords[j]
                z1 = self.sample_elevation(g1)
                z2 = self.sample_elevation(g2)
                dist = math.hypot(g1[0] - g2[0], g1[1] - g2[1])
                if dist > 1e-6:
                    slope_pct = (abs(z1 - z2) / dist) * 100.0
                    if slope_pct > max_slope_pct:
                        max_slope_pct = slope_pct

        if max_slope_pct > 10.0 + 1e-6:
            raise PhysicalInvariantError(
                f"ESA guy anchor maximum terrain slope {max_slope_pct:.2f}% exceeds 10.0% limit."
            )

        return float(max_slope_pct)
