"""Tests for tls_siting.terrain module."""

import math
import numpy as np
import pytest
import rasterio
from rasterio.transform import from_origin

from tls_siting.terrain import TerrainAuditor
from tls_siting.exceptions import PhysicalInvariantError


def create_synthetic_dem(data_array: np.ndarray, res: float = 1.0) -> rasterio.io.MemoryFile:
    """Helper to create in-memory GeoTIFF raster dataset."""
    memfile = rasterio.io.MemoryFile()
    transform = from_origin(0.0, 100.0, res, res)
    height, width = data_array.shape
    with memfile.open(
        driver="GTiff",
        height=height,
        width=width,
        count=1,
        dtype=data_array.dtype,
        crs="EPSG:32611",
        transform=transform,
    ) as dataset:
        dataset.write(data_array, 1)
    return memfile


def test_terrain_auditor_init_variants():
    grid = np.zeros((100, 100), dtype=np.float32)
    memfile = create_synthetic_dem(grid)
    auditor1 = TerrainAuditor(memfile)
    assert auditor1.sample_elevation((10.0, 10.0)) == 0.0

    ds = memfile.open()
    auditor2 = TerrainAuditor(ds)
    assert auditor2.sample_elevation((10.0, 10.0)) == 0.0

    with pytest.raises(ValueError):
        TerrainAuditor(123)  # Invalid type


def test_ground_drop_pass_and_fail():
    grid = np.zeros((100, 100), dtype=np.float32)
    # Crown at (10, 10) elev 100.0m -> row int((100 - 10)/1) = 90, col 10
    grid[90, 10] = 100.0
    grid[80, 20] = 99.0

    memfile = create_synthetic_dem(grid)
    auditor = TerrainAuditor(memfile)

    # Pass case
    dz = auditor.verify_ground_drop((10.0, 10.0), (20.0, 20.0))
    assert abs(dz - 1.0) <= 1e-3

    # Pass case with z passed in tuple
    dz_z = auditor.verify_ground_drop((10.0, 10.0, 100.0), (20.0, 20.0, 99.0))
    assert abs(dz_z - 1.0) <= 1e-3

    # Fail case: ASA ground drop 2.0m > 1.2192m -> FAIL
    grid_fail = np.zeros((100, 100), dtype=np.float32)
    grid_fail[90, 10] = 100.0
    grid_fail[80, 20] = 97.0
    memfile_fail = create_synthetic_dem(grid_fail)
    auditor_fail = TerrainAuditor(memfile_fail)

    with pytest.raises(PhysicalInvariantError):
        auditor_fail.verify_ground_drop((10.0, 10.0), (20.0, 20.0))


def test_calbit_sightline_pass_and_fail():
    grid = np.zeros((100, 100), dtype=np.float32)
    memfile = create_synthetic_dem(grid)
    auditor = TerrainAuditor(memfile)

    # Flat ground pass case
    calbit = (0.0, 50.0, 0.0)
    esa = (100.0, 50.0, 0.0)
    breach = auditor.verify_calbit_sightline(calbit, esa)
    assert breach <= 1.8

    # High terrain peak breaching chord in middle
    grid_mound = np.zeros((100, 100), dtype=np.float32)
    grid_mound[45:55, 45:55] = 10.0  # 10m high peak area in middle
    memfile_mound = create_synthetic_dem(grid_mound)
    auditor_mound = TerrainAuditor(memfile_mound)

    with pytest.raises(PhysicalInvariantError):
        auditor_mound.verify_calbit_sightline((0.0, 50.0, 0.0), (100.0, 50.0, 0.0))


def test_esa_guys_slope_pass_and_fail():
    grid = np.zeros((100, 100), dtype=np.float32)
    memfile = create_synthetic_dem(grid)
    auditor = TerrainAuditor(memfile)

    esa_coord = (50.0, 50.0, 0.0)
    slope = auditor.verify_esa_guys_slope(esa_coord)
    assert slope == 0.0

    # Test steep slope fail case
    grid_steep = np.zeros((100, 100), dtype=np.float32)
    # Put a steep hill under one guy anchor
    radius_m = 21.0 * 0.3048
    gx = 50.0 + radius_m
    gy = 50.0
    # row = int((100 - gy)/1) = 50, col = int(gx) = 56
    grid_steep[50, int(gx)] = 5.0
    memfile_steep = create_synthetic_dem(grid_steep)
    auditor_steep = TerrainAuditor(memfile_steep)

    with pytest.raises(PhysicalInvariantError):
        auditor_steep.verify_esa_guys_slope(esa_coord)
