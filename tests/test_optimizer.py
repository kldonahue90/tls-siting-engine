"""Tests for tls_siting.optimizer module focusing on Creech AFB (KINS) Runway 26."""

import json
import math
import pytest
import xml.etree.ElementTree as ET
from shapely.geometry import Polygon

from tls_siting.optimizer import TLSSiteOptimizer
from tls_siting.exceptions import PhysicalInvariantError, RegulatoryWarning


def test_creech_afb_kins_rwy26_benchmark():
    # Creech AFB (KINS) Runway 26 parameters
    lat0 = 36.583336
    lon0 = -115.664908
    elev0_m = 946.16  # 3,104.2 ft MSL
    hdg_deg = 273.0
    gpa_deg = 3.0
    tch_ft = 50.0

    # Define Taxiway Golf TOFA exclusion buffer polygon (>= 35m)
    # Taxiway Golf running parallel at y = 100m
    twy_golf_buffer = Polygon([(0, 95), (600, 95), (600, 105), (0, 105)])

    optimizer = TLSSiteOptimizer(
        lat0=lat0,
        lon0=lon0,
        elev0_m=elev0_m,
        hdg_deg=hdg_deg,
        gpa_deg=gpa_deg,
        tch_ft=tch_ft,
        siting_mode="right",
        exclusion_polygons=[],
        allow_waivers=False,
    )

    positions = optimizer.solve(
        asa_offset_m=60.0,
        esa_offset_m=160.0,
        calbit_upwind_m=120.0,
        shelter_angle_deg=35.0,
    )

    # 1. Assert ASA lateral clearance >= 45m from runway centerline (OFZ clear)
    asa_x, asa_y, _ = positions["ASA"]
    assert abs(asa_y) >= 45.0, f"ASA lateral offset {asa_y}m is within 45m OFZ"

    # 2. Assert Taxiway Golf clearance >= 35m
    # Taxiway Golf at y = 100m, ASA at y = 60m -> distance = 40m >= 35m
    dist_to_twy = abs(100.0 - asa_y)
    assert dist_to_twy >= 35.0, f"ASA clearance to TWY Golf {dist_to_twy}m < 35m"

    # 3. Assert Multilateration ATA oriented upwind (-X relative to ASA)
    ata_x, ata_y, _ = positions["ATA"]
    assert ata_x < asa_x, f"ATA position x={ata_x} is not upwind of ASA x={asa_x}"

    # 4. Assert all 5 cable reel inequalities strictly satisfied
    cables = optimizer.cable_lengths
    assert cables["Shelter->ESA"] <= 70.0
    assert cables["Shelter->ASA"] <= 100.0
    assert cables["Shelter->Cal/BIT"] <= 150.0
    assert cables["ASA->ATA"] <= 100.0
    assert cables["Shelter->Interrogator"] <= 13.0

    # 5. Export and validate GeoJSON and KML
    geojson_str = optimizer.export_geojson()
    geojson_obj = json.loads(geojson_str)
    assert geojson_obj["type"] == "FeatureCollection"
    assert len(geojson_obj["features"]) > 0

    kml_str = optimizer.export_kml()
    assert "<kml" in kml_str
    kml_root = ET.fromstring(kml_str)
    folder_names = [elem.text for elem in kml_root.findall(".//{*}Folder/{*}name")]
    expected_folders = [
        "Virtual Reference & Approach Fixes",
        "TLS Hardware Subsystems",
        "Conex Shelter",
        "Physical Cable Runs",
        "Protected Sensor Critical Boundaries",
        "Airfield Safety Zones",
    ]
    for folder_name in expected_folders:
        assert folder_name in folder_names, f"Folder '{folder_name}' missing from KML output"


def test_siting_modes_left_and_split():
    optimizer_left = TLSSiteOptimizer(
        lat0=36.583336, lon0=-115.664908, elev0_m=946.16, hdg_deg=273.0, siting_mode="left"
    )
    pos_left = optimizer_left.solve(asa_offset_m=60.0, esa_offset_m=160.0)
    assert pos_left["ASA"][1] < 0 and pos_left["ESA"][1] < 0

    # Split mode with waiver allowed since baseline across runway is wider
    optimizer_split = TLSSiteOptimizer(
        lat0=36.583336, lon0=-115.664908, elev0_m=946.16, hdg_deg=273.0, siting_mode="split", allow_waivers=True
    )
    with pytest.warns(RegulatoryWarning):
        pos_split = optimizer_split.solve(asa_offset_m=60.0, esa_offset_m=160.0)
    assert pos_split["ASA"][1] < 0 and pos_split["ESA"][1] > 0


def test_invalid_siting_mode():
    with pytest.raises(ValueError):
        TLSSiteOptimizer(lat0=0.0, lon0=0.0, elev0_m=0.0, hdg_deg=0.0, siting_mode="invalid")


def test_shelter_angle_out_of_bounds():
    optimizer = TLSSiteOptimizer(
        lat0=36.583336,
        lon0=-115.664908,
        elev0_m=946.16,
        hdg_deg=273.0,
    )
    with pytest.raises(PhysicalInvariantError):
        optimizer.solve(shelter_angle_deg=25.0)


def test_cable_limit_violation_raises():
    optimizer = TLSSiteOptimizer(
        lat0=36.583336,
        lon0=-115.664908,
        elev0_m=946.16,
        hdg_deg=273.0,
        allow_waivers=False,
    )
    with pytest.raises(PhysicalInvariantError):
        optimizer.solve(asa_offset_m=60.0, esa_offset_m=300.0)


def test_cable_limit_violation_warning_when_waiver_allowed():
    optimizer = TLSSiteOptimizer(
        lat0=36.583336,
        lon0=-115.664908,
        elev0_m=946.16,
        hdg_deg=273.0,
        allow_waivers=True,
    )
    with pytest.warns(RegulatoryWarning):
        optimizer.solve(asa_offset_m=60.0, esa_offset_m=300.0)


def test_exclusion_polygon_violation():
    # Polygon overlapping ASA at (290.796, 60.0)
    overlap_poly = Polygon([(280, 50), (300, 50), (300, 70), (280, 70)])
    optimizer = TLSSiteOptimizer(
        lat0=36.583336,
        lon0=-115.664908,
        elev0_m=946.16,
        hdg_deg=273.0,
        exclusion_polygons=[overlap_poly],
        allow_waivers=False,
    )
    with pytest.raises(PhysicalInvariantError):
        optimizer.solve()

    optimizer_waiver = TLSSiteOptimizer(
        lat0=36.583336,
        lon0=-115.664908,
        elev0_m=946.16,
        hdg_deg=273.0,
        exclusion_polygons=[overlap_poly],
        allow_waivers=True,
    )
    with pytest.warns(RegulatoryWarning):
        optimizer_waiver.solve()
