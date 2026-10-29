"""TLS Hardware Siting Optimizer and GeoJSON/3D KML Exporter."""

import json
import math
import warnings
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple, Any
from shapely.geometry import Point, Polygon, LineString

from tls_siting.exceptions import PhysicalInvariantError, RegulatoryWarning
from tls_siting.geometry import (
    cartesian_to_geodetic,
    calculate_touchdown_aiming_point,
    geodetic_inverse,
)


class TLSSiteOptimizer:
    """
    Constraint-satisfaction solver and exporter for TLS hardware placement.
    """

    def __init__(
        self,
        lat0: float,
        lon0: float,
        elev0_m: float,
        hdg_deg: float,
        gpa_deg: float = 3.0,
        tch_ft: float = 50.0,
        runway_dz_ft: float = 0.0,
        siting_mode: str = "right",
        exclusion_polygons: Optional[List[Polygon]] = None,
        allow_waivers: bool = False,
    ):
        self.lat0 = lat0
        self.lon0 = lon0
        self.elev0_m = elev0_m
        self.hdg_deg = hdg_deg
        self.gpa_deg = gpa_deg
        self.tch_ft = tch_ft
        self.runway_dz_ft = runway_dz_ft

        if siting_mode.lower() not in ["right", "left", "split"]:
            raise ValueError(f"Invalid siting_mode: {siting_mode}. Must be 'right', 'left', or 'split'.")
        self.siting_mode = siting_mode.lower()
        self.exclusion_polygons = exclusion_polygons or []
        self.allow_waivers = allow_waivers

        # Hardware positions in local Cartesian (x, y, z)
        self.positions: Dict[str, Tuple[float, float, float]] = {}
        # Conex shelter orientation angle relative to centerline in degrees
        self.shelter_angle_deg: float = 35.0
        # Cable lengths dict
        self.cable_lengths: Dict[str, float] = {}

    def solve(
        self,
        asa_offset_m: float = 60.0,
        esa_offset_m: float = 160.0,
        calbit_upwind_m: float = 120.0,
        shelter_angle_deg: float = 35.0,
    ) -> Dict[str, Tuple[float, float, float]]:
        """
        Solves TLS hardware layout in Cartesian coordinates (x, y, z).
        """
        if not (30.0 <= shelter_angle_deg <= 40.0):
            msg = f"Conex shelter orientation {shelter_angle_deg:.1f}° is outside allowable 30.0° - 40.0° range."
            raise PhysicalInvariantError(msg)
        self.shelter_angle_deg = shelter_angle_deg

        td_m = calculate_touchdown_aiming_point(self.tch_ft, self.gpa_deg, self.runway_dz_ft)

        # Base lateral sign multiplier
        if self.siting_mode == "right":
            asa_y = abs(asa_offset_m)
            esa_y = abs(esa_offset_m)
        elif self.siting_mode == "left":
            asa_y = -abs(asa_offset_m)
            esa_y = -abs(esa_offset_m)
        else:  # 'split': ASA/ATA left, ESA/Shelter right
            asa_y = -abs(asa_offset_m)
            esa_y = abs(esa_offset_m)

        # 1. ASA: phase center z = 1.5m (4.9 ft)
        asa_x = td_m
        asa_z = 0.0  # Ground z = 0, phase center height 1.5m handled during antenna height reporting
        asa_pos = (asa_x, asa_y, asa_z)

        # 2. ESA: phase center z = 4.6m (15.1 ft)
        esa_x = td_m
        esa_z = 0.0
        esa_pos = (esa_x, esa_y, esa_z)

        # 3. Multilateration ATA:
        # Perpendicular unit normal to ASA-ESA baseline directed into -X half-plane (upwind)
        # Vector ASA->ESA: dx = esa_x - asa_x, dy = esa_y - asa_y
        dx = esa_x - asa_x
        dy = esa_y - asa_y
        baseline_len = math.hypot(dx, dy)
        if baseline_len < 1e-6:
            raise PhysicalInvariantError("ASA and ESA coordinates are identical.")

        # Baseline normal pointing into -X:
        # Perpendicular candidates: (-dy, dx) or (dy, -dx)
        # We need normal_x < 0 (pointing upwind toward approaching aircraft)
        nx = -abs(dy) / baseline_len
        ny = dx / baseline_len if baseline_len > 1e-6 else 0.0

        if nx >= 0:
            msg = f"Multilateration ATA normal vector x component {nx:.3f} is not pointing upwind (-X)."
            raise PhysicalInvariantError(msg)

        # Equilateral triangle side L = baseline_len
        # Distance from baseline midpoint to ATA apex: h = (sqrt(3)/2) * L
        mid_x = (asa_x + esa_x) / 2.0
        mid_y = (asa_y + esa_y) / 2.0
        h = (math.sqrt(3.0) / 2.0) * baseline_len

        ata_x = mid_x + nx * h
        ata_y = mid_y + ny * h
        ata_z = 0.0
        ata_pos = (ata_x, ata_y, ata_z)

        # Verify ATA is upwind (-X relative to ASA/ESA baseline)
        if ata_x >= asa_x:
            msg = f"Multilateration ATA position x={ata_x:.2f} is downwind of ASA baseline x={asa_x:.2f}."
            raise PhysicalInvariantError(msg)

        # 4. Cal/BIT: sited 110m to 140m upwind from ASA along lateral offset line, aimed at ESA
        calbit_x = asa_x - abs(calbit_upwind_m)
        calbit_y = asa_y
        calbit_z = 0.0
        calbit_pos = (calbit_x, calbit_y, calbit_z)

        # 5. Electronics Conex Shelter & Interrogator
        # Place shelter near midpoint of ASA-ESA baseline or optimized for cable budget
        # Default shelter position:
        shelter_x = mid_x - 10.0
        shelter_y = mid_y
        shelter_z = 0.0
        shelter_pos = (shelter_x, shelter_y, shelter_z)

        # Interrogator: 11m to 13m from shelter
        interrogator_x = shelter_x - 12.0
        interrogator_y = shelter_y
        interrogator_z = 0.0
        interrogator_pos = (interrogator_x, interrogator_y, interrogator_z)

        self.positions = {
            "ASA": asa_pos,
            "ESA": esa_pos,
            "ATA": ata_pos,
            "Cal/BIT": calbit_pos,
            "Shelter": shelter_pos,
            "Interrogator": interrogator_pos,
        }

        # Calculate cable lengths
        self.cable_lengths = {
            "Shelter->ESA": math.hypot(shelter_x - esa_x, shelter_y - esa_y),
            "Shelter->ASA": math.hypot(shelter_x - asa_x, shelter_y - asa_y),
            "Shelter->Cal/BIT": math.hypot(shelter_x - calbit_x, shelter_y - calbit_y),
            "ASA->ATA": math.hypot(asa_x - ata_x, asa_y - ata_y),
            "Shelter->Interrogator": math.hypot(shelter_x - interrogator_x, shelter_y - interrogator_y),
        }

        # Validate cable reel constraints
        limits = {
            "Shelter->ESA": 70.0,
            "Shelter->ASA": 100.0,
            "Shelter->Cal/BIT": 150.0,
            "ASA->ATA": 100.0,
            "Shelter->Interrogator": 13.0,
        }

        for cable_name, limit in limits.items():
            dist = self.cable_lengths[cable_name]
            if dist > limit + 1e-6:
                msg = f"Cable constraint violated: {cable_name} length {dist:.2f}m > {limit:.1f}m limit."
                if self.allow_waivers:
                    warnings.warn(msg, RegulatoryWarning)
                else:
                    raise PhysicalInvariantError(msg)

        # Check exclusion polygons / airfield buffers
        for poly in self.exclusion_polygons:
            for name, pos in self.positions.items():
                pt = Point(pos[0], pos[1])
                if poly.contains(pt) or poly.distance(pt) < 1e-6:
                    msg = f"Hardware item '{name}' violates airfield exclusion zone / buffer."
                    if self.allow_waivers:
                        warnings.warn(msg, RegulatoryWarning)
                    else:
                        raise PhysicalInvariantError(msg)

        return self.positions

    def get_conex_bounding_box(
        self,
        length_m: float = 6.1,
        width_m: float = 2.44
    ) -> List[Tuple[float, float, float]]:
        """
        Computes 4-corner 3D bounding box for Conex Shelter oriented at angle relative to runway centerline.
        Returns 4 corner tuples (x, y, z) in local Cartesian.
        """
        sx, sy, sz = self.positions["Shelter"]
        angle_rad = math.radians(self.shelter_angle_deg)

        # Half dimensions
        dx = length_m / 2.0
        dy = width_m / 2.0

        local_corners = [(-dx, -dy), (dx, -dy), (dx, dy), (-dx, dy)]
        world_corners = []

        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        for lx, ly in local_corners:
            wx = sx + lx * cos_a - ly * sin_a
            wy = sy + lx * sin_a + ly * cos_a
            world_corners.append((wx, wy, sz))

        return world_corners

    def export_geojson(self) -> str:
        """
        Exports compliant GeoJSON representation of TLS site layout with dynamic metadata.
        """
        features = []

        # 1. Virtual Reference & Approach Fixes
        td_m = calculate_touchdown_aiming_point(self.tch_ft, self.gpa_deg, self.runway_dz_ft)
        fixes = {
            "Point T": (0.0, 0.0, 0.0),
            "TD": (td_m, 0.0, 0.0),
            "FTM": (td_m + 300.0, 0.0, 0.0),
            "Point C": (-1100.0, 0.0, 0.0),
            "Point B": (-6000.0, 0.0, 0.0),
            "Point A": (-18000.0, 0.0, 0.0),
            "VLP": (td_m, 0.0, 0.0),
        }

        for name, pos in fixes.items():
            lat, lon, elev = cartesian_to_geodetic(
                pos[0], pos[1], pos[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            features.append({
                "type": "Feature",
                "folder": "Virtual Reference & Approach Fixes",
                "geometry": {"type": "Point", "coordinates": [lon, lat, elev]},
                "properties": {"name": name, "folder": "Virtual Reference & Approach Fixes", "elevation_m": elev},
            })

        # 2. TLS Hardware Subsystems
        heights = {
            "ASA": 1.5,
            "ESA": 4.6,
            "ATA": 1.5,
            "Cal/BIT": 1.8,
            "Shelter": 2.6,
            "Interrogator": 6.1,
        }

        for name, pos in self.positions.items():
            lat, lon, elev = cartesian_to_geodetic(
                pos[0], pos[1], pos[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            features.append({
                "type": "Feature",
                "folder": "TLS Hardware Subsystems",
                "geometry": {"type": "Point", "coordinates": [lon, lat, elev]},
                "properties": {
                    "name": name,
                    "folder": "TLS Hardware Subsystems",
                    "mounting_height_m": heights.get(name, 0.0),
                    "elevation_m": elev,
                },
            })

        # Multilateration Tracking Triangle
        tri_pts = []
        for name in ["ASA", "ESA", "ATA", "ASA"]:
            pos = self.positions[name]
            lat, lon, elev = cartesian_to_geodetic(
                pos[0], pos[1], pos[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            tri_pts.append([lon, lat, elev])

        features.append({
            "type": "Feature",
            "folder": "TLS Hardware Subsystems",
            "geometry": {"type": "Polygon", "coordinates": [tri_pts]},
            "properties": {"name": "3D Multilateration Tracking Triangle", "folder": "TLS Hardware Subsystems"},
        })

        # 3. Conex Shelter 4-corner bounding box
        conex_cartesian = self.get_conex_bounding_box()
        conex_coords = []
        for c in conex_cartesian + [conex_cartesian[0]]:
            lat, lon, elev = cartesian_to_geodetic(
                c[0], c[1], c[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            conex_coords.append([lon, lat, elev])

        features.append({
            "type": "Feature",
            "folder": "Conex Shelter",
            "geometry": {"type": "Polygon", "coordinates": [conex_coords]},
            "properties": {
                "name": "Conex Shelter Bounding Box",
                "folder": "Conex Shelter",
                "orientation_deg": self.shelter_angle_deg,
            },
        })

        # 4. Physical Cable Runs
        cable_pairs = [
            ("Shelter->ESA", "Shelter", "ESA", 70.0),
            ("Shelter->ASA", "Shelter", "ASA", 100.0),
            ("Shelter->Cal/BIT", "Shelter", "Cal/BIT", 150.0),
            ("ASA->ATA", "ASA", "ATA", 100.0),
            ("Shelter->Interrogator", "Shelter", "Interrogator", 13.0),
        ]

        for cable_name, p1_key, p2_key, limit in cable_pairs:
            pos1 = self.positions[p1_key]
            pos2 = self.positions[p2_key]
            lat1, lon1, elev1 = cartesian_to_geodetic(
                pos1[0], pos1[1], pos1[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            lat2, lon2, elev2 = cartesian_to_geodetic(
                pos2[0], pos2[1], pos2[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            cable_len = self.cable_lengths[cable_name]
            desc = f"{cable_len:.1f}m / {limit:.1f}m Reel Limit"

            features.append({
                "type": "Feature",
                "folder": "Physical Cable Runs",
                "geometry": {"type": "LineString", "coordinates": [[lon1, lat1, elev1], [lon2, lat2, elev2]]},
                "properties": {
                    "name": cable_name,
                    "folder": "Physical Cable Runs",
                    "description": desc,
                    "length_m": cable_len,
                    "limit_m": limit,
                },
            })

        # 5. Protected Sensor Critical Boundaries (ASA FCBAB 700ft / 213.36m, ESA FCBEB 400ft / 121.92m)
        asa_pos = self.positions["ASA"]
        asa_lat, asa_lon, asa_elev = cartesian_to_geodetic(
            asa_pos[0], asa_pos[1], asa_pos[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
        )
        features.append({
            "type": "Feature",
            "folder": "Protected Sensor Critical Boundaries",
            "geometry": {"type": "Point", "coordinates": [asa_lon, asa_lat, asa_elev]},
            "properties": {
                "name": "ASA FCBAB Critical Boundary",
                "folder": "Protected Sensor Critical Boundaries",
                "radius_ft": 700.0,
                "radius_m": 213.36,
            },
        })

        esa_pos = self.positions["ESA"]
        esa_lat, esa_lon, esa_elev = cartesian_to_geodetic(
            esa_pos[0], esa_pos[1], esa_pos[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
        )
        features.append({
            "type": "Feature",
            "folder": "Protected Sensor Critical Boundaries",
            "geometry": {"type": "Point", "coordinates": [esa_lon, esa_lat, esa_elev]},
            "properties": {
                "name": "ESA FCBEB Critical Boundary",
                "folder": "Protected Sensor Critical Boundaries",
                "radius_ft": 400.0,
                "radius_m": 121.92,
            },
        })

        # 6. Airfield Safety Zones
        features.append({
            "type": "Feature",
            "folder": "Airfield Safety Zones",
            "geometry": {"type": "Point", "coordinates": [self.lon0, self.lat0, self.elev0_m]},
            "properties": {
                "name": "Runway Safety Area & OFZ Zone",
                "folder": "Airfield Safety Zones",
                "rsa_width_ft": 500.0,
                "ofz_width_ft": 400.0,
            },
        })

        geojson_obj = {"type": "FeatureCollection", "features": features}
        return json.dumps(geojson_obj, indent=2)

    def export_kml(self) -> str:
        """
        Exports compliant 3D KML representation of TLS site layout with structured folders and dynamic metadata.
        """
        kml_ns = "http://www.opengis.net/kml/2.2"
        ET.register_namespace("", kml_ns)

        kml = ET.Element(f"{{{kml_ns}}}kml")
        doc = ET.SubElement(kml, f"{{{kml_ns}}}Document")
        doc_name = ET.SubElement(doc, f"{{{kml_ns}}}name")
        doc_name.text = "TLS Siting Engine Output"

        folder_names = [
            "Virtual Reference & Approach Fixes",
            "TLS Hardware Subsystems",
            "Conex Shelter",
            "Physical Cable Runs",
            "Protected Sensor Critical Boundaries",
            "Airfield Safety Zones",
        ]

        folders: Dict[str, ET.Element] = {}
        for fname in folder_names:
            f_elem = ET.SubElement(doc, f"{{{kml_ns}}}Folder")
            f_title = ET.SubElement(f_elem, f"{{{kml_ns}}}name")
            f_title.text = fname
            folders[fname] = f_elem

        # Folder 1: Virtual Reference & Approach Fixes
        td_m = calculate_touchdown_aiming_point(self.tch_ft, self.gpa_deg, self.runway_dz_ft)
        fixes = {
            "Point T": (0.0, 0.0, 0.0),
            "TD": (td_m, 0.0, 0.0),
            "FTM": (td_m + 300.0, 0.0, 0.0),
            "Point C": (-1100.0, 0.0, 0.0),
            "Point B": (-6000.0, 0.0, 0.0),
            "Point A": (-18000.0, 0.0, 0.0),
            "VLP": (td_m, 0.0, 0.0),
        }

        f1 = folders["Virtual Reference & Approach Fixes"]
        for name, pos in fixes.items():
            lat, lon, elev = cartesian_to_geodetic(
                pos[0], pos[1], pos[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            pm = ET.SubElement(f1, f"{{{kml_ns}}}Placemark")
            pname = ET.SubElement(pm, f"{{{kml_ns}}}name")
            pname.text = name
            pdesc = ET.SubElement(pm, f"{{{kml_ns}}}description")
            pdesc.text = f"Fix {name} at Elevation {elev:.2f}m"

            pt = ET.SubElement(pm, f"{{{kml_ns}}}Point")
            alt_mode = ET.SubElement(pt, f"{{{kml_ns}}}altitudeMode")
            alt_mode.text = "absolute"
            coords = ET.SubElement(pt, f"{{{kml_ns}}}coordinates")
            coords.text = f"{lon:.8f},{lat:.8f},{elev:.2f}"

        # Folder 2: TLS Hardware Subsystems
        f2 = folders["TLS Hardware Subsystems"]
        heights = {
            "ASA": 1.5,
            "ESA": 4.6,
            "ATA": 1.5,
            "Cal/BIT": 1.8,
            "Shelter": 2.6,
            "Interrogator": 6.1,
        }

        for name, pos in self.positions.items():
            lat, lon, elev = cartesian_to_geodetic(
                pos[0], pos[1], pos[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            pm = ET.SubElement(f2, f"{{{kml_ns}}}Placemark")
            pname = ET.SubElement(pm, f"{{{kml_ns}}}name")
            pname.text = name
            pdesc = ET.SubElement(pm, f"{{{kml_ns}}}description")
            pdesc.text = f"{name} Hardware Subsystem - Height {heights.get(name, 0.0)}m AGL, Elev {elev:.2f}m"

            pt = ET.SubElement(pm, f"{{{kml_ns}}}Point")
            alt_mode = ET.SubElement(pt, f"{{{kml_ns}}}altitudeMode")
            alt_mode.text = "absolute"
            coords = ET.SubElement(pt, f"{{{kml_ns}}}coordinates")
            coords.text = f"{lon:.8f},{lat:.8f},{elev:.2f}"

        # Multilateration Tracking Triangle Polygon
        tri_pm = ET.SubElement(f2, f"{{{kml_ns}}}Placemark")
        tri_name = ET.SubElement(tri_pm, f"{{{kml_ns}}}name")
        tri_name.text = "3D Multilateration Tracking Triangle"

        poly = ET.SubElement(tri_pm, f"{{{kml_ns}}}Polygon")
        alt_mode = ET.SubElement(poly, f"{{{kml_ns}}}altitudeMode")
        alt_mode.text = "absolute"
        outer = ET.SubElement(poly, f"{{{kml_ns}}}outerBoundaryIs")
        lr = ET.SubElement(outer, f"{{{kml_ns}}}LinearRing")
        tri_coords = ET.SubElement(lr, f"{{{kml_ns}}}coordinates")

        coord_strs = []
        for name in ["ASA", "ESA", "ATA", "ASA"]:
            pos = self.positions[name]
            lat, lon, elev = cartesian_to_geodetic(
                pos[0], pos[1], pos[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            coord_strs.append(f"{lon:.8f},{lat:.8f},{elev:.2f}")
        tri_coords.text = " ".join(coord_strs)

        # Folder 3: Conex Shelter
        f3 = folders["Conex Shelter"]
        conex_pm = ET.SubElement(f3, f"{{{kml_ns}}}Placemark")
        cname = ET.SubElement(conex_pm, f"{{{kml_ns}}}name")
        cname.text = "Conex Shelter Bounding Box"
        cdesc = ET.SubElement(conex_pm, f"{{{kml_ns}}}description")
        cdesc.text = f"Conex Shelter Bounding Box oriented at {self.shelter_angle_deg:.1f}° relative to centerline"

        cpoly = ET.SubElement(conex_pm, f"{{{kml_ns}}}Polygon")
        alt_mode = ET.SubElement(cpoly, f"{{{kml_ns}}}altitudeMode")
        alt_mode.text = "absolute"
        couter = ET.SubElement(cpoly, f"{{{kml_ns}}}outerBoundaryIs")
        clr = ET.SubElement(couter, f"{{{kml_ns}}}LinearRing")
        ccoords = ET.SubElement(clr, f"{{{kml_ns}}}coordinates")

        conex_cartesian = self.get_conex_bounding_box()
        ccoord_strs = []
        for c in conex_cartesian + [conex_cartesian[0]]:
            lat, lon, elev = cartesian_to_geodetic(
                c[0], c[1], c[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            ccoord_strs.append(f"{lon:.8f},{lat:.8f},{elev:.2f}")
        ccoords.text = " ".join(ccoord_strs)

        # Folder 4: Physical Cable Runs
        f4 = folders["Physical Cable Runs"]
        cable_pairs = [
            ("Shelter->ESA", "Shelter", "ESA", 70.0),
            ("Shelter->ASA", "Shelter", "ASA", 100.0),
            ("Shelter->Cal/BIT", "Shelter", "Cal/BIT", 150.0),
            ("ASA->ATA", "ASA", "ATA", 100.0),
            ("Shelter->Interrogator", "Shelter", "Interrogator", 13.0),
        ]

        for cable_name, p1_key, p2_key, limit in cable_pairs:
            pos1 = self.positions[p1_key]
            pos2 = self.positions[p2_key]
            lat1, lon1, elev1 = cartesian_to_geodetic(
                pos1[0], pos1[1], pos1[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            lat2, lon2, elev2 = cartesian_to_geodetic(
                pos2[0], pos2[1], pos2[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
            )
            cable_len = self.cable_lengths[cable_name]

            c_pm = ET.SubElement(f4, f"{{{kml_ns}}}Placemark")
            c_name = ET.SubElement(c_pm, f"{{{kml_ns}}}name")
            c_name.text = cable_name
            c_desc = ET.SubElement(c_pm, f"{{{kml_ns}}}description")
            c_desc.text = f"{cable_len:.1f}m / {limit:.1f}m Reel Limit"

            ls = ET.SubElement(c_pm, f"{{{kml_ns}}}LineString")
            alt_mode = ET.SubElement(ls, f"{{{kml_ns}}}altitudeMode")
            alt_mode.text = "absolute"
            lcoords = ET.SubElement(ls, f"{{{kml_ns}}}coordinates")
            lcoords.text = f"{lon1:.8f},{lat1:.8f},{elev1:.2f} {lon2:.8f},{lat2:.8f},{elev2:.2f}"

        # Folder 5: Protected Sensor Critical Boundaries
        f5 = folders["Protected Sensor Critical Boundaries"]
        asa_pos = self.positions["ASA"]
        asa_lat, asa_lon, asa_elev = cartesian_to_geodetic(
            asa_pos[0], asa_pos[1], asa_pos[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
        )
        b1_pm = ET.SubElement(f5, f"{{{kml_ns}}}Placemark")
        b1_name = ET.SubElement(b1_pm, f"{{{kml_ns}}}name")
        b1_name.text = "ASA FCBAB Critical Boundary"
        b1_desc = ET.SubElement(b1_pm, f"{{{kml_ns}}}description")
        b1_desc.text = "700 ft (213.36 m) ASA Critical Boundary"
        b1_pt = ET.SubElement(b1_pm, f"{{{kml_ns}}}Point")
        ET.SubElement(b1_pt, f"{{{kml_ns}}}altitudeMode").text = "absolute"
        ET.SubElement(b1_pt, f"{{{kml_ns}}}coordinates").text = f"{asa_lon:.8f},{asa_lat:.8f},{asa_elev:.2f}"

        esa_pos = self.positions["ESA"]
        esa_lat, esa_lon, esa_elev = cartesian_to_geodetic(
            esa_pos[0], esa_pos[1], esa_pos[2], self.lat0, self.lon0, self.elev0_m, self.hdg_deg
        )
        b2_pm = ET.SubElement(f5, f"{{{kml_ns}}}Placemark")
        b2_name = ET.SubElement(b2_pm, f"{{{kml_ns}}}name")
        b2_name.text = "ESA FCBEB Critical Boundary"
        b2_desc = ET.SubElement(b2_pm, f"{{{kml_ns}}}description")
        b2_desc.text = "400 ft (121.92 m) ESA Critical Boundary"
        b2_pt = ET.SubElement(b2_pm, f"{{{kml_ns}}}Point")
        ET.SubElement(b2_pt, f"{{{kml_ns}}}altitudeMode").text = "absolute"
        ET.SubElement(b2_pt, f"{{{kml_ns}}}coordinates").text = f"{esa_lon:.8f},{esa_lat:.8f},{esa_elev:.2f}"

        # Folder 6: Airfield Safety Zones
        f6 = folders["Airfield Safety Zones"]
        z_pm = ET.SubElement(f6, f"{{{kml_ns}}}Placemark")
        z_name = ET.SubElement(z_pm, f"{{{kml_ns}}}name")
        z_name.text = "Runway Safety Area & OFZ Zone"
        z_desc = ET.SubElement(z_pm, f"{{{kml_ns}}}description")
        z_desc.text = "Runway Safety Area (500 ft) & OFZ Buffer (400 ft)"
        z_pt = ET.SubElement(z_pm, f"{{{kml_ns}}}Point")
        ET.SubElement(z_pt, f"{{{kml_ns}}}altitudeMode").text = "absolute"
        ET.SubElement(z_pt, f"{{{kml_ns}}}coordinates").text = f"{self.lon0:.8f},{self.lat0:.8f},{self.elev0_m:.2f}"

        return ET.tostring(kml, encoding="utf-8", xml_declaration=True).decode("utf-8")
