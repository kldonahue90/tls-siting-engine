# ANPC TLS Siting Engine - Agent System Directives

## Domain & System Overview
This repository implements the precision ground hardware siting, digital elevation model (DEM) terrain auditing, and obstacle clearance evaluation engine for the Advanced Navigation & Positioning Corporation (ANPC) Transponder Landing System (TLS / TTLS). All algorithms must strictly adhere to ANPC Field Service Manual (FSM) Doc #020-00103 Rev C, FAA Order 8200.47, FAA Order 8260.3G (TERPS), ICAO Annex 14, and ICAO Doc 8168 (PANS-OPS) Volume II.

## Non-Negotiable Physical & Geometric Laws
1. **Coordinate Reference System:**
   - Origin (0, 0, 0) is always the Landing Threshold Point (LTP / Point T).
   - +X is longitudinal downwind along the landing runway centerline.
   - +Y is lateral offset to the right of centerline; -Y is lateral offset to the left of centerline.
   - +Z is orthometric height above threshold elevation.
   - Siting mode can be 'left' (all -Y), 'right' (all +Y), or 'split' (ASA/ATA left, ESA/Shelter right).

2. **Guidance Termination at Threshold:**
   - TLS guidance beams are calculated virtually for a tracked transponder reply (Mode 3A/C or Mode S). Precision guidance terminates automatically when the aircraft crosses the runway threshold (LTP). The TLS does NOT guide missed approaches or departures.

3. **Virtual Emanation Points:**
   - Guidance signals emanate from computed virtual points on the runway, not physical antenna towers. All flight inspection points (Point A, B, C, T, VLP, GPI) are referenced to these virtual coordinates.

4. **Hardware Mounting Heights & Offsets:**
   - Azimuth Sensor Array (ASA): Quadpod beam phase center mounted at exactly 4.9 ft (1.5 m) AGL. Nominal lateral offset: 60.0 m.
   - Elevation Sensor Array (ESA): Triangular aluminum lattice tower midpoint phase center mounted at exactly 15.1 ft (4.6 m) AGL. Nominal lateral offset: 160.0 m (or site boundary limit >= 100.0 m).
   - Calibration / Built-In Test (Cal/BIT): Mounted at 6.0 ft (1.8 m) AGL, sited 110.0 m to 140.0 m upwind from the ASA along the lateral offset line, aimed directly at the ESA.
   - Electronics Conex Shelter: Must be oriented at an angle between 30.0° and 40.0° relative to runway centerline to eliminate 1090 MHz specular multipath.
   - Interrogator Mast: Top of tower at 20.0 ft (6.1 m) AGL, sited 11.0 m to 13.0 m from shelter.

5. **Multilateration Baseline & ATA Placement:**
   - The Alternate Time of Arrival (ATA) antenna must form a tracking triangle with ASA and ESA, oriented upwind toward approaching aircraft (-X half-plane / approach service volume).
   - ATA position must be calculated using the perpendicular unit normal to the ASA-to-ESA baseline pointing into the -X half-plane. Never use scalar Y-axis offsets.
   - Mathematical assertion required: `dist(ASA, ATA)` and `dist(ESA, ATA)` must match equilateral or authorized baseline bounds within 0.01 m.
   - Physical cable budget ceiling: `dist(ASA, ATA) <= 100.0 m` strictly enforced.

6. **Cable Reel Budget Ceilings:**
   - Shelter -> ESA: <= 70.0 m
   - Shelter -> ASA: <= 100.0 m
   - Shelter -> Cal/BIT: <= 150.0 m
   - ASA -> ATA: <= 100.0 m
   - Shelter -> Interrogator: <= 13.0 m

7. **Vertical Terrain Clearances & Tolerances:**
   - Cal/BIT to ESA Optical Sightline: Ground terrain between Cal/BIT (1.8 m AGL) and ESA (4.6 m AGL) must not rise > 6.0 ft (1.8 m) above their straight-line baseline chord.
   - ASA Ground Drop vs. Runway Crown: Ground elevation under the ASA mast must not drop > 4.0 ft (1.2 m) below runway centerline elevation.
   - ESA Guy Anchor Slope: Guy anchors (21.0 ft / 6.4 m radius at 120° spacing) must have a terrain slope <= 10% (5.7°) across the anchor footprint to prevent tower tilt exceeding the ±0.11° operating limit.
   - Runway Gradient Correction: Touchdown point distance must apply gradient correction: TD_dist = (TCH - dz) / tan(GPA).

8. **Coding & Verification Standards:**
   - Dynamic Metadata: Never hardcode cable distances, elevations, or coordinates into placemark strings or metadata. Always compute dynamically via f-strings.
   - Packages: Standard library (`math`, `json`, `xml.etree.ElementTree`), `shapely`, `pyproj`, `rasterio`, `numpy`.
   - Test Coverage: 100% test coverage using `pytest`. Every PR must pass all assertion checks.
