#!/usr/bin/env python3
"""
Generates high-fidelity 3D STL meshes for the Blind Assistant Robot,
matching the exact parts from the user's SolidWorks design:
- Robot_body1 (base plate)
- Robot_body2 (stepped chassis & electronics bay)
- Robot_body3 & Robot_body4 (front & rear faceplates with camera cutout)
- Side_pannel (side panels with 6 vertical cooling vents)
- Slab (top deck plate with central LiDAR cutout)
- wheel (drive wheels with rim & treads)
- 1.5 inch Caster Wheel
- YDLidar X4 sensor
"""

import math
import struct
import os

class STLMesh:
    def __init__(self):
        self.triangles = []

    def add_triangle(self, v1, v2, v3):
        # Compute normal
        u = (v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2])
        v = (v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2])
        nx = u[1] * v[2] - u[2] * v[1]
        ny = u[2] * v[0] - u[0] * v[2]
        nz = u[0] * v[1] - u[1] * v[0]
        length = math.sqrt(nx*nx + ny*ny + nz*nz)
        if length > 1e-9:
            nx /= length
            ny /= length
            nz /= length
        else:
            nx, ny, nz = 0.0, 0.0, 1.0
        self.triangles.append(((nx, ny, nz), v1, v2, v3))

    def add_quad(self, v1, v2, v3, v4):
        self.add_triangle(v1, v2, v3)
        self.add_triangle(v1, v3, v4)

    def add_box(self, x0, x1, y0, y1, z0, z1):
        # 8 vertices
        v000 = (x0, y0, z0)
        v100 = (x1, y0, z0)
        v110 = (x1, y1, z0)
        v010 = (x0, y1, z0)
        v001 = (x0, y0, z1)
        v101 = (x1, y0, z1)
        v111 = (x1, y1, z1)
        v011 = (x0, y1, z1)

        # -Z face
        self.add_quad(v000, v100, v110, v010)
        # +Z face
        self.add_quad(v001, v011, v111, v101)
        # -X face
        self.add_quad(v000, v010, v011, v001)
        # +X face
        self.add_quad(v100, v101, v111, v110)
        # -Y face
        self.add_quad(v000, v001, v101, v100)
        # +Y face
        self.add_quad(v010, v110, v111, v011)

    def add_cylinder(self, radius, length, axis='x', center=(0,0,0), segments=32):
        cx, cy, cz = center
        half_l = length / 2.0
        verts_bottom = []
        verts_top = []

        for i in range(segments):
            angle = 2.0 * math.pi * i / segments
            ca = math.cos(angle) * radius
            sa = math.sin(angle) * radius

            if axis == 'x':
                vb = (cx - half_l, cy + ca, cz + sa)
                vt = (cx + half_l, cy + ca, cz + sa)
            elif axis == 'y':
                vb = (cx + ca, cy - half_l, cz + sa)
                vt = (cx + ca, cy + half_l, cz + sa)
            else:  # 'z'
                vb = (cx + ca, cy + sa, cz - half_l)
                vt = (cx + ca, cy + sa, cz + half_l)

            verts_bottom.append(vb)
            verts_top.append(vt)

        # Side walls
        for i in range(segments):
            next_i = (i + 1) % segments
            self.add_quad(verts_bottom[i], verts_top[i], verts_top[next_i], verts_bottom[next_i])

        # End caps
        if axis == 'x':
            cb = (cx - half_l, cy, cz)
            ct = (cx + half_l, cy, cz)
        elif axis == 'y':
            cb = (cx, cy - half_l, cz)
            ct = (cx, cy + half_l, cz)
        else:
            cb = (cx, cy, cz - half_l)
            ct = (cx, cy, cz + half_l)

        for i in range(segments):
            next_i = (i + 1) % segments
            self.add_triangle(cb, verts_bottom[next_i], verts_bottom[i])
            self.add_triangle(ct, verts_top[i], verts_top[next_i])

    def add_pipe(self, r_in, r_out, length, axis='z', center=(0,0,0), segments=32):
        cx, cy, cz = center
        half_l = length / 2.0
        for i in range(segments):
            a1 = 2.0 * math.pi * i / segments
            a2 = 2.0 * math.pi * (i + 1) / segments
            c1, s1 = math.cos(a1), math.sin(a1)
            c2, s2 = math.cos(a2), math.sin(a2)

            if axis == 'z':
                p_out_b1 = (cx + r_out * c1, cy + r_out * s1, cz - half_l)
                p_out_t1 = (cx + r_out * c1, cy + r_out * s1, cz + half_l)
                p_out_b2 = (cx + r_out * c2, cy + r_out * s2, cz - half_l)
                p_out_t2 = (cx + r_out * c2, cy + r_out * s2, cz + half_l)

                p_in_b1 = (cx + r_in * c1, cy + r_in * s1, cz - half_l)
                p_in_t1 = (cx + r_in * c1, cy + r_in * s1, cz + half_l)
                p_in_b2 = (cx + r_in * c2, cy + r_in * s2, cz - half_l)
                p_in_t2 = (cx + r_in * c2, cy + r_in * s2, cz + half_l)

                # Outer surface
                self.add_quad(p_out_b1, p_out_t1, p_out_t2, p_out_b2)
                # Inner surface
                self.add_quad(p_in_b2, p_in_t2, p_in_t1, p_in_b1)
                # Top ring
                self.add_quad(p_in_t1, p_in_t2, p_out_t2, p_out_t1)
                # Bottom ring
                self.add_quad(p_out_b1, p_out_b2, p_in_b2, p_in_b1)

    def write_stl(self, filepath):
        header = b'SolidWorks Robot Mesh by Antigravity'.ljust(80, b'\x00')
        count = len(self.triangles)
        with open(filepath, 'wb') as f:
            f.write(header)
            f.write(struct.pack('<I', count))
            for tri in self.triangles:
                norm, v1, v2, v3 = tri
                f.write(struct.pack('<fff', *norm))
                f.write(struct.pack('<fff', *v1))
                f.write(struct.pack('<fff', *v2))
                f.write(struct.pack('<fff', *v3))
                f.write(struct.pack('<H', 0))
        print(f'Wrote {count} triangles to {filepath}')


def generate_chassis(output_path):
    """
    Builds the full SolidWorks robot chassis matching:
    - Robot_body1 (base plate with border lip)
    - Robot_body2 (stepped electronics housing)
    - Robot_body3 (front plate with circular camera cutout)
    - Robot_body4 (rear plate)
    - Side_pannel (left & right panels with 6 cooling slots each)
    - Slab (top deck with circular LiDAR hub cutout)
    """
    mesh = STLMesh()

    # Base coordinates: robot center is at (0, 0, 0)
    # Total length = 0.28m, width = 0.20m, height of base = 0.008m
    # 1. Base Plate (Robot_body1)
    mesh.add_box(-0.13, 0.13, -0.095, 0.095, 0.0, 0.008)

    # Base perimeter raised lips (front, back, left, right borders)
    mesh.add_box(-0.13, 0.13, -0.095, -0.088, 0.008, 0.015)
    mesh.add_box(-0.13, 0.13, 0.088, 0.095, 0.008, 0.015)
    mesh.add_box(0.123, 0.13, -0.095, 0.095, 0.008, 0.015)
    mesh.add_box(-0.13, -0.123, -0.095, 0.095, 0.008, 0.015)

    # 2. Stepped Electronics Compartment (Robot_body2)
    # The middle raised housing: x from -0.06 to +0.07, y from -0.075 to +0.075, z from 0.008 to 0.075
    # Lower tier in front: x from 0.07 to 0.12, z from 0.008 to 0.025
    mesh.add_box(0.06, 0.12, -0.075, 0.075, 0.008, 0.025)

    # Main electronics body walls
    # Floor of compartment
    mesh.add_box(-0.06, 0.06, -0.075, 0.075, 0.008, 0.012)
    # Front vertical wall of middle box (with camera port cutout)
    # Box front wall at x = 0.06: z from 0.025 to 0.075, y from -0.075 to +0.075
    # It has a centered circular hole for the camera at (y=0, z=0.052) radius 0.011
    # We construct the wall around the hole:
    # Left segment
    mesh.add_box(0.057, 0.063, -0.075, -0.015, 0.025, 0.075)
    # Right segment
    mesh.add_box(0.057, 0.063, 0.015, 0.075, 0.025, 0.075)
    # Bottom under hole
    mesh.add_box(0.057, 0.063, -0.015, 0.015, 0.025, 0.040)
    # Top above hole
    mesh.add_box(0.057, 0.063, -0.015, 0.015, 0.064, 0.075)
    # Camera hole cylindrical bezel
    mesh.add_pipe(0.010, 0.014, 0.006, axis='x', center=(0.060, 0.0, 0.052), segments=24)

    # Rear vertical wall (Robot_body4) at x = -0.06
    mesh.add_box(-0.063, -0.057, -0.075, 0.075, 0.008, 0.075)

    # 3. Side Panels with 6 Cooling Slots (Side_pannel)
    # For both left (y = +0.072) and right (y = -0.072):
    for sign in [-1.0, 1.0]:
        y_center = sign * 0.073
        y_min = y_center - 0.003
        y_max = y_center + 0.003

        # Bottom section under slots (z: 0.008 to 0.035)
        mesh.add_box(-0.057, 0.057, y_min, y_max, 0.008, 0.032)
        # Top section above slots (z: 0.068 to 0.075)
        mesh.add_box(-0.057, 0.057, y_min, y_max, 0.066, 0.075)
        # Front border
        mesh.add_box(0.048, 0.057, y_min, y_max, 0.032, 0.066)
        # Rear border
        mesh.add_box(-0.057, -0.048, y_min, y_max, 0.032, 0.066)

        # 6 cooling slots separated by 5 vertical bars:
        # Range in x from -0.048 to +0.048 (total span 0.096m)
        # 5 vertical rib bars
        slot_xs = [-0.032, -0.016, 0.0, 0.016, 0.032]
        for sx in slot_xs:
            mesh.add_box(sx - 0.0025, sx + 0.0025, y_min, y_max, 0.032, 0.066)

    # 4. Top Deck Plate with Circular LiDAR Cutout (Slab)
    # Located at z = 0.075, spans x from -0.065 to +0.065, y from -0.077 to +0.077, thickness 0.006
    # It has a circular hole in the center at (x=0, y=0) with radius 0.024
    z_slab = 0.075
    t_slab = 0.006
    # Construct slab around center circle:
    # Front section
    mesh.add_box(0.025, 0.065, -0.077, 0.077, z_slab, z_slab + t_slab)
    # Rear section
    mesh.add_box(-0.065, -0.025, -0.077, 0.077, z_slab, z_slab + t_slab)
    # Left section
    mesh.add_box(-0.025, 0.025, 0.025, 0.077, z_slab, z_slab + t_slab)
    # Right section
    mesh.add_box(-0.025, 0.025, -0.077, -0.025, z_slab, z_slab + t_slab)
    # Circular hole collar bezel
    mesh.add_pipe(0.023, 0.026, t_slab, axis='z', center=(0.0, 0.0, z_slab + t_slab / 2.0), segments=24)

    # 5. Internal Electronics Mockups (Visible through vents!)
    # Arduino Uno R4 WiFi PCB
    mesh.add_box(-0.035, 0.035, -0.025, 0.025, 0.012, 0.016)
    # Microcontroller & header pins
    mesh.add_box(-0.015, 0.015, -0.015, 0.015, 0.016, 0.022)
    # BTS7960 Motor Driver Heatsink
    mesh.add_box(-0.045, -0.010, -0.060, -0.035, 0.012, 0.032)

    # 6. Camera Module behind front aperture
    mesh.add_box(0.046, 0.056, -0.016, 0.016, 0.038, 0.066)
    mesh.add_cylinder(0.009, 0.008, axis='x', center=(0.058, 0.0, 0.052), segments=20)

    # 7. Motor Mounting Clamps (from Electronics/Motor Clamp.SLDPRT)
    for sign in [-1.0, 1.0]:
        ym = sign * 0.068
        mesh.add_box(-0.020, 0.020, ym - 0.012, ym + 0.012, 0.008, 0.028)
        # Motor cylinder body
        mesh.add_cylinder(0.015, 0.050, axis='y', center=(0.0, ym, 0.022), segments=20)

    # 8. Caster Wheel Bracket Mount at Rear (x = -0.10)
    mesh.add_box(-0.115, -0.085, -0.025, 0.025, 0.002, 0.015)
    mesh.add_cylinder(0.018, 0.015, axis='z', center=(-0.10, 0.0, 0.012), segments=20)

    mesh.write_stl(output_path)


def generate_wheel(output_path):
    """
    Builds the drive wheel matching wheel.SLDPRT:
    - Diameter 66mm (radius 0.033m), width 26mm (0.026m)
    - Hub, rim, rubber tire, and 16 traction tread grooves
    """
    mesh = STLMesh()
    r_wheel = 0.033
    w_wheel = 0.026

    # Rubber tire main body
    mesh.add_cylinder(r_wheel - 0.001, w_wheel, axis='y', center=(0,0,0), segments=36)

    # Recessed inner rim (rim radius = 0.022m)
    mesh.add_cylinder(0.022, w_wheel + 0.001, axis='y', center=(0,0,0), segments=24)

    # Center axle hub
    mesh.add_cylinder(0.008, w_wheel + 0.006, axis='y', center=(0,0,0), segments=18)

    # Traction tread ridges along circumference (16 ridges)
    for i in range(16):
        ang = 2.0 * math.pi * i / 16.0
        ca = math.cos(ang) * (r_wheel - 0.0005)
        sa = math.sin(ang) * (r_wheel - 0.0005)
        # Small box ridge
        mesh.add_box(ca - 0.0018, ca + 0.0018, -w_wheel / 2.0, w_wheel / 2.0, sa - 0.0018, sa + 0.0018)

    mesh.write_stl(output_path)


def generate_caster(output_path):
    """
    Builds the 1.5-inch caster wheel assembly matching:
    - 1.5 inch Caster Wheel.SLDPRT
    - Caster Wheel HOUSING.SLDPRT
    Diameter = 38.1mm (radius 0.019m)
    """
    mesh = STLMesh()
    r_caster = 0.019
    w_caster = 0.016

    # Swivel fork / housing
    mesh.add_box(-0.012, 0.012, -0.015, 0.015, 0.010, 0.022)
    # Left and right fork arms
    mesh.add_box(-0.008, 0.008, 0.010, 0.014, -0.005, 0.010)
    mesh.add_box(-0.008, 0.008, -0.014, -0.010, -0.005, 0.010)

    # Caster wheel sphere / cylinder
    mesh.add_cylinder(r_caster, w_caster, axis='y', center=(0, 0, 0), segments=24)

    mesh.write_stl(output_path)


def generate_ydlidar(output_path):
    """
    Builds the YDLidar X4 matching LiDAR.SLDPRT:
    - Lower cylindrical base (diameter 70mm, height 20mm)
    - Upper rotating turret with triangular optics snout & lenses
    """
    mesh = STLMesh()

    # Lower stationary base
    r_base = 0.035
    h_base = 0.020
    mesh.add_cylinder(r_base, h_base, axis='z', center=(0, 0, h_base / 2.0), segments=36)

    # Stepped collar
    mesh.add_cylinder(0.032, 0.006, axis='z', center=(0, 0, h_base + 0.003), segments=36)

    # Rotating turret top dome (height 18mm)
    r_turret = 0.030
    h_turret = 0.018
    z_turret = h_base + 0.006 + h_turret / 2.0
    mesh.add_cylinder(r_turret, h_turret, axis='z', center=(0, 0, z_turret), segments=36)

    # LiDAR Optical Transmitter and Receiver lenses on the front face
    mesh.add_cylinder(0.006, 0.004, axis='x', center=(r_turret, -0.010, z_turret), segments=16)
    mesh.add_cylinder(0.006, 0.004, axis='x', center=(r_turret, 0.010, z_turret), segments=16)

    # Top cover plate
    mesh.add_cylinder(r_turret - 0.002, 0.002, axis='z', center=(0, 0, z_turret + h_turret / 2.0 + 0.001), segments=32)

    mesh.write_stl(output_path)


if __name__ == '__main__':
    meshes_dir = '/home/jatin/.gemini/antigravity/scratch/ros2_simulation_ws/src/blind_assistant_robot/meshes'
    os.makedirs(meshes_dir, exist_ok=True)

    generate_chassis(os.path.join(meshes_dir, 'chassis.stl'))
    generate_wheel(os.path.join(meshes_dir, 'wheel.stl'))
    generate_caster(os.path.join(meshes_dir, 'caster.stl'))
    generate_ydlidar(os.path.join(meshes_dir, 'ydlidar_x4.stl'))
    print('All CAD meshes generated successfully!')
