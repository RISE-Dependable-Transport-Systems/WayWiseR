"""Top-view vehicle overlay helpers for the Control Tower map."""

from dataclasses import dataclass, field
import math
import os
import struct
import xml.etree.ElementTree as ET

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


MAX_MESH_OUTLINE_VERTICES = 6000


@dataclass
class Transform3D:
    """Rigid transform matching URDF xyz/rpy semantics."""

    xyz: tuple = (0.0, 0.0, 0.0)
    rotation: tuple = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )


@dataclass
class OverlayShape:
    """A top-view visual shape in vehicle base coordinates."""

    kind: str
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    width: float = 0.0
    length: float = 0.0
    color: str = '#111827'
    points: list = field(default_factory=list)


class VehicleOverlayModel:
    """Parse URDF visuals into top-view projected geometry."""

    def __init__(self):
        self.links = {}
        self.children_by_parent = {}
        self.root_link = None
        self.joint_positions = {}
        self.shapes = []
        self.mesh_cache = {}
        self.materials = {}
        self.joint_types = {}
        self.ignored_joint_names = set()
        self._rotor_joint_names = []  # rotor/propeller joints eligible for animation
        self._rotor_animation = []  # per-rotor entry: shape_indices + origin_xy
        self.version = 0

    def load_urdf(self, xml_text):
        """Load a URDF string and rebuild static overlay data."""
        root = ET.fromstring(xml_text)
        self.materials = self._parse_materials(root)
        self.joint_types = {}
        self.ignored_joint_names = set()
        links = {link.get('name'): link for link in root.findall('link') if link.get('name')}
        joints = []
        child_links = set()

        for joint in root.findall('joint'):
            parent = joint.find('parent')
            child = joint.find('child')
            if parent is None or child is None:
                continue
            parent_name = parent.get('link')
            child_name = child.get('link')
            if not parent_name or not child_name:
                continue

            joint_type = joint.get('type', 'fixed')
            joint_name = joint.get('name', '')
            joints.append(
                {
                    'name': joint_name,
                    'type': joint_type,
                    'parent': parent_name,
                    'child': child_name,
                    'origin': self._parse_origin(joint.find('origin')),
                    'axis': self._parse_axis(joint.find('axis')),
                }
            )
            self.joint_types[joint_name] = joint_type
            if self._is_spin_joint(joint_name, child_name, links.get(child_name)):
                self.ignored_joint_names.add(joint_name)
            child_links.add(child_name)

        root_candidates = [name for name in links if name not in child_links]
        self.root_link = root_candidates[0] if root_candidates else next(iter(links), None)
        self.links = links
        self.children_by_parent = {}
        for joint in joints:
            self.children_by_parent.setdefault(joint['parent'], []).append(joint)

        # Collect joints that drive rotors/propellers (subset of ignored_joint_names)
        # so tick() can animate them independently of incoming JointState messages.
        self._rotor_joint_names = [
            j['name']
            for j in joints
            if j['name'] in self.ignored_joint_names
            and any(
                t in (j['name'] + ' ' + j['child']).lower()
                for t in ('rotor', 'propeller', 'prop_')
            )
        ]

        self.shapes = self._build_shapes()
        self.version += 1

    def update_joint_states(self, names, positions):
        changed = False
        next_positions = dict(self.joint_positions)
        for name, position in zip(names, positions):
            if self._ignore_joint_state(name):
                continue
            position = float(position)
            if abs(next_positions.get(name, 0.0) - position) > 1e-3:
                next_positions[name] = position
                changed = True
        if not changed:
            return
        self.joint_positions = next_positions
        self.shapes = self._build_shapes()
        self.version += 1

    def tick(self, time_sec, spin_hz=3.0):
        """Advance rotor/propeller animation to wall-clock time *time_sec*.

        Applies a 2-D delta rotation to the pre-computed hull vertices of every
        rotor shape.  This is O(hull_vertices) — no mesh transforms or convex-hull
        recomputation — so it is safe to call at the full display refresh rate.
        Returns True when the version was incremented.
        """
        if not self._rotor_joint_names or not self._rotor_animation:
            return False

        new_angle = (time_sec * spin_hz * 2.0 * math.pi) % (2.0 * math.pi)
        old_angle = self.joint_positions.get(self._rotor_joint_names[0], 0.0)
        delta = new_angle - old_angle
        # Skip if effectively no movement (wrap-around safe check)
        if abs(delta) < 1e-4 and abs(abs(delta) - 2.0 * math.pi) > 1e-4:
            return False

        cos_d = math.cos(delta)
        sin_d = math.sin(delta)

        for entry in self._rotor_animation:
            cx, cy = entry['origin_xy']
            for idx in entry['shape_indices']:
                if idx >= len(self.shapes):
                    continue
                shape = self.shapes[idx]
                if not hasattr(shape, 'points') or not shape.points:
                    continue
                shape.points = [
                    (
                        cx + (px - cx) * cos_d - (py - cy) * sin_d,
                        cy + (px - cx) * sin_d + (py - cy) * cos_d,
                    )
                    for px, py in shape.points
                ]

        for name in self._rotor_joint_names:
            self.joint_positions[name] = new_angle
        self.version += 1
        return True

    def _ignore_joint_state(self, name):
        if not name:
            return True
        lowered = name.lower()
        if 'fifth_wheel' in lowered:
            return False
        return (
            name in self.ignored_joint_names
            or 'rotor' in lowered
            or 'prop' in lowered
            or ('wheel' in lowered and 'steer' not in lowered)
        )

    def _is_spin_joint(self, joint_name, child_name, child_link):
        search_text = ' '.join((joint_name or '', child_name or '')).lower()
        if 'fifth_wheel' in search_text:
            return False
        if any(token in search_text for token in ('rotor', 'propeller', 'prop_')):
            return True
        if 'wheel' in search_text and 'steer' not in search_text:
            return True
        if child_link is None:
            return False

        link_name = child_link.get('name', '')
        visuals = list(child_link.findall('visual')) + list(child_link.findall('collision'))
        visual_text = ' '.join(self._visual_search_text(link_name, visual) for visual in visuals)
        if 'fifth_wheel' in visual_text:
            return False
        return any(token in visual_text for token in ('wheel', 'rotor', 'propeller', 'prop_'))

    def _build_shapes(self):
        if not self.root_link:
            return []

        shapes = []
        rotor_anim = []
        rotor_joint_set = set(self._rotor_joint_names)
        self._walk_link(self.root_link, Transform3D(), shapes, rotor_joint_set, rotor_anim)
        self._rotor_animation = rotor_anim
        return shapes

    def _walk_link(self, link_name, transform, shapes, rotor_joint_set, rotor_anim):
        link = self.links.get(link_name)
        if link is not None:
            shapes.extend(self._link_shapes(link, transform))

        for joint in self.children_by_parent.get(link_name, []):
            joint_transform = self._compose(transform, joint['origin'])
            joint_value = self.joint_positions.get(joint['name'], 0.0)
            child_transform = self._apply_joint_motion(joint_transform, joint, joint_value)

            if joint['name'] in rotor_joint_set:
                # Collect this rotor subtree's shapes separately so tick() can
                # rotate them cheaply without a full _build_shapes() call.
                start = len(shapes)
                self._walk_link(joint['child'], child_transform, shapes, set(), [])
                end = len(shapes)
                if end > start:
                    rotor_anim.append(
                        {
                            'shape_indices': list(range(start, end)),
                            'origin_xy': (joint_transform.xyz[0], joint_transform.xyz[1]),
                        }
                    )
            else:
                self._walk_link(
                    joint['child'], child_transform, shapes, rotor_joint_set, rotor_anim
                )

    def _link_shapes(self, link, parent_transform):
        shapes = []
        visuals = link.findall('visual') or link.findall('collision')
        for visual in visuals:
            geometry = visual.find('geometry')
            if geometry is None:
                continue

            local = self._parse_origin(visual.find('origin'))
            transform = self._compose(parent_transform, local)
            geometry_shapes = self._shape_from_geometry(
                link.get('name', ''), visual, geometry, transform
            )
            if isinstance(geometry_shapes, list):
                shapes.extend(geometry_shapes)
            elif geometry_shapes is not None:
                shapes.append(geometry_shapes)
        return shapes

    def _shape_from_geometry(self, link_name, visual, geometry, transform):
        color = self._visual_color(visual, '#374151')

        box = geometry.find('box')
        if box is not None and box.get('size'):
            values = [float(value) for value in box.get('size').split()[:3]]
            values += [0.1] * (3 - len(values))
            polygon = self._projected_box_polygon(transform, values)
            return OverlayShape('polygon', color=color, points=polygon)

        cylinder = geometry.find('cylinder')
        if cylinder is not None:
            radius = float(cylinder.get('radius', '0.1'))
            length = float(cylinder.get('length', str(radius * 2.0)))
            return self._projected_cylinder_shape(transform, radius, length, color)

        sphere = geometry.find('sphere')
        if sphere is not None:
            center = self._transform_point(transform, (0.0, 0.0, 0.0))
            radius = float(sphere.get('radius', '0.1'))
            return OverlayShape(
                'circle', center[0], center[1], 0.0, radius * 2.0, radius * 2.0, color
            )

        mesh = geometry.find('mesh')
        if mesh is not None:
            if self._is_wheel_visual(link_name, visual, mesh):
                polygon = self._mesh_rect_polygon(mesh, transform)
                if polygon:
                    return OverlayShape('polygon', color=color, points=polygon)
            polygon, mesh_color = self._mesh_polygon(mesh, transform)
            if polygon:
                return OverlayShape('polygon', color=mesh_color or color, points=polygon)

        return None

    def _is_wheel_visual(self, link_name, visual, mesh):
        search_text = self._visual_search_text(link_name, visual, mesh)
        return 'wheel' in search_text and 'fifth_wheel' not in search_text

    @staticmethod
    def _visual_search_text(link_name, visual, mesh=None):
        parts = [link_name or '', visual.get('name', '') if visual is not None else '']
        if mesh is None and visual is not None:
            geometry = visual.find('geometry')
            mesh = geometry.find('mesh') if geometry is not None else None
        if mesh is not None and mesh.get('filename'):
            filename = mesh.get('filename')
            parts.extend((filename, os.path.basename(filename)))
        return ' '.join(parts).lower()

    @staticmethod
    def _parse_origin(origin):
        if origin is None:
            return Transform3D()
        xyz = [0.0, 0.0, 0.0]
        rpy = [0.0, 0.0, 0.0]
        if origin.get('xyz'):
            xyz = [float(value) for value in origin.get('xyz').split()[:3]]
            xyz += [0.0] * (3 - len(xyz))
        if origin.get('rpy'):
            rpy = [float(value) for value in origin.get('rpy').split()[:3]]
            rpy += [0.0] * (3 - len(rpy))
        return Transform3D(tuple(xyz[:3]), VehicleOverlayModel._rpy_to_matrix(*rpy[:3]))

    @staticmethod
    def _parse_axis(axis):
        if axis is None or not axis.get('xyz'):
            return (0.0, 0.0, 1.0)
        values = [float(value) for value in axis.get('xyz').split()[:3]]
        values += [0.0] * (3 - len(values))
        length = math.sqrt(sum(value * value for value in values[:3]))
        if length <= 1e-9:
            return (0.0, 0.0, 1.0)
        return tuple(value / length for value in values[:3])

    @staticmethod
    def _compose(parent, child):
        return Transform3D(
            VehicleOverlayModel._add(
                parent.xyz, VehicleOverlayModel._rotate(parent.rotation, child.xyz)
            ),
            VehicleOverlayModel._matmul(parent.rotation, child.rotation),
        )

    def _apply_joint_motion(self, transform, joint, value):
        joint_type = joint['type']
        axis = joint['axis']
        if joint_type in ('revolute', 'continuous'):
            return self._compose(
                transform, Transform3D(rotation=self._axis_angle_to_matrix(axis, value))
            )
        if joint_type == 'prismatic':
            return self._compose(
                transform,
                Transform3D((axis[0] * value, axis[1] * value, axis[2] * value)),
            )
        return transform

    def _mesh_polygon(self, mesh, transform):
        color, vertices = self._cached_mesh_vertices(mesh)
        if not vertices:
            return [], ''

        points = [
            (point[0], point[1])
            for point in (self._transform_point(transform, vertex) for vertex in vertices)
        ]
        return self._convex_hull(points), color

    def _mesh_rect_polygon(self, mesh, transform):
        _, vertices = self._cached_mesh_vertices(mesh)
        if not vertices:
            return []

        points = [
            (point[0], point[1])
            for point in (self._transform_point(transform, vertex) for vertex in vertices)
        ]
        axes = self._footprint_axes(transform)
        return self._projected_rect(points, axes)

    def _cached_mesh_vertices(self, mesh):
        filename = mesh.get('filename')
        mesh_path = self._resolve_mesh_filename(filename)
        if not mesh_path:
            return '', []

        scale = self._mesh_scale(mesh)
        cache_key = (mesh_path, tuple(scale))
        if cache_key not in self.mesh_cache:
            color, vertices = self._load_mesh_vertices(mesh_path)
            if not vertices:
                self.mesh_cache[cache_key] = ('', [])
            else:
                scaled = [
                    (vertex[0] * scale[0], vertex[1] * scale[1], vertex[2] * scale[2])
                    for vertex in self._sample_vertices(vertices)
                ]
                self.mesh_cache[cache_key] = (color, scaled)

        color, vertices = self.mesh_cache.get(cache_key, ('', []))
        return color, vertices

    @staticmethod
    def _footprint_axes(transform):
        projected_axes = []
        for local_axis in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)):
            axis = VehicleOverlayModel._rotate(transform.rotation, local_axis)
            length = math.hypot(axis[0], axis[1])
            if length > 1e-6:
                projected_axes.append((length, (axis[0] / length, axis[1] / length)))

        projected_axes.sort(key=lambda item: item[0], reverse=True)
        if not projected_axes:
            return (1.0, 0.0), (0.0, 1.0)

        first_axis = projected_axes[0][1]
        for _, candidate_axis in projected_axes[1:]:
            cross = abs(first_axis[0] * candidate_axis[1] - first_axis[1] * candidate_axis[0])
            if cross > 1e-3:
                return first_axis, candidate_axis
        return first_axis, (-first_axis[1], first_axis[0])

    @staticmethod
    def _projected_rect(points, axes):
        if not points:
            return []
        axis_x, axis_y = axes
        projections_x = [point[0] * axis_x[0] + point[1] * axis_x[1] for point in points]
        projections_y = [point[0] * axis_y[0] + point[1] * axis_y[1] for point in points]
        min_x, max_x = min(projections_x), max(projections_x)
        min_y, max_y = min(projections_y), max(projections_y)
        return [
            (axis_x[0] * min_x + axis_y[0] * min_y, axis_x[1] * min_x + axis_y[1] * min_y),
            (axis_x[0] * max_x + axis_y[0] * min_y, axis_x[1] * max_x + axis_y[1] * min_y),
            (axis_x[0] * max_x + axis_y[0] * max_y, axis_x[1] * max_x + axis_y[1] * max_y),
            (axis_x[0] * min_x + axis_y[0] * max_y, axis_x[1] * min_x + axis_y[1] * max_y),
        ]

    @staticmethod
    def _mesh_scale(mesh):
        scale = [1.0, 1.0, 1.0]
        if mesh.get('scale'):
            scale = [float(value) for value in mesh.get('scale').split()[:3]]
            scale += [1.0] * (3 - len(scale))
        return scale[:3]

    @staticmethod
    def _projected_box_polygon(transform, size):
        lx, ly, lz = size[0] * 0.5, size[1] * 0.5, size[2] * 0.5
        corners = []
        for px in (-lx, lx):
            for py in (-ly, ly):
                for pz in (-lz, lz):
                    point = VehicleOverlayModel._transform_point(transform, (px, py, pz))
                    corners.append((point[0], point[1]))
        return VehicleOverlayModel._convex_hull(corners)

    @staticmethod
    def _projected_cylinder_shape(transform, radius, length, color):
        center = VehicleOverlayModel._transform_point(transform, (0.0, 0.0, 0.0))
        axis = VehicleOverlayModel._rotate(transform.rotation, (0.0, 0.0, 1.0))
        axis_xy_length = math.hypot(axis[0], axis[1])
        if axis_xy_length <= 1e-6:
            return OverlayShape(
                'circle',
                center[0],
                center[1],
                0.0,
                radius * 2.0,
                radius * 2.0,
                color,
            )

        points = []
        cap_steps = 24
        for local_z in (-length * 0.5, length * 0.5):
            for index in range(cap_steps):
                angle = 2.0 * math.pi * index / cap_steps
                point = VehicleOverlayModel._transform_point(
                    transform,
                    (radius * math.cos(angle), radius * math.sin(angle), local_z),
                )
                points.append((point[0], point[1]))
        return OverlayShape(
            'polygon', color=color, points=VehicleOverlayModel._convex_hull(points)
        )

    @staticmethod
    def _parse_materials(root):
        materials = {}
        for material in root.findall('material'):
            name = material.get('name')
            color = material.find('color')
            if name and color is not None and color.get('rgba'):
                materials[name] = VehicleOverlayModel._rgba_to_hex(color.get('rgba'))
        return materials

    def _visual_color(self, visual, default):
        material = visual.find('material')
        if material is None:
            return default
        color = material.find('color')
        if color is not None and color.get('rgba'):
            return self._rgba_to_hex(color.get('rgba'))
        name = material.get('name')
        return self.materials.get(name, default)

    @staticmethod
    def _resolve_mesh_filename(filename):
        if not filename:
            return ''
        if filename.startswith('package://'):
            package_and_path = filename[len('package://') :]
            package_name, _, relative_path = package_and_path.partition('/')
            if not package_name or not relative_path:
                return ''
            if get_package_share_directory is not None:
                try:
                    mesh_path = os.path.join(
                        get_package_share_directory(package_name),
                        relative_path,
                    )
                    if os.path.exists(mesh_path):
                        return mesh_path
                except Exception:
                    pass
            return ''
        if filename.startswith('file://'):
            filename = filename[len('file://') :]
        return filename if os.path.exists(filename) else ''

    def _load_mesh_vertices(self, mesh_path):
        extension = os.path.splitext(mesh_path)[1].lower()
        try:
            if extension == '.dae':
                return self._load_dae_vertices(mesh_path)
            if extension == '.stl':
                return '', self._load_stl_vertices(mesh_path)
        except Exception:
            return '', []
        return '', []

    @staticmethod
    def _load_dae_vertices(mesh_path):
        root = ET.parse(mesh_path).getroot()
        unit_scale = VehicleOverlayModel._dae_unit_scale(root)
        material_colors = VehicleOverlayModel._dae_material_colors(root)
        sources = VehicleOverlayModel._dae_position_sources(root, unit_scale)
        color = next((value for value in material_colors.values() if value), '')
        vertices = []
        for positions in sources.values():
            vertices.extend(positions)
        return color, vertices

    @staticmethod
    def _dae_unit_scale(root):
        for node in root.iter():
            if VehicleOverlayModel._tag_name(node) == 'unit' and node.get('meter'):
                return float(node.get('meter'))
        return 1.0

    @staticmethod
    def _dae_position_sources(root, unit_scale):
        # Collect IDs of sources referenced with semantic="POSITION" in <vertices> elements.
        # This avoids loading normal vectors, UV maps, etc. which are also stored as
        # float arrays but have values in [-1, 1] that corrupt the bounding box for
        # small-scale meshes (e.g. semitrailer wheels where positions are ~0.53 m).
        position_source_ids = set()
        for vertices_el in root.iter():
            if VehicleOverlayModel._tag_name(vertices_el) != 'vertices':
                continue
            for inp in vertices_el:
                if (
                    VehicleOverlayModel._tag_name(inp) == 'input'
                    and inp.get('semantic', '').upper() == 'POSITION'
                ):
                    source_ref = inp.get('source', '').lstrip('#')
                    if source_ref:
                        position_source_ids.add(source_ref)

        sources = {}
        for source in root.iter():
            if VehicleOverlayModel._tag_name(source) != 'source':
                continue
            source_id = source.get('id')
            if not source_id:
                continue

            # Skip sources that are not vertex positions.
            if position_source_ids:
                if source_id not in position_source_ids:
                    continue
            else:
                # Fallback when <vertices> POSITION semantic is absent: exclude obvious
                # non-position sources by ID keyword.
                sid_lower = source_id.lower()
                if any(
                    kw in sid_lower for kw in ('normal', 'texcoord', 'map', 'color', 'tangent')
                ):
                    continue

            float_array = next(
                (
                    item
                    for item in source.iter()
                    if VehicleOverlayModel._tag_name(item) == 'float_array'
                ),
                None,
            )
            if float_array is None or not float_array.text:
                continue

            values = [float(value) for value in float_array.text.split()]
            stride = 3
            accessor = next(
                (
                    item
                    for item in source.iter()
                    if VehicleOverlayModel._tag_name(item) == 'accessor'
                ),
                None,
            )
            if accessor is not None:
                stride = max(1, int(accessor.get('stride', '3')))

            positions = []
            for index in range(0, len(values) - 2, stride):
                positions.append(
                    (
                        values[index] * unit_scale,
                        values[index + 1] * unit_scale,
                        values[index + 2] * unit_scale,
                    )
                )
            sources[source_id] = positions
        return sources

    @staticmethod
    def _dae_material_colors(root):
        effect_colors = {}
        for effect in root.iter():
            if VehicleOverlayModel._tag_name(effect) != 'effect' or not effect.get('id'):
                continue
            color_node = VehicleOverlayModel._dae_effect_color_node(effect)
            if color_node is not None and color_node.text:
                effect_colors[effect.get('id')] = VehicleOverlayModel._rgba_to_hex(color_node.text)

        material_colors = {}
        for material in root.iter():
            if VehicleOverlayModel._tag_name(material) != 'material' or not material.get('id'):
                continue
            instance = next(
                (
                    node
                    for node in material
                    if VehicleOverlayModel._tag_name(node) == 'instance_effect'
                ),
                None,
            )
            if instance is None:
                continue
            effect_id = instance.get('url', '').lstrip('#')
            if effect_id in effect_colors:
                material_colors[material.get('id')] = effect_colors[effect_id]
                if material.get('name'):
                    material_colors[material.get('name')] = effect_colors[effect_id]
        return material_colors

    @staticmethod
    def _dae_effect_color_node(effect):
        for channel_name in ('diffuse', 'ambient', 'emission', 'specular'):
            for node in effect.iter():
                if VehicleOverlayModel._tag_name(node) != channel_name:
                    continue
                color_node = next(
                    (
                        child
                        for child in node.iter()
                        if VehicleOverlayModel._tag_name(child) == 'color'
                    ),
                    None,
                )
                if color_node is not None and color_node.text:
                    return color_node
        return None

    @staticmethod
    def _load_stl_vertices(mesh_path):
        with open(mesh_path, 'rb') as mesh_file:
            header = mesh_file.read(80)
            count_bytes = mesh_file.read(4)
            if len(count_bytes) != 4:
                return []
            triangle_count = struct.unpack('<I', count_bytes)[0]
            expected_size = 84 + triangle_count * 50
            actual_size = os.path.getsize(mesh_path)
            if not header.lstrip().lower().startswith(b'solid') or expected_size == actual_size:
                mesh_file.seek(84)
                vertices = []
                for _ in range(triangle_count):
                    data = mesh_file.read(50)
                    if len(data) != 50:
                        break
                    values = struct.unpack('<12fH', data)
                    vertices.extend(
                        (
                            (values[3], values[4], values[5]),
                            (values[6], values[7], values[8]),
                            (values[9], values[10], values[11]),
                        )
                    )
                if vertices:
                    return vertices

        vertices = []
        with open(mesh_path, 'r', encoding='utf-8', errors='ignore') as mesh_file:
            for line in mesh_file:
                parts = line.strip().split()
                if len(parts) == 4 and parts[0].lower() == 'vertex':
                    vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
        return vertices

    @staticmethod
    def _sample_vertices(vertices):
        if len(vertices) <= MAX_MESH_OUTLINE_VERTICES:
            return vertices
        stride = max(1, math.ceil(len(vertices) / MAX_MESH_OUTLINE_VERTICES))
        return vertices[::stride]

    @staticmethod
    def _transform_point(transform, point):
        return VehicleOverlayModel._add(
            transform.xyz, VehicleOverlayModel._rotate(transform.rotation, point)
        )

    @staticmethod
    def _rpy_to_matrix(roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )

    @staticmethod
    def _axis_angle_to_matrix(axis, angle):
        x, y, z = axis
        c = math.cos(angle)
        s = math.sin(angle)
        t = 1.0 - c
        return (
            (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
            (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
            (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
        )

    @staticmethod
    def _rotate(matrix, vector):
        return (
            matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
            matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
            matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
        )

    @staticmethod
    def _matmul(left, right):
        return tuple(
            tuple(
                sum(left[row][index] * right[index][column] for index in range(3))
                for column in range(3)
            )
            for row in range(3)
        )

    @staticmethod
    def _add(left, right):
        return (left[0] + right[0], left[1] + right[1], left[2] + right[2])

    @staticmethod
    def _convex_hull(points):
        unique_points = sorted({(round(x, 6), round(y, 6)) for x, y in points})
        if len(unique_points) <= 1:
            return unique_points

        def cross(origin, a, b):
            return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (
                b[0] - origin[0]
            )

        lower = []
        for point in unique_points:
            while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
                lower.pop()
            lower.append(point)

        upper = []
        for point in reversed(unique_points):
            while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
                upper.pop()
            upper.append(point)

        return lower[:-1] + upper[:-1]

    @staticmethod
    def _rgba_to_hex(text):
        values = [float(value) for value in text.split()[:4]]
        values += [1.0] * (4 - len(values))
        return '#{:02x}{:02x}{:02x}'.format(
            int(max(0.0, min(1.0, values[0])) * 255),
            int(max(0.0, min(1.0, values[1])) * 255),
            int(max(0.0, min(1.0, values[2])) * 255),
        )

    @staticmethod
    def _tag_name(element):
        return element.tag.split('}')[-1]
