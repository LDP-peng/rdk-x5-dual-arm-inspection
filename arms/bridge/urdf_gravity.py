#!/usr/bin/env python3
"""从 URDF 计算机械臂静态重力补偿力矩，不依赖 Pinocchio。

只实现本机 Panthera-HT 所需的固定关节和转动关节。重力方向为世界坐标
Z 轴负方向，默认 g = 9.81 m/s²。
"""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np


def _vector(text, default):
    if text is None:
        return np.asarray(default, dtype=float)
    values = [float(value) for value in text.split()]
    if len(values) != 3:
        raise ValueError(f'需要 3 个数值，实际为：{text!r}')
    return np.asarray(values, dtype=float)


def _rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _origin_transform(element):
    transform = np.eye(4)
    if element is None:
        return transform
    origin = element.find('origin')
    if origin is None:
        return transform
    transform[:3, :3] = _rpy_matrix(_vector(origin.get('rpy'), [0.0, 0.0, 0.0]))
    transform[:3, 3] = _vector(origin.get('xyz'), [0.0, 0.0, 0.0])
    return transform


def _axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(3)
    x, y, z = axis / norm
    c, s = math.cos(angle), math.sin(angle)
    one_c = 1.0 - c
    return np.array([
        [c + x * x * one_c, x * y * one_c - z * s, x * z * one_c + y * s],
        [y * x * one_c + z * s, c + y * y * one_c, y * z * one_c - x * s],
        [z * x * one_c - y * s, z * y * one_c + x * s, c + z * z * one_c],
    ])


class UrdfGravityModel:
    def __init__(self, urdf_path, joint_names=None, gravity=9.81):
        self.urdf_path = os.path.abspath(os.path.expanduser(urdf_path))
        self.gravity = float(gravity)
        self.joint_names = list(joint_names or [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
        ])
        self.q_index = {name: index for index, name in enumerate(self.joint_names)}

        root = ET.parse(self.urdf_path).getroot()
        self.links = {}
        self.joints = {}
        self.children = {}
        child_links = set()

        for link in root.findall('link'):
            name = link.get('name')
            inertial = link.find('inertial')
            mass = 0.0
            com = np.zeros(3)
            if inertial is not None:
                mass_element = inertial.find('mass')
                if mass_element is not None:
                    mass = float(mass_element.get('value', '0'))
                origin = inertial.find('origin')
                if origin is not None:
                    com = _vector(origin.get('xyz'), [0.0, 0.0, 0.0])
            self.links[name] = {'mass': mass, 'com': com, 'parent': None}

        for joint in root.findall('joint'):
            name = joint.get('name')
            parent_element = joint.find('parent')
            child_element = joint.find('child')
            if parent_element is None or child_element is None:
                continue
            parent = parent_element.get('link')
            child = child_element.get('link')
            axis_element = joint.find('axis')
            axis = _vector(
                axis_element.get('xyz') if axis_element is not None else None,
                [0.0, 0.0, 0.0],
            )
            self.joints[name] = {
                'parent': parent,
                'child': child,
                'type': joint.get('type', 'fixed'),
                'origin': _origin_transform(joint),
                'axis': axis,
            }
            self.children.setdefault(parent, []).append(name)
            child_links.add(child)
            if child in self.links:
                self.links[child]['parent'] = parent

        roots = [name for name in self.links if name not in child_links]
        if len(roots) != 1:
            raise ValueError(f'URDF 应有唯一根连杆，实际为：{roots}')
        self.root_link = roots[0]

        for name in self.joint_names:
            if name not in self.joints:
                raise ValueError(f'URDF 中缺少关节 {name}')
            if self.joints[name]['type'] not in ('revolute', 'continuous'):
                raise ValueError(f'关节 {name} 不是转动关节')

        self._descendants = {
            name: self._collect_descendant_links(self.joints[name]['child'])
            for name in self.joint_names
        }

    def _collect_descendant_links(self, link_name):
        result = [link_name]
        for joint_name in self.children.get(link_name, []):
            result.extend(
                self._collect_descendant_links(self.joints[joint_name]['child'])
            )
        return result

    def _forward(self, q):
        transforms = {self.root_link: np.eye(4)}
        axes_world = {}
        pivots_world = {}

        def visit(link_name):
            parent_transform = transforms[link_name]
            for joint_name in self.children.get(link_name, []):
                joint = self.joints[joint_name]
                joint_transform = parent_transform @ joint['origin']
                axis_world = joint_transform[:3, :3] @ joint['axis']
                axes_world[joint_name] = axis_world
                pivots_world[joint_name] = joint_transform[:3, 3].copy()

                child_transform = joint_transform.copy()
                if joint_name in self.q_index:
                    angle = float(q[self.q_index[joint_name]])
                    child_transform[:3, :3] = (
                        joint_transform[:3, :3] @ _axis_rotation(joint['axis'], angle)
                    )
                elif joint['type'] == 'prismatic':
                    distance = 0.0
                    child_transform[:3, 3] += (
                        joint_transform[:3, :3] @ joint['axis'] * distance
                    )

                transforms[joint['child']] = child_transform
                visit(joint['child'])

        visit(self.root_link)
        return transforms, axes_world, pivots_world

    def gravity_torques(self, q):
        q = np.asarray(q, dtype=float)
        if q.size != len(self.joint_names):
            raise ValueError(f'需要 {len(self.joint_names)} 个关节角')

        transforms, axes_world, pivots_world = self._forward(q)
        torques = np.zeros(len(self.joint_names), dtype=float)

        for joint_name, output_index in self.q_index.items():
            axis = axes_world[joint_name]
            pivot = pivots_world[joint_name]
            total = 0.0
            for link_name in self._descendants[joint_name]:
                link = self.links[link_name]
                if link['mass'] <= 0.0:
                    continue
                com_world = transforms[link_name][:3, :3] @ link['com'] + transforms[link_name][:3, 3]
                velocity_z = np.cross(axis, com_world - pivot)[2]
                total += link['mass'] * self.gravity * velocity_z
            torques[output_index] = total
        return torques

    def potential_energy(self, q):
        q = np.asarray(q, dtype=float)
        transforms, _, _ = self._forward(q)
        total = 0.0
        for link_name, link in self.links.items():
            if link['mass'] <= 0.0 or link_name == self.root_link:
                continue
            com_world = transforms[link_name][:3, :3] @ link['com'] + transforms[link_name][:3, 3]
            total += link['mass'] * self.gravity * com_world[2]
        return total

    def numerical_torques(self, q, delta=1e-5):
        q = np.asarray(q, dtype=float)
        result = np.zeros_like(q)
        for index in range(q.size):
            upper = q.copy()
            lower = q.copy()
            upper[index] += delta
            lower[index] -= delta
            result[index] = (
                self.potential_energy(upper) - self.potential_energy(lower)
            ) / (2.0 * delta)
        return result
