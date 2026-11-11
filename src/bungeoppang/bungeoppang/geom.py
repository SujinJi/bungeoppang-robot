"""벡터·자세 계산 도우미 (ROS 노드와 무관한 순수 함수)"""
import math

from builtin_interfaces.msg import Duration


def gdir(pitch):
    """그리퍼가 향하는 방향 (pitch 0 = 바로 아래)"""
    a = math.radians(pitch)
    return [-math.sin(a), 0.0, -math.cos(a)]


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def add(v, w, s=1.0):
    return [v[i] + s * w[i] for i in range(3)]


def up(v, dz):
    return [v[0], v[1], v[2] + dz]


def lerp(a, b, r):
    """(tcp, pitch, roll) 두 자세 사이를 r 비율로 보간"""
    (t0, p0, r0), (t1, p1, r1) = a, b
    return (add(t0, [t1[i] - t0[i] for i in range(3)], r), p0 + (p1 - p0) * r, r0 + (r1 - r0) * r)


def to_duration(t):
    sec = int(t)
    return Duration(sec=sec, nanosec=int((t - sec) * 1e9))
