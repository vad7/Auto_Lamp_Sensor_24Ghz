"""Проверка STL на нависания, как в слайсере: грани выше стола, смотрящие вниз круче порога (печать без поддержек).

Запуск: python Scripts/stl_overhang.py <файл.stl> [порог_°]
Из скрипта: overhangs(path) - вызывает Scripts/enclosure.py для деталей, которые печатаются без мостиков.
"""
import math
import struct
import sys

ANGLE_DEG = 45.0               # порог нависания по умолчанию: грань положе этого от горизонтали вниз - без опоры, °
BED_EPS_MM = 0.01              # грань ниже этой высоты лежит на столе, мм
AREA_MIN_MM2 = 0.01            # мельче - погрешность сетки, мм²
HEADER_BYTES = 80              # двоичный STL: заголовок
TRI_BYTES = 50                 # двоичный STL: нормаль + 3 вершины (12 float) + атрибут


def triangles(path):
    """Треугольники STL (двоичный или текстовый) - списки из 3 вершин (x, y, z)."""
    data = open(path, "rb").read()
    n = struct.unpack_from("<I", data, HEADER_BYTES)[0] if len(data) > HEADER_BYTES + 4 else 0
    if len(data) == HEADER_BYTES + 4 + n * TRI_BYTES:
        for i in range(n):
            f = struct.unpack_from("<12f", data, HEADER_BYTES + 4 + i * TRI_BYTES)
            yield [f[3:6], f[6:9], f[9:12]]
        return
    v = []
    for line in data.decode("ascii", "replace").splitlines():
        p = line.split()
        if p and p[0] == "vertex":
            v.append(tuple(map(float, p[1:4])))
            if len(v) == 3:
                yield v
                v = []


def overhangs(path, angle=ANGLE_DEG):
    """Нависания: {высота над столом мм (округл. 0.1): [площадь мм², xmin, xmax, ymin, ymax]}, только >= AREA_MIN_MM2."""
    limit = math.sin(math.radians(90 - angle))
    tris = list(triangles(path))
    zmin = min(p[2] for t in tris for p in t)
    spots = {}
    for a, b, c in tris:
        u = [b[i] - a[i] for i in range(3)]
        w = [c[i] - a[i] for i in range(3)]
        nx, ny, nz = u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0]
        ln = math.sqrt(nx * nx + ny * ny + nz * nz)
        if ln == 0 or nz / ln > -limit:
            continue
        z = min(a[2], b[2], c[2]) - zmin
        if z < BED_EPS_MM:
            continue
        s = spots.setdefault(round(z, 1), [0.0, 1e9, -1e9, 1e9, -1e9])
        s[0] += ln / 2
        xs, ys = (a[0], b[0], c[0]), (a[1], b[1], c[1])
        s[1], s[2] = min(s[1], *xs), max(s[2], *xs)
        s[3], s[4] = min(s[3], *ys), max(s[4], *ys)
    return {k: s for k, s in spots.items() if s[0] >= AREA_MIN_MM2}


def report(spots, indent="  "):
    """Строки отчёта по нависаниям."""
    return ["%sz %.1f мм: без опоры %.2f мм² (X %.1f..%.1f, Y %.1f..%.1f)" % ((indent, k) + tuple(spots[k]))
            for k in sorted(spots)]


if __name__ == "__main__":
    sp = overhangs(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else ANGLE_DEG)
    print(sys.argv[1])
    print("\n".join(report(sp)) if sp else "нависаний нет")
