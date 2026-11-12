# Вид ночника снаружи (VTK, без окна) - картинки для Readme: корпус, крышка, кнопка BOOT, рассеиватель - из
# Scripts/enclosure.py; ночник горит: рассеиватель светится сам, на стене за корпусом - тёплый ореол над окном COB.
#   python Scripts/render_view.py  ->  Images/render_34.png (3/4 сверху справа)
# Система координат - как в enclosure.py: X вправо, Y вверх, Z к зрителю, лицо крышки Z = 0, основание - Z = -D.
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import enclosure as E                                     # noqa: E402
import vtk                                                # noqa: E402
from PIL import Image                                     # noqa: E402

OUT_DIR = os.path.join(E.ROOT, "Images")
WORK_PNG = os.path.join(E.ROOT, "work", "png")
IMG_W, IMG_H = 1200, 900                                  # размер картинки, px
TESS_TOL, TESS_ANG = 0.05, 0.2                            # точность сетки: мм, рад
FEATURE_DEG = 30                                          # нормали: острее - ребро (без сглаживания)
VIEW_ANGLE = 22                                           # угол обзора камеры, °
COL_CASE = (0.93, 0.92, 0.89)                             # ABS белый
COL_GLOW = (1.00, 0.84, 0.52)                             # свет COB через рассеиватель (тёплый белый)
COL_WALL = (0.20, 0.20, 0.23)                             # стена ночью
COL_BG = (0.08, 0.08, 0.10)                               # фон за стеной
GLOW_UP = 38.0                                            # ореол на стене: спад вверх от верха корпуса (в e раз), мм
GLOW_SIDE = 22.0                                          # ореол: спад в стороны за краем окна (сигма), мм
GLOW_BELOW = 6.0                                          # ореол: спад вниз от верха корпуса (свет вверх - мал), мм
WALL_PX_MM = 4                                            # текстура стены: пикселей на мм
WALL_X = 3.5                                              # стена: полуширина в ширинах корпуса (край - вне кадра)
WALL_Y = (-3.5, 3.0)                                      # стена: низ и верх в высотах корпуса от центра
CASE_LIGHT = 0.55                                         # общий свет сцены (ночь - тусклее дневного)
# виды: имя файла, направление на камеру от центра кадра, запас кадра (1 - габарит во весь кадр по высоте)
VIEWS = [("render_34.png", (0.6, 0.45, 1.0), 2.0)]
CROP_SIDE = 0.12                                          # обрезка пустых полей слева и справа, доля ширины кадра
CROP_BOTTOM = 0.14                                        # обрезка пустого поля снизу, доля высоты кадра


def poly_of(shape):
    """CadQuery Workplane / Shape -> vtkPolyData (треугольники) с нормалями по рёбрам."""
    s = shape.val() if hasattr(shape, "val") else shape
    verts, tris = s.tessellate(TESS_TOL, TESS_ANG)
    pts, cells = vtk.vtkPoints(), vtk.vtkCellArray()
    for v in verts:
        pts.InsertNextPoint(v.x, v.y, v.z)
    for t in tris:
        cells.InsertNextCell(3, t)
    pd = vtk.vtkPolyData()
    pd.SetPoints(pts)
    pd.SetPolys(cells)
    n = vtk.vtkPolyDataNormals()
    n.SetInputData(pd)
    n.SetFeatureAngle(FEATURE_DEG)
    n.SplittingOn()
    n.Update()
    return n.GetOutput()


def actor_of(pd, color, spec=0.25, lit=True):
    m = vtk.vtkPolyDataMapper()
    m.SetInputData(pd)
    a = vtk.vtkActor()
    a.SetMapper(m)
    p = a.GetProperty()
    p.SetColor(*color)
    p.SetSpecular(spec)
    p.SetSpecularPower(30)
    p.SetLighting(lit)                                    # False - светится сам (рассеиватель)
    return a


def wall_texture(path, x0, x1, y0, y1):
    """Стена с ореолом: яркость спадает вверх от верха корпуса и в стороны от окна COB."""
    w, h = int((x1 - x0) * WALL_PX_MM), int((y1 - y0) * WALL_PX_MM)
    top = E.H / 2
    im = Image.new("RGB", (w, h))
    px = im.load()
    for j in range(h):
        y = y1 - (j + 0.5) / WALL_PX_MM                   # строка 0 - верх картинки
        dy = y - top
        fy = math.exp(-dy / GLOW_UP) if dy >= 0 else math.exp(dy / GLOW_BELOW)
        for i in range(w):
            x = x0 + (i + 0.5) / WALL_PX_MM
            dx = max(0.0, abs(x) - E.WIN_HALF)
            g = fy * math.exp(-dx * dx / (2 * GLOW_SIDE * GLOW_SIDE))
            px[i, j] = tuple(int(255 * min(1.0, c * (1 - g) + gc * g)) for c, gc in zip(COL_WALL, COL_GLOW))
    im.save(path)


def wall_actor():
    x0, x1 = -WALL_X * E.W, WALL_X * E.W
    y0, y1 = WALL_Y[0] * E.H, WALL_Y[1] * E.H
    os.makedirs(WORK_PNG, exist_ok=True)
    tex_png = os.path.join(WORK_PNG, "render_wall.png")
    wall_texture(tex_png, x0, x1, y0, y1)
    z = -E.D - 0.05
    pl = vtk.vtkPlaneSource()
    pl.SetOrigin(x0, y0, z)
    pl.SetPoint1(x1, y0, z)
    pl.SetPoint2(x0, y1, z)
    rd = vtk.vtkPNGReader()
    rd.SetFileName(tex_png)
    tex = vtk.vtkTexture()
    tex.SetInputConnection(rd.GetOutputPort())
    tex.InterpolateOn()
    m = vtk.vtkPolyDataMapper()
    m.SetInputConnection(pl.GetOutputPort())
    a = vtk.vtkActor()
    a.SetMapper(m)
    a.SetTexture(tex)
    a.GetProperty().SetLighting(False)                    # свет на стене уже в текстуре
    return a


def glow_light():
    """Тёплый свет от рассеивателя: подсвечивает верх корпуса и кромки."""
    lt = vtk.vtkLight()
    lt.SetPositional(True)
    lt.SetPosition(0, E.H / 2 + 3, -E.D / 2)
    lt.SetFocalPoint(0, E.H / 2 + 40, -E.D / 2)
    lt.SetConeAngle(80)
    lt.SetColor(*COL_GLOW)
    lt.SetIntensity(0.8)
    return lt


def main():
    print("модель корпуса...")
    ren = vtk.vtkRenderer()
    ren.SetBackground(*COL_BG)
    for shape in (E.make_body(), E.make_cover(), E.make_button()):
        ren.AddActor(actor_of(poly_of(shape), COL_CASE))
    ren.AddActor(actor_of(poly_of(E.make_diffuser()), COL_GLOW, spec=0.0, lit=False))
    ren.AddActor(wall_actor())
    kit = vtk.vtkLightKit()
    kit.SetKeyLightIntensity(CASE_LIGHT)
    kit.AddLightsToRenderer(ren)
    ren.AddLight(glow_light())
    win = vtk.vtkRenderWindow()
    win.SetOffScreenRendering(1)
    win.SetSize(IMG_W, IMG_H)
    win.SetMultiSamples(8)
    win.AddRenderer(ren)
    cx, cy, cz, size = 0.0, E.H * 0.2, -E.D / 2, max(E.W, E.H)   # центр кадра чуть выше корпуса - видно ореол
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, (dx, dy, dz), margin in VIEWS:
        ln = (dx * dx + dy * dy + dz * dz) ** 0.5
        k = margin * 0.5 / math.tan(math.radians(VIEW_ANGLE / 2))
        cam = ren.GetActiveCamera()
        cam.SetFocalPoint(cx, cy, cz)
        cam.SetPosition(cx + dx / ln * size * k, cy + dy / ln * size * k, cz + dz / ln * size * k)
        cam.SetViewUp(0, 1, 0)
        cam.SetViewAngle(VIEW_ANGLE)
        ren.ResetCameraClippingRange()
        win.Render()
        grab = vtk.vtkWindowToImageFilter()
        grab.SetInput(win)
        grab.Update()
        wr = vtk.vtkPNGWriter()
        p = os.path.join(OUT_DIR, name)
        wr.SetFileName(p)
        wr.SetInputConnection(grab.GetOutputPort())
        wr.Write()
        im = Image.open(p)
        dx_px = int(CROP_SIDE * im.width)
        im.crop((dx_px, 0, im.width - dx_px, im.height - int(CROP_BOTTOM * im.height))).save(p)
        print(p)


if __name__ == "__main__":
    main()
