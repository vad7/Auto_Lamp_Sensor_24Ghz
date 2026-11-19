# Корпус ночника (вариант rp2040zero_night): плоская коробка со скруглёнными углами, COB на верхней грани.
#   PYTHONIOENCODING=utf-8 python Scripts/enclosure.py
# Выход: Build/enclosure/*.stl (в положении печати), _assembly.step, _layout.png, _dims.json.
# Требуется: pip install cadquery (2.8), matplotlib.
#
# Система координат модели: X - вправо при виде спереди, Y - вверх (как висит на стене), Z - на зрителя.
# Лицевая сторона крышки - плоскость Z = 0, корпус уходит в минус Z; «глубина» d = -Z (от лица внутрь).
# Детали: body - основание (к стене) со стенками, печать основанием на стол; cover - лицевая крышка
# (4 самореза спереди); diffuser - рассеиватель COB (вклеивается в выемку, пока - печатная пластина);
# clamp - прижимная пластина платы (2 шт.: LD2410S и RP2040-Zero, саморез 2,5 в стойку); holder - T-держатель
# на стену для корпуса COB вниз (в модели - в системе корпуса).
import json
import math
import os
import sys

import cadquery as cq

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import stl_overhang  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "Build", "enclosure")

# ---------------- PARAMETERS: корпус ----------------
W = 84.0                # ширина снаружи (X), мм: прямой участок верха W - 2R = 64 под рассеиватель 63
H = 63.0                # высота снаружи (Y), мм: +3 - COB дальше от плат (пользователь; было 60)
D = 27.0                # толщина снаружи (Z), мм: HLK-5LD05 19 + выводы 3,5 (даташит, гл. 9) + зазор 0,5 + стенки
R = 10.0                # радиус углов при виде спереди, мм (было 20 - не помещается COB 60 мм при W 84)
T_WALL = 2.0            # боковые стенки, мм
T_FRONT = 2.0           # лицевая крышка, мм (потай самореза 1,15 мм)
T_BACK = 2.0            # основание, мм
DB = D - T_BACK         # глубина внутренней стороны основания, мм
SLICE_FIRST = 0.22      # слайсер: первый слой, мм (задание Picaso, work/clamp_radar_clamp_rp_cover.plgx)
SLICE_LAYER = 0.20      # слайсер: высота слоя, мм (там же)
SLICE_LINE = 0.33       # слайсер: ширина линии периметра, мм (там же; сопло 0,3)
SLICE_PROBE = 0.1       # половина стороны пробы материала в срезе, мм
LED_WIN_T = 0.28        # окно индикатора, толщина в модели, мм: между 1-м и 2-м срезом слайсера - ровно один слой при
                        # первом слое 0,20-0,22 и слое 0,2 (срез по середине или верху слоя); 0,2 при первом слое
                        # 0,22 не напечаталось (задание Picaso, сопло 0,3)
FIT = 0.3               # зазор посадки деталей в гнёзда, мм
GUIDE_T = 1.2           # толщина направляющих и рёбер, мм
SLOPE = 1.5             # откос под клиньями: глубина / вылет (56° от стола - печать без поддержек)

# ---------------- PARAMETERS: крепёж крышки и пластин ----------------
BOSS_R = 3.5            # радиус стойки крышки, мм
BOSS_OFF = (R - T_WALL) - BOSS_R + 1.0   # от центра дуги угла до стойки (стойка врезана в стенку на 1 мм), мм
PILOT_D = 2.0           # пилотное отверстие под саморез 2,5 мм (скилл enclosure), мм
PILOT_DEPTH = 8.0       # глубина пилота от торца стойки, мм
SCREW_L = 8.0           # саморез 2,5 x 8 (задал пользователь), мм
SCREW_HOLE_D = 2.9      # отверстие под саморез в крышке и пластине, мм
SCREW_HEAD_D = 5.2      # потай: диаметр зенковки 90°, мм
SINK = (SCREW_HEAD_D - SCREW_HOLE_D) / 2   # глубина зенковки, мм
COVER_RIB_T = GUIDE_T   # рёбра жёсткости крышки по краям (крышка гнулась - пользователь): толщина, мм
COVER_RIB_H = 3.0       # высота рёбер от внутренней стороны крышки, мм (выбрал ассистент; дальше - платы и выводы)

# ---------------- PARAMETERS: ввод провода (выдавливаемые карманы) ----------------
KO_D = 6.5              # диаметр под провод 6 мм (задал пользователь), мм
KO_SKIN = 0.6           # остающаяся снаружи «кожа» - продавить или просверлить, мм
KO_DEPTH = 19.0         # глубина центра кармана в стенках (d), мм
KO_SIDE_Y = -H / 2 + 25.0   # карман в правой стенке (у HLK-5LD05), Y (от нижней стенки, как платы), мм
KO_BOTTOM_X = 18.0      # карман в нижней стенке - под HLK-5LD05, X, мм
KO_TIP_HALF = 40.0      # полуугол носика капли, °: стороны 50° от стола - круче порога 45° с запасом на сетку

# ---------------- PARAMETERS: COB, профиль, рассеиватель ----------------
PROF_W = 15.0           # П-профиль 15 x 15 x 1,5 (задал пользователь): полка (по Z), мм
PROF_H = 15.0           # стенки профиля (вниз, -Y), мм
PROF_T = 1.5            # толщина, мм
PROF_L = 60.0           # длина = длине COB, мм
PROF_D0 = T_FRONT + FIT # глубина передней стенки профиля: от выпадания держит крышка, мм
TAPE = 0.3              # термоскотч, мм
COB_L, COB_W = 60.0, 15.0     # пластина COB 6015, мм (продавец)
COB_T = 1.2             # алюминиевая подложка, мм - ОЦЕНКА по фото
EMIT_L, EMIT_W, EMIT_T = 40.0, 9.0, 0.8   # светящееся поле, мм (продавец; толщина - ОЦЕНКА)
WALL_IN_Y = H / 2 - T_WALL                  # внутренняя поверхность верхней стенки, мм
DIFF_T = 1.5            # рассеиватель: плоская пластина, мм (рифы к COB убраны - расплывались при печати;
                        # лицо верхним слоем печати шершавое - рассеивает лучше, пользователь)
DIFF_GAP = 3.3          # светящееся поле - рассеиватель, мм
PROF_Y1 = WALL_IN_Y - DIFF_T - DIFF_GAP - EMIT_T - COB_T - TAPE   # верх полки профиля, мм
WIN_HALF = 29.5         # окно в верхней стенке: половина ширины, мм
WIN_D1 = 20.0           # окно от торца стенок (открыто к крышке - печать без мостика) до глубины, мм
DIFF_PLUG = T_WALL      # рассеиватель: выступ в окно (заподлицо снаружи, стенки прямые - окно корпуса прежнее,
                        # без перепечатки) - толще на просвет (0,8 мм - видны светодиоды COB, пользователь), мм
DIFF_HALF = 31.5        # рассеиватель: половина ширины (перекрывает окно на 2 мм; шире - упор во внутреннее
                        # скругление R8 угла), мм
DIFF_D1 = WIN_D1 + 2.0  # рассеиватель: задний край, мм
DIFF_LEDGE_W = 1.0      # полки-рельсы под боковыми краями рассеивателя (держат от провала внутрь, пользователь):
                        # заходят под край, мм - в перекрытие окна 2 мм, свет не загораживают
DIFF_LEDGE_T = 1.0      # толщина полок, мм (3 линии 0,33)
BLOCK_X = (PROF_L / 2 - 2.0, PROF_L / 2 + FIT + 2.0)   # упоры концов профиля (|x|), мм
WALL_HOLE_XY = (0.0, PROF_Y1 - PROF_H / 2)   # отверстие крепления к стене (карман в основании): под профилем COB
                        # (пользователь; было под макеткой - она стояла на 6,5 мм над головкой самореза), мм
COB_BOARD_GAP = 3.0     # COB дальше от плат: добавка к зазору профиль - крючки плат, мм (пользователь)
WIRE_CHAMFER = 2.0     # скос 45° внешнего верхнего угла упора (катет): щель до стойки самореза крышки под провод
                        # COB - около 2,4 мм (было 1,05 - от угла упора до стойки), пользователь; упор держит
                        # профиль ниже скоса, мм

# ---------------- PARAMETERS: крепление плат (стойка + прижимная пластина снизу, крючки сверху) ----------------
SO_D = 6.0              # стойка под саморез, мм
LEDGE_L = 1.5           # полка под край платы, вылет, мм
HOOK_T = 1.5            # колонна крючка, толщина, мм
HOOK_FIT = 0.15         # зазор колонна - край платы, мм
HOOK_L = 1.0            # крючок над лицом платы, вылет, мм
LIP_H = 1.6             # высота крючка (низ - откос), мм
CLAMP_W = 7.0           # прижимная пластина, ширина, мм
CLAMP_T = 2.0           # прижимная пластина, толщина (потай 1,15 мм), мм
TOP_Y = PROF_Y1 - PROF_H - FIT - HOOK_FIT - HOOK_T - COB_BOARD_GAP   # верхний край плат (крючки ниже профиля), мм

# ---------------- PARAMETERS: LD2410S (Docs/HLK-LD2410S User manual-V1.3.pdf гл. 9, Docs/HLK-LD2410S-3D.step) ----
RADAR_S = 20.0          # плата 20 x 20, мм (чертёж)
RADAR_T = 1.2           # толщина платы, мм (чертёж)
RADAR_PINS_BACK = 9.8   # выводы J2 назад от лица антенны, мм (STEP: z -9.8)
RADAR_PINS_FRONT = 1.7  # концы выводов вперёд от лица антенны, мм (STEP: z 1.7)
RADAR_PINS = (17.21, 19.75, 6.95, 19.65)   # колонка выводов J2 (x0, x1, y0, y1) от угла платы, мм (STEP)
RX0, RY0 = -39.0, TOP_Y - RADAR_S   # левый нижний угол платы (вид спереди, антенна к зрителю), мм
RADAR_HOOKS = ((0.0, 3.0), (11.0, 14.0))   # крючки по верхнему краю (x от угла платы) - мимо выводов J2, мм
F_RADAR = 24.125e9      # средняя частота, Гц (24,0-24,25 ГГц)
C_MM = 299.792458e9     # скорость света, мм/с
RADOME_ER = 2.8         # проницаемость ABS (пластик задал пользователь) - ОЦЕНКА: TI SWRA705 табл. 3-1 - ABS 2,0-3,5,
                        # гл. 5 - ABS-HG_FR 2,8 (взят он)
LAMBDA0 = C_MM / F_RADAR                           # длина волны в воздухе, мм
RADOME_T = LAMBDA0 / (2 * math.sqrt(RADOME_ER))    # прежняя стенка перед антенной λm/2 (SWRA705 гл. 2.3), мм: от неё
                                                   # считаны глубина антенны и упоры COB - корпус не меняется
RADOME_GAP = LAMBDA0 / 2                           # антенна - прежняя стенка λ0/2 (SWRA705 гл. 2.4), мм
ANT_D = RADOME_T + RADOME_GAP                      # глубина лица антенны, мм
RADOME_WALL = 0.7       # крышка перед антенной, мм (пользователь, белый ABS): с утолщением λm/2 датчик срабатывал через
                        # ~2 с, без крышки - сразу; выемка изнутри по плате радара, в обход рёбер
RADAR_SO_X = 18.5       # стойка радара: x от левого угла платы (у правого угла - мимо фоторезистора), мм
RADAR_CHAMFER = 3.0     # скос левого нижнего угла пластины радара (к трубке фоторезистора), катет, мм

# ---------------- PARAMETERS: RP2040-Zero (Docs/RP2040-Zero.pdf, Dimension) ----------------
RP_W, RP_H = 18.0, 23.5 # плата, мм (чертёж); USB-C вверх, компоненты к лицу
RP_T = 1.2              # толщина платы, мм - ОЦЕНКА
RP_X0, RP_YT = -17.0, TOP_Y   # левый край и верхний (у USB) край платы, мм
USB_H = 3.2             # высота USB-C над платой, мм - ОЦЕНКА (задаёт зазор до лица)
RP_LOWER = 1.0          # плата глубже от крышки, мм: нажатие на крышку нажимало кнопку BOOT (задал пользователь)
RP_D = T_FRONT + USB_H + FIT + RP_LOWER   # глубина верхней стороны платы, мм
USB_BOX = (4.5, 13.5, -0.5, 7.3)   # USB-C (x0, x1, y0, y1) от левого верхнего угла, y вниз, мм - ОЦЕНКА по рисунку
BOOT_XY = (4.2, 16.3)   # центр кнопки BOOT от левого верхнего угла, y вниз, мм - ОЦЕНКА по рисунку
LED_XY = (8.5, 15.8)    # центр WS2812B, мм - ОЦЕНКА по рисунку
BTN_H = 1.5             # высота кнопки над платой, мм - ОЦЕНКА (данных нет)
LED_WIN_D = 5.0         # окно индикатора (один слой), диаметр у плёнки, мм (задал пользователь: радиус 2,5)
LED_WIN_SLOPE = 30.0    # откос от плёнки окна к полной толщине крышки, угол к плоскости лица, ° (задал пользователь)
RP_HOOKS = ((-3.0, 4.0), (14.0, 21.0))  # крючки по верхнему краю - по сторонам USB-C, мм: ширина 7, зацеп за плату по 4
                                        # (задал пользователь; до USB-C остаётся 0,5), остальное - за краем платы

# ---------------- PARAMETERS: печатная кнопка BOOT (упирается торцом в кнопку платы) ----------------
CAP_D = 4.0             # стержень, мм (задал пользователь)
CAP_OUT = 4.0           # выступает над лицом крышки, мм (задал пользователь; при опоре на кнопку платы, BTN_H - оценка)
CAP_TOP_R = 1.0         # скругление верхнего края, мм
CAP_FL_D = 6.0          # фланец изнутри крышки (не даёт выпасть), мм; 8 закрывал светодиод платы
CAP_FL_T = 1.0          # толщина фланца, мм
CAP_HOLE_D = 4.6        # отверстие в крышке, мм: зазор 0,3 на сторону; при 0,15 покупной колпачок Ø6 в напечатанном
                        # отверстии 6,3 сидел очень плотно (пользователь) - пружина кнопки может не вернуть

# ---------------- PARAMETERS: фоторезистор ----------------
LDR_XY = (-27.0, -H / 2 + 6.5)   # центр (от нижней стенки, как платы), мм
LDR_D = 5.2             # гнездо в крышке под фоторезистор 5 мм (снаружи закрыто плёнкой LED_WIN_T), мм
LDR_BOSS_D = 8.0        # трубка-держатель, мм
LDR_BOSS_H = 5.0        # высота трубки от крышки внутрь, мм

# ---------------- PARAMETERS: HLK-5LD05 (Docs/HLK-5LD05_5W-LD_datasheet_V1.0.pdf гл. 9) ----------------
HLK_S = 25.4            # основание, мм
HLK_H = 19.0            # высота, мм
HLK_PIN = 3.5           # выводы, мм
HLK_PIN_D = 0.8         # диаметр вывода, мм (чертёж)
HLK_PINS = ((10.16, 10.16), (10.16, 5.08), (-10.16, 0.0), (-10.16, 10.16))   # выводы 1, 2, 4, 5 от центра
                        # (вид со стороны выводов = спереди, чертёж «Bottom view»), мм; модуль повёрнут на 180°
                        # (задал пользователь): сеть (1 - N, 2 - L) - к правой стенке, выход (4, 5) - к RP2040-Zero
HLK_X0, HLK_Y0 = 5.5, TOP_Y - HLK_S  # левый нижний угол, мм (стоит на основании, выводы - к крышке)
HLK_GUIDE_H = 6.0       # высота уголков-направляющих, мм
HLK_PRESS_D = 5.0       # штырь на крышке, прижимает HLK по центру (выводы - по краям, 2,54 мм от кромки), мм

# ---------------- PARAMETERS: макетка (Q1 P75N02LDG, R1-R3, C1) - за RP2040-Zero, на основании ----------------
# Обрезок с заводского конца макетки 2 x 8 см (фото пользователя - work/protoboard/photo_end.png): 6 рядов 1-6
# поперёк (X), столбцы A-G вдоль (Y, A - у заводского торца); у торца - 2 крепёжных отверстия и овальные площадки.
# Заводской торец - вниз.
PB_X0, PB_Y0 = -16.3, -H / 2 + 10.4   # левый нижний угол, мм: между стойками радара и RP2040-Zero и уголками HLK
PB_W = 20.0             # ширина макетки (заводская), мм
PB_H = 22.1             # от заводского торца до реза за столбцом G, мм - ОЦЕНКА по фото
PB_END = 5.6            # заводской торец - столбец A (там крепёжные отверстия, площадки), мм - ОЦЕНКА по фото
PB_T = 1.6              # толщина макетки, мм - ОЦЕНКА (типовой стеклотекстолит)
PB_RAISE = 3.4          # макетка над основанием, мм: около половины 6,5 (пользователь; 3,25 + 0,15 - пилот под
                        # саморез PB_SCREW_L с запасом 0,5) - саморез крепления к стене перенесён под COB
PB_SCREW_L = 6.0        # саморез макетки 2,5 x 6 (пользователь): 2,5 x 8 при низкой опоре вышел бы из основания, мм
PB_POST = 3.0           # опоры под углами, сторона квадрата, мм
PB_SCREW_D = 5.0        # опора под саморез 2,5 x 8, диаметр, мм
PB_SCREW_OFF = (1.85, 2.1)   # крепёжное отверстие макетки у левого нижнего угла (от боковой кромки, от торца), мм -
                             # ОЦЕНКА по фото; диаметр по фото около 2,3 - тесно для 2,5: рассверлить до 2,7
PB_PILOT = PB_RAISE + T_BACK - 0.5   # пилот: через опору в основание, снаружи 0,5 мм, мм
PB_ZONE_GAP = 0.5       # детали на макетке: зазор до платы RP2040-Zero, мм

# ---------------- PARAMETERS: T-держатель (корпус COB вниз, держатель саморезами на стену) ----------------
# Пластина T за основанием: перекладина - «руки», загнуты вперёд по боковым стенкам, с зацепом на лицо крышки; ножка -
# вниз (к COB) с полочкой под корпусом. Модель - в системе корпуса (COB вверху, +Y): в работе всё повёрнуто на 180°
# вокруг Z. Корпус вставляется сверху (в работе) между руками до полочки. Печать - пластиной на стол, руки и полочка вверх.
HOLDER_T = 3.0          # толщина, мм (пользователь)
HOLDER_W = 10.0         # ширина рук, мм (пользователь)
HOLDER_STEM_W = 14.0    # ширина ножки и полочки, мм (пользователь; было 10)
HOLDER_BRACE_IN = 0.0   # косое утолщение перекладины к ножке начинается от внутренней грани рук на, мм (пользователь;
                        # было 3)
HOLDER_BRACE_W = 16.0   # ширина перекладины у ножки (пользователь; было 14) (низ - по прямой от HOLDER_W), мм (пользователь): прочнее у отверстий
HOLDER_KO_GAP = 1.0     # руки дальше от COB, чем отверстие у HLK, на, мм (пользователь; было 3)
HOLDER_PRELOAD = 0.2    # натяг зацепа на край крышки, мм (пользователь: болтался - было 0,75 мм люфта): руки пружинят
HOLDER_LIP = 2.0        # зацеп рук на лицо крышки, вылет внутрь, мм (выбрал ассистент): низ - откос SLOPE (печать без
                        # поддержек), касается края крышки - держит корпус от наклона от стены
HOLDER_LIP_TOP = 1.0    # толщина зацепа у кончика, мм
HOLDER_SHELF = 5.0      # полочка под корпусом, вылет от основания, мм: под сплошной частью стенки за окном COB
HOLDER_SCREW_D = 3.5    # отверстие под саморез в стену, мм (выбрал ассистент: саморез 3-3,5 под дюбель)
HOLDER_HEAD_D = 7.0     # потай под голову, мм (зенковка 90° со стороны корпуса - голова заподлицо)

# Платы: (x0, y0, ширина, высота, глубина лица, толщина, крючки, x стойки от левого угла,
#         скос левого нижнего угла пластины)
MOUNTS = {
    "radar": (RX0, RY0, RADAR_S, RADAR_S, ANT_D, RADAR_T, RADAR_HOOKS, RADAR_SO_X, RADAR_CHAMFER),
    "rp": (RP_X0, RP_YT - RP_H, RP_W, RP_H, RP_D, RP_T, RP_HOOKS, RP_W / 2, 0.0),
}

CHECK_VOL = 0.05        # пересечение меньше - касание (погрешность), мм³


def blk(x0, x1, y0, y1, d0, d1):
    """Брусок по координатам; d0..d1 - глубина от лица."""
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, d1 - d0, centered=False).translate((x0, y0, -d1))


def cyl(x, y, dia, d0, d1):
    """Цилиндр вдоль Z, d0..d1 - глубина."""
    return cq.Workplane("XY").circle(dia / 2).extrude(d1 - d0).translate((x, y, -d1))


def rrect(w, h, r, d0, d1):
    """Скруглённый прямоугольник (углы при виде спереди), d0..d1 - глубина."""
    return cq.Workplane("XY").rect(w, h).extrude(d1 - d0).edges("|Z").fillet(r).translate((0, 0, -d1))


def prism(pts, x0, x1):
    """Призма вдоль X: сечение - точки (y, d)."""
    return (cq.Workplane("YZ").workplane(offset=x0).polyline([(y, -d) for y, d in pts]).close()
            .extrude(x1 - x0))


def countersink(x, y, d):
    """Зенковка 90° под потай от глубины d внутрь."""
    cone = cq.Solid.makeCone(SCREW_HEAD_D / 2, SCREW_HOLE_D / 2, SINK, cq.Vector(x, y, -d), cq.Vector(0, 0, -1))
    return cq.Workplane().add(cone)


def teardrop_x(x0, x1, y0, d, dia):
    """Карман-капля вдоль X (в стенке, перпендикулярной X): носик к крышке - вверх при печати."""
    r = dia / 2
    beta = math.radians(KO_TIP_HALF)
    kx, kz = r * math.cos(beta), r * math.sin(beta)     # точки касания сторон носика
    wp = cq.Workplane("YZ").workplane(offset=x0)
    c = wp.center(y0, -d).circle(r).extrude(x1 - x0)
    tip = (cq.Workplane("YZ").workplane(offset=x0)
           .polyline([(y0 - kx, -d + kz), (y0, -d + r / math.sin(beta)), (y0 + kx, -d + kz), (y0, -d)]).close()
           .extrude(x1 - x0))
    return c.union(tip)


def boss_xy():
    cx, cy = W / 2 - R, H / 2 - R
    o = BOSS_OFF / math.sqrt(2)
    return [(sx * (cx + o), sy * (cy + o)) for sx in (-1, 1) for sy in (-1, 1)]


def corner_guides(x0, x1, y0, y1, d0, d1, corners):
    """Уголки-направляющие снаружи прямоугольника (x0..x1, y0..y1) на углах corners ('LB', 'RT', ...)."""
    g = None
    n = 4.0
    for c in corners:
        x = x0 - FIT - GUIDE_T if c[0] == "L" else x1 + FIT
        y = y0 - FIT - GUIDE_T if c[1] == "B" else y1 + FIT
        bx = x if c[0] == "L" else x - n
        by = y if c[1] == "B" else y - n
        part = (blk(x, x + GUIDE_T, by, by + n + GUIDE_T, d0, d1)
                .union(blk(bx, bx + n + GUIDE_T, y, y + GUIDE_T, d0, d1)))
        g = part if g is None else g.union(part)
    return g


def standoff_xy(m):
    return m[0] + m[7], m[1] - FIT - SO_D / 2


def chamfer_cut(m, d0, d1):
    """Срезаемый треугольник у левого нижнего угла пластины (катет m[8]), d0..d1 - глубина."""
    cx, cy = standoff_xy(m)
    x0, y0, k = cx - CLAMP_W / 2, cy - SO_D / 2, m[8]
    return (cq.Workplane("XY").polyline([(x0 - 5, y0 - 5), (x0 + k + 5, y0 - 5), (x0 - 5, y0 + k + 5)]).close()
            .extrude(d1 - d0).translate((0, 0, -d1)))


def board_mount(m):
    """Стойка с полкой под нижний край платы (саморез через прижимную пластину) и крючки по верхнему краю."""
    x0, y0, w, h, df, t, hooks = m[:7]
    db = df + t
    cx, cy = standoff_xy(m)
    yc = y0 - FIT
    part = cyl(cx, cy, SO_D, df, DB + 0.01).cut(cyl(cx, cy, PILOT_D, df - 1, df + PILOT_DEPTH))
    part = part.union(prism([(yc - 2, db), (y0 + LEDGE_L, db), (yc - 2, db + (LEDGE_L + FIT + 2) * SLOPE)],
                            cx - SO_D / 2 + 0.5, cx + SO_D / 2 - 0.5))
    y1 = y0 + h
    yc = y1 + HOOK_FIT
    for a, b in hooks:
        part = part.union(blk(x0 + a, x0 + b, yc, yc + HOOK_T, df - LIP_H, DB + 0.01))
        ga, gb = x0 + max(a, 0.0), x0 + min(b, w)   # зацеп и полка - только над платой: колонна может быть шире её края
        part = part.union(prism([(yc + 0.01, df - LIP_H), (y1 - HOOK_L, df - LIP_H), (yc + 0.01, df)], ga, gb))
        part = part.union(prism([(yc + 0.01, db), (y1 - LEDGE_L, db),
                                 (yc + 0.01, db + (LEDGE_L + HOOK_FIT) * SLOPE)], ga, gb))
    return part


def make_body():
    body = rrect(W, H, R, T_FRONT, D).cut(rrect(W - 2 * T_WALL, H - 2 * T_WALL, R - T_WALL, T_FRONT - 1, DB))

    # Стойки под саморезы крышки (растут из основания)
    for x, y in boss_xy():
        body = (body.union(cyl(x, y, 2 * BOSS_R, T_FRONT, DB + 0.01))
                .cut(cyl(x, y, PILOT_D, T_FRONT - 1, T_FRONT + PILOT_DEPTH)))

    # Платы: стойка + крючки
    for m in MOUNTS.values():
        body = body.union(board_mount(m))

    # Макетка: опоры под углами (у левого нижнего - под саморез), уголки слева снизу и справа сверху
    pb_d = DB - PB_RAISE
    px1, py1 = PB_X0 + PB_W, PB_Y0 + PB_H
    sx, sy = PB_X0 + PB_SCREW_OFF[0], PB_Y0 + PB_SCREW_OFF[1]
    for x, y in ((PB_X0, py1 - PB_POST), (px1 - PB_POST, PB_Y0), (px1 - PB_POST, py1 - PB_POST)):
        body = body.union(blk(x, x + PB_POST, y, y + PB_POST, pb_d, DB + 0.01))
    body = (body.union(cyl(sx, sy, PB_SCREW_D, pb_d, DB + 0.01))
            .cut(cyl(sx, sy, PILOT_D, pb_d - 1, pb_d + PB_PILOT)))
    body = body.union(corner_guides(PB_X0, px1, PB_Y0, py1, pb_d - PB_T - 1.0, DB + 0.01, ("LB", "RT")))
    # Крючки плат - одним целым с соседями (пользователь, чтобы не ломались): левый крючок радара - со стенкой,
    # правый - с упором профиля COB над ним, левый крючок RP2040-Zero - с верхней левой опорой макетки под ним
    hy0, hy1 = TOP_Y + HOOK_FIT, TOP_Y + HOOK_FIT + HOOK_T   # колонны крючков по Y (верхний край плат - TOP_Y)
    rx, rd = MOUNTS["radar"][0], MOUNTS["radar"][4] - LIP_H   # левый край радара, глубина начала крючков
    body = body.union(blk(-W / 2 + T_WALL - 0.01, rx + RADAR_HOOKS[0][0] + 0.01, hy0, hy1, rd, DB + 0.01))
    hx = rx + RADAR_HOOKS[1][0]
    body = body.union(blk(hx - GUIDE_T, hx + GUIDE_T, hy1 - 0.01, PROF_Y1 - PROF_H - 0.2 - 1.5 + 0.01, rd, DB + 0.01))
    body = body.union(blk(PB_X0, PB_X0 + PB_POST, py1 + FIT, hy0 + 0.01, pb_d - PB_T - 1.0, DB + 0.01))
    body = body.union(blk(PB_X0, PB_X0 + PB_POST, py1 - 0.01, py1 + FIT + 0.01, pb_d, DB + 0.01))   # за макеткой

    # HLK-5LD05: уголки на основании
    body = body.union(corner_guides(HLK_X0, HLK_X0 + HLK_S, HLK_Y0, HLK_Y0 + HLK_S, DB - HLK_GUIDE_H, DB + 0.01,
                                    ("LB", "RB", "LT", "RT")))

    # COB: упоры концов профиля (гнездо открыто к крышке - профиль вставляется спереди, держит крышка)
    for s in (-1, 1):
        a, b = sorted((s * BLOCK_X[0], s * BLOCK_X[1]))
        pa, pb = sorted((s * (BLOCK_X[0] - 0.1), s * (PROF_L / 2 + FIT)))
        py0 = PROF_Y1 - PROF_H - 0.2
        blkp = blk(a, b, py0 - 1.5, PROF_Y1 + 0.2, RADOME_T + FIT, DB + 0.01)
        pocket = blk(pa, pb, py0, PROF_Y1 + 1, 0, PROF_D0 + PROF_W + 0.2)
        # Скос внешнего верхнего угла упора: щель до стойки самореза - провод COB (стенки по глубине - без поддержек)
        xo, yt, c = s * BLOCK_X[1], PROF_Y1 + 0.2, WIRE_CHAMFER
        bevel = (cq.Workplane("XY").workplane(offset=-(DB + 0.1))
                 .polyline([(xo + s * 0.1, yt + 0.1), (xo - s * c, yt + 0.1), (xo + s * 0.1, yt - c - 0.1)]).close()
                 .extrude(DB + 0.2))
        body = body.union(blkp.cut(pocket).cut(bevel))

    # Окно COB - вырез от торца стенок; выемка рассеивателя - рёбра по бокам и сзади (до основания)
    body = body.cut(blk(-WIN_HALF, WIN_HALF, WALL_IN_Y - 0.1, H / 2 + 1, 0, WIN_D1))
    y0 = WALL_IN_Y - DIFF_T
    r0, r1 = DIFF_HALF + FIT, DIFF_HALF + FIT + GUIDE_T
    ly1 = y0 - FIT                   # верх полки-рельса под рассеивателем (зазор FIT - вдвигается спереди)
    for s in (-1, 1):
        a, b = sorted((s * r0, s * r1))
        body = body.union(blk(a, b, ly1 - DIFF_LEDGE_T, WALL_IN_Y + 0.01, T_FRONT, DB + 0.01))
        a, b = sorted((s * (DIFF_HALF - DIFF_LEDGE_W), s * r0))   # полка под краем рассеивателя: не проваливается
        body = body.union(blk(a, b, ly1 - DIFF_LEDGE_T, ly1, T_FRONT, DB + 0.01))   # до основания - печать без опоры
    body = body.union(blk(-r1, r1, y0, WALL_IN_Y + 0.01, DIFF_D1 + FIT, DB + 0.01))

    # Карманы под провод у HLK-5LD05: справа и снизу (носик капли - к крышке); карман крепления к стене
    body = body.cut(teardrop_x(W / 2 - T_WALL - 0.5, W / 2 - KO_SKIN, KO_SIDE_Y, KO_DEPTH, KO_D))
    y0, y1 = -H / 2 + KO_SKIN, -H / 2 + T_WALL + 0.5
    body = body.cut(teardrop_x(y0, y1, -KO_BOTTOM_X, KO_DEPTH, KO_D).rotate((0, 0, 0), (0, 0, 1), 90))
    return body.cut(cyl(WALL_HOLE_XY[0], WALL_HOLE_XY[1], KO_D, DB - 0.01, D - KO_SKIN))


def make_cover():
    cover = rrect(W, H, R, 0, T_FRONT)
    for x, y in boss_xy():
        cover = cover.cut(cyl(x, y, SCREW_HOLE_D, -1, T_FRONT + 1)).cut(countersink(x, y, 0))

    # LD2410S: тонкая стенка RADOME_WALL перед антенной - выемка изнутри по плате радара, в обход рёбер
    o = T_WALL + FIT + COVER_RIB_T
    cover = cover.cut(blk(RX0, RX0 + RADAR_S, RY0, RY0 + RADAR_S, RADOME_WALL, T_FRONT + 0.01)
                      .intersect(rrect(W - 2 * o, H - 2 * o, R - o, 0, T_FRONT + 0.01)))

    # Фоторезистор: гнездо + трубка; снаружи - плёнка LED_WIN_T, как окно индикатора (закрыто от пыли и влаги,
    # пользователь; пропускание плёнки ABS - не известно, проверить перепад «темно / светло» в консоли show)
    lx, ly = LDR_XY
    cover = (cover.union(cyl(lx, ly, LDR_BOSS_D, T_FRONT - 0.01, T_FRONT + LDR_BOSS_H))
             .cut(cyl(lx, ly, LDR_D, LED_WIN_T, T_FRONT + LDR_BOSS_H + 1)))

    # HLK-5LD05: штырь до лица модуля с выводами - прижимает модуль к основанию
    cover = cover.union(cyl(HLK_X0 + HLK_S / 2, HLK_Y0 + HLK_S / 2, HLK_PRESS_D, T_FRONT - 0.01, DB - HLK_H))

    # Рёбра жёсткости по краям: изнутри вдоль стенок (зазор FIT), в обход стоек саморезов и выемки рассеивателя COB
    o = T_WALL + FIT
    d1 = T_FRONT + COVER_RIB_H
    rib = rrect(W - 2 * o, H - 2 * o, R - o, T_FRONT - 0.01, d1).cut(
        rrect(W - 2 * (o + COVER_RIB_T), H - 2 * (o + COVER_RIB_T), R - o - COVER_RIB_T, T_FRONT - 1, d1 + 1))
    for x, y in boss_xy():
        rib = rib.cut(cyl(x, y, 2 * (BOSS_R + FIT), 0, d1 + 1))
    by = max(y for _, y in boss_xy())      # сверху ребра нет: выемка рассеивателя COB во всю ширину между стойками
    cover = cover.union(rib.cut(blk(-W, W, by, H, 0, d1 + 1)))

    # Индикатор: окно в один слой с откосом к полной толщине, не заходит на опору фланца колпачка BOOT;
    # кнопка BOOT: отверстие под печатную кнопку
    led_x, led_y = RP_X0 + LED_XY[0], RP_YT - LED_XY[1]
    boot_x, boot_y = RP_X0 + BOOT_XY[0], RP_YT - BOOT_XY[1]
    r0 = LED_WIN_D / 2
    h = T_FRONT + 0.1 - LED_WIN_T
    window = cq.Workplane("XY").add(cq.Solid.makeCone(r0, r0 + h / math.tan(math.radians(LED_WIN_SLOPE)), h,
                                                      cq.Vector(led_x, led_y, -LED_WIN_T), cq.Vector(0, 0, -1)))
    window = window.cut(cyl(boot_x, boot_y, CAP_FL_D, -1, T_FRONT + 1))
    cover = cover.cut(window)
    return cover.cut(cyl(boot_x, boot_y, CAP_HOLE_D, -1, T_FRONT + 0.1))


def make_clamp(m):
    """Прижимная пластина: лежит на торце стойки и на крае платы, саморез 2,5 в стойку."""
    df = m[4]
    cx, cy = standoff_xy(m)
    plate = blk(cx - CLAMP_W / 2, cx + CLAMP_W / 2, cy - SO_D / 2, m[1] + LEDGE_L, df - CLAMP_T, df)
    if m[8]:
        plate = plate.cut(chamfer_cut(m, df - CLAMP_T - 1, df + 1))
    return plate.cut(cyl(cx, cy, SCREW_HOLE_D, df - CLAMP_T - 1, df + 1)).cut(countersink(cx, cy, df - CLAMP_T))


def make_diffuser(plug=True):
    """Рассеиватель: плоская пластина DIFF_T под окном COB.
    plug - выступ в окно (PETG, толще на просвет); False - тонкий вариант для ABS (окно утоплено на стенку)."""
    z0 = T_FRONT + 0.1
    base = blk(-DIFF_HALF, DIFF_HALF, WALL_IN_Y - DIFF_T, WALL_IN_Y, z0, DIFF_D1)
    if not plug:
        return base
    # Выступ в окно заподлицо с корпусом, с зазором FIT по бокам и сзади
    return base.union(blk(-WIN_HALF + FIT, WIN_HALF - FIT, WALL_IN_Y - 0.01, H / 2, z0, WIN_D1 - FIT))


def make_button():
    """Печатная кнопка BOOT: стержень через крышку со скруглённым верхом, фланец изнутри; торец - на кнопке платы."""
    bx, by = RP_X0 + BOOT_XY[0], RP_YT - BOOT_XY[1]
    cap_end = RP_D - BTN_H
    stem = cyl(bx, by, CAP_D, -CAP_OUT, cap_end - CAP_FL_T + 0.01).edges(">Z").fillet(CAP_TOP_R)
    return stem.union(cyl(bx, by, CAP_FL_D, cap_end - CAP_FL_T, cap_end))


def holder_arm_y():
    """Руки T-держателя по Y (система корпуса): от отверстия у HLK на HOLDER_KO_GAP дальше от COB."""
    y1 = KO_SIDE_Y - KO_D / 2 - HOLDER_KO_GAP
    return y1 - HOLDER_W, y1


def holder_screws():
    """Саморезы в стену: на концах и в центре перекладины (между рук - пользователь), внизу ножки (у полочки)."""
    ya0, _ = holder_arm_y()
    xb = W / 2 + FIT - HOLDER_BRACE_IN   # начало скоса перекладины
    sw = HOLDER_STEM_W / 2

    def mid(x):   # середина ширины перекладины в точке x (со скосом к ножке) - отверстие по центру (пользователь)
        k = min(max((xb - abs(x)) / (xb - sw), 0.0), 1.0)
        return ya0 + (HOLDER_W + (HOLDER_BRACE_W - HOLDER_W) * k) / 2

    xs = W / 2 - 12.0
    return [(-xs, mid(xs)), (0.0, mid(0.0)), (xs, mid(xs)), (0.0, H / 2 - 8.0)]


def make_holder():
    ya0, ya1 = holder_arm_y()
    t = HOLDER_T
    xo, xt = W / 2 + FIT, W / 2 + FIT + t             # руки: внутренняя и наружная грань по X
    d0 = HOLDER_PRELOAD + FIT * SLOPE                   # низ зацепа у грани руки: откос заходит на край крышки (x = W/2)
                                                        # на HOLDER_PRELOAD
    d_top = d0 - HOLDER_LIP * SLOPE - HOLDER_LIP_TOP    # верх рук (перед лицом крышки), глубина
    sw = HOLDER_STEM_W / 2
    xb = xo - HOLDER_BRACE_IN                       # начало косого утолщения перекладины
    yb = ya0 + HOLDER_BRACE_W                       # низ перекладины у ножки (к COB)
    plate = [(-xt, ya0), (xt, ya0), (xt, ya1), (xb, ya1), (sw, yb), (sw, H / 2 + t), (-sw, H / 2 + t), (-sw, yb),
             (-xb, ya1), (-xt, ya1)]   # T: перекладина с косыми утолщениями к ножке, ножка
    h = cq.Workplane("XY").polyline(plate).close().extrude(t).translate((0, 0, -(D + t)))
    h = h.union(blk(-sw, sw, H / 2, H / 2 + t, D - HOLDER_SHELF, D + t))   # полочка под корпусом
    for s in (-1, 1):
        a, b = sorted((s * xo, s * xt))
        h = h.union(blk(a, b, ya0, ya1, d_top, D + 0.01))                  # рука вдоль боковой стенки
        # Зацеп на лицо крышки: сечение (x, d), низ - откос от грани руки (зазор FIT) к кончику
        pts = [(xo, d0), (xo - HOLDER_LIP, d0 - HOLDER_LIP * SLOPE), (xo - HOLDER_LIP, d_top), (xo + 0.01, d_top),
               (xo + 0.01, d0)]
        lip = cq.Workplane("XZ").polyline([(s * x, -d) for x, d in pts]).close().extrude(HOLDER_W)
        bb = lip.val().BoundingBox()
        h = h.union(lip.translate((0, ya0 - bb.ymin, 0)))
    for x, y in holder_screws():
        cone = cq.Solid.makeCone(HOLDER_HEAD_D / 2, HOLDER_SCREW_D / 2, (HOLDER_HEAD_D - HOLDER_SCREW_D) / 2,
                                 cq.Vector(x, y, -D + 0.01), cq.Vector(0, 0, -1))
        h = h.cut(cq.Workplane().add(cone)).cut(cyl(x, y, HOLDER_SCREW_D, D - 1, D + t + 1))
    return h


def make_components():
    """Покупные детали (для проверки зазоров и сборки STEP)."""
    c = {}
    pb_d = DB - PB_RAISE   # макетка - первой: на виде спереди её закрывает RP2040-Zero
    c["protoboard"] = blk(PB_X0, PB_X0 + PB_W, PB_Y0, PB_Y0 + PB_H, pb_d - PB_T, pb_d)
    c["ld2410s"] = (blk(RX0, RX0 + RADAR_S, RY0, RY0 + RADAR_S, ANT_D, ANT_D + RADAR_T)
                    .union(blk(RX0 + RADAR_PINS[0], RX0 + RADAR_PINS[1], RY0 + RADAR_PINS[2], RY0 + RADAR_PINS[3],
                               ANT_D - RADAR_PINS_FRONT, ANT_D + RADAR_PINS_BACK)))
    ux0, ux1, uy0, uy1 = USB_BOX
    bx, by = RP_X0 + BOOT_XY[0], RP_YT - BOOT_XY[1]
    c["rp2040zero"] = (blk(RP_X0, RP_X0 + RP_W, RP_YT - RP_H, RP_YT, RP_D, RP_D + RP_T)
                       .union(blk(RP_X0 + ux0, RP_X0 + ux1, RP_YT - uy1, RP_YT - uy0, RP_D - USB_H, RP_D))
                       .union(blk(bx - 1.5, bx + 1.5, by - 2, by + 2, RP_D - BTN_H, RP_D)))
    c["hlk5ld05"] = blk(HLK_X0, HLK_X0 + HLK_S, HLK_Y0, HLK_Y0 + HLK_S, DB - HLK_H, DB)
    hx, hy = HLK_X0 + HLK_S / 2, HLK_Y0 + HLK_S / 2
    for px, py in HLK_PINS:
        c["hlk5ld05"] = c["hlk5ld05"].union(cyl(hx + px, hy + py, HLK_PIN_D, DB - HLK_H - HLK_PIN, DB - HLK_H + 0.01))
    py0 = PROF_Y1 - PROF_H
    pd1 = PROF_D0 + PROF_W
    c["profile"] = (blk(-PROF_L / 2, PROF_L / 2, PROF_Y1 - PROF_T, PROF_Y1, PROF_D0, pd1)
                    .union(blk(-PROF_L / 2, PROF_L / 2, py0, PROF_Y1, PROF_D0, PROF_D0 + PROF_T))
                    .union(blk(-PROF_L / 2, PROF_L / 2, py0, PROF_Y1, pd1 - PROF_T, pd1)))
    cy0 = PROF_Y1 + TAPE
    cd = PROF_D0 + PROF_W / 2
    c["cob"] = (blk(-COB_L / 2, COB_L / 2, cy0, cy0 + COB_T, PROF_D0, pd1)
                .union(blk(-EMIT_L / 2, EMIT_L / 2, cy0 + COB_T, cy0 + COB_T + EMIT_T, cd - EMIT_W / 2, cd + EMIT_W / 2)))
    return c


def vol(a, b):
    try:
        return a.intersect(b).val().Volume()
    except Exception:   # пустое пересечение
        return 0.0


def slices_with_material(part, x, y, thin, to_depth):
    """Сколько срезов слайсера содержат материал в точке (x, y) тонкого слоя толщиной thin у стола: [по верху, по середине].
    Picaso, похоже, режет по верху слоя (окно 0,20 при первом слое Z0.22 в G-коде отсутствует); режим слайсера в файле не
    указан, поэтому проверяются оба. to_depth(z) - глубина модели на высоте z над столом."""
    counts = []
    for first, second in ((SLICE_FIRST, SLICE_FIRST + SLICE_LAYER), (SLICE_FIRST / 2, SLICE_FIRST + SLICE_LAYER / 2)):
        n = 0
        z = first
        while z <= thin + 2 * SLICE_LAYER:   # выше - уже не тонкий слой
            d = to_depth(z)
            probe = blk(x - SLICE_PROBE, x + SLICE_PROBE, y - SLICE_PROBE, y + SLICE_PROBE, d - 0.005, d + 0.005)
            n += vol(part, probe) > 0.5 * probe.val().Volume()
            z = second if z == first else z + SLICE_LAYER
        counts.append(n)
    return counts


def slice_checks(printed):
    """Тонкие слои - по срезам настроек слайсера; узкие стенки - по ширине линии."""
    ok = True
    led_x, led_y = RP_X0 + LED_XY[0], RP_YT - LED_XY[1]
    for name, (fx, fy) in (("окна индикатора", (led_x, led_y)), ("над фоторезистором", LDR_XY)):
        counts = slices_with_material(printed["cover"], fx, fy, LED_WIN_T, lambda z: z)
        s_ok = counts == [1, 1]   # плёнка - ровно один слой
        print("[срезы %.2f/%.2f] плёнка %s %.2f мм, слоёв (по верху, по середине) %s: %s" %
              (SLICE_FIRST, SLICE_LAYER, name, LED_WIN_T, counts, "OK" if s_ok else "ЕСТЬ ПРОБЛЕМЫ"))
        ok &= s_ok
    counts = slices_with_material(printed["body"], WALL_HOLE_XY[0], WALL_HOLE_XY[1], KO_SKIN, lambda z: D - z)
    s_ok = min(counts) >= 1   # кожа кармана крепления лежит на столе
    print("[срезы %.2f/%.2f] кожа кармана на стену %.1f мм, слоёв %s: %s" %
          (SLICE_FIRST, SLICE_LAYER, KO_SKIN, counts, "OK" if s_ok else "ЕСТЬ ПРОБЛЕМЫ"))
    ok &= s_ok
    counts = slices_with_material(printed["cover"], RX0 + RADAR_S / 2, RY0 + RADAR_S / 2, RADOME_WALL, lambda z: z)
    s_ok = min(counts) >= 2   # стенка перед радаром - не меньше двух слоёв (пользователь)
    print("[срезы %.2f/%.2f] стенка перед радаром %.2f мм, слоёв %s: %s" %
          (SLICE_FIRST, SLICE_LAYER, RADOME_WALL, counts, "OK" if s_ok else "ЕСТЬ ПРОБЛЕМЫ"))
    ok &= s_ok
    for name, width, lines in (("кожа карманов у HLK", KO_SKIN, 1),
                               ("кольцо под фланец BOOT", (CAP_FL_D - CAP_HOLE_D) / 2, 2)):
        s_ok = width >= lines * SLICE_LINE
        print("[линии %.2f] %s %.2f мм, нужно линий %d: %s" % (SLICE_LINE, name, width, lines, "OK" if s_ok else "ЕСТЬ ПРОБЛЕМЫ"))
        ok &= s_ok
    return ok


def build_one():
    os.makedirs(OUT, exist_ok=True)
    ok = True
    printed = {"body": make_body(), "cover": make_cover(), "diffuser": make_diffuser(), "boot_cap": make_button(),
               "clamp_radar": make_clamp(MOUNTS["radar"]), "clamp_rp": make_clamp(MOUNTS["rp"]),
               "holder": make_holder()}
    comps = make_components()

    print("[габарит] %.0f x %.0f x %.0f мм, углы R%.0f; внутри по глубине %.1f мм" % (W, H, D, R, DB - T_FRONT))
    print("[радар] λ0 %.2f мм; стенка перед антенной %.2f мм, антенна от неё %.2f мм" %
          (LAMBDA0, RADOME_WALL, ANT_D - RADOME_WALL))
    print("[COB] окно %.0f x %.0f мм, рассеиватель %.0f x %.0f x %.1f мм + выступ в окно %.1f мм "
          "(diffuser_abs - без выступа), без рифов" %
          (2 * WIN_HALF, WIN_D1 - T_FRONT, 2 * DIFF_HALF, DIFF_D1 - T_FRONT - 0.1, DIFF_T, DIFF_PLUG))

    # 1. Пересечения (крышка закрыта): покупные с печатными, покупные между собой, печатные между собой
    allp = dict(printed)
    allp.update(comps)
    names = list(allp)
    touch = ({"cob", "profile"}, {"rp2040zero", "boot_cap"},   # COB на профиле, кнопка на кнопке платы - по замыслу
             {"holder", "cover"})                               # натяг зацепов держателя - отдельная проверка 2е
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            v = vol(allp[a], allp[b])
            if v > CHECK_VOL and {a, b} not in touch:
                print("  ПЕРЕСЕЧЕНИЕ %s - %s: %.2f мм³" % (a, b, v))
                ok = False
    print("[зазоры] детали и корпус: %s" % ("OK" if ok else "ЕСТЬ ПРОБЛЕМЫ"))

    # 2. Между антенной и крышкой - никаких покупных (металлических) деталей (руководство LD2410S гл. 10)
    zone = blk(RX0, RX0 + RADAR_S, RY0, RY0 + RADAR_S, RADOME_WALL + 0.01, ANT_D - 0.01)
    bad = [n for n, s in comps.items() if n != "ld2410s" and vol(zone, s) > CHECK_VOL]
    print("[радар] перед антенной пусто: %s" % ("OK" if not bad else "ЕСТЬ ПРОБЛЕМЫ " + ", ".join(bad)))
    ok &= not bad

    # 2а. Стойки и пластины не под трубкой фоторезистора (по всей глубине, зазор 0,5 мм)
    ldr = cyl(LDR_XY[0], LDR_XY[1], LDR_BOSS_D + 1.0, T_FRONT, DB - 0.01)
    v = sum(vol(ldr, printed[n]) for n in ("body", "clamp_radar", "clamp_rp"))
    print("[фоторезистор] стойки и пластины в стороне: %s" % ("OK" if v <= CHECK_VOL else "ЕСТЬ ПРОБЛЕМЫ"))
    ok &= v <= CHECK_VOL

    # 2б. Над макеткой до платы RP2040-Zero - место под детали (Q1, R1-R3, C1)
    pb_top = DB - PB_RAISE - PB_T
    zone_d0 = RP_D + RP_T + PB_ZONE_GAP
    zone = blk(PB_X0, PB_X0 + PB_W, PB_Y0 + PB_END - 1.27, PB_Y0 + PB_H, zone_d0, pb_top - 0.01)   # от столбца A
    bad = [n for n, s in allp.items() if n != "protoboard" and vol(zone, s) > CHECK_VOL]
    print("[макетка] %.1f x %.1f мм, над ней свободно %.1f мм: %s" %
          (PB_W, PB_H, pb_top - zone_d0, "OK" if not bad else "ЕСТЬ ПРОБЛЕМЫ " + ", ".join(bad)))
    ok &= not bad

    # 2в. Окно индикатора не задевает зону под кнопку: кольцо отверстие - фланец колпачка в крышке полной толщины
    bx, by = RP_X0 + BOOT_XY[0], RP_YT - BOOT_XY[1]
    seat = cyl(bx, by, CAP_FL_D, 0, T_FRONT).cut(cyl(bx, by, CAP_HOLE_D, -1, T_FRONT + 1))
    v = seat.val().Volume() - vol(seat, printed["cover"])
    print("[индикатор] зона под кнопку BOOT цела: %s" % ("OK" if v <= CHECK_VOL else "ЕСТЬ ПРОБЛЕМЫ %.2f мм³" % v))
    ok &= v <= CHECK_VOL

    # 2г. Тонкие элементы при настройках слайсера
    ok &= slice_checks(printed)

    # 2д. Отверстия под саморезы в крышке - сквозные (без материала по оси на всю толщину)
    v = sum(vol(cyl(x, y, SCREW_HOLE_D - 0.1, 0, T_FRONT), printed["cover"]) for x, y in boss_xy())
    print("[крепёж] отверстия в крышке сквозные: %s" % ("OK" if v <= CHECK_VOL else "ЕСТЬ ПРОБЛЕМЫ %.2f мм³" % v))
    ok &= v <= CHECK_VOL

    # 2е. T-держатель: отверстие под провод у HLK открыто (провод выходит наружу), окно COB не закрыто полочкой,
    # корпус проходит между зацепами рук (вставляется вдоль Y)
    ya0, ya1 = holder_arm_y()
    hole = (cq.Workplane("YZ").workplane(offset=W / 2 - 1).center(KO_SIDE_Y, -KO_DEPTH)
            .circle(KO_D / 2 + HOLDER_KO_GAP / 2).extrude(30))
    v_ko = vol(hole, printed["holder"])
    v_win = vol(blk(-WIN_HALF, WIN_HALF, H / 2 - 0.01, H / 2 + 10, -1, WIN_D1), printed["holder"])
    # Натяг: клин откоса зацепа на краю крышки, обе руки (сечение - треугольник PRELOAD x PRELOAD / SLOPE)
    v_pre = 2 * HOLDER_W * HOLDER_PRELOAD * (HOLDER_PRELOAD / SLOPE) / 2
    v_cov = vol(printed["cover"], printed["holder"])   # на месте
    v_pass = vol(rrect(W, H, R, 0, D).translate((0, -15.0, 0)), printed["holder"])   # корпус на полпути в руках
    pre_ok = v_cov <= 2 * v_pre and v_pass <= 2 * v_pre
    print("[держатель] руки %.1f..%.1f мм от COB-края (в работе - снизу), отверстие у HLK %.1f мм (до рук %.1f): "
          "свободно %s; окно COB: %s; натяг зацепов %.1f мм (пересечение %.2f / %.2f мм³, расчёт %.2f): %s" %
          (H / 2 - ya1, H / 2 - ya0, H / 2 - KO_SIDE_Y, HOLDER_KO_GAP,
           "OK" if v_ko <= CHECK_VOL else "НЕТ", "OK" if v_win <= CHECK_VOL else "ЗАКРЫТО",
           HOLDER_PRELOAD, v_cov, v_pass, v_pre, "OK" if pre_ok else "ЕСТЬ ПРОБЛЕМЫ"))
    ok &= v_ko <= CHECK_VOL and v_win <= CHECK_VOL and pre_ok

    # 3. Саморезы: длина в стойке не больше пилота (крышка, прижимная пластина - потай; макетка - без зенковки)
    for name, length, t, pilot in (("крышка", SCREW_L, T_FRONT - SINK, PILOT_DEPTH),
                                   ("пластина", SCREW_L, CLAMP_T - SINK, PILOT_DEPTH),
                                   ("макетка", PB_SCREW_L, PB_T, PB_PILOT)):
        engage = length - t
        s_ok = engage + 0.5 <= pilot
        print("[крепёж] %s, саморез 2,5 x %.0f: в стойке %.1f мм, пилот %.1f мм: %s" %
              (name, length, engage, pilot, "OK" if s_ok else "ЕСТЬ ПРОБЛЕМЫ"))
        ok &= s_ok

    # 4. Экспорт в положении печати и проверка нависаний
    place = {
        "body": printed["body"],                                               # основанием на стол
        "cover": printed["cover"].rotate((0, 0, 0), (1, 0, 0), 180),          # лицом на стол
        "diffuser": printed["diffuser"].rotate((0, 0, 0), (1, 0, 0), 90),     # стороной к COB на стол, лицо (выступ)
                                                                               # вверх - шершавый верхний слой
        "diffuser_abs": make_diffuser(False).rotate((0, 0, 0), (1, 0, 0), 90),   # тонкий (ABS), так же
        "boot_cap": printed["boot_cap"],                                       # фланцем на стол
        "clamp_radar": printed["clamp_radar"],                                 # зенковкой вверх
        "clamp_rp": printed["clamp_rp"],                                       # зенковкой вверх
        "holder": printed["holder"],                                           # пластиной на стол, руки и полочка вверх
    }
    for n, s in place.items():
        bb = s.val().BoundingBox()
        s = s.translate((0, 0, -bb.zmin))
        path = os.path.join(OUT, n + ".stl")
        cq.exporters.export(s, path, tolerance=0.02, angularTolerance=0.1)
        spots = stl_overhang.overhangs(path)
        if spots:
            print("[печать] %s: нависания без опоры:" % n)
            print("\n".join(stl_overhang.report(spots)))
            ok = False
        else:
            print("[печать] %s: без поддержек OK" % n)

    asm = cq.Assembly()
    for n, s in allp.items():
        asm.add(s, name=n)
    asm.save(os.path.join(OUT, "_assembly.step"))
    layout(printed, comps, os.path.join(OUT, "_layout.png"))
    json.dump({"outer_mm": [W, H, D], "corner_r": R, "inner_depth": DB - T_FRONT, "radome_wall": RADOME_WALL,
               "antenna_gap": round(ANT_D - RADOME_WALL, 2), "screws": "2.5x%.0f x6 (крышка 4, пластины 2), потай" % SCREW_L},
              open(os.path.join(OUT, "_dims.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    return ok


def layout(printed, comps, path):
    """Вид спереди без крышки (без основания) и разрезы по X через радар и кнопку BOOT."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    colors = {"body": "#d0d0d0", "cover": "#b0c4de", "diffuser": "#fff3b0", "clamp_radar": "#8c564b",
              "clamp_rp": "#8c564b", "ld2410s": "#3a7bd5","rp2040zero": "#2e8b57",
              "boot_cap": "#d62728", "hlk5ld05": "#444444", "profile": "#9aa5b1", "cob": "#ffcc00",
              "protoboard": "#c8a165", "holder": "#6a3d9a"}

    def polys(shape, proj):
        verts, faces = cq.Compound.makeCompound(shape.vals()).tessellate(0.2)
        return [[proj(verts[i]) for i in f] for f in faces]

    fig, axes = plt.subplots(1, 3, figsize=(17, 5.5), gridspec_kw={"width_ratios": [W, 45, 45]})
    ax = axes[0]
    inner = {n: s for n, s in printed.items() if n != "cover"}
    inner.update(comps)
    cut = blk(-W, W, -H, H, -1, DB - 0.05)   # без основания - видно стойки и рёбра
    for n, s in inner.items():
        s = s.intersect(cut) if n == "body" else s
        ax.add_collection(PolyCollection(polys(s, lambda v: (v.x, v.y)), facecolor=colors[n], edgecolor="none",
                                         alpha=0.9 if n != "body" else 0.6))
    ax.set_title("вид спереди без крышки")
    ax.set_xlim(-W / 2 - 3, W / 2 + 3)
    ax.set_ylim(-H / 2 - 3, H / 2 + 3)

    for ax, x, title in ((axes[1], RX0 + RADAR_SO_X - 1, "разрез X = %.1f (стойка радара)"),
                         (axes[2], RP_X0 + BOOT_XY[0], "разрез X = %.1f (кнопка BOOT)")):
        slab = blk(x - 0.05, x + 0.05, -H, H, -10, D + 1)
        allp = dict(printed)
        allp.update(comps)
        for n, s in allp.items():
            try:
                sec = s.intersect(slab)
                if not sec.vals():
                    continue
                ax.add_collection(PolyCollection(polys(sec, lambda v: (v.z, v.y)), facecolor=colors[n], edgecolor="k",
                                                 linewidth=0.1))
            except Exception:
                pass
        ax.set_title(title % x)
        ax.set_xlim(-D - 2, 10)
        ax.set_ylim(-H / 2 - 3, H / 2 + 3)
        ax.set_xlabel("Z (крышка - справа, Z = 0)")
    for ax in axes:
        ax.set_aspect("equal")
        ax.grid(True, linewidth=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    print("OK" if build_one() else "ЕСТЬ ПРОБЛЕМЫ")
