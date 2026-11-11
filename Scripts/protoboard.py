# Раскладка деталей ночника на обрезке макетки 2 x 8 см (Q1 P75N02LDG, R1-R3, C1) -> Images/protoboard.svg.
#   python Scripts/protoboard.py   (PNG для проверки глазами - work/png/protoboard.png)
# Вид со стороны деталей (сторона с надписями - фото work/protoboard/photo_end.png) - как макетка стоит в корпусе
# (спереди, при снятой RP2040-Zero): заводской торец с крепёжными отверстиями - вниз, верх - к COB, слева - радар.
# Отверстия: столбцы A-G (A - у торца), ряды 1-6 (как напечатано на плате), шаг 2,54 мм. Размеры обрезка и опор -
# Scripts/enclosure.py (PB_*); корпус Q1 - Docs/P75N02LDG.PDF, стр. 5; цепи - Images/schematic_night.svg.
# Требуется: matplotlib.
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PITCH = 2.54            # шаг отверстий, мм
LETTERS, ROWS = "ABCDEFG", 6   # столбцы (от торца) и ряды на плате
BOARD_W = 20.0          # заводская ширина макетки, мм
BOARD_H = 22.1          # от торца до реза за столбцом G, мм - ОЦЕНКА по фото
END = 5.6               # торец - столбец A, мм - ОЦЕНКА по фото
MARGIN = (BOARD_W - (ROWS - 1) * PITCH) / 2   # ряд 6 от левой кромки, мм
MOUNT = (1.85, 2.1)     # крепёжные отверстия у торца: от боковой кромки, от торца, мм - ОЦЕНКА по фото
MOUNT_D = 2.3           # их диаметр по фото, мм - ОЦЕНКА
POST = 3.0              # опоры корпуса под углами (под макеткой), мм
SCREW_POST_D = 5.0      # опора под саморез, мм
PAD_R = 0.75            # радиус площадки, мм
DPAK_BODY = (6.5, 6.1)  # Q1 TO-252: корпус I x G (стр. 5), мм
DPAK_TAB = 1.0          # выступ пластины стока за корпус (H), мм
C1_SIZE = (11.0, 5.0)   # C1 220 мкФ 10 В лёжа: длина x диаметр, мм (Ø5 x 11)

# Цепи: перемычки снизу (проводом или каплей припоя) между отверстиями, цвет
NETS = {
    "+5V": ("#d62728", ["F6-F5", "F5-G5", "G5-G4", "G4-G3"]),
    "GND": ("#000000", ["E6-D6", "E6-E5", "E5-E4", "E4-E3", "E3-E2", "E2-E1", "E1-D1", "D1-C1", "C1-B1",
                        "C2-C1"]),
    "затвор": ("#1f77b4", ["D4-D3", "D3-D2"]),
    "GP4": ("#ff7f0e", ["C3-B3"]),
    "GP26": ("#2ca02c", ["B2-A2", "A2-A1"]),
    "COB -": ("#9467bd", ["A3-A4"]),
}
# Провода к макетке: отверстие -> подпись
WIRES = {
    "G5": "+5V <- HLK +Vo", "G4": "+5V -> RP 5V", "G3": "+5V -> COB +",
    "C1": "GND <- HLK -Vo", "B1": "GND -> RP GND", "E1": "GND -> LD2410S J2-2", "B3": "GP4 <- RP", "A3": "COB -",
    "A2": "GP26 <- RP", "A1": "LDR (2-й вывод - к 3V3 RP)",
}
# Стоящие резисторы: (отверстие под корпусом, отверстие загнутого вывода, подпись, промежуток рядов для выноски:
# -1 - к столбцу G, +1 - к столбцу A; выбран так, чтобы выноска не пересекала выводы)
# последнее поле - отверстие провода, выноске которого параллелен конец выноски резистора (None - горизонтально)
RESISTORS = [("C3", "D3", "R2 100 Ом", -1, None), ("D1", "D2", "R3 100 кОм", -1, None),
             ("B2", "C2", "R1 10 кОм*", +1, "B3")]
R_BODY = 1.15           # корпус стоящего резистора на рисунке, радиус, мм
R_LEAD_W = 3.0          # толщина линии загнутого вывода, пт
R_DOT = 0.38            # точка вывода в отверстии, радиус, мм


def hole(name):
    """Центр отверстия по имени (A1 ... G6), мм: X вправо, Y вниз от реза."""
    x = MARGIN + (ROWS - int(name[1:])) * PITCH
    return x, BOARD_H - END - LETTERS.index(name[0]) * PITCH


def main():
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.add_patch(Rectangle((0, 0), BOARD_W, BOARD_H, facecolor="#2e8b57", edgecolor="k", alpha=0.25))

    # Опоры корпуса под макеткой (штриховка), крепёжные отверстия у торца; саморез - в левое
    hatch = dict(fill=False, hatch="///", linestyle="--", edgecolor="#777777")
    for x, y in ((0, 0), (BOARD_W - POST, 0), (BOARD_W - POST, BOARD_H - POST)):
        ax.add_patch(Rectangle((x, y), POST, POST, **hatch))
    for x in (MOUNT[0], BOARD_W - MOUNT[0]):
        ax.add_patch(Circle((x, BOARD_H - MOUNT[1]), MOUNT_D / 2, facecolor="white", edgecolor="k", zorder=3))
    sx, sy = MOUNT[0], BOARD_H - MOUNT[1]
    ax.add_patch(Circle((sx, sy), SCREW_POST_D / 2, **hatch))
    ax.annotate("саморез 2,5 x 8\n(отверстие платы,\nрассверлить до 2,7)", (sx, sy), (-4, BOARD_H + 1.5),
                fontsize=8, ha="right", va="center", arrowprops=dict(arrowstyle="-", lw=0.6))

    for c in LETTERS:
        for r in range(1, ROWS + 1):
            ax.add_patch(Circle(hole("%s%d" % (c, r)), PAD_R, facecolor="#d4a017", edgecolor="#8a6d0b", lw=0.5))
    for c in LETTERS:
        ax.text(-0.8, hole(c + "1")[1], c, va="center", ha="right", fontsize=9)
    for r in range(1, ROWS + 1):
        ax.text(hole("G%d" % r)[0], -0.6, str(r), ha="center", fontsize=9)

    # Перемычки снизу
    for net, (color, links) in NETS.items():
        for k, link in enumerate(links):
            a, b = (hole(h) for h in link.split("-"))
            ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=2.2, linestyle=(0, (3, 1.5)), zorder=3,
                    label=net if k == 0 else None)

    # C1 лёжа: выводы F6 (+), E6 (-), корпус вправо над рядами 5-2
    (lx, fy), (_, ey) = hole("F6"), hole("E6")
    cy = (fy + ey) / 2
    cx0 = lx + 1.1
    ax.add_patch(FancyBboxPatch((cx0, cy - C1_SIZE[1] / 2), C1_SIZE[0], C1_SIZE[1],
                                boxstyle="round,pad=0,rounding_size=1.2", facecolor="#4a6fa5", alpha=0.85, zorder=4))
    for y in (fy, ey):
        ax.plot([lx, cx0], [y, y], color="#555555", lw=1.2, zorder=4)
    ax.text(cx0 + C1_SIZE[0] / 2, cy, "C1 220 мкФ 10 В\n(лёжа, Ø5 x 11)", ha="center", va="center", color="white",
            fontsize=8, zorder=5)
    ax.text(lx - 1.6, fy, "+", fontsize=10, va="center", fontweight="bold", zorder=5)

    # Q1 TO-252: выводы вверх (3 - исток D6, 2 - срезать, 1 - затвор D4), корпус над A-C, пластина стока - к торцу
    (x6, dy), (x4, _) = hole("D6"), hole("D4")
    qx = (x6 + x4) / 2
    qy0 = dy + 1.0
    ax.add_patch(Rectangle((qx - DPAK_BODY[0] / 2, qy0), DPAK_BODY[0], DPAK_BODY[1], facecolor="#333333", zorder=4))
    ax.add_patch(Rectangle((qx - 2.6, qy0 + DPAK_BODY[1]), 5.2, DPAK_TAB, facecolor="#b0b0b0", edgecolor="k",
                           lw=0.5, zorder=4))
    for x in (x6, x4):
        ax.plot([x, x], [dy, qy0], color="#b0b0b0", lw=3, zorder=4)
    ax.plot([qx, qx], [qy0 - 0.4, qy0], color="#b0b0b0", lw=3, zorder=4)
    ax.text(qx, qy0 + DPAK_BODY[1] / 2, "Q1\nP75N02LDG", ha="center", va="center", color="white", fontsize=8,
            zorder=5)
    ax.annotate("сток: пластина - припаять\nснизу через A4-A6", hole("A5"), (-4, hole("A5")[1] + 1.0), fontsize=8,
                ha="right", va="center", arrowprops=dict(arrowstyle="-", lw=0.6))
    ax.text(0, BOARD_H + 4.5, "Q1: вывод 3 (исток) - D6, 1 (затвор) - D4, средний 2 - срезать (сток - пластина).\n"
            "Штриховка - опоры корпуса под макеткой: на этих отверстиях снизу ни припоя, ни перемычек.\n"
            "Резисторы - стоя (высота до 9,7 мм - плата RP2040-Zero), C1 - лёжа. Размеры обрезка - по фото, оценка.",
            fontsize=8, va="top")

    # Стоящие резисторы: корпус над первым отверстием, второй вывод загнут во второе (жирная линия, точки - выводы
    # в отверстиях); выноска - по диагонали в промежуток между рядами и по нему к правой кромке
    labels = []         # подписи справа: (y на кромке, текст, точки выноски, жирно, отверстие провода для параллели)
    for a, b, text, side, par in RESISTORS:
        (ax_, ay), (bx_, by_) = hole(a), hole(b)
        ax.plot([ax_, bx_], [ay, by_], color="#222222", lw=R_LEAD_W, solid_capstyle="round", zorder=6)
        ax.add_patch(Circle((ax_, ay), R_BODY, facecolor="#e8c48a", edgecolor="#7a5a20", lw=1.5, zorder=6))
        for x, y in ((ax_, ay), (bx_, by_)):
            ax.add_patch(Circle((x, y), R_DOT, facecolor="#222222", zorder=7))
        gy = ay + side * PITCH / 2
        labels.append((gy, "%s: корпус %s, вывод в %s" % (text, a, b),
                       [(ax_ + R_BODY * 0.7, ay + side * R_BODY * 0.7), (ax_ + PITCH / 2, gy), (BOARD_W + 0.3, gy)],
                       True, par))

    # Провода: белый кружок - отверстие провода, тонкая выноска - прямо к подписи
    for h, text in WIRES.items():
        x, y = hole(h)
        ax.add_patch(Circle((x, y), PAD_R * 0.6, facecolor="white", edgecolor="k", zorder=7))
        labels.append((y, "%s: %s" % (h, text), [(x, y)], False, None))

    # Подписи справа столбиком сверху вниз, не ближе 1,6 мм друг к другу
    lx = BOARD_W + 1.8  # конец выносок у подписей, мм
    ty, placed = -3.5, []
    for lab in sorted(labels, key=lambda v: (v[0], -v[2][0][0])):
        ty = max(lab[0], ty + 1.6)
        placed.append((ty, lab))
    wire_ty = {lab[1].split(":")[0]: (ty, lab[2][0]) for ty, lab in placed if not lab[3]}
    for ty, (y, text, pts, bold, par) in placed:
        if par:     # последний отрезок - к выноске провода par, чтобы выноски не сходились
            wty, (wx, wy) = wire_ty[par]
            px, py = pts[-2]
            slope = (wty - wy) / (lx - wx)
            kx = lx - (ty - py) / slope if slope else px
            pts = pts[:-1] + [(max(kx, px), py)]
        xs, ys = zip(*pts, (lx, ty))
        ax.plot(xs, ys, color="#7a5a20" if bold else "#444444", lw=0.9 if bold else 0.5, zorder=8)
        ax.text(BOARD_W + 2, ty, text, fontsize=8, ha="left", va="center", zorder=8,
                fontweight="bold" if bold else "normal", color="#5a3d00" if bold else "black")

    ax.legend(title="перемычки снизу", loc="upper left", bbox_to_anchor=(1.0, 0.75), fontsize=8, title_fontsize=8)
    ax.text(0, BOARD_H + 9.0, "Резистор: кружок - корпус стоя над первым отверстием, жирная линия - загнутый вывод, "
            "чёрные точки - выводы в отверстиях.\nБелый кружок - отверстие провода (тонкая выноска к подписи справа).",
            fontsize=8, va="top")
    ax.set_title("Макетка ночника %.0f x %.1f мм - вид со стороны деталей (как в корпусе)" % (BOARD_W, BOARD_H),
                 fontsize=10)
    ax.text(BOARD_W / 2, BOARD_H + 0.9, "заводской торец (крепёжные отверстия, площадки)", ha="center", fontsize=7,
            color="#555555")
    ax.text(BOARD_W / 2, -2.2, "рез за столбцом G (к COB)", ha="center", fontsize=7, color="#555555")
    ax.set_xlim(-17, BOARD_W + 28)
    ax.set_ylim(BOARD_H + 13, -5)
    ax.set_aspect("equal")
    ax.axis("off")
    fig.tight_layout()
    os.makedirs(os.path.join(ROOT, "work", "png"), exist_ok=True)
    for path in (os.path.join(ROOT, "Images", "protoboard.svg"), os.path.join(ROOT, "work", "png", "protoboard.png")):
        fig.savefig(path, dpi=150, metadata={"Date": None} if path.endswith(".svg") else None)
        print("Готово:", path)


if __name__ == "__main__":
    main()
