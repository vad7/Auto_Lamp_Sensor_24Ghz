# Принципиальные схемы (условные обозначения), общее - RP2040-Zero, HLK-LD2410S, фоторезистор:
#   лампа 230 В - конденсаторный блок питания с контролем стабилитрона, MOC3043M -> Images/schematic.svg;
#   ночник - HLK-5LD05, COB 5 В через P75N02LDG (ШИМ)                           -> Images/schematic_night.svg.
#   python Scripts/schematic.py   (PNG для проверки глазами - work/png/)
# Требуется: pip install schemdraw (0.23). Выводы - по Readme «Распиновка»; детали - Docs/ (MOC304xM, HLK-5LD05,
# P75N02LDG), расчёты - Readme «Блок питания», «Ночник».
import os

import schemdraw
import schemdraw.elements as elm

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_NAMES = {False: "schematic", True: "schematic_night"}   # вариант (ночник?) -> имя файла схемы
PIN_STEP = 1.2      # шаг выводов микросхем на схеме
FONT = 11           # размер шрифта
BRIDGE = 4.24       # диагональ моста schemdraw (от минуса до плюса)

schemdraw.config(fontsize=FONT)


def power_supply(d, out, adc):
    # Конденсаторный блок: L - R3 - (C1 || R4) - мост - D1 (основной) и D2 (запасной) стабилитроны, C2 - D3 - вывод 5V
    # (VSYS); делитель R5 / R6 (C3) с узла стабилитронов на вход АЦП adc.
    # Мост schemdraw: плюс справа (E), минус слева (W), переменка сверху (N) и снизу (S).
    bx = out[0] - 14
    br = d.add(elm.Rectifier().right().anchor("E").at((bx, out[1])))
    d.add(elm.Label().at((bx - BRIDGE / 2, out[1])).label("BR1\n2W10"))
    d.add(elm.Ground().at(br.W))

    # Выход: плюс моста - D1 4,3 В (основной), D2 5,1 В (запасной: при обрыве D1 напряжение растёт - видит АЦП),
    # C2 на землю - делитель - D3 (5 В от USB не идут в стабилитроны) - вывод 5V
    d.add(elm.Line().right(1.5).at(br.E))
    d.add(elm.Dot())
    d.add(elm.Zener().down(3).reverse().label("D1\n1N4731A\n4,3 В", loc="bottom"))
    d.add(elm.Ground())
    d.add(elm.Line().right(2.6).at((bx + 1.5, out[1])))
    d.add(elm.Dot())
    d.add(elm.Zener().down(3).reverse().label("D2\n1N4733A\n5,1 В", loc="bottom"))
    d.add(elm.Ground())
    d.add(elm.Line().right(2.6).at((bx + 4.1, out[1])))
    d.add(elm.Dot())
    d.add(elm.Capacitor(polar=True).down(3).label("C2\n470 мкФ\n16 В", loc="bottom"))
    d.add(elm.Ground())
    d.add(elm.Line().right(2.4).at((bx + 6.7, out[1])))
    node = d.add(elm.Dot())
    d.add(elm.Schottky().right(3).label("D3 1N5819", loc="top"))
    d.add(elm.Line().endpoints(d.here, out))

    # Делитель на АЦП: R5 сверху, R6 и C3 на землю
    d.add(elm.Resistor().down(4.5).at(node.center).label("R5\n100 кОм", loc="bottom"))
    j = d.add(elm.Dot())
    d.add(elm.Resistor().down(2.5).label("R6\n150 кОм", loc="bottom"))
    d.add(elm.Ground())
    d.add(elm.Line().right(2.4).at(j.center))
    k = d.add(elm.Dot())
    d.add(elm.Capacitor().down(2.5).label("C3\n100 нФ", loc="bottom"))
    d.add(elm.Ground())
    d.add(elm.Line().at(k.center).tox(adc[0]))
    d.add(elm.Line().toy(adc[1]))

    # Сеть: L через R3 и C1 (R4 - разряд C1) на переменку сверху, N на переменку снизу
    x0 = br.W[0] - 9
    d.add(elm.Dot(open=True).at((x0, br.N[1])).label("L  230 В~", loc="left"))
    d.add(elm.Resistor().right(3).label("R3\n15 Ом 1 Вт", loc="bottom"))
    a = d.add(elm.Dot())
    d.add(elm.Capacitor().right(3).label("C1\n0,68 мкФ\nX2 275 В~", loc="bottom"))
    b = d.add(elm.Dot())
    d.add(elm.Line().endpoints(b.center, br.N))
    d.add(elm.Line().up(1.6).at(a.center))
    d.add(elm.Resistor().right(3).label("R4 1 МОм", loc="top"))
    d.add(elm.Line().toy(b.center[1]))
    d.add(elm.Dot(open=True).at((x0, br.S[1])).label("N", loc="left"))
    d.add(elm.Line().tox(br.S[0]))


def power_night(d, out):
    # Блок питания ночника: L - F1 - HLK-5LD05 (развязка 2500 В~) - C1 - вывод 5V; RV1 - выбросы сети.
    # F1 и RV1 даташит HLK-5LD05 требует обязательно (гл. 6.1), C1 - гл. 6.2.
    psu = d.add(elm.Ic(pins=[
        elm.IcPin(name="L", pin="2", side="left", slot="2/2"),
        elm.IcPin(name="N", pin="1", side="left", slot="1/2"),
        elm.IcPin(name="+Vo", pin="5", side="right", slot="2/2"),
        elm.IcPin(name="-Vo", pin="4", side="right", slot="1/2"),
    ], pinspacing=3, edgepadW=1.5, leadlen=0.6, label="U4\nHLK-5LD05\n5 В 1 А")
        .right().anchor("+Vo").at((out[0] - 7, out[1])))
    d.add(elm.Line().right(2).at(getattr(psu, "+Vo")))
    c = d.add(elm.Dot())
    d.add(elm.Capacitor(polar=True).down(3).label("C1\n220 мкФ\n10 В", loc="bottom"))
    d.add(elm.Ground())
    d.add(elm.Line().endpoints(c.center, out).label("+5V", loc="top"))
    d.add(elm.Line().right(0.6).at(getattr(psu, "-Vo")))
    d.add(elm.Ground())

    x0 = psu.L[0] - 7
    d.add(elm.Dot(open=True).at((x0, psu.L[1])).label("L  230 В~", loc="left"))
    d.add(elm.Fuse().right(3).label("F1\nT1A 250 В", loc="top"))
    a = d.add(elm.Dot())
    d.add(elm.Line().endpoints(a.center, psu.L))
    d.add(elm.Resistor().down().at(a.center).toy(psu.N[1]).label("RV1\n10D561K", loc="bottom"))
    d.add(elm.Dot())
    d.add(elm.Dot(open=True).at((x0, psu.N[1])).label("N", loc="left"))
    d.add(elm.Line().tox(psu.N[0]))


def lamp_key(d, gp4):
    # Ключ лампы 230 В: GP4 - R2 - светодиод MOC3043M (1 анод, 2 катод), выходы 6, 4 - к силовой части
    d.add(elm.Line().down(3.6).at(gp4))
    d.add(elm.Resistor().right(3).label("R2\n180 Ом", loc="bottom"))
    opto = d.add(elm.Ic(pins=[
        elm.IcPin(name="A", pin="1", side="left", slot="2/2"),
        elm.IcPin(name="K", pin="2", side="left", slot="1/2"),
        elm.IcPin(name="MT2", pin="6", side="right", slot="2/2"),
        elm.IcPin(name="MT1", pin="4", side="right", slot="1/2"),
    ], pinspacing=PIN_STEP, edgepadW=1.2, leadlen=0.6, label="U3\nMOC3043M").right().anchor("A"))
    d.add(elm.Line().left(0.6).at(opto.K))
    d.add(elm.Ground())
    d.add(elm.Line().right(1.2).at(opto.MT2))
    d.add(elm.Dot(open=True).label("к симистору 230 В", loc="right"))
    d.add(elm.Line().right(1.2).at(opto.MT1))
    d.add(elm.Dot(open=True).label("(силовая часть)", loc="right"))


def night_key(d, gp4):
    # Ночник: GP4 (ШИМ) - R2 - затвор Q1 (R3 держит закрытым без МК); сток - COB (резисторы 2,4 Ом на модуле) - +5V
    d.add(elm.Line().down(3.6).at(gp4))
    d.add(elm.Resistor().right(3).label("R2\n100 Ом", loc="bottom"))
    g = d.add(elm.Dot())
    d.add(elm.Resistor().down(3).at(g.center).label("R3\n100 кОм", loc="bottom"))
    d.add(elm.Ground())
    d.add(elm.Line().right(1.5).at(g.center))
    q = d.add(elm.NFet().reverse().anchor("gate").label("Q1\nP75N02LDG\n(1 G, 2 D, 3 S)", loc="right"))
    d.add(elm.Ground().at(q.source))
    d.add(elm.LED().up(3).at(q.drain).reverse().label("HL1\nCOB 6015\n5 В 2 Вт", loc="bottom"))
    d.add(elm.Vdd().label("+5V"))


def draw(night):
    d = schemdraw.Drawing()
    # RP2040-Zero: слева питание, справа сигналы (порядок выводов - под разводку без пересечений)
    mcu = d.add(elm.Ic(pins=[
        elm.IcPin(name="3V3", side="left", slot="3/3"),
        elm.IcPin(name="5V", side="left", slot="2/3"),
        elm.IcPin(name="GND", side="left", slot="1/3"),
        elm.IcPin(name="GP26", side="right", slot="5/5"),
        elm.IcPin(name="GP0", side="right", slot="4/5"),
        elm.IcPin(name="GP1", side="right", slot="3/5"),
        elm.IcPin(name="GP2", side="right", slot="2/5"),
        elm.IcPin(name="GP4", side="right", slot="1/5"),
    ] + ([elm.IcPin(name="GP7", side="bottom", slot="1/2"), elm.IcPin(name="GP8", side="bottom", slot="2/2")] if night
         else [elm.IcPin(name="GP28", side="bottom", slot="1/3"), elm.IcPin(name="GP7", side="bottom", slot="2/3"),
               elm.IcPin(name="GP8", side="bottom", slot="3/3")]),
        pinspacing=PIN_STEP, edgepadW=4.0, leadlen=0.6, label="U1\nRP2040-Zero\n(3V3 - выход\nRT9013 платы)").right())
    d.add(elm.Line().left(0.6).at(mcu.GND))
    d.add(elm.Ground())
    if night:
        power_night(d, getattr(mcu, "5V"))
    else:
        power_supply(d, getattr(mcu, "5V"), mcu.GP28)

    # HLK-LD2410S (J2): RX, OT1, OT2 - напротив GP0, GP1, GP2
    ld = d.add(elm.Ic(pins=[
        elm.IcPin(name="RX", pin="4", side="left", slot="3/3"),
        elm.IcPin(name="OT1", pin="3", side="left", slot="2/3"),
        elm.IcPin(name="OT2", pin="5", side="left", slot="1/3"),
        elm.IcPin(name="3V3", pin="1", side="right", slot="2/2"),
        elm.IcPin(name="GND", pin="2", side="right", slot="1/2"),
    ], pinspacing=PIN_STEP, edgepadW=3.0, leadlen=0.6, label="U2\nHLK-LD2410S")
        .right().anchor("RX").at((mcu.GP0[0] + 7, mcu.GP0[1])))   # направление явно: иначе - от прошлого элемента
    d.add(elm.Line().endpoints(mcu.GP0, ld.RX).label("TX", loc="bottom"))
    d.add(elm.Line().endpoints(mcu.GP1, ld.OT1).label("RX", loc="bottom"))
    d.add(elm.Line().endpoints(mcu.GP2, ld.OT2).label("OT2 - присутствие", loc="bottom"))
    d.add(elm.Line().right(0.6).at(ld.GND))
    d.add(elm.Ground())

    # Общая цепь 3V3 (выход стабилизатора платы): RP2040, LD2410S, верх фоторезистора
    y_top = mcu.GP26[1] + 3
    mcu_3v3 = getattr(mcu, "3V3")
    d.add(elm.Line().left(0.6).at(mcu_3v3))
    d.add(elm.Line().toy(y_top))
    rail_left = d.here
    d.add(elm.Line().right(0.6).at(getattr(ld, "3V3")))
    d.add(elm.Line().toy(y_top))
    d.add(elm.Line().endpoints(rail_left, d.here).label("3V3 - общее питание RP2040 и LD2410S", loc="top"))

    # Делитель фоторезистора: GP26 - узел; LDR к 3V3, R1 к GND
    d.add(elm.Line().right(1.5).at(mcu.GP26))
    node = d.add(elm.Dot())
    d.add(elm.Photoresistor().up().toy(y_top).at(node.center).label("LDR", loc="bottom"))
    d.add(elm.Dot())
    d.add(elm.Resistor().right(3).at(node.center).label("R1 10 кОм*", loc="top"))
    d.add(elm.Ground())

    (night_key if night else lamp_key)(d, mcu.GP4)

    # J1: перемычка GP7 - GP8 (соседние выводы в углу платы) - замкнута при включении: консоль настройки по USB
    d.add(elm.Line().down(1.2).at(mcu.GP7))
    j_a = d.here
    d.add(elm.Line().down(1.2).at(mcu.GP8))
    d.add(elm.Switch().endpoints(j_a, d.here).label("J1 перемычка:\nзамкнуть - настройка по USB", loc="bottom"))
    return d


def main():
    os.makedirs(os.path.join(ROOT, "Images"), exist_ok=True)
    os.makedirs(os.path.join(ROOT, "work", "png"), exist_ok=True)
    for night, name in OUT_NAMES.items():
        d = draw(night)
        svg = os.path.join(ROOT, "Images", name + ".svg")
        d.save(svg)
        d.save(os.path.join(ROOT, "work", "png", name + ".png"), dpi=110)
        print("Схема: " + svg)


if __name__ == "__main__":
    main()
