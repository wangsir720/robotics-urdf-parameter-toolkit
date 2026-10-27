# 机器人 URDF 关节参数库（解析器 + 模型体检）

把散在 URDF XML 里的机器人参数，抽成一张**可查、可比、可校验**的参数表。

## 为什么做这个

客户/方案讨论里最常被问的是「这台够不够用」：工作半径多少、负载多大、有几个自由度、末端要串几级。
真实回答方式通常是工程师现翻模型文件、人工抄参数——慢、易错、不可比。

这个工具把这一步变成一次解析：

```
URDF 文件  →  解析器  →  参数表（CSV）+ 结构化 JSON + 模型体检报告 + 构型对比矩阵
```

零第三方依赖（只用 Python 标准库 `xml.etree`），可在无网、客户现场直接运行。

## 产出物

| 文件 | 内容 |
|---|---|
| `report/<robot>/joints.csv` | 关节参数表，22 列：类型 / 轴向 / 原点 xyz·rpy / 限位上下界与行程 / 力矩 / 速度 / mimic / 自由度 |
| `report/<robot>/links.csv` | 连杆参数表，7 列：质量 / 主惯量 Ixx·Iyy·Izz / 几何类型 / 尺寸 |
| `report/<robot>/summary.json` | 自由度、总质量、根节点、最长串联链、链长、异常清单 |
| `report/compare.csv` | 多构型对比矩阵（自由度 / 总质量 / 串联深度 / 链长 / 异常数） |

## 8 类模型体检规则

| 代码 | 规则 | 级别 |
|---|---|---|
| A1 | 关节的 parent / child 连杆不存在（断链） | high |
| A2 | continuous 关节误写了上下限位 | mid |
| A3 | revolute / prismatic 关节缺限位 | high |
| A4 | 声明了 inertial 但质量 ≤ 0 | mid |
| A5 | 声明了 inertial 但主惯量全 0 | mid |
| A6 | 轴向量的模长 ≠ 1（未归一化） | mid |
| A7 | 存在多个根节点（模型被拆成多段） | high |
| A8 | 同一连杆被多个关节挂载（重复 / 循环） | high |

> 纯坐标系连杆（无 `<inertial>`）不计入 A4/A5，属正常建模习惯。

## 实测结果

```
agv_diff_drive     links= 5 joints= 4 dof=2 mass= 48.000kg chain=1 reach≈0.323m 异常=0
broken_case        links= 4 joints= 4 dof=3 mass=  3.000kg chain=2 reach≈0.300m 异常=7
scara_4axis        links= 5 joints= 4 dof=4 mass= 18.950kg chain=4 reach≈1.020m 异常=0
ur5_like_6dof      links= 8 joints= 7 dof=6 mass= 20.994kg chain=7 reach≈1.329m 异常=0
```

`broken_case.urdf` 是故意埋错的用例（幽灵父连杆 / 连续关节带限位 / 缺限位 / 零质量 / 零惯量 /
轴未归一化 / 连杆重复挂载），用来验证体检规则是否真的能抓出来——实测命中 7 类。

## 测试

```
python tests/test_urdf_param_lib.py     # 13 项，全绿
```

覆盖：解析正确性、自由度计数（含 continuous 与 prismatic）、总质量、链长、参数表口径、
8 类体检规则命中与不误报。

## 用法

```python
import urdf_param_lib as L

rb = L.parse("samples/scara_4axis.urdf")
print(rb.summary())
rb.to_csv("out/scara")            # 写出 joints.csv / links.csv / summary.json
```

批量：

```
python urdf_param_lib.py          # 跑 samples/ 全量，输出到 report/
```

## 参数 → 指标 → 场景

专业参数本身客户听不懂，所以额外做了一层翻译：

| 层 | 例子 |
|---|---|
| 参数 | 关节限位 ±2.967 rad、力矩 120 N·m、速度 6.28 rad/s |
| 指标 | 工作半径约 1.02 m、4 个自由度、总质量 18.95 kg；UR5 构型链长 1.33 m |
| 场景 | SCARA → 3C 平面分拣/装配；6 轴串联臂 → 多姿态抓取；差速轮 AGV → 移动搬运 |

## 说明

`samples/` 下 3 个构型的关节原点与质量取自厂商公开规格整理而成，**不是厂商原始文件**，
仅用于验证解析器；`broken_case.urdf` 为自建异常用例。解析器本身可处理任意符合 URDF 规范的模型。

---

2026-10 ｜ 独立完成 ｜ Python 标准库 · 零依赖 · 13 项单测
