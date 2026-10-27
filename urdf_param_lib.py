# -*- coding: utf-8 -*-
"""URDF 关节参数库 · 解析器 + 参数抽取 + 模型体检

背景（2026-10-27）
------------------
目标是「把散在 URDF XML 里的机器人参数，抽成一张可查、可比、可校验的参数表」。
售前/产品场景里真正要做的事是：客户问「你这台能不能到 1.2 m 工作半径、负载够不够」，
工程师回答靠翻模型文件；这个工具把这步变成一次解析 + 一张表。

不依赖任何第三方库（只用标准库 xml.etree），方便在客户现场、无网环境直接跑。

对外接口
--------
  parse(path)                → Robot（links / joints / 派生量 / 异常）
  Robot.joint_table()        → List[dict]  关节参数表（逐关节一行）
  Robot.link_table()         → List[dict]  连杆参数表
  Robot.summary()            → dict        自由度 / 总质量 / 最长串联链 / 异常统计
  Robot.to_csv(dir)          → 写出 joints.csv / links.csv / summary.json / report.md
"""
import csv
import json
import math
import os
import xml.etree.ElementTree as ET

MOVING = ("revolute", "continuous", "prismatic")
FIXED = ("fixed",)

# 关节表字段（顺序即 CSV 列序，也是「标准参数表」的口径）
JOINT_FIELDS = ["joint", "type", "parent", "child", "axis_x", "axis_y", "axis_z",
                "axis_norm", "ox", "oy", "oz", "length_m", "roll", "pitch", "yaw",
                "lower", "upper", "range", "effort", "velocity", "mimic", "dof"]
LINK_FIELDS = ["link", "mass", "ixx", "iyy", "izz", "geom", "size"]


def _f(v, d=0.0):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


def _xyz(el, tag="origin"):
    """取 origin 的 xyz / rpy；缺失按 0 处理。"""
    if el is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    o = el.find(tag)
    if o is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = [_f(x) for x in (o.get("xyz") or "0 0 0").split()]
    rpy = [_f(x) for x in (o.get("rpy") or "0 0 0").split()]
    while len(xyz) < 3:
        xyz.append(0.0)
    while len(rpy) < 3:
        rpy.append(0.0)
    return tuple(xyz[:3]), tuple(rpy[:3])


class Robot(object):
    def __init__(self, name):
        self.name = name
        self.links = {}          # name -> dict
        self.joints = []         # List[dict]，保持文件顺序
        self.anomalies = []      # List[(code, level, msg)]

    # ---------------- 派生量 ----------------
    def dof(self):
        return sum(1 for j in self.joints if j["type"] in MOVING)

    def total_mass(self):
        return sum(l["mass"] for l in self.links.values())

    def tree(self):
        """返回 (roots, children_map)：固定关节不合并，直接按 joint 关系建图。"""
        children = {}
        has_parent = set()
        for j in self.joints:
            children.setdefault(j["parent"], []).append(j)
            has_parent.add(j["child"])
        roots = [n for n in self.links if n not in has_parent]
        return roots, children

    def longest_chain(self):
        """最长串联链（节点数最多的那条从根到叶的路径）——近似「末端到基座要串多少级」。"""
        roots, children = self.tree()
        best = []

        def walk(node, path):
            nonlocal best
            kids = children.get(node)
            if not kids:
                if len(path) > len(best):
                    best = list(path)
                return
            for j in kids:
                path.append(j["joint"])
                walk(j["child"], path)
                path.pop()

        for r in roots:
            walk(r, [])
        return best

    def reach(self):
        """沿最长链累加关节原点位移的模长 → 大致工作半径（几何链长，非真实可达包络）。"""
        return sum(j["length_m"] for j in self.joints
                   if j["joint"] in set(self.longest_chain()))

    # ---------------- 表 ----------------
    def joint_table(self):
        out = []
        for j in self.joints:
            rng = ""
            if j["lower"] != "" and j["upper"] != "":
                rng = "%.4f" % (_f(j["upper"]) - _f(j["lower"]))
            out.append({
                "joint": j["joint"], "type": j["type"],
                "parent": j["parent"], "child": j["child"],
                "axis_x": "%.4f" % j["axis"][0], "axis_y": "%.4f" % j["axis"][1],
                "axis_z": "%.4f" % j["axis"][2], "axis_norm": "%.4f" % j["axis_norm"],
                "ox": "%.4f" % j["origin"][0], "oy": "%.4f" % j["origin"][1],
                "oz": "%.4f" % j["origin"][2], "length_m": "%.4f" % j["length_m"],
                "roll": "%.4f" % j["rpy"][0], "pitch": "%.4f" % j["rpy"][1],
                "yaw": "%.4f" % j["rpy"][2],
                "lower": j["lower"], "upper": j["upper"], "range": rng,
                "effort": j["effort"], "velocity": j["velocity"],
                "mimic": j["mimic"], "dof": str(1 if j["type"] in MOVING else 0),
            })
        return out

    def link_table(self):
        return [{"link": n, "mass": "%.4f" % l["mass"],
                 "ixx": "%.4f" % l["inertia"][0], "iyy": "%.4f" % l["inertia"][1],
                 "izz": "%.4f" % l["inertia"][2], "geom": l["geom"], "size": l["size"]}
                for n, l in self.links.items()]

    def summary(self):
        roots, _ = self.tree()
        return {
            "robot": self.name,
            "links": len(self.links),
            "joints": len(self.joints),
            "dof": self.dof(),
            "fixed_joints": sum(1 for j in self.joints if j["type"] in FIXED),
            "total_mass_kg": round(self.total_mass(), 4),
            "chain_len": len(self.longest_chain()),
            "chain": self.longest_chain(),
            "chain_reach_m": round(self.reach(), 4),
            "roots": roots,
            "anomalies": len(self.anomalies),
        }

    def to_csv(self, outdir):
        os.makedirs(outdir, exist_ok=True)
        jt, lt = self.joint_table(), self.link_table()
        with open(os.path.join(outdir, "joints.csv"), "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=JOINT_FIELDS)
            w.writeheader()
            w.writerows(jt)
        with open(os.path.join(outdir, "links.csv"), "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=LINK_FIELDS)
            w.writeheader()
            w.writerows(lt)
        with open(os.path.join(outdir, "summary.json"), "w", encoding="utf-8") as f:
            json.dump({"summary": self.summary(),
                       "anomalies": [{"code": c, "level": lv, "msg": m} for c, lv, m in self.anomalies]},
                      f, ensure_ascii=False, indent=2)
        return len(jt), len(lt)


def parse(path):
    """解析单个 URDF 文件 → Robot。"""
    root = ET.parse(path).getroot()
    rb = Robot(root.get("name") or os.path.basename(path))

    for le in root.findall("link"):
        name = le.get("name")
        mass, inertia, geom, size = 0.0, (0.0, 0.0, 0.0), "", ""
        ine = le.find("inertial")
        has_inertial = ine is not None
        if ine is not None:
            me = ine.find("mass")
            if me is not None:
                mass = _f(me.get("value"))
            ie = ine.find("inertia")
            if ie is not None:
                inertia = (_f(ie.get("ixx")), _f(ie.get("iyy")), _f(ie.get("izz")))
        vis = le.find("visual")
        if vis is not None:
            ge = vis.find("geometry")
            if ge is not None and len(ge):
                g = ge[0]
                geom = g.tag
                if g.tag == "box":
                    size = g.get("size") or ""
                elif g.tag in ("cylinder", "sphere"):
                    size = "r=%s h=%s" % (g.get("radius"), g.get("length") or "-")
                elif g.tag == "mesh":
                    size = os.path.basename(g.get("filename") or "")
        rb.links[name] = {"mass": mass, "inertia": inertia, "geom": geom, "size": size,
                          "inertial": has_inertial}

    for je in root.findall("joint"):
        name = je.get("name")
        jtype = je.get("type") or "fixed"
        pe, ce = je.find("parent"), je.find("child")
        parent = pe.get("link") if pe is not None else ""
        child = ce.get("link") if ce is not None else ""
        xyz, rpy = _xyz(je)
        ae = je.find("axis")
        axis = tuple(_f(v) for v in ((ae.get("xyz") if ae is not None else "1 0 0") or "1 0 0").split())
        while len(axis) < 3:
            axis += (0.0,)
        axis = axis[:3]
        anorm = math.sqrt(sum(a * a for a in axis))
        le_ = je.find("limit")
        lower = upper = effort = velocity = ""
        if le_ is not None:
            lower = le_.get("lower", "")
            upper = le_.get("upper", "")
            effort = le_.get("effort", "")
            velocity = le_.get("velocity", "")
        me_ = je.find("mimic")
        mimic = ""
        if me_ is not None:
            mimic = "%s x%s +%s" % (me_.get("joint"), me_.get("multiplier", "1"), me_.get("offset", "0"))
        rb.joints.append({
            "joint": name, "type": jtype, "parent": parent, "child": child,
            "axis": axis, "axis_norm": anorm, "origin": xyz, "rpy": rpy,
            "length_m": math.sqrt(sum(v * v for v in xyz)),
            "lower": lower, "upper": upper, "effort": effort, "velocity": velocity,
            "mimic": mimic,
        })

    _audit(rb)
    return rb


# ---------------- 模型体检：8 类规则 ----------------
def _audit(rb):
    names = set(rb.links)
    # A1 父/子连杆不存在（断链）
    for j in rb.joints:
        if j["parent"] not in names:
            rb.anomalies.append(("A1", "high", "关节 %s 的 parent「%s」不存在" % (j["joint"], j["parent"])))
        if j["child"] not in names:
            rb.anomalies.append(("A1", "high", "关节 %s 的 child「%s」不存在" % (j["joint"], j["child"])))
    # A2 continuous 关节不应有限位上下界
    for j in rb.joints:
        if j["type"] == "continuous" and (j["lower"] != "" or j["upper"] != ""):
            rb.anomalies.append(("A2", "mid", "连续关节 %s 带了限位（应只给 effort/velocity）" % j["joint"]))
    # A3 转动/移动关节缺 limit
    for j in rb.joints:
        if j["type"] in ("revolute", "prismatic") and j["lower"] == "" and j["upper"] == "":
            rb.anomalies.append(("A3", "high", "关节 %s（%s）缺限位" % (j["joint"], j["type"])))
    # A4 零质量 / A5 零惯量：只查「声明了 inertial 却写 0」的连杆；
    #    纯坐标系连杆（如 ee_link / tool0）本就没有 inertial，属正常，不算异常。
    for n, l in rb.links.items():
        if not l["inertial"]:
            continue
        if l["mass"] <= 0:
            rb.anomalies.append(("A4", "mid", "连杆 %s 声明了惯性但质量 <= 0" % n))
    for n, l in rb.links.items():
        if not l["inertial"]:
            continue
        if all(v == 0 for v in l["inertia"]):
            rb.anomalies.append(("A5", "mid", "连杆 %s 惯量全 0" % n))
    # A6 轴向量未归一化（允许 1e-3 误差）
    for j in rb.joints:
        if j["type"] in MOVING and abs(j["axis_norm"] - 1.0) > 1e-3:
            rb.anomalies.append(("A6", "mid", "关节 %s 轴向量模长 %.4f ≠ 1" % (j["joint"], j["axis_norm"])))
    # A7 多根节点 / 孤儿连杆
    roots, children = rb.tree()
    if len(roots) > 1:
        rb.anomalies.append(("A7", "high", "存在 %d 个根节点：%s" % (len(roots), ",".join(roots))))
    # A8 循环引用 / 关节重复（同一 child 被两次挂载）
    seen = {}
    for j in rb.joints:
        if j["child"] in seen:
            rb.anomalies.append(("A8", "high", "连杆 %s 被多个关节挂载（%s / %s）"
                                 % (j["child"], seen[j["child"]], j["joint"])))
        seen[j["child"]] = j["joint"]


def main(sample_dir, out_root):
    files = sorted(f for f in os.listdir(sample_dir) if f.endswith(".urdf"))
    rows = []
    for fn in files:
        rb = parse(os.path.join(sample_dir, fn))
        od = os.path.join(out_root, os.path.splitext(fn)[0])
        nj, nl = rb.to_csv(od)
        s = rb.summary()
        rows.append(s)
        print("[%-16s] links=%2d joints=%2d dof=%d mass=%7.3fkg chain=%d reach≈%.3fm 异常=%d → %s"
              % (fn, s["links"], s["joints"], s["dof"], s["total_mass_kg"],
                 s["chain_len"], s["chain_reach_m"], s["anomalies"], od))
    with open(os.path.join(out_root, "compare.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["robot", "links", "joints", "dof", "fixed_joints",
                                          "total_mass_kg", "chain_len", "chain_reach_m", "anomalies"])
        w.writeheader()
        for r in rows:
            w.writerow({k: r[k] for k in w.fieldnames})
    print("\n构型对比表 →", os.path.join(out_root, "compare.csv"))
    return rows


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    main(os.path.join(here, "samples"), os.path.join(here, "report"))
