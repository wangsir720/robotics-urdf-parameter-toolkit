# -*- coding: utf-8 -*-
"""URDF 参数库单元测试：11 项，覆盖解析、派生量、参数表口径与 8 类体检规则。"""
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

import urdf_param_lib as L

S = os.path.join(os.path.dirname(HERE), "samples")


def load(name):
    return L.parse(os.path.join(S, name))


class TestParse(unittest.TestCase):
    def test_robot_name(self):
        self.assertEqual(load("ur5_like_6dof.urdf").name, "ur5_like_6dof")

    def test_link_joint_count(self):
        rb = load("ur5_like_6dof.urdf")
        self.assertEqual(len(rb.links), 8)
        self.assertEqual(len(rb.joints), 7)

    def test_dof_6axis(self):
        self.assertEqual(load("ur5_like_6dof.urdf").dof(), 6)

    def test_dof_scara_mixes_revolute_and_prismatic(self):
        # 2 转动 + 1 移动 + 1 转动 = 4
        self.assertEqual(load("scara_4axis.urdf").dof(), 4)

    def test_dof_agv_continuous_counts(self):
        self.assertEqual(load("agv_diff_drive.urdf").dof(), 2)

    def test_total_mass(self):
        rb = load("ur5_like_6dof.urdf")
        self.assertAlmostEqual(rb.total_mass(), 20.994, places=3)

    def test_axis_norm_detects_unnormalized(self):
        rb = load("broken_case.urdf")
        j2 = [j for j in rb.joints if j["joint"] == "j2"][0]
        self.assertAlmostEqual(j2["axis_norm"], 2.0, places=4)

    def test_length_from_origin_xyz(self):
        rb = load("ur5_like_6dof.urdf")
        j = [x for x in rb.joints if x["joint"] == "shoulder_pan_joint"][0]
        self.assertAlmostEqual(j["length_m"], 0.089159, places=5)

    def test_joint_table_schema(self):
        rb = load("scara_4axis.urdf")
        t = rb.joint_table()
        self.assertEqual(len(t), 4)
        self.assertEqual(list(t[0].keys()), L.JOINT_FIELDS)
        self.assertEqual(t[0]["dof"], "1")

    def test_range_computed(self):
        rb = load("scara_4axis.urdf")
        t = {r["joint"]: r for r in rb.joint_table()}
        self.assertAlmostEqual(float(t["joint_3"]["range"]), 0.20, places=4)


class TestAudit(unittest.TestCase):
    def codes(self, name):
        return {c for c, _, _ in load(name).anomalies}

    def test_clean_models_have_no_anomaly(self):
        for n in ("ur5_like_6dof.urdf", "scara_4axis.urdf", "agv_diff_drive.urdf"):
            self.assertEqual(load(n).anomalies, [], "%s 不应有异常" % n)

    def test_broken_case_hits_all_rules(self):
        c = self.codes("broken_case.urdf")
        # A1 幽灵父连杆 / A2 连续关节带限位 / A3 缺限位 / A4 零质量 / A5 零惯量 / A6 轴未归一化 / A8 重复挂载
        for code in ("A1", "A2", "A3", "A4", "A5", "A6", "A8"):
            self.assertIn(code, c, "应命中 %s" % code)

    def test_frame_link_without_inertial_not_flagged(self):
        # ee_link 没有 inertial 元素，属纯坐标系连杆，不应报 A4/A5
        msgs = " ".join(m for _, _, m in load("ur5_like_6dof.urdf").anomalies)
        self.assertNotIn("ee_link", msgs)


if __name__ == "__main__":
    unittest.main(verbosity=2)
