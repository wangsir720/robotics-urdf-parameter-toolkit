# -*- coding: utf-8 -*-
"""初始化仓库、提交、双推 Gitee + GitHub，并核对两端 HEAD。"""
import os
import subprocess
import sys

sys.stdout.reconfigure(encoding="utf-8")
REPO_DIR = os.path.dirname(os.path.abspath(__file__))
REPO = "robotics-urdf-parameter-toolkit"
GITEE = "git@gitee.com:binliwang/%s.git" % REPO
GITHUB = "git@github.com:wangsir720/%s.git" % REPO


def sh(args, cwd=REPO_DIR, timeout=240):
    p = subprocess.run(args, cwd=cwd, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", timeout=timeout)
    out = ((p.stdout or "") + (p.stderr or "")).strip()
    print("$ %s\n%s\n" % (" ".join(args), out[:1500]))
    return p.returncode, out


# 清理临时脚本（不纳入仓库）
for f in ("_gh_setup.py", "_gitee_public.py"):
    fp = os.path.join(REPO_DIR, f)
    if os.path.exists(fp):
        os.remove(fp)
        print("removed", f)

sh(["git", "init", "-q"])
sh(["git", "add", "-A"])
rc, _ = sh(["git", "-c", "user.name=Wang Libin", "-c", "user.email=15616823512@163.com",
            "commit", "-m", "feat: URDF 关节参数解析与模型体检工具（4 构型 · 13 项测试）"])
sh(["git", "branch", "-M", "main"])

sh(["git", "remote", "remove", "gitee"])
sh(["git", "remote", "remove", "github"])
sh(["git", "remote", "add", "gitee", GITEE])
sh(["git", "remote", "add", "github", GITHUB])
sh(["git", "push", "-u", "gitee", "main"], timeout=300)
sh(["git", "push", "-u", "github", "main"], timeout=300)

rc, head = sh(["git", "rev-parse", "HEAD"])
head = head.split("\n")[-1]
for name, url in (("gitee", GITEE), ("github", GITHUB)):
    rc, out = sh(["git", "ls-remote", "--heads", url, "main"])
    remote = out.split()[0] if out else ""
    print("== %s remote=%s local=%s  %s" % (name, remote[:12], head[:12],
                                            "OK" if remote == head else "MISMATCH"))
