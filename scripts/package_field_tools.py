#!/usr/bin/env python3
"""生成可通过远程桌面传输的现场工具包；不包含模型、实机配置、控制器或二进制。"""

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile


def main():
    project = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=project / "output/field_tools_1013.tar.gz")
    args = parser.parse_args()
    if args.output.exists() or args.output.is_symlink():
        raise ValueError("输出包已存在；请使用 --output 指定新文件名")
    files = [
        "contest/live_pixel_probe.cpp", "contest/analyze_field_session.py",
        "contest/field_preflight.py", "contest/evaluate_pnp_session.py",
        "contest/render_field_session.py", "tests/field_tools_selfcheck.py",
        "calibration/calibrate_from_video.py", "tests/camera_test.cpp",
        "docs/现场数据工具_使用说明.md",
    ]
    sources = {relative: project / relative for relative in files}
    sources["docs/现场调试计划_10月13日起.md"] = (
        project.parent / "中期评定_10月17日/07_后续现场调试详细计划.md")
    readme = """# 现场数据工具包（10 月 13 日）

适用于已有自瞄比赛工程。安装前先关掉正在运行的相机/识别测试程序。

1. 在本目录运行 `sha256sum -c SHA256SUMS` 核对传输完整性。
2. 预览：`python3 install_field_tools.py /home/blue/robomaster-workspace-main/自瞄比赛项目`
3. 备份并复制：在上一行末尾加 `--apply`。
4. 在项目根目录运行 `cmake --build build --target srm_live_pixel_probe camera_test -j4`。
5. 读项目 `docs/现场数据工具_使用说明.md`，按 `docs/现场调试计划_10月13日起.md` 逐项执行。

安装器不修改模型、现场 YAML、CMake、驱动或现有跟随控制器。默认预览不会更改项目。
需要已有项目含 srm_live_pixel_probe、camera_test 两个编译目标。
标定需要 Python OpenCV/numpy，PnP 还需要 PyYAML。程序不自动安装依赖。

本包在开发电脑通过离线/合成数据检查。NUC 编译、相机采集、实际标定和控制均需现场复核。
若编译失败，保留第一个 error 与备份路径；不要删除旧录像，也不要覆盖现有现场配置。

相机参数、画面方向或颜色若在 NUC 有独立改动，安装本包会保留它们。
新探针只读串口；另外两个 Python 分析程序和标定程序只读离线文件。

更详细说明在 payload/docs/现场数据工具_使用说明.md（安装后出现在项目 docs/）。
"""
    with tempfile.TemporaryDirectory(prefix="field_tools_package_") as temporary:
        base = Path(temporary) / "field_tools_1013"
        base.mkdir()
        entries = []
        for relative, source in sources.items():
            if not source.is_file():
                raise ValueError(f"打包所需文件缺失：{source}")
            destination = base / "payload" / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
            if relative == "docs/现场调试计划_10月13日起.md":
                # 中期文档装进工程 docs 后，相对链接的起点随之改变。
                content = destination.read_text(encoding="utf-8")
                content = content.replace("../自瞄比赛项目/docs/", "")
                content = content.replace("../校内赛规则/", "../../校内赛规则/")
                destination.write_text(content, encoding="utf-8")
            entries.append({"path": relative, "sha256": hashlib.sha256(destination.read_bytes()).hexdigest()})
        shutil.copyfile(project / "scripts/install_field_tools.py", base / "install_field_tools.py")
        (base / "manifest.json").write_text(json.dumps({"schema": 1, "files": entries}, ensure_ascii=False, indent=2), encoding="utf-8")
        (base / "README.md").write_text(readme, encoding="utf-8")
        checksums = []
        for path in sorted(base.rglob("*")):
            if path.is_file():
                checksums.append(hashlib.sha256(path.read_bytes()).hexdigest() + "  " + path.relative_to(base).as_posix())
        (base / "SHA256SUMS").write_text("\n".join(checksums) + "\n", encoding="utf-8")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        # x 模式再次防止检查后意外覆盖已有文件。
        with args.output.open("xb") as stream:
            with tarfile.open(fileobj=stream, mode="w:gz") as archive:
                archive.add(base, arcname=base.name)
    checksum = hashlib.sha256(args.output.read_bytes()).hexdigest()
    print(f"生成 {args.output}，{args.output.stat().st_size} 字节")
    print(f"SHA-256: {checksum}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as error:
        raise SystemExit(f"打包失败：{error}")
