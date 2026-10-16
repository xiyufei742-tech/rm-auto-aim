#!/usr/bin/env python3
"""安装现场数据工具包。默认仅预览；--apply 备份后复制源码，不运行设备。"""

import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import tempfile


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def relative_name(text):
    path = PurePosixPath(text)
    if path.is_absolute() or not path.parts or any(part in ("..", ".") for part in path.parts):
        raise ValueError(f"包内相对路径不合法：{text}")
    if path.parts[0] not in {"contest", "calibration", "tests", "docs"}:
        raise ValueError(f"包内路径不在约定目录：{text}")
    return Path(*path.parts)


def reject_symlinks(root, relative):
    current = root
    for component in relative.parts:
        current = current / component
        if current.is_symlink():
            raise ValueError(f"目标含软链接，需人工确认路径：{current}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project", type=Path, help="NUC 上已有项目根目录")
    parser.add_argument("--apply", action="store_true", help="核对通过后备份并复制；否则只预览")
    args = parser.parse_args()
    bundle = Path(__file__).resolve().parent
    project = args.project.resolve(strict=True)
    if not (project / "CMakeLists.txt").is_file() or not (project / "io/camera.hpp").is_file():
        raise ValueError("目标不是已有自瞄项目：缺少 CMakeLists.txt 或 io/camera.hpp")
    cmake = (project / "CMakeLists.txt").read_text(encoding="utf-8")
    for target in ("srm_live_pixel_probe", "camera_test"):
        if target not in cmake:
            raise ValueError(f"现有 CMakeLists.txt 没有 {target}；请先确认 NUC 项目版本")
    manifest = json.loads((bundle / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("schema") != 1 or not isinstance(manifest.get("files"), list):
        raise ValueError("无法识别包清单")
    entries = []
    seen = set()
    for entry in manifest["files"]:
        relative = relative_name(entry["path"])
        if relative in seen:
            raise ValueError(f"包清单重复路径：{relative}")
        seen.add(relative)
        source, destination = bundle / "payload" / relative, project / relative
        reject_symlinks(bundle / "payload", relative)
        reject_symlinks(project, relative)
        if not source.is_file() or digest(source) != entry["sha256"]:
            raise ValueError(f"包文件 SHA-256 不符：{relative}")
        if destination.exists() and not destination.is_file():
            raise ValueError(f"目标应为文件：{relative}")
        same = destination.is_file() and digest(destination) == entry["sha256"]
        print(f"{'相同，跳过' if same else '将备份并更新' if destination.exists() else '将新增'}：{relative}")
        if not same:
            entries.append((relative, source, destination))
    if not entries:
        print("这些文件已与工具包一致，无需复制。")
        return
    if not args.apply:
        print("预览完成，未修改项目。确认路径正确后，在同一命令末尾加 --apply。")
        return

    reject_symlinks(project, Path("output"))
    (project / "output").mkdir(exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_")
    backup = Path(tempfile.mkdtemp(prefix="field_tools_backup_" + timestamp,
                                  dir=project / "output"))
    # 所有旧文件先完整备份，再开始覆盖。未存在的路径也写进清单，便于追溯。
    changes = []
    for relative, source, destination in entries:
        old_hash = None
        if destination.exists():
            old_hash = digest(destination)
            saved = backup / relative
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(destination, saved)
            if digest(saved) != old_hash:
                raise ValueError(f"备份校验失败，未覆盖源码：{relative}")
        changes.append({"path": relative.as_posix(), "old_sha256": old_hash,
                        "new_sha256": digest(source)})
    (backup / "changes.json").write_text(json.dumps(changes, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"旧文件备份：{backup}")
    for relative, source, destination in entries:
        destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as stream:
                temporary = Path(stream.name)
                stream.write(source.read_bytes())
                stream.flush()
                os.fsync(stream.fileno())
            os.chmod(temporary, 0o644)
            os.replace(temporary, destination)
            if digest(destination) != digest(source):
                raise ValueError(f"复制后校验失败：{relative}；旧文件在 {backup}")
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
    print("复制与校验完成。未修改现场配置、控制程序、CMakeLists.txt；未连接设备。")
    print("下一步在项目目录编译：cmake --build build --target srm_live_pixel_probe camera_test -j4")
    print("若没有 build，按工具包 README 先配置 CMake。")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError) as error:
        raise SystemExit(f"安装停止：{error}")
