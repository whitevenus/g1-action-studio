"""审计实际 wheel；--installed 同时核对项目目录外安装的包与资源。"""

import argparse
import configparser
import importlib
import importlib.metadata
import xml.etree.ElementTree as ET
from email.parser import Parser
from hashlib import sha256
from pathlib import Path
from zipfile import ZipFile

PACKAGE = "g1_action_studio"
MODULES = {"__init__", "__main__", "cli", "motion", "model", "planner", "robot", "teach", "replay", "preview"}
LICENSES = {"LICENSE", "LICENSE-UNITREE", "LICENSE-UNITREE-MODEL", "LICENSE-APACHE-2.0"}
MODEL_COMMIT = "ab03eec53238487c4a7c7f61dd8e29a2f66abc04"
# 固定上游38个文件，按相对路径排序，将 path + NUL + 原始字节依次加入 SHA-256。
MODEL_SHA256 = "01b3afdc76c1c6b68a90b44f2337f9b740c91038094534baeba6aadff64b0f84"


def verify_archive(archive):
    members = [name for name in archive.namelist() if not name.endswith("/")]
    names = set(members)
    assert len(names) == len(members), "wheel 有重复文件"
    assert archive.testzip() is None, "wheel ZIP 校验失败"
    metadata_dirs = {name.split("/")[0] for name in names if ".dist-info/" in name}
    assert len(metadata_dirs) == 1, f"wheel 元信息目录错误：{metadata_dirs}"
    metadata_dir = metadata_dirs.pop()
    metadata = Parser().parsestr(archive.read(f"{metadata_dir}/METADATA").decode("utf-8"))
    assert metadata["Name"] == "g1-action-studio", metadata["Name"]
    assert metadata_dir == f"g1_action_studio-{metadata['Version']}.dist-info", metadata_dir
    assert metadata["Requires-Python"] == ">=3.11", metadata["Requires-Python"]
    assert set(metadata.get_all("License-File", [])) == LICENSES, "缺少发行许可声明"
    entry_points = configparser.ConfigParser()
    entry_points.read_string(archive.read(f"{metadata_dir}/entry_points.txt").decode("utf-8"))
    assert entry_points.sections() == ["console_scripts"], "存在额外入口类型"
    assert dict(entry_points["console_scripts"]) == {"g1-action-studio": "g1_action_studio.cli:main"}
    assert archive.read(f"{metadata_dir}/top_level.txt").decode("utf-8").strip() == PACKAGE

    resource_dir = f"{PACKAGE}/resources"
    xml = ET.fromstring(archive.read(f"{resource_dir}/g1_29dof.xml"))
    assert xml.find("compiler").attrib["meshdir"] == "meshes"
    meshes = {node.attrib["file"] for node in xml.findall("./asset/mesh")}
    assert len(meshes) == 36, f"固定模型应有36个引用网格，实际{len(meshes)}"
    assert all(Path(name).name == name and name.endswith(".STL") for name in meshes)
    assets = {"g1_29dof.xml", "LICENSE"} | {f"meshes/{name}" for name in meshes}
    digest = sha256()
    for name in sorted(assets):
        content = archive.read(f"{resource_dir}/{name}")
        assert content, f"资源为空：{name}"
        digest.update(name.encode("utf-8") + b"\0" + content)
    assert digest.hexdigest() == MODEL_SHA256, "固定上游模型、网格或许可字节发生变化"
    assert MODEL_COMMIT in archive.read(f"{resource_dir}/ORIGIN.md").decode("utf-8"), "缺少固定模型来源"
    assert b"BSD 3-Clause License" in archive.read(f"{resource_dir}/LICENSE"), "缺少模型 BSD 许可"
    for name in LICENSES:
        assert archive.read(f"{metadata_dir}/licenses/{name}").strip(), f"发行许可为空：{name}"

    package_files = {f"{PACKAGE}/{name}.py" for name in MODULES}
    package_files |= {f"{resource_dir}/{name}" for name in assets | {"ORIGIN.md"}}
    metadata_files = {
        f"{metadata_dir}/{name}" for name in ("METADATA", "WHEEL", "entry_points.txt", "top_level.txt", "RECORD")
    }
    metadata_files |= {f"{metadata_dir}/licenses/{name}" for name in LICENSES}
    expected = package_files | metadata_files
    assert names == expected, f"wheel 文件不符合合同；多余={sorted(names - expected)}；缺少={sorted(expected - names)}"
    print(f"wheel 内容通过：8个业务模块、36个网格、固定模型及全部许可；{metadata['Version']}")
    return package_files


def verify_installed(archive, package_files):
    import sys

    source_root = Path(__file__).resolve().parents[1]
    assert not Path.cwd().resolve().is_relative_to(source_root), "安装验证必须从项目目录外运行"
    distribution = importlib.metadata.distribution("g1-action-studio")
    package = importlib.import_module(PACKAGE)
    imported = Path(package.__file__).resolve()
    installed = Path(distribution.locate_file(f"{PACKAGE}/__init__.py")).resolve()
    assert imported == installed, f"导入路径与安装元信息不一致：{imported} != {installed}"
    assert imported.is_relative_to(Path(sys.prefix).resolve()), f"包未安装到当前环境：{imported}"
    assert not imported.is_relative_to(source_root), f"错误使用源码包：{imported}"
    for name in sorted(package_files):
        installed_file = Path(distribution.locate_file(name))
        assert installed_file.read_bytes() == archive.read(name), f"安装文件与 wheel 不一致：{name}"
    actual_modules = {path.stem for path in imported.parent.glob("*.py")}
    assert actual_modules == MODULES, f"安装包中有缺失或旧模块：{actual_modules}"
    print(f"目录外安装通过：{imported}；全部模块和资源与 wheel 一致")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wheel", type=Path)
    parser.add_argument("--installed", action="store_true")
    args = parser.parse_args()
    with ZipFile(args.wheel) as archive:
        package_files = verify_archive(archive)
        if args.installed:
            verify_installed(archive, package_files)


if __name__ == "__main__":
    main()
