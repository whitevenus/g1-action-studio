"""分层运行离线测试；Linux 完整层不接受 skip。"""

import argparse
import sys
import unittest
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite", choices=("numerical", "full", "workflow"))
    args = parser.parse_args(argv)
    if args.suite != "numerical" and sys.platform != "linux":
        parser.error("完整规划及官方 SDK 流程需要 Linux；文件/数值范围使用 numerical")
    patterns = {
        "numerical": ("test_motion.py", "test_cli.py"),
        "full": ("test_*.py",),
        "workflow": ("test_workflow.py",),
    }
    directory = str(Path(__file__).resolve().parent)
    suite = unittest.TestSuite(
        unittest.defaultTestLoader.discover(directory, pattern) for pattern in patterns[args.suite]
    )
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if args.suite != "numerical" and result.skipped:
        print("完整流程不允许跳过测试；请安装规划及官方 SDK 依赖。", file=sys.stderr)
        return 1
    return 0 if result.wasSuccessful() and result.testsRun else 1


if __name__ == "__main__":
    raise SystemExit(main())
