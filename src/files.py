"""JSON 文件读写：中文输出、新文件独占创建，避免覆盖已有数据。"""

import json
from pathlib import Path


def read_json(path, *, encoding="utf-8-sig"):
    return json.loads(Path(path).read_text(encoding=encoding))


def dump_json(data, stream, *, compact=False, allow_nan=False):
    options = {"separators": (",", ":")} if compact else {"indent": 2}
    json.dump(data, stream, ensure_ascii=False, allow_nan=allow_nan, **options)
    stream.write("\n")


def write_json(data, output, *, compact=True, allow_nan=False):
    with Path(output).open("x", encoding="utf-8") as stream:
        dump_json(data, stream, compact=compact, allow_nan=allow_nan)
