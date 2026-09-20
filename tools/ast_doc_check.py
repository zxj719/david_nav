# -*- coding: utf-8 -*-
"""证明一批改动只动了 docstring：去掉说明后的 AST 必须与给定提交完全一致。"""

import ast
import subprocess
import sys


def strip(tree):
    """删除模块/类/函数的首个字符串表达式语句，并清空位置信息。"""
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            body = node.body
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                node.body = body[1:] or [ast.Pass()]
    return ast.dump(ast.fix_missing_locations(tree), include_attributes=False)


def main(rev, paths):
    bad = []
    for path in paths:
        old = subprocess.run(['git', 'show', rev + ':' + path], capture_output=True, text=True)
        if old.returncode:
            bad.append((path, 'missing in ' + rev))
            continue
        with open(path, encoding='utf-8') as handle:
            new_source = handle.read()
        if strip(ast.parse(old.stdout)) != strip(ast.parse(new_source)):
            bad.append((path, 'non-doc AST differs'))
        else:
            print('ok (doc-only):', path)
    if bad:
        for item in bad:
            print('FAIL:', item[0], '-', item[1])
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2:]))
