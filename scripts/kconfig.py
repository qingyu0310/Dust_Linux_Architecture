#!/usr/bin/env python3
"""Kconfig 前端（本工程子集）：解析 Kconfig → 求值 → 生成 .config / kconfig.cmake / autoconf.h

用法：
  python3 scripts/kconfig.py [--defconfig configs/xxx_defconfig] [--config 现有.config] --out build/kconfig

支持的语法：mainmenu / source / config + bool / default [y|n] [if 表达式] /
           select [if 表达式] / depends on 表达式 / if 块 / help（忽略）/ # 注释
支持的表达式：符号名、y、n、`X = y`、&&、||、!、()
"""

import argparse
import os
import re
import sys

MAX_ITER = 64
TOKEN_RE = re.compile(r"\s*(&&|\|\||[!()=]|[A-Za-z0-9_]+)")


class Symbol:
    def __init__(self, name, origin):
        self.name = name
        self.origin = origin
        self.type = None
        self.defaults = []      # [(条件表达式, 'y'/'n')]
        self.selects = []       # [(被强制拉起的符号名, 条件表达式)]
        self.depends = []       # [条件表达式]，空表示无条件
        self.value = "n"
        self.user = None        # defconfig / .config 给定的值


def tokenize(text):
    tokens, pos = [], 0
    while pos < len(text):
        match = TOKEN_RE.match(text, pos)
        if not match:
            raise ValueError(f"无法解析表达式: {text!r}")
        tokens.append(match.group(1))
        pos = match.end()
    return tokens


def parse_expr(text):
    tokens = tokenize(text)
    pos = [0]

    def parse_or():
        node = parse_and()
        while pos[0] < len(tokens) and tokens[pos[0]] == "||":
            pos[0] += 1
            node = ("or", node, parse_and())
        return node

    def parse_and():
        node = parse_not()
        while pos[0] < len(tokens) and tokens[pos[0]] == "&&":
            pos[0] += 1
            node = ("and", node, parse_not())
        return node

    def parse_not():
        if pos[0] < len(tokens) and tokens[pos[0]] == "!":
            pos[0] += 1
            return ("not", parse_not())
        return parse_atom()

    def parse_atom():
        if pos[0] < len(tokens) and tokens[pos[0]] == "(":
            pos[0] += 1
            node = parse_or()
            if pos[0] >= len(tokens) or tokens[pos[0]] != ")":
                raise ValueError(f"表达式缺少右括号: {text!r}")
            pos[0] += 1
            return node
        if pos[0] >= len(tokens):
            raise ValueError(f"表达式不完整: {text!r}")
        name = tokens[pos[0]]
        pos[0] += 1
        if pos[0] < len(tokens) and tokens[pos[0]] == "=":
            pos[0] += 1
            value = tokens[pos[0]] if pos[0] < len(tokens) else None
            pos[0] += 1
            return ("cmp", name, value)
        return ("sym", name)

    node = parse_or()
    if pos[0] != len(tokens):
        raise ValueError(f"表达式有多余内容: {text!r}")
    return node


def eval_expr(node, values):
    kind = node[0]
    if kind == "sym":
        name = node[1]
        if name == "y":
            return True
        if name == "n":
            return False
        return values.get(name, "n") == "y"
    if kind == "cmp":
        return values.get(node[1], "n") == (node[2] or "n").strip('"')
    if kind == "and":
        return eval_expr(node[1], values) and eval_expr(node[2], values)
    if kind == "or":
        return eval_expr(node[1], values) or eval_expr(node[2], values)
    return not eval_expr(node[1], values)


def strip_comment(line):
    out, in_quote = [], False
    for char in line:
        if char == '"':
            in_quote = not in_quote
        if char == "#" and not in_quote:
            break
        out.append(char)
    return "".join(out)


def parse_kconfig(path, symbols, order, cond_stack):
    with open(path, encoding="utf-8") as handle:
        lines = handle.readlines()
    current = None
    help_indent = None
    for lineno, raw in enumerate(lines, 1):
        line = raw.rstrip("\n")
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())
        if help_indent is not None:
            if not stripped or indent > help_indent:
                continue
            help_indent = None
        body = strip_comment(line).strip()
        if not body:
            continue
        origin = f"{path}:{lineno}"

        if body.startswith("mainmenu"):
            continue
        if body.startswith("source "):
            match = re.match(r'source\s+"([^"]+)"', body)
            if not match:
                raise ValueError(f"{origin}: source 缺少引号路径")
            nested = os.path.join(os.path.dirname(path), match.group(1))
            parse_kconfig(nested, symbols, order, cond_stack)
            continue
        if body == "endif" or body.startswith("endif "):
            if cond_stack:
                cond_stack.pop()
            continue
        if body.startswith("if "):
            cond_stack.append(parse_expr(body[3:].strip()))
            continue
        if body in ("menu", "endmenu") or body.startswith("menu "):
            continue
        if body.startswith("config "):
            name = body.split()[1]
            if name in symbols:
                raise ValueError(f"{origin}: 符号 {name} 重复定义")
            current = Symbol(name, origin)
            current.depends = list(cond_stack)
            symbols[name] = current
            order.append(name)
            continue
        if current is None:
            raise ValueError(f"{origin}: 这条语句不在任何 config 块里: {body}")

        match = re.match(r"(bool|tristate|int|string|hex)\b(.*)$", body)
        if match:
            current.type = match.group(1)
            rest = match.group(2)
            cond = re.search(r"\bif\s+(.+)$", rest)
            if cond:
                current.depends.append(parse_expr(cond.group(1).strip()))
            continue
        if body.startswith("default "):
            rest = body[len("default "):].strip()
            cond = re.search(r"\bif\s+(.+)$", rest)
            value = rest[:cond.start()].strip() if cond else rest
            if value not in ("y", "n"):
                raise ValueError(f"{origin}: 只支持 bool 默认值 y/n，收到 {value!r}")
            current.defaults.append((parse_expr(cond.group(1).strip()) if cond else parse_expr("y"),
                                     value))
            continue
        if body.startswith("select "):
            rest = body[len("select "):].strip()
            cond = re.search(r"\bif\s+(.+)$", rest)
            target = rest[:cond.start()].strip() if cond else rest
            current.selects.append((target, parse_expr(cond.group(1).strip()) if cond
                                    else parse_expr("y")))
            continue
        if body.startswith("depends on "):
            current.depends.append(parse_expr(body[len("depends on "):].strip()))
            continue
        if body.startswith("prompt "):
            cond = re.search(r"\bif\s+(.+)$", body)
            if cond:
                current.depends.append(parse_expr(cond.group(1).strip()))
            continue
        if body in ("help", "---help---"):
            help_indent = indent
            continue
        raise ValueError(f"{origin}: 不支持的语句: {body}")


def load_assignments(path, symbols, warnings):
    """读取 defconfig / .config：CONFIG_X=y、CONFIG_X=n、# CONFIG_X is not set"""
    assignments = {}
    with open(path, encoding="utf-8") as handle:
        for lineno, raw in enumerate(handle, 1):
            line = raw.strip()
            if not line:
                continue
            match = re.match(r"^#\s*CONFIG_([A-Za-z0-9_]+)\s+is not set$", line)
            if match:
                assignments[match.group(1)] = "n"
                continue
            match = re.match(r"^CONFIG_([A-Za-z0-9_]+)\s*=\s*(\S+)$", line)
            if not match:
                continue
            name, value = match.group(1), match.group(2)
            if value not in ("y", "n"):
                raise ValueError(f"{path}:{lineno}: 只支持 y/n，收到 {name}={value}")
            assignments[name] = value
    for name in assignments:
        if name not in symbols:
            raise ValueError(f"{path}: 赋值了 Kconfig 里不存在的符号 {name}")
    return assignments


def resolve(symbols, order):
    """不动点求值：select 强制拉起 > 用户赋值（依赖满足时）> default > n"""
    selected_by = {}
    for name in order:
        for target, cond in symbols[name].selects:
            if target not in symbols:
                raise ValueError(f"{symbols[name].origin}: select 了不存在的符号 {target}")
            selected_by.setdefault(target, []).append((name, cond))

    for _ in range(MAX_ITER):
        values = {name: symbols[name].value for name in order}
        changed = False
        for name in order:
            symbol = symbols[name]
            deps_ok = all(eval_expr(expr, values) for expr in symbol.depends)
            forced_y = any(values.get(src) == "y" and eval_expr(cond, values)
                           for src, cond in selected_by.get(name, []))
            if forced_y:
                value = "y"
            elif not deps_ok:
                value = "n"
            elif symbol.user is not None:
                value = symbol.user
            else:
                value = "n"
                for cond, default in symbol.defaults:
                    if eval_expr(cond, values):
                        value = default
                        break
            if value != symbol.value:
                symbol.value = value
                changed = True
        if not changed:
            break
    else:
        raise RuntimeError("求值未收敛：检查是否有循环 select")

    # 收敛后再判诊断：迭代中途的依赖状态还不是最终值，那时报会是假警
    problems = []
    values = {name: symbols[name].value for name in order}
    for name in order:
        symbol = symbols[name]
        deps_ok = all(eval_expr(expr, values) for expr in symbol.depends)
        forced_y = any(values.get(src) == "y" and eval_expr(cond, values)
                       for src, cond in selected_by.get(name, []))
        if forced_y and not deps_ok:
            problems.append(f"{name} 被 select 拉起，但 depends on 不满足（{symbol.origin}）")
        elif not deps_ok and symbol.user == "y":
            problems.append(f"{name} 依赖不满足，但配置要求 =y（{symbol.origin}）")
    return problems


def write_outputs(out_dir, symbols, order):
    os.makedirs(out_dir, exist_ok=True)

    with open(os.path.join(out_dir, ".config"), "w", encoding="utf-8") as handle:
        handle.write("# 由 scripts/kconfig.py 生成，不要手改；改 defconfig 后重新生成\n")
        for name in order:
            if symbols[name].value == "y":
                handle.write(f"CONFIG_{name}=y\n")
            else:
                handle.write(f"# CONFIG_{name} is not set\n")

    with open(os.path.join(out_dir, "kconfig.cmake"), "w", encoding="utf-8") as handle:
        handle.write("# 由 scripts/kconfig.py 生成，不要手改\n")
        for name in order:
            state = "ON" if symbols[name].value == "y" else "OFF"
            handle.write(f"set(CONFIG_{name} {state})\n")

    with open(os.path.join(out_dir, "autoconf.h"), "w", encoding="utf-8") as handle:
        handle.write("/* 由 scripts/kconfig.py 生成，不要手改 */\n")
        for name in order:
            if symbols[name].value == "y":
                handle.write(f"#define CONFIG_{name} 1\n")
            else:
                handle.write(f"/* #undef CONFIG_{name} */\n")


def main():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description="Kconfig -> .config / kconfig.cmake / autoconf.h")
    parser.add_argument("--kconfig", default=os.path.join(root, "Kconfig"))
    parser.add_argument("--config", help="已存在的 .config，作为 base")
    parser.add_argument("--defconfig", action="append", default=[], help="配置片段，可多次给")
    parser.add_argument("--out", default=os.path.join(root, "build", "kconfig"))
    args = parser.parse_args()

    symbols, order = {}, []
    parse_kconfig(args.kconfig, symbols, order, [])

    warnings = []
    assignments = {}
    if args.config:
        assignments.update(load_assignments(args.config, symbols, warnings))
    for path in args.defconfig:
        assignments.update(load_assignments(path, symbols, warnings))
    for name, value in assignments.items():
        symbols[name].user = value

    problems = resolve(symbols, order)

    for name in order:
        symbol = symbols[name]
        print(f"{'y' if symbol.value == 'y' else 'n'}  CONFIG_{name}")
    for problem in problems:
        print(f"错误: {problem}", file=sys.stderr)
    for warning in warnings:
        print(f"警告: {warning}", file=sys.stderr)
    if problems:
        return 1

    write_outputs(args.out, symbols, order)
    print(f"已写出 {args.out}/.config、kconfig.cmake、autoconf.h")
    return 0


if __name__ == "__main__":
    sys.exit(main())
