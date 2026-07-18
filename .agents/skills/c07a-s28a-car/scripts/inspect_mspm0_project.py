#!/usr/bin/env python3
"""Read-only preflight indexer for this repository's MSPM0 SysConfig project."""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import Iterable


SKIP_DIRS = {
    ".git",
    ".agents",
    ".svn",
    ".hg",
    "node_modules",
    "__pycache__",
    "tmp",
}
BUILD_DIRS = {"Debug", "Release", "build", "out", "Objects", "Listings"}
GENERATED_NAMES = {"ti_msp_dl_config.h", "ti_msp_dl_config.c"}
PIN_PATTERN = re.compile(r"\bP[AB]\d{1,2}\b")


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def relative(path: Path, root: Path) -> str:
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def iter_files(root: Path) -> Iterable[Path]:
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(name for name in dirnames if name not in SKIP_DIRS)
        for filename in sorted(filenames):
            yield Path(dirpath) / filename


def is_build_copy(path: Path, root: Path) -> bool:
    parts = path.relative_to(root).parts
    return any(part in BUILD_DIRS or part.startswith("cmake-build") for part in parts)


def find_hardware_fact(project_root: Path) -> Path | None:
    candidates = [project_root, *project_root.parents]
    script_path = Path(__file__).resolve()
    candidates.extend([script_path.parent, *script_path.parents])
    seen: set[Path] = set()
    for directory in candidates:
        if directory in seen:
            continue
        seen.add(directory)
        candidate = directory / "硬件.md"
        if candidate.is_file():
            return candidate.resolve()
    return None


def cli_value(text: str, key: str) -> str | None:
    pattern = re.compile(
        rf"--{re.escape(key)}(?:=|\s+)(?:\"([^\"]+)\"|'([^']+)'|([^\s*/]+))"
    )
    match = pattern.search(text)
    if not match:
        return None
    return next(value for value in match.groups() if value is not None)


def parse_syscfg(path: Path) -> dict[str, object]:
    text = read_text(path)
    versions_match = re.search(r"@versions\s+(\{[^\r\n]+\})", text)
    versions_raw = versions_match.group(1).strip() if versions_match else None
    tool_version = None
    if versions_raw:
        tool_match = re.search(
            r"[\"'](?:tool|sysconfig)[\"']\s*:\s*[\"']([^\"']+)", versions_raw
        )
        if tool_match:
            tool_version = tool_match.group(1)

    product = cli_value(text, "product")
    sdk_version = None
    if product and "@" in product:
        sdk_version = product.rsplit("@", 1)[1]

    assigned_pins = sorted(
        set(
            re.findall(r"assignedPin\s*=\s*[\"'](P[AB]\d{1,2})[\"']", text)
            + re.findall(r"\.\$assign\s*=\s*[\"'](P[AB]\d{1,2})[\"']", text)
        )
    )

    return {
        "device": cli_value(text, "device"),
        "package": cli_value(text, "package"),
        "part": cli_value(text, "part"),
        "sdk_product": product,
        "sdk_version": sdk_version,
        "sysconfig_version": tool_version,
        "versions_raw": versions_raw,
        "has_cli_args": "@cliArgs" in text or "@v2CliArgs" in text,
        "has_versions": "@versions" in text,
        "assigned_pins": assigned_pins,
    }


def parse_generated_header(path: Path, root: Path) -> dict[str, object]:
    text = read_text(path)
    macros = sorted(set(re.findall(r"^\s*#define\s+([A-Za-z_]\w*)", text, re.MULTILINE)))
    init_functions = sorted(
        set(re.findall(r"\bvoid\s+(SYSCFG_DL_\w*[Ii]nit)\s*\(", text))
    )
    return {
        "path": relative(path, root),
        "init_functions": init_functions,
        "instance_macros": [name for name in macros if "_INST" in name],
        "irq_and_handler_macros": [
            name for name in macros if "IRQ" in name.upper() or "HANDLER" in name.upper()
        ],
        "gpio_port_pin_macros": [
            name
            for name in macros
            if ("PORT" in name.upper() or "PIN" in name.upper())
            and not name.startswith("__")
        ],
        "macro_count": len(macros),
    }


def find_toolchain_evidence(files: list[Path], root: Path) -> dict[str, object]:
    def paths_matching(predicate) -> list[str]:
        return sorted(relative(path, root) for path in files if predicate(path))

    ccs_project_files = paths_matching(
        lambda path: path.name in {".project", ".cproject"}
        or path.suffix.lower() == ".projectspec"
    )
    target_configs = paths_matching(
        lambda path: path.suffix.lower() == ".ccxml"
        and "targetConfigs" in path.parts
    )
    makefiles = paths_matching(
        lambda path: path.name.lower() in {"makefile", "subdir_rules.mk"}
        and any(part in {"Debug", "Release"} for part in path.parts)
    )
    keil_projects = paths_matching(lambda path: path.suffix.lower() == ".uvprojx")
    cmake_files = paths_matching(
        lambda path: path.name in {"CMakeLists.txt", "CMakePresets.json"}
    )

    sysconfig_command_fragments: list[dict[str, object]] = []
    for path in files:
        if path.name.lower() not in {"makefile", "subdir_rules.mk", "subdir_vars.mk"}:
            continue
        for lineno, line in enumerate(read_text(path).splitlines(), start=1):
            lowered = line.lower()
            if "sysconfig" in lowered and ("--product" in line or "sysconfig_cli" in lowered):
                sysconfig_command_fragments.append(
                    {
                        "path": relative(path, root),
                        "line": lineno,
                        "text": line.strip(),
                    }
                )
                if len(sysconfig_command_fragments) >= 10:
                    break
        if len(sysconfig_command_fragments) >= 10:
            break

    if ccs_project_files or makefiles or target_configs:
        detected = "CCS/SysConfig"
    elif keil_projects:
        detected = "Keil/uVision"
    elif cmake_files:
        detected = "CMake"
    else:
        detected = "unknown"

    return {
        "detected": detected,
        "ccs_project_files": ccs_project_files,
        "target_configs": target_configs,
        "generated_build_rules": makefiles,
        "keil_projects": keil_projects,
        "cmake_files": cmake_files,
        "sysconfig_command_fragments": sysconfig_command_fragments,
    }


def inspect(project_root: Path) -> tuple[dict[str, object], int]:
    files = list(iter_files(project_root))
    syscfg_files = sorted(
        path
        for path in files
        if path.suffix.lower() == ".syscfg" and not is_build_copy(path, project_root)
    )
    generated_files = sorted(path for path in files if path.name in GENERATED_NAMES)
    generated_headers = [path for path in generated_files if path.name.endswith(".h")]
    generated_sources = [path for path in generated_files if path.name.endswith(".c")]
    hardware_path = find_hardware_fact(project_root)
    hardware_pins = sorted(set(PIN_PATTERN.findall(read_text(hardware_path)))) if hardware_path else []

    syscfg_details = [
        {"path": relative(path, project_root), **parse_syscfg(path)} for path in syscfg_files
    ]
    header_details = [
        parse_generated_header(path, project_root) for path in generated_headers
    ]
    assigned_pins = sorted(
        {
            pin
            for detail in syscfg_details
            for pin in detail.get("assigned_pins", [])
            if isinstance(pin, str)
        }
    )

    warnings: list[str] = []
    exit_code = 0
    if not hardware_path:
        status = "blocked_hardware_fact_missing"
        warnings.append("Repository-root 硬件.md was not found; do not make board-level changes.")
        exit_code = 2
    elif not syscfg_files:
        status = "blocked_no_syscfg"
        warnings.append("No active .syscfg was found; this directory is not yet an inspectable MSPM0 project.")
        exit_code = 2
    elif len(syscfg_files) > 1:
        status = "blocked_multiple_syscfg"
        warnings.append("Multiple .syscfg files were found; identify the active CCS project before editing.")
        exit_code = 3
    else:
        detail = syscfg_details[0]
        missing = [
            key
            for key in ("device", "package", "sdk_product", "sysconfig_version")
            if not detail.get(key)
        ]
        if missing:
            status = "blocked_incomplete_metadata"
            warnings.append("Active .syscfg metadata is missing: " + ", ".join(missing))
            exit_code = 4
        elif not generated_headers:
            status = "syscfg_ready_generated_header_missing"
            warnings.append(
                "No ti_msp_dl_config.h was found; regenerate/build before writing application C names."
            )
        else:
            status = "ready_for_manual_pin_conflict_check"

    if generated_headers and len(generated_headers) > 1:
        warnings.append(
            "Multiple generated headers were found; trace the active build configuration before using macros."
        )
    if not generated_sources:
        warnings.append("No generated ti_msp_dl_config.c was found.")

    undocumented_assigned_pins = sorted(set(assigned_pins) - set(hardware_pins))
    if undocumented_assigned_pins:
        warnings.append(
            "Assigned pins absent from 硬件.md require purpose-level review; this is not an automatic conflict: "
            + ", ".join(undocumented_assigned_pins)
        )

    result: dict[str, object] = {
        "project_root": str(project_root),
        "status": status,
        "hardware_fact": str(hardware_path) if hardware_path else None,
        "hardware_pins": hardware_pins,
        "syscfg_files": syscfg_details,
        "generated_headers": header_details,
        "generated_sources": [relative(path, project_root) for path in generated_sources],
        "assigned_pins_absent_from_hardware_map": undocumented_assigned_pins,
        "toolchain": find_toolchain_evidence(files, project_root),
        "warnings": warnings,
        "next_required_checks": [
            "Read the complete hardware fact file.",
            "Read the active .syscfg and confirm device/package/SDK/SysConfig metadata.",
            "List pins involved in the requested change and compare their purposes with 硬件.md.",
            "After regeneration, reread the active generated header before writing C code.",
        ],
    }
    return result, exit_code


def print_items(label: str, items: list[object]) -> None:
    print(f"{label}:")
    if not items:
        print("  - [none]")
        return
    for item in items:
        print(f"  - {item}")


def print_report(result: dict[str, object]) -> None:
    print(f"Project root: {result['project_root']}")
    print(f"Status: {result['status']}")
    print(f"Hardware fact: {result['hardware_fact'] or '[missing]'}")
    print_items("Hardware-map pins", list(result["hardware_pins"]))

    syscfg_files = list(result["syscfg_files"])
    print("SysConfig files:")
    if not syscfg_files:
        print("  - [none]")
    for detail in syscfg_files:
        assert isinstance(detail, dict)
        print(f"  - {detail['path']}")
        for key in (
            "device",
            "package",
            "sdk_product",
            "sdk_version",
            "sysconfig_version",
            "versions_raw",
        ):
            print(f"      {key}: {detail.get(key) or '[missing]'}")
        print(f"      assigned_pins: {', '.join(detail['assigned_pins']) or '[none found]'}")

    headers = list(result["generated_headers"])
    print("Generated headers (read-only evidence):")
    if not headers:
        print("  - [none]")
    for header in headers:
        assert isinstance(header, dict)
        print(f"  - {header['path']}")
        print(f"      init_functions: {', '.join(header['init_functions']) or '[none found]'}")
        print(f"      instance_macros: {', '.join(header['instance_macros']) or '[none found]'}")
        print(
            "      irq_and_handler_macros: "
            + (", ".join(header["irq_and_handler_macros"]) or "[none found]")
        )
        print(
            "      gpio_port_pin_macros: "
            + (", ".join(header["gpio_port_pin_macros"]) or "[none found]")
        )

    print_items("Generated sources (do not edit)", list(result["generated_sources"]))
    toolchain = result["toolchain"]
    assert isinstance(toolchain, dict)
    print(f"Toolchain evidence: {toolchain['detected']}")
    for key in (
        "ccs_project_files",
        "target_configs",
        "generated_build_rules",
        "keil_projects",
        "cmake_files",
    ):
        print_items(f"  {key}", list(toolchain[key]))
    print_items("Warnings", list(result["warnings"]))
    print_items("Next required checks", list(result["next_required_checks"]))


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only MSPM0 SysConfig preflight for the C07A + S28A project."
    )
    parser.add_argument("project", nargs="?", default=".", help="Project directory to inspect.")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON.")
    args = parser.parse_args()

    project_root = Path(args.project).expanduser().resolve()
    if not project_root.is_dir():
        parser.error(f"project is not a directory: {project_root}")

    result, exit_code = inspect(project_root)
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
    else:
        print_report(result)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
