# Bundled C07A and S28A board documents

These are canonical board-source files bundled with the Skill. They establish board wiring and connector labels but do not establish generated macros, SysConfig schema, IRQ names, or DriverLib APIs.

## C07A

- [`C07A核心板原理图_V1.1（MSPM0G3507）.pdf`](c07a/C07A核心板原理图_V1.1（MSPM0G3507）.pdf): C07A V1.1 core-board schematic; identifies MSPM0G3507SPTR.

## S28A

- [`0.S28A转接板使用必读.doc`](s28a/0.S28A转接板使用必读.doc): original adapter-board usage notes.
- [`1.C07A搭配S28A底板丝印图.png`](s28a/1.C07A搭配S28A底板丝印图.png): silkscreen map.
- [`2.C07A搭配S28A底板丝印实物说明图.png`](s28a/2.C07A搭配S28A底板丝印实物说明图.png): annotated physical-board image.
- [`3.C07A搭配S28A底板资源分配表25.7.29.pdf`](s28a/3.C07A搭配S28A底板资源分配表25.7.29.pdf): resource allocation table.
- [`4.C07A适配S28A底板原理图.pdf`](s28a/4.C07A适配S28A底板原理图.pdf): adapter/bottom-board schematic.

The former nested S28A directory contained byte-identical duplicates of these five files and was removed after SHA-256 verification.
