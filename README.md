# colmap-model-audit

Read-only COLMAP text-model checks for camera fields, reciprocal tracks, disconnected view graphs and merge overlap.

**English** · [简体中文](README.zh-CN.md) · [Demand evidence](docs/EVIDENCE.md) · [License](LICENSE)

Independent, experimental utility maintained by **Afloat16**. Original implementation; no upstream endorsement is implied.

## Install

Python 3.10+.

```bash
git clone https://github.com/Afloat16/colmap-model-audit.git
cd colmap-model-audit
python -m venv .venv
# Activate the virtual environment for your OS, then:
python -m pip install -e '.[dev]'
```

The core package uses the Python standard library only. No GPU, ROS runtime, API key, hosted service or model weights are required for the core offline workflow.

## First run

```bash
colmap-model-audit examples/model --strict
colmap-model-audit model_txt --images-root photos --compare-model other_model_txt --output model-report.json
colmap model_converter --input_path sparse/0 --output_path model_txt --output_type TXT
```

## Inputs and checks

COLMAP text sparse-model directory with cameras.txt, images.txt and points3D.txt. Binary files must be exported with official model_converter first. The original reader preserves non-contiguous IDs, filenames with spaces and empty observation lines.

Checks legacy camera parameter counts, focal lengths, unit quaternions, absent IDs, point2D indices, reciprocal tracks, duplicate track entries and disconnected view graphs. Optional `--images-root` checks safe relative paths and file existence without decoding photos. Pinhole residuals are computed only for PINHOLE/SIMPLE_PINHOLE. Unknown/new camera models are explicitly unvalidated.

## Boundaries

Exact-name overlap is only a merge candidate; three common names do not prove alignment, database compatibility or successful merging. Does not merge/reconstruct/filter models, fix COLMAP/GLOMAP or establish ground-truth accuracy. Newer rig/frame files are not validated; scope is the legacy camera/image/point projection representation. Each text file is limited to 64 MiB.

## Output and development

Audit commands exit 0 for no error findings, 1 for errors (also warnings with --strict), and 2 for malformed input/report-write failure. --output creates a new JSON file exclusively; existing files are never overwritten. Detailed audit findings are capped at 200 while total severity counts remain accurate.

```bash
python -m unittest discover -s tests -v
python -m compileall -q src tests
python -m build
```

The configured CI matrix covers Ubuntu, Windows and macOS with Python 3.10/3.13; inspect Actions for actual results. Fixtures are synthetic. No private captures or upstream user attachments are redistributed.

Report problems with command/version, expected/actual result and a minimal synthetic reproduction. Keep source attribution and compatible licensing. MIT license, copyright Afloat16.


## Related projects

[Browse Afloat16's project guide](https://github.com/Afloat16/research-to-fix/blob/main/docs/PROJECTS.md) for robotics diagnostics, model interfaces, 3D geometry and engineering workflows.
