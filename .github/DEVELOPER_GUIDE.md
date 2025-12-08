# Developer Guide

This guide sets up a development environment with [Visual Studio Code](https://code.visualstudio.com/) as the recommended IDE. This setup ensures code style consistency and easy testing within VS Code.

## Prerequisites

Install these VS Code extensions:

- [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
- [Ruff](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff)
- [C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
- [Uncrustify](https://marketplace.visualstudio.com/items?itemName=zachflower.uncrustify)
- [CMake Language Support](https://marketplace.visualstudio.com/items?itemName=josetr.cmake-language-support-vscode)
- [shell-format](https://marketplace.visualstudio.com/items?itemName=foxundermoon.shell-format)
- [XML](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-xml)
- [YAML](https://marketplace.visualstudio.com/items?itemName=redhat.vscode-yaml)

## Configure Ruff for Python

Create or update `.vscode/ruff.toml` with:

```toml
# Allow lines to be as long as 99
line-length = 99

[lint]
# Select E (PEP8), W (warnings), F (pyflakes), C (complexity), Q (quotes), I (imports)
select = ["E", "F", "W", "C", "Q", "I"]

[format]
quote-style = "single"
docstring-code-format = true
docstring-code-line-length = 99

[lint.isort]
known-first-party = ["waywiser_core", "waywiser_py", "waywiser_test_runner", "waywiser_twist_safety"]
known-third-party = ["rclpy", "ament_index_python", "matplotlib", "numpy", "pymap3d", "shapely", "tf_transformations"]
force-sort-within-sections = true
order-by-type = false
```

## Configure Uncrustify for C++

Download [ament_code_style.cfg](https://github.com/ament/ament_lint/blob/humble/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg) to `.vscode` directory.

## Configure VS Code Settings

Create or update `.vscode/settings.json` with:

```json
{
  "[python]": {
    "editor.defaultFormatter": "charliermarsh.ruff",
    "editor.codeActionsOnSave": {
      "source.organizeImports": "explicit",
      "source.fixAll": "explicit"
    }
  },
  "python.analysis.typeCheckingMode": "basic",
  "python.analysis.autoImportCompletions": true,
  "ruff.configuration": "./.vscode/ruff.toml",
  "ruff.organizeImports": true,
  "editor.formatOnSave": true,
  "[cpp]": {
    "editor.defaultFormatter": "zachflower.uncrustify"
  },
  "uncrustify.configPath.linux": ".vscode/ament_code_style.cfg",
  "C_Cpp.codeAnalysis.clangTidy.enabled": true,
  "[cmake]": {
    "editor.defaultFormatter": "josetr.cmake-language-support-vscode"
  },
  "[shellscript]": {
    "editor.defaultFormatter": "foxundermoon.shell-format"
  },
  "[xml]": {
    "editor.defaultFormatter": "redhat.vscode-xml"
  },
  "[yaml]": {
    "editor.defaultFormatter": "redhat.vscode-yaml"
  },
  "[jsonc]": {
    "editor.defaultFormatter": "vscode.json-language-features"
  }
}
```

## Running Tests

To run tests, build the workspace and then do:

```bash
cd $WAYWISER_WS
colcon test --base-paths src/WayWiseR/
colcon test-result --verbose
```
