# Developer Guide

This guide sets up a development environment with [Visual Studio Code](https://code.visualstudio.com/) as the recommended IDE. This setup ensures code style consistency and easy testing within VS Code.

## Prerequisites

Install these VS Code extensions:

- [Python](https://marketplace.visualstudio.com/items?itemName=ms-python.python)
- [Ruff](https://marketplace.visualstudio.com/items?itemName=charliermarsh.ruff)
- [isort](https://marketplace.visualstudio.com/items?itemName=ms-python.isort)
- [C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
- [Uncrustify](https://marketplace.visualstudio.com/items?itemName=zachflower.uncrustify)

## Configure Ruff for Python

Create or update `.vscode/ruff.toml` with:
```
line-length = 99

[format]
quote-style = "single"
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
            "source.fixAll": "explicit",
            "source.fixAll.ruff": "explicit",
            "source.organizeImports.ruff": "explicit"
        }
    },
    "python.analysis.typeCheckingMode": "basic",
    "python.analysis.autoImportCompletions": true,
    "isort.args": ["--profile", "google"],
    "ruff.format.args": ["--config=./.vscode/ruff.toml"],
    "ruff.organizeImports": false,
    "ruff.showNotifications": "always",
    "editor.formatOnSave": true,
    "[cpp]": {
        "editor.defaultFormatter": "zachflower.uncrustify"
    },
    "uncrustify.configPath.linux": ".vscode/ament_code_style.cfg",
    "C_Cpp.codeAnalysis.clangTidy.enabled": true
}
```

## Running Tests
To run tests, build the workspace and then do:
```
colcon test
colcon test-result --verbose
```
