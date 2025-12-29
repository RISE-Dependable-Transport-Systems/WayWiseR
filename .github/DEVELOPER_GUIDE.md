# Developer Guide

This guide sets up a development environment with [Visual Studio Code](https://code.visualstudio.com/) as the recommended IDE. This setup ensures code style consistency and easy testing within VS Code.

## Prerequisites

To install the recommended VS Code extensions, run the following command in your terminal:

```bash
code --install-extension ms-python.python
code --install-extension charliermarsh.ruff
code --install-extension ms-vscode.cpptools
code --install-extension zachflower.uncrustify
code --install-extension josetr.cmake-language-support-vscode
code --install-extension foxundermoon.shell-format
code --install-extension esbenp.prettier-vscode
```

## Configure Uncrustify for C++

Run the following command to download the `ament_code_style.cfg` to your `.vscode` directory:

```bash
wget https://raw.githubusercontent.com/ament/ament_lint/humble/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg -O .vscode/ament_code_style.cfg
```

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
    "editor.defaultFormatter": "esbenp.prettier-vscode"
  },
  "[yaml]": {
    "editor.defaultFormatter": "esbenp.prettier-vscode"
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
colcon test --base-paths src/WayWiseR/ --packages-skip $WAYWISER_SKIPPED_PACKAGES
colcon test-result --verbose
```
