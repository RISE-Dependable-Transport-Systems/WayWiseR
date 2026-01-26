# Developer Guide

This guide sets up a development environment with [Visual Studio Code](https://code.visualstudio.com/) as the recommended IDE. This setup ensures code style consistency and easy testing within VS Code.

## Prerequisites

To install the recommended VS Code extensions, run the following command in your terminal:

```bash
code --install-extension ms-python.python
code --install-extension charliermarsh.ruff
code --install-extension llvm-vs-code-extensions.vscode-clangd
code --install-extension zachflower.uncrustify
code --install-extension josetr.cmake-language-support-vscode
code --install-extension shakram02.bash-beautify
code --install-extension esbenp.prettier-vscode
code --install-extension DotJoshJohnson.xml
```

## Configure Uncrustify for C++

Run the following command to download the `ament_code_style.cfg` to your `.vscode` directory:

```bash
wget https://raw.githubusercontent.com/ament/ament_lint/humble/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg -O $WAYWISER_WS/src/WayWiseR/.vscode/ament_code_style.cfg
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
    },
    "editor.formatOnSave": true
  },
  "python.analysis.typeCheckingMode": "basic",
  "python.analysis.autoImportCompletions": true,
  "ruff.organizeImports": true,
  "editor.formatOnSave": true,
  "[cpp]": {
    "editor.defaultFormatter": "zachflower.uncrustify"
  },
  "uncrustify.configPath.linux": ".vscode/ament_code_style.cfg",
  "[cmake]": {
    "editor.defaultFormatter": "josetr.cmake-language-support-vscode"
  },
  "cmake.ignoreCMakeListsMissing": true,
  "cmakeFormat.args": ["--max-pargs-hwrap=6"],
  "[shellscript]": {
    "editor.defaultFormatter": "shakram02.bash-beautify"
  },
  "[xml]": {
    "editor.defaultFormatter": "DotJoshJohnson.xml"
  },
  "[yaml]": {
    "editor.defaultFormatter": "esbenp.prettier-vscode"
  },
  "[jsonc]": {
    "editor.defaultFormatter": "esbenp.prettier-vscode"
  },
  "clangd.arguments": [
    "--background-index",
    "--pretty",
    "--clang-tidy",
    "--query-driver=/usr/bin/g++",
    "--header-insertion=never",
    "--compile-commands-dir=${workspaceFolder}/../../build"
  ],
  "C_Cpp.intelliSenseEngine": "disabled"
}
```

## Running Tests

To run tests, build the workspace and then do:

```bash
cd $WAYWISER_WS
colcon test --base-paths src/WayWiseR/ --packages-skip $WAYWISER_SKIPPED_PACKAGES
colcon test-result --verbose
```

## Local CI Pipeline (Docker & act)

To verify changes in an environment identical to the GitHub Actions runner, you can run the CI pipeline locally using [Docker](https://docs.docker.com/engine) and [act](https://nektosact.com/).

### Installation

1. **Docker**: Install `Docker engine` for your platform by following the [official guide](https://docs.docker.com/engine/install/).
2. **act**: Install `act` CLI by following the [official guide](https://nektosact.com/installation/).

### Running the Pipeline

Before running the pipeline for the first time or after changing the CI environment, build the custom CI image locally:

```bash
# Build the CI image
cd $WAYWISER_WS/src/WayWiseR
docker build -t ghcr.io/rise-dependable-transport-systems/waywiser/ci-image:humble -f .github/workflows/Dockerfile.ci .
```

Then, run the build and test job using `act`. The `--pull=false` flag ensures `act` uses your local image:

```bash
# Run the CI pipeline locally
cd $WAYWISER_WS/src/WayWiseR
act -j build-and-test --pull=false -P ubuntu-22.04=ghcr.io/rise-dependable-transport-systems/waywiser/ci-image:humble
```
