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

## Configure VS Code Workspace

To keep `src/`, `resources/`, and workspace-level build outputs visible in Explorer while still using repository-managed VS Code settings, create a workspace file at `$WAYWISER_WS` and symlink the `.vscode` folder:

```bash
cd $WAYWISER_WS
ln -sfn src/WayWiseR/.vscode .vscode
cat > waywiser.code-workspace <<'EOF'
{
  "folders": [
    {
      "path": "."
    }
  ],
  "settings": {
    "git.scanRepositories": ["src/WayWiseR"]
  }
}
EOF
```

Then open this workspace in VS Code:

```bash
code $WAYWISER_WS/waywiser.code-workspace
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
make test
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
docker build -t ghcr.io/das-rise/waywiser/ci-image:humble \
  -f $WAYWISER_WS/src/WayWiseR/.github/workflows/Dockerfile.ci \
  $WAYWISER_WS/src/WayWiseR
```

Then, run the build and test job using `act`. The `--pull=false` flag ensures `act` uses your local image:

> [!NOTE]
> If you are using **rootless Docker**, you may need to specify the `DOCKER_HOST` environment variable so `act` can find the local Docker socket:
>
> ```bash
> export DOCKER_HOST=$(docker context inspect rootless --format '{{.Endpoints.docker.Host}}')
> ```

```bash
# Run the CI pipeline locally
act -j build-and-test --pull=false -P ubuntu-22.04=ghcr.io/das-rise/waywiser/ci-image:humble -C $WAYWISER_WS/src/WayWiseR
```
