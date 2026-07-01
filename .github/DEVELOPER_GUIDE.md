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

## Develop in a Dev Container

If you want to keep ROS, apt, and Python dependencies off the host, use the committed devcontainer in `.devcontainer/`.

1. Install Docker Engine and the VS Code Dev Containers extension.
2. Open the repository in VS Code and run `Dev Containers: Reopen in Container`.
3. The container mounts this repository at `/workspaces/waywiser_ws/src/WayWiseR`, sets `WAYWISER_WS=/workspaces/waywiser_ws`, and keeps `build/`, `install/`, `log/`, `.venv/`, rosdep state, ccache, and `/etc/waywiser/waywiser.env` in Docker volumes instead of the host checkout.
4. After the first create, `.devcontainer/post-create.sh` writes the container runtime config to `/etc/waywiser/waywiser.env`, enables `waywiser_gazebo` by default, and runs `make setup ARGS="--quiet"` inside the container.

The devcontainer also forwards `DISPLAY` and mounts `/tmp/.X11-unix` so Gazebo GUI applications can connect to the host X server. If Gazebo starts but the GUI cannot open, allow local Docker clients on the host before reopening the container:

```bash
xhost +local:
```

Build and test from a terminal in the container:

```bash
cd $WAYWISER_WS
make build
make test
```

Launch Gazebo from inside the container with:

```bash
cd $WAYWISER_WS
source .venv/bin/activate
ros2 launch waywiser_gazebo gazebo.launch.py
```

`make configure` also targets `/etc/waywiser/waywiser.env` when `WAYWISER_ENV_FILE` is set, so container-side configuration stays inside Docker-managed storage.

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
# Build the amd64 CI image
docker buildx build --platform linux/amd64 --load \
  --build-arg ACT_COMPAT=true \
  -t ghcr.io/das-rise/waywiser/ci-image-amd64:humble \
  -f $WAYWISER_WS/src/WayWiseR/.github/workflows/Dockerfile.ci.amd64 \
  $WAYWISER_WS/src/WayWiseR
```

Then, run the build and test job using `act`. The `--pull=false` flag ensures `act` uses your local image:

```bash
# Run the CI pipeline locally for amd64
act --pull=false \
  --env-file /dev/null \
  -C $WAYWISER_WS/src/WayWiseR \
  --matrix arch:amd64 \
  --env WAYWISER_CHECKOUT_PATH=. \
  --env WAYWISER_WORKING_DIRECTORY=. \
  -j build-and-test
```

## (Experimental) Build amd64 packages

Build amd64 packages:

```bash
make package
```
Packages are written to architecture-specific directories:

```text
$WAYWISER_WS/deb_dist/amd64/
```

Install packages from a local release directory with the generated helper:

```bash
cd $WAYWISER_WS/deb_dist/amd64/
./install-waywiser-debs.bash
```

### Runtime Configuration (deb install)

After installing the Debian packages on a target machine, configure runtime
settings and create a local virtual environment for pip-only WayWiseR
dependencies:

```bash
ros2 run waywiser initialize
```

The initializer updates `/etc/waywiser/waywiser.env`, reads the installed
`pyproject.toml`, detects installed
`ros-humble-waywiser-*` packages, and selects the matching extras automatically.
Use `--print-extras` to preview the selection:

```bash
ros2 run waywiser initialize --print-extras
```

Also, installing `ros-humble-waywiser` creates `/etc/waywiser/waywiser.env` on first install from the bundled `waywiser.env.example`. It is a dpkg **conffile**, so package upgrades never overwrite edits you have made.

You can still run either step separately:

```bash
ros2 run waywiser configure_env
ros2 run waywiser setup_venv
```

After configuring, re-source the ROS setup file so the WayWiseR environment hook loads the updated settings:

```bash
source /opt/ros/humble/setup.bash
```
