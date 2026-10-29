#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# Some packages (notably rtabmap_sync) have memory-intensive C++ translation
# units. colcon limits package-level parallelism, while Make/Ninja controls
# source-file parallelism within a package, so constrain both levels here.
# Use BUILD_JOBS=2 ./scripts/build.sh to opt into a higher limit.
BUILD_JOBS="${BUILD_JOBS:-1}"
if ! [[ "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]]; then
  echo "BUILD_JOBS must be a positive integer (got: $BUILD_JOBS)" >&2
  exit 2
fi

export CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS"
export MAKEFLAGS="-j$BUILD_JOBS"

# 底盘适配层（hexplorer_adapter）依赖 vendor 的 custom_msg（RobotCommand/
# RobotState），它以预编译包的形式装在 robot_controller_release 里；有就叠进来。
# 注意：本机 miniconda 按用户手册默认不激活，全程用系统 Python 3.10。
set +u
if [ -f "$HOME/robot_controller_release/ros2_packages/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/robot_controller_release/ros2_packages/setup.bash"
fi
set -u

CMAKE_EXTRA_ARGS=(-DCMAKE_BUILD_TYPE=Release)

# 没有 sudo 装 libasio-dev 时（ugv_sdk 需要），允许把 standalone asio 头文件
# 放在 ~/.local/include；装了系统包的话这段不生效。
set +u
if ! [ -e /usr/include/asio.hpp ] && [ -e "$HOME/.local/include/asio.hpp" ]; then
  export CPATH="$HOME/.local/include${CPATH:+:$CPATH}"
fi

# ~/.local 里还有免 sudo 装的 gtsam / oneTBB(2022) / robin / kiss_matcher
# （原方案要 sudo 装到 /usr/local）。GTSAM 是对着这份 TBB 编译的，系统的
# libtbb(2021.5) 缺它引用的符号，因此所有链接都要先搜 ~/.local 并带 rpath。
# 用显式 cmake-args 传：命令行 -D 优先于 CMakeCache；环境变量 LDFLAGS 只在
# 首次 configure 时生效，对已配置过的 build 目录不可靠。
if [ -d "$HOME/.local/lib/cmake" ]; then
  export CMAKE_PREFIX_PATH="$HOME/.local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
  _LOCAL_LINK="-L$HOME/.local -Wl,-rpath,$HOME/.local"
  CMAKE_EXTRA_ARGS+=(
    "-DCMAKE_EXE_LINKER_FLAGS=$_LOCAL_LINK"
    "-DCMAKE_SHARED_LINKER_FLAGS=$_LOCAL_LINK"
    "-DCMAKE_MODULE_LINKER_FLAGS=$_LOCAL_LINK"
  )
fi
set -u

colcon build --symlink-install \
  --executor sequential \
  --parallel-workers 1 \
  --cmake-args "${CMAKE_EXTRA_ARGS[@]}"
