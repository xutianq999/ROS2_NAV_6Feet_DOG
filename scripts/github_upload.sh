#!/usr/bin/env bash
# Run from a local terminal. Authentication is handled by Git; do not put tokens here.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
REPOSITORY_URL="https://github.com/xutianq999/ROS2_NAV_6Feet_DOG.git"
REMOTE_NAME=origin
cd "$WORKSPACE_ROOT"

if git rev-parse --show-toplevel >/dev/null 2>&1; then
  if [ "$(git rev-parse --show-toplevel)" != "$WORKSPACE_ROOT" ]; then
    echo '错误：项目位于另一个 Git 仓库内，请先确认仓库位置。' >&2
    exit 1
  fi
else
  if ! git init -b main; then
    echo '无法初始化 Git。请在机器人电脑的本机终端运行本脚本，并确认 .git 可以写入。' >&2
    exit 1
  fi
fi

if ! git symbolic-ref -q HEAD >/dev/null; then
  echo '错误：当前处于 detached HEAD 状态，请先切换到工作分支。' >&2
  exit 1
fi

if ! git var GIT_AUTHOR_IDENT >/dev/null 2>&1; then
  echo '请先设置提交者身份，再重新运行本脚本：' >&2
  echo '  git config user.name "你的名字"' >&2
  echo '  git config user.email "你的 GitHub 邮箱或 noreply 邮箱"' >&2
  exit 1
fi

matches_repository() {
  case "$1" in
    "$REPOSITORY_URL"|"${REPOSITORY_URL%.git}"|git@github.com:xutianq999/ROS2_NAV_6Feet_DOG.git) return 0 ;;
    *) return 1 ;;
  esac
}

if git remote get-url origin >/dev/null 2>&1; then
  if ! matches_repository "$(git remote get-url origin)"; then
    # Preserve an existing upstream remote.
    REMOTE_NAME=personal-github
  fi
fi
if git remote get-url "$REMOTE_NAME" >/dev/null 2>&1; then
  if ! matches_repository "$(git remote get-url "$REMOTE_NAME")"; then
    echo "错误：远程 $REMOTE_NAME 已指向其他仓库，请先检查 git remote -v。" >&2
    exit 1
  fi
  while IFS= read -r push_url; do
    if ! matches_repository "$push_url"; then
      echo "错误：远程 $REMOTE_NAME 的推送地址指向其他仓库，请先检查 git remote -v。" >&2
      exit 1
    fi
  done < <(git remote get-url --push --all "$REMOTE_NAME")
else
  git remote add "$REMOTE_NAME" "$REPOSITORY_URL"
fi

git add --all -- .
git diff --cached --stat
if ! git diff --cached --quiet; then
  git commit -m "Sync navigation project source"
fi
if ! git rev-parse --verify HEAD >/dev/null 2>&1; then
  echo '没有可上传的提交。' >&2
  exit 1
fi

echo "即将推送到 $REPOSITORY_URL 的 main 分支。"
echo 'HTTPS 认证请使用本机凭据管理器，或在 Git 的 Password 提示处输入新 Token。'
echo '请勿把 Token 放入仓库地址、脚本或聊天中。'
git push -u "$REMOTE_NAME" HEAD:refs/heads/main
echo '上传完成。后续可再次运行本脚本提交并推送本地修改。'
echo '开始编辑前，请在工作目录干净时运行 git pull --ff-only 拉取更新。'
