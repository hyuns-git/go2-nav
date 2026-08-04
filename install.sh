#!/bin/bash
# go2-nav 설치 스크립트
# 사용: bash install.sh
set -e

REPO="$(cd "$(dirname "$0")" && pwd)"
WS=~/ros2_ws
PKG=$WS/src/go2_nav_bridge
STAMP=$(date +%Y%m%d_%H%M%S)

echo "════════════════════════════════════════"
echo " go2-nav 설치"
echo " 저장소: $REPO"
echo " 워크스페이스: $WS"
echo "════════════════════════════════════════"

# 환경 확인
if [ -z "$ROS_DISTRO" ]; then
  echo "!! ROS 환경이 없습니다. 먼저 실행하세요:"
  echo "   source ~/unitree_ros2/setup.sh"
  exit 1
fi

# 기존 백업
if [ -d "$PKG" ]; then
  BK=~/go2_backup_$STAMP
  mkdir -p "$BK"
  cp -r "$PKG" "$BK/" 2>/dev/null || true
  echo "[백업] $BK"
fi

# 패키지 복사 (nav2_go2.yaml 은 사용자가 만든 것을 보존)
mkdir -p "$PKG"
KEEP=""
if [ -f "$PKG/config/nav2_go2.yaml" ]; then
  cp "$PKG/config/nav2_go2.yaml" /tmp/nav2_go2.yaml.keep
  KEEP=1
fi
cp -r "$REPO/src/go2_nav_bridge/." "$PKG/"
if [ -n "$KEEP" ]; then
  cp /tmp/nav2_go2.yaml.keep "$PKG/config/nav2_go2.yaml"
  echo "[보존] 기존 nav2_go2.yaml"
else
  echo "[안내] nav2_go2.yaml 이 없습니다."
  echo "       config/NAV2_PATCH.md 를 따라 만드세요."
fi

# 도구 복사
mkdir -p ~/go2_tools ~/maps ~/logs
cp "$REPO"/tools/* ~/go2_tools/
chmod +x ~/go2_tools/*
echo "[복사] ~/go2_tools/"

# 빌드
cd "$WS"
colcon build --symlink-install --packages-select go2_nav_bridge
echo ""
echo "════════════════════════════════════════"
echo " 완료"
echo ""
echo "   source ~/ros2_ws/install/setup.bash"
echo "   ros2 pkg executables go2_nav_bridge"
echo ""
echo " 다음: docs/02-mapping.md"
echo "════════════════════════════════════════"
