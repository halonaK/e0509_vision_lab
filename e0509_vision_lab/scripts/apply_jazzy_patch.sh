#!/usr/bin/env bash
# =============================================================================
# doosan-robot2 (jazzy) 에 Flange Serial 서비스 4개 추가
#   FlangeSerialOpen / Close / Write / Read
#   -> /<ns>/gripper/flange_serial_{open,close,write,read}
#   RH-P12-RN-A 그리퍼를 Tool Flange RS-485(Modbus RTU)로 제어할 때 필요
#
# 사용법:  bash apply_jazzy_patch.sh ~/doosan_ws/src
# 여러 번 실행해도 안전 (이미 적용된 부분은 건너뜀)
# =============================================================================
set -euo pipefail

SRC="${1:-$HOME/doosan_ws/src}"
DSR="$SRC/doosan-robot2"

[ -d "$DSR/dsr_msgs2" ] || { echo "❌ $DSR 없음 (jazzy 브랜치 clone 필요)"; exit 1; }

BR=$(git -C "$DSR" rev-parse --abbrev-ref HEAD 2>/dev/null || echo "?")
[ "$BR" = "jazzy" ] || echo "⚠️  doosan-robot2 브랜치가 '$BR' 입니다 (jazzy 권장)"

# -----------------------------------------------------------------------------
# 1-1. srv 파일 추가
# -----------------------------------------------------------------------------
SRV="$DSR/dsr_msgs2/srv"
cat > "$SRV/FlangeSerialOpen.srv" <<'EOF'
int32 port
int32 baudrate
int32 bytesize
int32 parity
int32 stopbits
---
bool success
EOF
cat > "$SRV/FlangeSerialClose.srv" <<'EOF'
int32 port
---
bool success
EOF
cat > "$SRV/FlangeSerialWrite.srv" <<'EOF'
int32 port
uint8[] data
---
bool success
EOF
cat > "$SRV/FlangeSerialRead.srv" <<'EOF'
int32 port
float32 timeout
---
bool success
int32 size
uint8[] data
EOF
echo "✅ srv 4개 생성"

# -----------------------------------------------------------------------------
# 1-2 ~ 1-4. dsr_msgs2 CMakeLists / controller hpp / controller cpp 수정
# -----------------------------------------------------------------------------
python3 - "$DSR" <<'PY'
import sys, re, pathlib
dsr = pathlib.Path(sys.argv[1])
NAMES = ["Open", "Close", "Write", "Read"]

def edit(path, marker, anchor, new, after=True, regex=False):
    p = dsr / path
    s = p.read_text()
    if marker in s:
        print(f"⏭  {path}: 이미 적용됨"); return
    if regex:
        m = re.search(anchor, s)
        if not m: sys.exit(f"❌ {path}: 기준 위치를 찾지 못함 -> {anchor}")
        i = m.end() if after else m.start()
    else:
        if anchor not in s: sys.exit(f"❌ {path}: 기준 위치를 찾지 못함 -> {anchor}")
        i = s.index(anchor) + (len(anchor) if after else 0)
    p.write_text(s[:i] + new + s[i:])
    print(f"✅ {path}")

# (a) dsr_msgs2/CMakeLists.txt : srv 등록
edit("dsr_msgs2/CMakeLists.txt", "FlangeSerialOpen.srv",
     '"srv/SerialSendData.srv"',
     "".join(f'\n  "srv/FlangeSerial{n}.srv"' for n in NAMES))

# (b) controller hpp : include + 멤버 변수
hpp = "dsr_controller2/include/dsr_controller2/dsr_controller2.hpp"
edit(hpp, "flange_serial_open.hpp",
     '#include "dsr_msgs2/srv/drl_start.hpp"',
     "".join(f'\n#include "dsr_msgs2/srv/flange_serial_{n.lower()}.hpp"' for n in NAMES))
edit(hpp, "m_nh_srv_flange_serial_open",
     r"rclcpp::Service<dsr_msgs2::srv::DrlStart>::SharedPtr\s+m_nh_srv_drl_start;",
     "".join(f"\n  rclcpp::Service<dsr_msgs2::srv::FlangeSerial{n}>::SharedPtr m_nh_srv_flange_serial_{n.lower()};" for n in NAMES),
     regex=True)

# (c) controller cpp : 콜백 + 서비스 등록
cpp = "dsr_controller2/src/dsr_controller2.cpp"
CB = r'''
//----- GRIPPER (Flange Serial) Service Call-back functions [jazzy port] ---------------------------
auto flange_serial_open_cb = [this](const std::shared_ptr<dsr_msgs2::srv::FlangeSerialOpen::Request> req, std::shared_ptr<dsr_msgs2::srv::FlangeSerialOpen::Response> res) -> void
{
    res->success = false;
    if(g_bIsEmulatorMode) {
        RCLCPP_WARN(rclcpp::get_logger("dsr_controller2"),"flange_serial_open is not available in emulator mode.");
        return;
    }
    int port = (req->port > 0) ? req->port : 1;
    int baudrate = (req->baudrate > 0) ? req->baudrate : 115200;
    BYTE_SIZE bytesize = (req->bytesize == 5) ? BYTE_SIZE_FIVEBITES :
                         (req->bytesize == 6) ? BYTE_SIZE_SIXBITS :
                         (req->bytesize == 7) ? BYTE_SIZE_SEVENBITS : BYTE_SIZE_EIGHTBITS;
    PARITY_CHECK parity = (req->parity == 1) ? PARITY_CHECK_ODD :
                          (req->parity == 2) ? PARITY_CHECK_EVEN : PARITY_CHECK_NONE;
    STOP_BITS stopbits = (req->stopbits == 2) ? STOPBITS_TWO : STOPBITS_ONE;
    res->success = Drfl->flange_serial_open(port, baudrate, bytesize, parity, stopbits);
};

auto flange_serial_close_cb = [this](const std::shared_ptr<dsr_msgs2::srv::FlangeSerialClose::Request> req, std::shared_ptr<dsr_msgs2::srv::FlangeSerialClose::Response> res) -> void
{
    res->success = false;
    if(g_bIsEmulatorMode) {
        RCLCPP_WARN(rclcpp::get_logger("dsr_controller2"),"flange_serial_close is not available in emulator mode.");
        return;
    }
    int port = (req->port > 0) ? req->port : 1;
    res->success = Drfl->flange_serial_close(port);
};

auto flange_serial_write_cb = [this](const std::shared_ptr<dsr_msgs2::srv::FlangeSerialWrite::Request> req, std::shared_ptr<dsr_msgs2::srv::FlangeSerialWrite::Response> res) -> void
{
    res->success = false;
    if(g_bIsEmulatorMode) {
        RCLCPP_WARN(rclcpp::get_logger("dsr_controller2"),"flange_serial_write is not available in emulator mode.");
        return;
    }
    int port = (req->port > 0) ? req->port : 1;
    std::vector<char> buf(req->data.begin(), req->data.end());
    res->success = Drfl->flange_serial_write(static_cast<int>(buf.size()), buf.data(), port);
};

auto flange_serial_read_cb = [this](const std::shared_ptr<dsr_msgs2::srv::FlangeSerialRead::Request> req, std::shared_ptr<dsr_msgs2::srv::FlangeSerialRead::Response> res) -> void
{
    res->success = false;
    res->size = 0;
    if(g_bIsEmulatorMode) {
        RCLCPP_WARN(rclcpp::get_logger("dsr_controller2"),"flange_serial_read is not available in emulator mode.");
        return;
    }
    int port = (req->port > 0) ? req->port : 1;
    LPFLANGE_SER_RXD_INFO_EX rxd = Drfl->flange_serial_read(req->timeout, port);
    if(rxd != nullptr && rxd->_iSize > 0) {
        res->size = rxd->_iSize;
        res->data.assign(reinterpret_cast<uint8_t*>(rxd->_cRxd),
                         reinterpret_cast<uint8_t*>(rxd->_cRxd) + rxd->_iSize);
        res->success = true;
    }
};

'''
edit(cpp, "flange_serial_open_cb",
     "//----- TCP Service Call-back functions", CB, after=False)

REG = "".join(
    f'\n  m_nh_srv_flange_serial_{n.lower()} = get_node()->create_service<dsr_msgs2::srv::FlangeSerial{n}>(svc_prefix_ + "gripper/flange_serial_{n.lower()}", flange_serial_{n.lower()}_cb);'
    for n in NAMES)
edit(cpp, "create_service<dsr_msgs2::srv::FlangeSerialOpen>",
     r"m_nh_srv_drl_start\s*=\s*get_node\(\)->create_service<dsr_msgs2::srv::DrlStart>\([^;]*\);",
     REG, regex=True)
PY

echo
echo "🎉 완료. 빌드는 dsr_msgs2 먼저:"
echo "   cd $(dirname "$SRC")"
echo "   colcon build --symlink-install --packages-select dsr_msgs2 && source install/setup.bash"
echo "   colcon build --symlink-install && source install/setup.bash"
