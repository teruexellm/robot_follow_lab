# 追従状態機械の状態 (spec v2.0 §43, §44, §55)
#
# spec が TARGET_LOST / SEARCHING / TARGET_REDETECTED というイベント名を要求するため、
# telemetry と video overlay が参照できる形で publish する。

uint8 INIT=0
uint8 DETECTED=1
uint8 LOST=2
uint8 SEARCHING=3

std_msgs/Header header

uint8 state
string state_label

# 最後の有効な検出からの経過時間 [s]
float32 time_since_detection

# 直前の有効な検出で得た推定距離 [m]。未検出時は負値
float32 last_estimated_distance

uint32 lost_count
uint32 redetect_count
