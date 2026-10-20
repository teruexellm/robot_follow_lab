# Camera から得た Leader の検出結果 (spec v2.0 §37)
#
# bounding box の値は画像サイズで正規化した 0.0〜1.0 とする (spec §23)。
# detected が false のとき、bbox と confidence の値は意味を持たない。

std_msgs/Header header

bool detected
float32 confidence

float32 center_x
float32 center_y

float32 width
float32 height
