# apu_medication_support

# Author
tsukamoto tomoki

# Date
2021/08/20

# 構成
## launch
- identify_case
薬の認識のみ実行
- carry_case(_rsh)
薬の認識、運搬

## script
- hsrb_task_apu_components
基本的な動作（吸引、移動、発話など）の関数

- carry_case
薬ケース運搬動作
- move_medicine_calendar
薬カレンダー前に移動

## src
- identify_case：薬ケース認識
- edge_detection：エッジ検出
- transparent_detection：透明検出
