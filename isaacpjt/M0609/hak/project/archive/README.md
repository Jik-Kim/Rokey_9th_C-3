# archive

지금 쓰지 않는 스크립트입니다. **지우지 마세요** — 씬을 특정 시점으로 되돌릴 때
그 상태를 재현하는 유일한 기록입니다.

래퍼(`run_*.sh`)는 `dirname $0` 로 경로를 잡으므로 이 폴더에서 그대로 실행됩니다.

| 파일 | 왜 아카이브했나 |
|---|---|
| `add_pushers.py` | 키네매틱 밀대로 박스를 분기로 밀던 방식. A43 소터의 내장 팝업 휠로 대체 |
| `fix_graph_wiring.py` | `fix_graph_refs.py` 가 더 포괄적 (relationship 뿐 아니라 connection 까지) |
| `spawn_boxes_offline.py` | 오프라인 일괄 배치. `spawn_feeder.py` 의 실시간 투입으로 대체 |
| `set_conveyor.py` | 08-20 초기 실험 |
| `set_conveyor_offline.py` | 08-20 초기 실험. Sdf 레이어 직접 편집 방식의 원형 |
| `setup_sorting.py` | 소터 도입 전(12:33)의 분류 설정 |
| `fix_track01.py` | 08-20, 특정 조각 일회성 수정 |
| `*.bak`, `*_bak` | 스크립트 이전 버전. `sorter.py.pusher_bak` 은 밀대 방식, `sorter.py.velocity_bak` 은 `physics:velocity` 덮어쓰기 방식 |

## 대체 방식이 실패한 이유 (다시 시도하지 않도록)

**`physics:velocity` 덮어쓰기** (`sorter.py.velocity_bak`) — 속도를 순간적으로
바꾸니 박스가 뒤집혔습니다.

**키네매틱 밀대** (`sorter.py.pusher_bak`, `add_pushers.py`) — 뒤집힘은 없었지만
스트로크가 모자라 박스를 끝까지 밀어내지 못했습니다.

둘 다 A43 에셋 안의 `Sorter/ActionGraph` (팝업 휠 기구) 로 대체했습니다.
`binary_switch` 하나로 켜고 끕니다.
