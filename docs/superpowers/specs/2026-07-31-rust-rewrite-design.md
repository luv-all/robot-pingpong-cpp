# Rust Rewrite Design — robot-pingpong

## Goal

C++ 로봇 탁구 시스템을 동일한 실시간 제어 루프·알고리즘·하드웨어 인터페이스를 유지한 채 Rust로 재작성한다.

## Architecture

```
Cameras → Vision(Capture/Tracker/DLT) → Predictor → LinearMotor(Y) + Arm(Z/hit)
                                              ↘ Visualizer
```

모듈 경계는 기존 C++ 구조를 따른다.

| Module | Responsibility |
|--------|----------------|
| `constants` | 테이블/카메라/모터 ID 상수 |
| `utils` | FPS 타이머, 다항 회귀 |
| `vision` | 캡처·마스크·스테레오 삼각측량·시각화 |
| `predictor` | Kalman + Y/Z 궤적 예측 + hit timing |
| `dynamixel` | Protocol 2 시리얼 버스·제어 테이블 |
| `control` | Arm IK + LinearMotor(AJINEXTEK FFI / stub) |

## Tech Stack

- Rust 2021, `anyhow` / `thiserror`
- `opencv` — 카메라·이미지 처리·삼각측량·GUI
- `nalgebra` — Kalman / DLT 보조 수치
- `serialport` — Dynamixel Protocol 2
- Windows `ajinextek` feature — AXL/AXM FFI

## Behavior Notes

- 메인 루프·좌표 변환·IK·예측식은 C++와 동등하게 유지
- `resetByZ` bulk write에 `txPacket` 누락 버그는 수정
- Linux에서는 LinearMotor stub, Dynamixel은 `/dev/ttyUSB0`
- 캘리브레이션 파일 `mask.yml` / `points.yml` 포맷은 OpenCV FileStorage 호환 유지

## Testing

- 회귀·IK·예측기·Protocol CRC 단위 테스트 (하드웨어 불필요)
- 전체 바이너리는 카메라/모터 환경에서 통합 검증
