# 현재 개발보드 배선

| 출발 | 도착 |
|---|---|
| 차량 CAN H / CAN L | CAN 모듈 H / L |
| 차량 12V / GND | 기존 BEC 입력 |
| BEC 5V | ESP32-S3 5V 및 TFT VCC |
| ESP32-S3 3V3 | 3.3V CAN 모듈 VCC |
| GPIO4 | CAN TXD/D, 기존 10kΩ→3.3V 풀업 유지 |
| GPIO5 | CAN RXD/R |
| GPIO6 | CAN RS/STB가 노출된 기존 모듈에서만 해당 |
| GPIO8 / 9 / 10 | TFT RST / DC / CS |
| GPIO11 / 12 | TFT MOSI / SCK |
| GPIO13 | TFT LED 제어 입력. 기존 LED→3.3V 연결은 제거 |
| GND | BEC·ESP·CAN·TFT·차량 공통 기준 |

TFT VCC는5V, 논리신호는3.3V입니다. GPIO13에 백라이트 전력을 직접 공급하는 모듈을 연결하면 안 됩니다. TFT LED 입력 회로의 실제 리비전 확인은 필요합니다. MISO/터치/SD는 현재 사용하지 않습니다.

OBD 표준 핀16은 배터리+,6은CAN H,14는CAN L입니다. 차량 접지는 기존 검증된 케이블 접지 배선을 유지합니다. 커스텀 차량 케이블의 커넥터 모양만 보고 일반 USB 핀맵을 적용하지 않습니다. 추가120Ω 저항을 병렬로 넣지 않습니다.

업로드는 주차·OBD/BEC 분리·USB 단독 상태에서 진행합니다. 차량 전원을 사용하는 동안은 기존 USB VBUS 분리 구성을 유지하여 노트북5V와BEC5V를 합치지 않습니다.

근거: 프로젝트 board_pins.hpp, 현재 auto_sleep 환경, 사용자의 GPIO13 배선 확인. CAN RS/STB의 실제 연결 여부, BEC 정격 및 케이블 내부 도통은 이 개요도만으로 검증되지 않습니다.
