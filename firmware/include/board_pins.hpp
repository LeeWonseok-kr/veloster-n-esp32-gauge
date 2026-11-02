#pragma once

namespace pins {
constexpr int kCanTx = 4;
#ifdef GAUGE_BOARD_LOLIN_D32
constexpr int kCanRx = 21;
constexpr int kCanStandby = 22;
constexpr int kTftReset = 25;
constexpr int kTftDc = 26;
constexpr int kTftCs = 27;
constexpr int kTftMosi = 23;
constexpr int kTftSck = 18;
// LED is physically wired to 3V. GPIO13 remains unused by the D32 logger.
constexpr bool kTftBacklightAlwaysOn = true;
#else
constexpr int kCanRx = 5;
constexpr int kCanStandby = 6;
constexpr int kTftReset = 8;
constexpr int kTftDc = 9;
constexpr int kTftCs = 10;
constexpr int kTftMosi = 11;
constexpr int kTftSck = 12;
constexpr bool kTftBacklightAlwaysOn = false;
#endif
constexpr int kTftBacklight = 13;
}
