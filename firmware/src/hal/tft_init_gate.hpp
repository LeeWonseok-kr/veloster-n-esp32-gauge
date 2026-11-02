#pragma once

#include <cstdint>

#include <dashboard_model.hpp>

namespace tft_init {

constexpr bool hasExpectedGeometry(int32_t width, int32_t height) {
  return width == layout::kScreenWidth && height == layout::kScreenHeight;
}

}  // namespace tft_init
