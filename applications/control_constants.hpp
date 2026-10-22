#ifndef CONTROL_CONSTANTS_HPP
#define CONTROL_CONSTANTS_HPP

namespace control_constants
{
// Temporary RC/CAN diagnosis. false restores the selected power plot.
constexpr bool CHASSIS_CONTROL_DIAGNOSTICS = false;

// GY-56 receive test: physical 3-pin UART1 = USART6.
constexpr bool INFRARED_PLOT_TEST = false;
constexpr bool INFRARED_PLOT_RAW_FIELDS = true;   // 8 channels; false restores 4.
constexpr unsigned INFRARED_UART_BAUD = 115200U;  // Set 115200 if module was configured so.
constexpr bool INFRARED_AUTO_BAUD = true;
constexpr unsigned INFRARED_BAUD_PROBE_MS = 2000U;
// 0: accept 3/4-byte payloads; nonzero: require this exact payload length.
constexpr unsigned INFRARED_FRAME_DATA_LENGTH = 0U;
// Assumed GY-56 distance field is cm. Use 0.1 if its manual specifies mm.
constexpr float INFRARED_RAW_TO_CM = 1.0f;
constexpr unsigned INFRARED_VALID_TIMEOUT_MS = 500U;
static_assert(
  INFRARED_UART_BAUD == 9600U || INFRARED_UART_BAUD == 115200U, "Unsupported GY-56 baud rate");
static_assert(
  (INFRARED_FRAME_DATA_LENGTH == 0U ||
   (INFRARED_FRAME_DATA_LENGTH >= 2U && INFRARED_FRAME_DATA_LENGTH <= 8U)),
  "Invalid GY-56 payload length");
static_assert(INFRARED_RAW_TO_CM > 0.0f, "Invalid distance unit conversion");

// Initial tuning values only; NOT calibrated for this chassis.
// Torque: gearbox output shaft N*m; speed: same shaft rad/s.
// Refit K1/K2 if the shaft convention or actual gear ratio changes.
// K1 是铜损系数；按参考文档它还承担"加减速阶段由加速度引起的额外能耗"。
// 【2026-10 标定尝试记录】用 data_1（1291 采样，扣除 7.4% 刹车/再生）最小二乘得
//   K1 = 3.97, K2 ≈ 0, K3 = 4.82，残差 ±1.49 W、R² = 0.981。
//   但该批数据最高只到 62 W，76~80 W 属于外推 → 实机表现反而比下面这组经验值差，
//   所以暂时回到"经验值 + 8 W 裕量"的方案。
//   若要让标定真正生效：把 CHASSIS_POWER_LIMIT_W 临时设 200、绘图模式设 4，
//   录一段真正跑到 90~100 W 的数据重新拟合，再用 3~4 W 裕量。
// K2 must be calibrated in the actual shaft-speed coordinates.
// Rotor-to-output conversion multiplies the speed-squared coefficient by ratio^2.
constexpr float POWER_K1 = 2.80f;  // bias-era coefficient  // bias-era coefficient   // W / (N*m)^2
constexpr float POWER_K2 =
  0.009f;                         // bias-era coefficient  // bias-era coefficient  // W / (rad/s)^2
constexpr float POWER_K3 = 4.0f;  // bias-era coefficient
// ---- Measured-power slow correction (bias model) ----
// effective limit = (limit - margin) x bias, bias in [MIN, MAX], regulated on the
// low-pass filtered measured power so the mean sits on the limit.
constexpr float POWER_BIAS_DOWN_PER_W = 5e-5f;
constexpr float POWER_BIAS_UP_PER_W = 2e-5f;
constexpr float POWER_BIAS_MIN = 0.6f;
constexpr float POWER_BIAS_MAX = 1.3f;
constexpr float POWER_BIAS_RECOVER_BAND_W = 2.0f;
constexpr float POWER_BIAS_FILTER_S = 0.3f;
constexpr float POWER_MECH_LOSS_RATIO = 0.25f;  // transmission/ESC loss
constexpr float POWER_RESERVE_LEAD_S = 0.05f;
constexpr float POWER_RESERVE_MAX_W = 15.0f;  // bias-era coefficient    // W, 整机空载损耗

constexpr bool POWER_MODEL_CALIBRATED = false;

constexpr float CHASSIS_POWER_LIMIT_W =
  80.0f;  // hard limit  // restored: real 80 W limit  // temporarily raised: compare prediction vs measured without limiting
// Measured-power PI target = LIMIT - MARGIN; model budget = target + bounded PI trim.
// 标定后模型误差只有 ±1.5 W（最大 6.5 W），所以裕量可以从 8 W 降回 4 W：
//   限幅点 76 W，均值贴 76，峰值约 80~82。
// 想更激进用 3；想更保守用 5~6。
constexpr float POWER_LIMIT_MARGIN_W =
  8.0f;  // nominal budget 72 W  // limit target 77 W (80 - 3)  // bias-era margin (target 72 W)
static_assert(CHASSIS_POWER_LIMIT_W > 0.0f, "Power limit must be positive");
static_assert(
  POWER_LIMIT_MARGIN_W >= 0.0f && POWER_LIMIT_MARGIN_W < CHASSIS_POWER_LIMIT_W,
  "Invalid power margin");
constexpr unsigned POWER_LIMIT_SEARCH_STEPS = 24;
static_assert(POWER_LIMIT_SEARCH_STEPS > 0, "Search needs at least one step");

// 功率计多久没更新就视为无效（不再参与修正�?
constexpr unsigned POWER_METER_TIMEOUT_MS = 200U;

// Measured-power PI (budget correction in W), matching power_model.cpp.
constexpr float POWER_PI_KP = 0.4f;
constexpr float POWER_PI_KI = 1.5f;
constexpr float POWER_PI_FILTER_S = 0.03f;
constexpr float POWER_PI_DEADBAND_W = 0.3f;
constexpr float POWER_PI_TRIM_MIN_W = -40.0f;
constexpr float POWER_PI_TRIM_MAX_W = 12.0f;
constexpr float POWER_PI_RISE_W_PER_S = 20.0f;
constexpr float POWER_FAST_DROP_KP = 2.0f;
constexpr float POWER_FAST_OVER_MAX_SCALE = 0.95f;
static_assert(
  POWER_PI_KP >= 0.0f && POWER_PI_KI >= 0.0f && POWER_PI_FILTER_S > 0.0f &&
    POWER_PI_DEADBAND_W >= 0.0f && POWER_PI_TRIM_MIN_W <= 0.0f && POWER_PI_TRIM_MAX_W >= 0.0f &&
    POWER_PI_RISE_W_PER_S > 0.0f,
  "Invalid measured-power PI");
static_assert(
  POWER_FAST_DROP_KP > 0.0f && POWER_FAST_OVER_MAX_SCALE > 0.0f && POWER_FAST_OVER_MAX_SCALE < 1.0f,
  "Invalid fast power guard");

// Fast measured-power trend guard, separate from the original prediction formula.
constexpr float POWER_TREND_LEAD_S = 0.05f;    // Forecast horizon.
constexpr float POWER_TREND_FILTER_S = 0.04f;  // Filter power slope only.
constexpr float POWER_TREND_RECOVER_PER_S =
  2.0f;  // Recovery rate: up to two scale units per second.
constexpr float POWER_TREND_RECOVER_BAND_W = 0.25f;
static_assert(
  POWER_TREND_LEAD_S >= 0.0f && POWER_TREND_LEAD_S <= 0.5f, "Invalid power forecast horizon");
static_assert(
  POWER_TREND_FILTER_S > 0.0f && POWER_TREND_RECOVER_PER_S > 0.0f &&
    POWER_TREND_RECOVER_BAND_W >= 0.0f,
  "Invalid power trend guard parameters");

}  // namespace control_constants

#endif  // CONTROL_CONSTANTS_HPP
