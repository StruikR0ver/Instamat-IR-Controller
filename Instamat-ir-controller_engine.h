#pragma once
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace radiator {

inline uint8_t crc8_maxim(const uint8_t *d, int n) {
  uint8_t crc = 0;
  for (int i = 0; i < n; i++) {
    crc ^= d[i];
    for (int k = 0; k < 8; k++) crc = (crc & 1) ? (uint8_t)((crc >> 1) ^ 0x8C) : (uint8_t)(crc >> 1);
  }
  return crc;
}

// modusnaam -> modusbits (bits 2..7 van byte 1 van frame F02) en byte 3 van frame F10
inline bool mode_to_bits(const std::string &mode, uint8_t &bits, uint8_t &f10_b3) {
  f10_b3 = 0x06;
  if (mode == "Off") { bits = 0x04; return true; }
  if (mode == "Comfort") { bits = 0x01; return true; }
  if (mode == "Night") { bits = 0x02; return true; }
  if (mode == "Fill-Pilot") { bits = 0x10; return true; }
  if (mode == "Clock program") { bits = 0x2F; return true; }
  if (mode == "Frost protection") { bits = 0x08; f10_b3 = 0x02; return true; }
  if (mode == "Boost") { bits = 0x00; f10_b3 = 0x02; return true; }
  return false;
}

inline void add_frame(std::vector<int32_t> &out, const uint8_t f[4], bool last) {
  const uint8_t d[5] = {f[0], f[1], f[2], f[3], crc8_maxim(f, 4)};
  out.push_back(9525);   // header-puls
  out.push_back(-4069);  // header-pauze
  for (int i = 0; i < 5; i++) {
    for (int k = 7; k >= 0; k--) {
      out.push_back(580);                              // bit-puls
      out.push_back(((d[i] >> k) & 1) ? -1638 : -598);  // pauze: lang = 1, kort = 0
    }
  }
  out.push_back(580);                    // stop-puls
  out.push_back(last ? -1000 : -60930);  // stilte tussen frames
}

// wday: 0 = maandag .. 6 = zondag
inline std::vector<int32_t> build(const std::string &mode, float temp, int wday, int hour, int minute, int second) {
  std::vector<int32_t> out;
  uint8_t bits, f10_b3;
  if (!mode_to_bits(mode, bits, f10_b3)) {
#ifdef ESP_LOGW
    ESP_LOGW("radiator", "Onbekend programma '%s': er wordt niets gestuurd (check de namen in de select en in mode_to_bits)",
             mode.c_str());
#endif
    return out;
  }

  if (temp < 15.0f) temp = 15.0f;
  if (temp > 24.0f) temp = 24.0f;
  const int val20 = (int) lroundf(temp * 20.0f);
  const int val10 = (int) lroundf(temp * 10.0f);

  out.reserve(10 * 86);

  const uint8_t f1[4] = {0x00, (uint8_t)(((wday & 7) << 5) | (hour & 0x1F)), (uint8_t) minute, (uint8_t) second};
  add_frame(out, f1, false);

  const uint8_t f2[4] = {0x01, (uint8_t)((bits << 2) | ((val20 >> 8) & 3)), (uint8_t)(val20 & 0xFF), (uint8_t) val10};
  add_frame(out, f2, false);

  for (uint8_t id = 0x08; id <= 0x0E; id++) {  // leeg weekschema
    const uint8_t fs[4] = {id, 0x00, 0x00, 0x00};
    add_frame(out, fs, false);
  }

  const uint8_t f10[4] = {0x02, 0x40, 0x06, f10_b3};
  add_frame(out, f10, true);
  return out;
}

// ------------------------------------------------------------------------------------------------
// ONTVANGEN: het signaal van de echte remote decoderen (voor ESPFill-Pilot on_raw van remote_receiver)
// ------------------------------------------------------------------------------------------------

// Een ontvangen frame (header + 40 bits + stop-puls = 83 waarden) naar 5 bytes; false bij een fout of CRC-fout.
inline bool decode_frame(const std::vector<int32_t> &x, uint8_t d[5]) {
  if (x.size() < 83 || x.size() > 84) return false;
  if (x[0] < 8000 || x[0] > 11000) return false;    // header-puls ca. 9,5 ms
  if (x[1] > -3000 || x[1] < -5000) return false;   // header-pauze ca. 4,1 ms
  for (int i = 0; i < 5; i++) d[i] = 0;
  for (int i = 0; i < 40; i++) {
    const int space = -x[3 + 2 * i];
    if (space < 300 || space > 2500) return false;
    d[i / 8] = (uint8_t)((d[i / 8] << 1) | (space > 1100 ? 1 : 0));
  }
  return crc8_maxim(d, 4) == d[4];
}

struct Decoded {
  bool valid = false;
  std::string mode = "onbekend";
  float temp = 0;
  bool temp_valid = false;   
  bool has_clock = false;
  int wday = 0, hour = 0, minute = 0, second = 0;
  bool has_schedule = false;
  uint32_t schedule[7] = {0, 0, 0, 0, 0, 0, 0};
};

struct RxPress {
  uint8_t f[10][5];
  uint16_t have = 0;
  uint32_t last_ms = 0;
};
static RxPress rx;

inline int id_index(uint8_t id) {
  static const uint8_t ids[10] = {0x00, 0x01, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x02};
  for (int i = 0; i < 10; i++) if (ids[i] == id) return i;
  return -1;
}

inline float round_half(float v) { return roundf(v * 2.0f) / 2.0f; }

inline Decoded interpret(const RxPress &p) {
  Decoded r;
  if (!(p.have & 0x002)) return r;
  const uint8_t *f2 = p.f[1];
  const uint8_t bits = f2[1] >> 2;
  const int val20 = ((f2[1] & 3) << 8) | f2[2];
  const float t20 = val20 / 20.0f;
  const float t10 = f2[3] / 10.0f;
  const bool f10var = (p.have & 0x200) && p.f[9][1] == 0x40 && p.f[9][2] == 0x06 && p.f[9][3] == 0x02;

  bool use_temp = true;
  float temp = t20;
  switch (bits) {
    case 0x04: r.mode = "Off"; break;
    case 0x01: r.mode = "Comfort"; break;
    case 0x02: r.mode = "Night"; break;
    case 0x10: r.mode = "Fill-Pilot"; break;
    case 0x08: r.mode = "Frost protection"; use_temp = false; break;
    case 0x2F: r.mode = "Clock program"; temp = t10; break;
    case 0x00: if (f10var) { r.mode = "Boost"; use_temp = false; } break;
    default: break;
  }
  if (use_temp && r.mode != "onbekend") {
    const float t = round_half(temp);
    if (t >= 15.0f && t <= 24.0f) { r.temp = t; r.temp_valid = true; }
  }
  r.valid = true;

  if (p.have & 0x001) {
    r.has_clock = true;
    r.wday = p.f[0][1] >> 5;
    r.hour = p.f[0][1] & 0x1F;
    r.minute = p.f[0][2];
    r.second = p.f[0][3];
  }
  if ((p.have & 0x1FC) == 0x1FC) {
    r.has_schedule = true;
    for (int d = 0; d < 7; d++) {
      const uint8_t *f = p.f[2 + d];
      r.schedule[d] = ((uint32_t) f[1] << 16) | ((uint32_t) f[2] << 8) | f[3];
    }
  }
  return r;
}

inline bool rx_add(const uint8_t d[5], uint32_t now_ms, Decoded &out) {
  const int idx = id_index(d[0]);
  if (idx < 0) return false;
  if (idx == 0 || (now_ms - rx.last_ms) > 500) rx.have = 0;
  for (int i = 0; i < 5; i++) rx.f[idx][i] = d[i];
  rx.have |= (uint16_t)(1u << idx);
  rx.last_ms = now_ms;
  if (idx != 9) return false;
  out = interpret(rx);
  rx.have = 0;
  return out.valid;
}

inline std::string describe(const Decoded &r) {
  static const char *dagen[7] = {"ma", "di", "wo", "do", "vr", "za", "zo"};
  char buf[96];
  std::string s;
  if (r.has_clock) {
    snprintf(buf, sizeof(buf), "%s %02d:%02d ", dagen[r.wday > 6 ? 6 : r.wday], r.hour, r.minute);
    s += buf;
  }
  s += r.mode;
  if (r.temp_valid) {
    snprintf(buf, sizeof(buf), " %.1f C", r.temp);
    s += buf;
  }
  return s;
}

inline bool schedule_any(const uint32_t *sched) {
  for (int d = 0; d < 7; d++) if (sched[d]) return true;
  return false;
}

inline std::string describe_schedule(const uint32_t *sched) {
  std::string s;
  char buf[16];
  for (int d = 0; d < 7; d++) {
    snprintf(buf, sizeof(buf), "dag %d:", d + 1);
    s += buf;
    if (!sched[d]) s += " -";
    for (int h = 0; h < 24; h++) {
      if (sched[d] & (1UL << h)) {
        snprintf(buf, sizeof(buf), " %02d", h);
        s += buf;
      }
    }
    if (d < 6) s += " | ";
  }
  if (s.size() > 250) s = s.substr(0, 247) + "...";
  return s;
}

}
