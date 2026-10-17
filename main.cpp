#include <GLFW/glfw3.h>

#include <cfloat>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "sim.hpp"

using namespace nprn;

static const int COLRGB[4][3] = {{0x52, 0xd1, 0xc4},
                                 {0xd9, 0xa4, 0x41},
                                 {0xe0, 0x63, 0x7a},
                                 {0x8a, 0x7f, 0xd6}};
static ImU32 morphCol(int i, int a = 255) {
  return IM_COL32(COLRGB[i][0], COLRGB[i][1], COLRGB[i][2], a);
}
static ImVec4 morphVec(int i) {
  return ImVec4(COLRGB[i][0] / 255.f, COLRGB[i][1] / 255.f,
                COLRGB[i][2] / 255.f, 1.f);
}
static const ImU32 C_GRID = IM_COL32(0x1c, 0x24, 0x30, 255),
                   C_BORDER = IM_COL32(0x2a, 0x33, 0x40, 255);
static const ImU32 C_MUTED = IM_COL32(0x7c, 0x8b, 0xa0, 255),
                   C_AMBER = IM_COL32(0xd9, 0xa4, 0x41, 255);
static const ImU32 C_WHITE = IM_COL32(255, 255, 255, 255);

static ImU32 hsl(float hDeg, float s, float l) {
  const float c = (1 - std::fabs(2 * l - 1)) * s,
              hp = std::fmod(hDeg, 360.f) / 60.f;
  const float x = c * (1 - std::fabs(std::fmod(hp, 2.f) - 1));
  float r = 0, g = 0, b = 0;
  if (hp < 1) {
    r = c;
    g = x;
  } else if (hp < 2) {
    r = x;
    g = c;
  } else if (hp < 3) {
    g = c;
    b = x;
  } else if (hp < 4) {
    g = x;
    b = c;
  } else if (hp < 5) {
    r = x;
    b = c;
  } else {
    r = c;
    b = x;
  }
  const float m = l - c / 2;
  return IM_COL32(int((r + m) * 255), int((g + m) * 255), int((b + m) * 255),
                  255);
}

struct Cv {
  ImDrawList *dl;
  ImVec2 o, sz;
  float sc;
  ImVec2 P(double x, double y) const {
    return ImVec2(o.x + (float)x * sc, o.y + (float)y * sc);
  }
  void text(double x, double y, ImU32 c, const std::string &s) const {
    dl->AddText(P(x, y - 10), c, s.c_str());
  }
};
static Cv beginCv(float lh) {
  Cv c;
  const float w = std::max(120.f, ImGui::GetContentRegionAvail().x);
  c.sc = w / 640.f;
  c.sz = ImVec2(w, lh * c.sc);
  c.o = ImGui::GetCursorScreenPos();
  c.dl = ImGui::GetWindowDrawList();
  ImGui::Dummy(c.sz);
  c.dl->AddRectFilled(c.o, ImVec2(c.o.x + c.sz.x, c.o.y + c.sz.y),
                      IM_COL32(0, 0, 0, 255), 6.f);
  c.dl->AddRect(c.o, ImVec2(c.o.x + c.sz.x, c.o.y + c.sz.y), C_BORDER, 6.f);
  c.dl->PushClipRect(c.o, ImVec2(c.o.x + c.sz.x, c.o.y + c.sz.y), true);
  return c;
}
static void endCv(Cv &c) { c.dl->PopClipRect(); }

static int domReg(const Neuron &n) {
  int k = 0;
  for (int i = 0; i < 4; i++)
    if (n.wins[i] > n.wins[k])
      k = i;
  return n.wins[k] > 50 ? k : -1;
}

static void drawRobot(const Sim &s) {
  Cv c = beginCv(200);
  const double K = 120, gy = 176;
  const Robot &r = s.rob;
  const int reg = SEQ[s.schedI];
  const ImU32 mc = morphCol(reg);
  const double X = 320 + r.s * K, Y = gy - WR * K;

  c.dl->AddLine(c.P(0, gy), c.P(640, gy), C_BORDER, 1.f);
  for (int i = -10; i <= 10; i++) {
    const double gx = 320 + i * .25 * K;
    c.dl->AddLine(c.P(gx, gy), c.P(gx - 6, gy + 6), C_BORDER, 1.f);
  }
  for (int m = -2; m <= 2; m++)
    c.text(320 + m * K - 10, gy + 18, IM_COL32(0x55, 0x55, 0x66, 255),
           fmt("%d m", m));
  for (int sg = -1; sg <= 1; sg += 2)
    c.dl->AddLine(c.P(320 + sg * WALL * K, gy),
                  c.P(320 + sg * WALL * K, gy - 40), morphCol(2), 3.f * c.sc);

  c.dl->AddCircle(c.P(X, Y), float(WR * K) * c.sc,
                  IM_COL32(0xee, 0xee, 0xee, 255), 40, 3.f * c.sc);
  c.dl->AddLine(c.P(X, Y),
                c.P(X + std::cos(r.wa) * WR * K, Y + std::sin(r.wa) * WR * K),
                IM_COL32(0xee, 0xee, 0xee, 255), 1.5f * c.sc);

  const double ct = std::cos(r.th), st = std::sin(r.th);
  auto rot = [&](double x, double y) {
    return c.P(X + x * ct - y * st, Y + x * st + y * ct);
  };
  const double Hl = r.h - .14, L = .25;
  const double kx = std::sqrt(std::max(0., L * L - Hl * Hl / 4)) * K;
  ImVec2 leg[3] = {rot(0, 0), rot(kx, -Hl * K / 2), rot(0, -Hl * K)};
  c.dl->AddPolyline(leg, 3, mc, ImDrawFlags_RoundCornersAll, 4.f * c.sc);
  const double bw = (.10 + .018 * r.m) * K, bh = .14 * K;
  ImVec2 box[4] = {rot(-bw / 2, -Hl * K - bh), rot(bw / 2, -Hl * K - bh),
                   rot(bw / 2, -Hl * K), rot(-bw / 2, -Hl * K)};
  c.dl->AddConvexPolyFilled(
      box, 4, hsl(float(210 - r.m * 12), .45f, float(30 + r.m * 2) / 100.f));
  c.dl->AddPolyline(box, 4, mc, ImDrawFlags_Closed, 2.f * c.sc);

  c.text(10, 16, C_MUTED,
         fmt("h=%.2f m  mass=%.1f kg  x=%.2f m  pitch=%.1f deg  v=%.2f m/s",
             r.h, r.m, r.s, r.th * 57.3, r.v));
  c.text(10, 30, mc,
         r.fall > 0  ? fmt("FALLEN, righting in %.1f s", r.fall * .05)
         : s.pad.man ? "manual height"
                     : fmt("true morph %d", reg));
  endCv(c);
}

static void drawPhase(const Sim &s) {
  Cv c = beginCv(160);
  const double cx = 320, cy = 80, S = 140;
  c.dl->AddLine(c.P(0, cy), c.P(640, cy), C_GRID);
  c.dl->AddLine(c.P(cx, 0), c.P(cx, 160), C_GRID);
  const size_t n = s.trail.size();
  for (size_t i = 0; i < n; i++) {
    const auto &p = s.trail[i];
    const int a = int(255 * (.15 + .85 * double(i) / double(n)));
    c.dl->AddCircleFilled(c.P(cx + p.x[0] * S, cy - p.x[1] * S), 2.5f * c.sc,
                          morphCol(p.reg, a), 12);
  }
  if (!s.ns.empty()) {
    const Neuron &wn = s.ns[std::min<size_t>(s.w, s.ns.size() - 1)];
    c.dl->AddCircle(c.P(cx + wn.pred[0] * S, cy - wn.pred[1] * S), 7.f * c.sc,
                    C_WHITE, 24, 1.5f);
  }
  endCv(c);
}

static void drawErr(const Sim &s) {
  Cv c = beginCv(100);
  const double dx = .64;
  const size_t N = s.errH.size();
  const double o = (1000.0 - double(N)) * dx;
  for (size_t i = 0; i < N;) {
    size_t j = i;
    while (j < N && s.errH[j].reg == s.errH[i].reg)
      j++;
    c.dl->AddRectFilled(c.P(o + i * dx, 0), c.P(o + j * dx, 100),
                        morphCol(s.errH[i].reg, 51));
    i = j;
  }
  const double ty = 98 - std::min(1., s.params.thr / .4) * 94;
  for (double x = 0; x < 640; x += 8)
    c.dl->AddLine(c.P(x, ty), c.P(x + 4, ty), C_AMBER);
  std::vector<ImVec2> pts;
  pts.reserve(N);
  for (size_t i = 0; i < N; i++)
    pts.push_back(c.P(o + i * dx, 98 - std::min(1., s.errH[i].e / .4) * 94));
  if (pts.size() > 1)
    c.dl->AddPolyline(pts.data(), (int)pts.size(), C_WHITE, ImDrawFlags_None,
                      1.2f);
  endCv(c);
}

static void drawNeurons(const Sim &s) {
  Cv c = beginCv(80);
  const int N = (int)s.ns.size();
  const double bw = std::min(28., 620. / std::max(1, N));
  for (int i = 0; i < N; i++) {
    const Neuron &n = s.ns[i];
    const double h = std::max(2., (1 - std::min(1., n.eS / .3)) * 56),
                 x = 10 + i * (bw + 2);
    const int d = domReg(n);
    c.dl->AddRectFilled(c.P(x, 62 - h), c.P(x + bw, 62),
                        d < 0 ? IM_COL32(0x55, 0x55, 0x66, 255) : morphCol(d));
    if (i == s.w)
      c.dl->AddRect(c.P(x, 62 - h), c.P(x + bw, 62), C_WHITE, 0.f, 0, 1.5f);
    c.text(x, 75, C_MUTED, fmt("%d", n.id));
  }
  endCv(c);
}

static bool toggleBtn(const char *label, bool on) {
  if (on)
    ImGui::PushStyleColor(ImGuiCol_Text,
                          ImGui::ColorConvertU32ToFloat4(C_AMBER));
  const bool r = ImGui::Button(label);
  if (on)
    ImGui::PopStyleColor();
  return r;
}
static void statRow(const char *k, const std::string &v) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextDisabled("%s", k);
  ImGui::TableSetColumnIndex(1);
  ImGui::TextUnformatted(v.c_str());
}
static void dot(int reg, int id) {
  ImGui::PushID(id);
  ImGui::ColorButton("##d", morphVec(reg),
                     ImGuiColorEditFlags_NoTooltip |
                         ImGuiColorEditFlags_NoDragDrop |
                         ImGuiColorEditFlags_NoInputs,
                     ImVec2(9, 9));
  ImGui::PopID();
  ImGui::SameLine(0, 4);
}

struct PadUI {
  double kx = .5, ky = (.4 - .25) / .35, ky0 = (.4 - .25) / .35;
};
static void drivePad(Sim &s, PadUI &u) {
  const ImVec2 sz(200, 200);
  ImGui::SetCursorPosX((ImGui::GetWindowWidth() - sz.x) * .5f);
  const ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton("pad", sz);
  if (ImGui::IsItemActivated())
    u.ky0 = u.ky;
  if (ImGui::IsItemActive()) {
    const ImVec2 m = ImGui::GetIO().MousePos;
    u.kx = clampd((m.x - p.x) / sz.x, 0, 1);
    u.ky = clampd(1 - (m.y - p.y) / sz.y, 0, 1);
    if (!s.pad.man && std::fabs(u.ky - u.ky0) > .03)
      s.pad.man = true;
  }
  if (ImGui::IsItemDeactivated())
    u.kx = .5;
  s.pad.vx = (u.kx - .5) * 2;
  s.pad.h = .25 + .35 * u.ky;

  ImDrawList *dl = ImGui::GetWindowDrawList();
  dl->AddRectFilled(p, ImVec2(p.x + sz.x, p.y + sz.y), IM_COL32(0, 0, 0, 255),
                    10.f);
  dl->AddRect(p, ImVec2(p.x + sz.x, p.y + sz.y), C_BORDER, 10.f);
  dl->AddLine(ImVec2(p.x + sz.x / 2, p.y), ImVec2(p.x + sz.x / 2, p.y + sz.y),
              C_GRID);
  dl->AddLine(ImVec2(p.x, p.y + sz.y / 2), ImVec2(p.x + sz.x, p.y + sz.y / 2),
              C_GRID);
  dl->AddText(ImVec2(p.x + 6, p.y + sz.y / 2 - 6), C_MUTED, "< left");
  dl->AddText(ImVec2(p.x + sz.x - 52, p.y + sz.y / 2 - 6), C_MUTED, "right >");
  dl->AddText(ImVec2(p.x + sz.x / 2 - 10, p.y + 4), C_MUTED, "tall");
  dl->AddText(ImVec2(p.x + sz.x / 2 - 12, p.y + sz.y - 16), C_MUTED, "short");
  dl->AddCircleFilled(
      ImVec2(p.x + float(u.kx) * sz.x, p.y + float(1 - u.ky) * sz.y), 14.f,
      C_AMBER, 24);
}

int main() {
  if (!glfwInit())
    return 1;
#if defined(__APPLE__)
  const char *glsl = "#version 150";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
  const char *glsl = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif
  GLFWwindow *win = glfwCreateWindow(
      1440, 900, "NPRN - wheeled biped, direct readout", nullptr, nullptr);
  if (!win) {
    glfwTerminate();
    return 1;
  }
  glfwMakeContextCurrent(win);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.IniFilename = nullptr;
  ImGui::StyleColorsDark();
  ImGuiStyle &st = ImGui::GetStyle();
  st.WindowRounding = 8.f;
  st.FrameRounding = 5.f;
  st.Colors[ImGuiCol_WindowBg] = ImVec4(0, 0, 0, .96f);
  ImGui_ImplGlfw_InitForOpenGL(win, true);
  ImGui_ImplOpenGL3_Init(glsl);

  Sim sim;
  PadUI padUI;
  bool paused = false;
  std::vector<char> rpBuf(1, 0);

  while (!glfwWindowShouldClose(win)) {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    if (!paused)
      for (int k = 0; k < sim.params.spf; k++)
        sim.tick();

    const ImVec2 ds = io.DisplaySize;
    const float ctrlW = 380.f, diagW = 300.f, gap = 10.f;

    ImGui::SetNextWindowPos(ImVec2(gap, gap), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(
        ImVec2(ds.x - ctrlW - diagW - 4 * gap, ds.y - 2 * gap),
        ImGuiCond_FirstUseEver);
    ImGui::Begin("Viewport");
    drawRobot(sim);
    drawPhase(sim);
    drawErr(sim);
    drawNeurons(sim);

    if (ImGui::Button("Reset"))
      sim.init();
    ImGui::SameLine();
    if (toggleBtn(paused ? "Resume###pause" : "Pause###pause", paused))
      paused = !paused;
    ImGui::SameLine();
    if (ImGui::Button("Force next morphology"))
      sim.forceNextMorph();
    ImGui::SameLine();
    if (toggleBtn(sim.frozen ? "Unfreeze structure###frz"
                             : "Freeze structure###frz",
                  sim.frozen)) {
      sim.frozen = !sim.frozen;
      sim.log("boot", sim.frozen ? "structure frozen" : "structure unfrozen");
    }
    if (ImGui::Button("Push robot"))
      sim.push();
    ImGui::SameLine();
    if (toggleBtn(sim.adapt ? "PD fallback: ON###ad" : "PD fallback: OFF###ad",
                  sim.adapt))
      sim.adapt = !sim.adapt;
    ImGui::SameLine();
    if (toggleBtn(sim.netOn ? "Network: ON###net" : "Network: OFF###net",
                  sim.netOn)) {
      sim.netOn = !sim.netOn;
      if (!sim.netOn && sim.adapt)
        sim.adapt = false;

      sim.log("boot", sim.netOn ? "network resumed" : "NETWORK STOPPED");
    }
    ImGui::SameLine();
    if (toggleBtn(sim.rock ? "Rocking: ON###rock" : "Rocking: OFF###rock",
                  sim.rock))
      sim.rock = !sim.rock;

    ImGui::SeparatorText("Morphology visits (reuse test)");
    if (ImGui::BeginTable("visits", 7,
                          ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                              ImGuiTableFlags_SizingStretchProp,
                          ImVec2(0, 0))) {
      ImGui::TableSetupScrollFreeze(0, 1);
      ImGui::TableSetupColumn("#");
      ImGui::TableSetupColumn("morph");
      ImGui::TableSetupColumn("adapt err (first 150)");
      ImGui::TableSetupColumn("steady err (last 300)");
      ImGui::TableSetupColumn("spawns");
      ImGui::TableSetupColumn("dominant neuron");
      ImGui::TableSetupColumn("verdict");
      ImGui::TableHeadersRow();
      for (const auto &v : sim.visits) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%d", v.n);
        ImGui::TableSetColumnIndex(1);
        dot(v.reg, v.n);
        ImGui::Text("%d", v.reg);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%.3f", v.adapt);
        ImGui::TableSetColumnIndex(3);
        ImGui::Text("%.3f", v.steady);
        ImGui::TableSetColumnIndex(4);
        ImGui::Text("%d", v.spawns);
        ImGui::TableSetColumnIndex(5);
        ImGui::Text("%d", v.dom);
        ImGui::TableSetColumnIndex(6);
        ImGui::TextUnformatted(v.verdict.c_str());
      }
      if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 1.f)
        ImGui::SetScrollHereY(1.f);
      ImGui::EndTable();
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(ds.x - ctrlW - diagW - 2 * gap, gap),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(ctrlW, ds.y - 2 * gap),
                             ImGuiCond_FirstUseEver);
    ImGui::Begin("Controls");
    ImGui::SeparatorText("Drive pad");
    drivePad(sim, padUI);
    if (toggleBtn(sim.pad.man ? "Manual height: ON###man"
                              : "Manual height: OFF###man",
                  sim.pad.man))
      sim.pad.man = !sim.pad.man;

    ImGui::SeparatorText("Morphologies (height / mass)");
    const char *names[4] = {"0 nominal", "1 tall, light", "2 short, heavy",
                            "3 tall, heavy"};
    for (int i = 0; i < 4; i++) {
      dot(i, 100 + i);
      ImGui::Text("%-15s %.2f m / %.1f kg", names[i], R[i].h, R[i].m);
    }

    ImGui::SeparatorText("Parameters");
    float thr = (float)sim.params.thr, mu = (float)sim.params.mu;
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::TextUnformatted("spawn / learn threshold");
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::SliderFloat("##thr", &thr, .02f, .3f, "%.3f"))
      sim.params.thr = thr;
    ImGui::TextUnformatted("morphology length (steps)");
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::SliderInt("##len", &sim.params.len, 300, 3000, "%d");
    ImGui::TextUnformatted("steps per frame");
    ImGui::SetNextItemWidth(-FLT_MIN);
    ImGui::SliderInt("##spf", &sim.params.spf, 1, 8);
    ImGui::TextUnformatted("control learning rate");
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::SliderFloat("##mu", &mu, .02f, 1.5f, "%.2f"))
      sim.params.mu = mu;

    ImGui::SeparatorText("Status");
    if (ImGui::BeginTable("status", 2,
                          ImGuiTableFlags_SizingStretchProp |
                              ImGuiTableFlags_RowBg)) {
      ImGui::TableSetupColumn("k", ImGuiTableColumnFlags_WidthStretch, 1.f);
      ImGui::TableSetupColumn("v", ImGuiTableColumnFlags_WidthStretch, 1.f);
      const Robot &r = sim.rob;
      statRow("neurons", fmt("%d", (int)sim.ns.size()));
      statRow("step / morph now", fmt("%d / %d", sim.T, SEQ[sim.schedI]));
      statRow("height / mass", fmt("%.2f m / %.1f kg", r.h, r.m));
      std::string g = "-";
      if (!sim.ns.empty()) {
        const Neuron &wn = sim.ns[std::min<size_t>(sim.w, sim.ns.size() - 1)];
        g = fmt("%.1f , %.1f , %.1f", wn.K[0], wn.K[1], wn.K[2]);
      }
      statRow("winner K / falls", g + fmt(" / %d", sim.falls));
      statRow("Control: ",
              sim.netOn ? (sim.adapt ? "network + PD fallback" : "network only")
                        : (sim.adapt ? "PD fallback only" : "NOTHING"));
      statRow("net / safety cmd", fmt("%.1f / %.1f", r.lu, r.ls));
      statRow("spawns / prunes / merges",
              fmt("%d / %d / %d", sim.spawnC, sim.pruneC, sim.mergeC));
      statRow("net error (EMA)", fmt("%.3f", sim.errEma));
      ImGui::EndTable();
    }
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(ds.x - diagW - gap, gap),
                            ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(diagW, ds.y - 2 * gap),
                             ImGuiCond_FirstUseEver);
    ImGui::Begin("Logs");
    ImGui::BeginChild("log", ImVec2(0, 0), ImGuiChildFlags_Borders);
    int shown = 0;
    for (const auto &e : sim.events) {
      if (shown++ >= 12)
        break;
      ImGui::TextDisabled("%d %s %s", e.t, e.k.c_str(), e.m.c_str());
    }
    ImGui::EndChild();
    ImGui::End();

    ImGui::Render();
    int fw, fh;
    glfwGetFramebufferSize(win, &fw, &fh);
    glViewport(0, 0, fw, fh);
    glClearColor(.83f, .86f, .91f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(win);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(win);
  glfwTerminate();
  return 0;
}
