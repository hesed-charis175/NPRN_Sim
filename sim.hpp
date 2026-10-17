#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace nprn {

struct Morph {
  double h, m;
};
inline constexpr Morph R[4] = {{.40, 4}, {.60, 2.5}, {.25, 7}, {.60, 7}};
inline constexpr int SEQ[10] = {0, 1, 2, 0, 1, 3, 2, 0, 3, 1};
inline constexpr int SEQ_N = 10;

inline constexpr int PROB = 400;     // probation ticks
inline constexpr int MAXN = 40;      // max neurons
inline constexpr int SUB = 5;        // physics substeps per tick
inline constexpr double UTAU = 8000; // utilisation time constant
inline constexpr double MERGE = .06; // merge distance on P
inline constexpr double DT = .01, G0 = 9.81;
inline constexpr double KT = 18, KW = 2.2;    // fallback PD gains
inline constexpr double MW = 1, IB = .01;     // wheel mass, body inertia factor
inline constexpr double TMAX = 2.5;           // torque limit
inline constexpr double WALL = 2.6, WR = .08; // wall, wheel radius

inline double clampd(double v, double a, double b) {
  return std::max(a, std::min(b, v));
}
inline double sgn(double v) { return (v > 0) - (v < 0); }

inline std::string fmt(const char *f, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

struct Params {
  double thr = .1;
  int len = 1200;
  int spf = 2;
  double mu = .3;
};
struct Pad {
  double vx = 0, h = .4;
  bool man = false;
};
struct Event {
  int t;
  std::string k, m;
};

struct Neuron {
  int id = 0;
  double A[4], B[4], P[4] = {0, 0, 0, 0};
  double h[2], pred[2] = {0, 0};
  double eS = 1;
  int age = 0;
  double util = .05;
  int wins[4] = {0, 0, 0, 0};
  int prob = PROB;
  double K[3] = {0, 0, 0};

  void learn(const double e[2], double lr) { // NLMS on P
    const double k = lr / (.02 + h[0] * h[0] + h[1] * h[1]);
    P[0] += k * e[0] * h[0];
    P[1] += k * e[0] * h[1];
    P[2] += k * e[1] * h[0];
    P[3] += k * e[1] * h[1];
  }
  void step(const double x[2]) {
    const double h0 = h[0], h1 = h[1];
    h[0] = std::tanh(A[0] * h0 + A[1] * h1 + B[0] * x[0] + B[1] * x[1]);
    h[1] = std::tanh(A[2] * h0 + A[3] * h1 + B[2] * x[0] + B[3] * x[1]);
    pred[0] = P[0] * h[0] + P[1] * h[1];
    pred[1] = P[2] * h[0] + P[3] * h[1];
  }
};

struct Visit {
  int i = 0, reg = 0;
  std::vector<double> errs;
  int spawns = 0;
  std::map<int, int> wins;
};
struct VisitRec {
  int n, reg;
  double adapt, steady;
  int spawns, dom;
  std::string verdict;
};
struct Robot {
  double th = 0, om = 0, s = 0, v = 0, h = .4, m = 4, wa = 0, t = 0, tr = 0;
  double lu = 0, ls = 0, hd = 0, F = 0;
  int fall = 0;
  bool sat = false;
};
struct TrailPt {
  double x[2];
  int reg;
};
struct ErrPt {
  double e;
  int reg;
};

class Sim {
public:
  Params params;
  Pad pad;
  bool frozen = false, adapt = true, netOn = true, rock = false;

  std::vector<Neuron> ns;
  int T = 0, schedI = 0, schedLeft = 0;
  int spawnC = 0, pruneC = 0, mergeC = 0, falls = 0;
  int nov = 0, w = 0, nid = 0;
  long long lastSpawn = -1000000000LL;
  double errEma = .3;
  Robot rob;
  Visit cv;
  std::vector<VisitRec> visits;
  std::deque<Event> events;
  std::deque<ErrPt> errH;
  std::deque<TrailPt> trail;

  explicit Sim(unsigned seed = std::random_device{}()) : rng_(seed) { init(); }

  void init() {
    ns.clear();
    T = 0;
    schedI = 0;
    schedLeft = params.len;
    spawnC = pruneC = mergeC = 0;
    events.clear();
    nov = 0;
    lastSpawn = -1000000000LL;
    w = 0;
    visits.clear();
    errH.clear();
    trail.clear();
    nid = 0;
    errEma = .3;
    falls = 0;
    rob = Robot{};
    rob.h = R[0].h;
    rob.m = R[0].m;
    cv = newVisit();
  }

  void log(const std::string &k, const std::string &m) {
    events.push_front({T, k, m});
    if (events.size() > 40)
      events.pop_back();
  }

  void tick() {
    if (schedLeft <= 0) {
      closeVisit();
      schedI = (schedI + 1) % SEQ_N;
      schedLeft = params.len;
      cv = newVisit();
    }
    schedLeft--;
    const int reg = SEQ[schedI];
    const Morph &tg = R[reg];

    rob.hd = ((pad.man ? pad.h : tg.h) - rob.h) * (pad.man ? .02 : .004) /
             (SUB * DT);
    rob.m += (tg.m - rob.m) * .004;
    const double wl = std::fabs(rob.s) > 2.2
                          ? -sgn(rob.s) * (std::fabs(rob.s) - 2.2) * 1.5
                          : 0;
    rob.tr =
        clampd(.2 * (pad.vx * 1.5 + (rock ? .3 * std::sin(.5 * rob.t) : 0) +
                     (pad.vx == 0 ? -.5 * rob.s : 0) + wl - rob.v),
               -.3, .3);

    const bool haveC = w >= 0 && w < (int)ns.size();
    double f[3] = {0, 0, 0};
    if (haveC) {
      f[0] = ns[w].h[0];
      f[1] = ns[w].h[1];
      f[2] = .2;
    }
    const double u = (haveC && netOn) ? ns[w].K[0] * f[0] + ns[w].K[1] * f[1] +
                                            ns[w].K[2] * f[2]
                                      : 0;

    physics(u);

    if (netOn && haveC && rob.fall <= 0 && !rob.sat &&
        (ns[w].prob > 0 || ns[w].eS < params.thr * 2)) {
      const double y = (rob.th - rob.tr) + .15 * rob.om;
      const double k =
          params.mu * y / (.02 + f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
      for (int i = 0; i < 3; i++)
        ns[w].K[i] = clampd(ns[w].K[i] + k * f[i], -15, 15);
    }

    double x[2] = {(rob.th - rob.tr) * 5 + .01 * randn(),
                   rob.om * .5 + .01 * randn()};
    T++;
    if (!netOn || rob.fall > 0) {
      pushTrail(x, reg);
      return;
    }
    if (ns.empty())
      spawn(x, "bootstrap");

    const int N = (int)ns.size();
    std::vector<std::array<double, 2>> eV(N);
    std::vector<double> eN(N);
    for (int i = 0; i < N; i++) {
      eV[i] = {x[0] - ns[i].pred[0], x[1] - ns[i].pred[1]};
      eN[i] = std::hypot(eV[i][0], eV[i][1]);
    }
    const double netErr = eN[std::min(w, N - 1)];
    errEma = .99 * errEma + .01 * netErr;
    for (int i = 0; i < N; i++)
      ns[i].eS = .8 * ns[i].eS + .2 * eN[i];
    int b = 0;
    for (int i = 0; i < N; i++)
      if (ns[i].eS < ns[b].eS)
        b = i;

    for (int i = 0; i < N; i++) {
      Neuron &n = ns[i];
      if (n.prob > 0 || (i == b && n.eS < params.thr * 1.5))
        n.learn(eV[i].data(),
                .4 * std::min(2., std::max(.2, n.eS / params.thr)));
      if (n.prob > 0)
        n.prob--;
      n.age++;
      n.util = n.util * (1 - 1 / UTAU) + (i == b ? 1 / UTAU : 0);
    }
    ns[b].wins[reg]++;
    cv.wins[ns[b].id]++;
    for (auto &n : ns)
      n.step(x);

    cv.errs.push_back(netErr);
    errH.push_back({netErr, reg});
    if (errH.size() > 1000)
      errH.pop_front();
    pushTrail(x, reg);

    if (!frozen) {
      nov = ns[b].eS > params.thr ? nov + 1 : 0;
      if (nov > 15 && T - lastSpawn > PROB && (int)ns.size() < MAXN) {
        spawn(x, fmt("novel input, best err %.3f (morph %d)", ns[b].eS, reg));
        nov = 0;
        lastSpawn = T;
        cv.spawns++;
      }
      if (T % 200 == 0)
        pruneAndMerge();
    }
    w = 0;
    for (int i = 0; i < (int)ns.size(); i++)
      if (ns[i].eS < ns[w].eS)
        w = i;
  }

  void push() {
    rob.om += (uni() < .5 ? -1 : 1) * 1.2 / (rob.m * rob.h);
    log("push", "impulse");
  }
  void forceNextMorph() { schedLeft = 0; }

  std::string report() const {
    std::ostringstream L;
    L << fmt("NPRN WHEELED-BIPED REPORT step=%d N=%d "
             "spawns/prunes/merges=%d/%d/%d falls=%d frozen=%s safety=%s\n",
             T, (int)ns.size(), spawnC, pruneC, mergeC, falls,
             frozen ? "true" : "false", adapt ? "true" : "false");
    L << fmt("params={\"thr\":%g,\"len\":%d,\"spf\":%d,\"mu\":%g}\n",
             params.thr, params.len, params.spf, params.mu);
    L << "morphs (h m / kg): ";
    for (int i = 0; i < 4; i++)
      L << i << ":" << R[i].h << "/" << R[i].m << "  ";
    L << "; sequence ";
    for (int i = 0; i < SEQ_N; i++)
      L << SEQ[i] << (i + 1 < SEQ_N ? "," : "");
    L << "\n\nVISITS (#,morph,adapt,steady,spawns,dominant neuron,verdict):\n";
    for (auto &v : visits)
      L << fmt("  %d,%d,%.3f,%.3f,%d,%d,%s\n", v.n, v.reg, v.adapt, v.steady,
               v.spawns, v.dom, v.verdict.c_str());
    L << "\nNEURONS (id, age, usage, |P|, K, wins per morph 0-3):\n";
    for (auto &n : ns)
      L << fmt("  %d, %d, %.3f, %.2f, %.1f/%.1f/%.1f, %d/%d/%d/%d\n", n.id,
               n.age, n.util,
               std::sqrt(n.P[0] * n.P[0] + n.P[1] * n.P[1] + n.P[2] * n.P[2] +
                         n.P[3] * n.P[3]),
               n.K[0], n.K[1], n.K[2], n.wins[0], n.wins[1], n.wins[2],
               n.wins[3]);
    L << "\nEVENTS (newest first):\n";
    for (auto &e : events)
      L << "  " << e.t << " " << e.k << " " << e.m << "\n";
    return L.str();
  }

private:
  std::mt19937 rng_;
  double randn() { return std::normal_distribution<double>(0, 1)(rng_); }
  double uni() { return std::uniform_real_distribution<double>(0, 1)(rng_); }

  Neuron makeNeuron(const double x[2]) {
    Neuron n;
    n.id = nid++;
    n.A[0] = .25 + .03 * randn();
    n.A[1] = .03 * randn();
    n.A[2] = .03 * randn();
    n.A[3] = .25 + .03 * randn();
    n.B[0] = .6 + .03 * randn();
    n.B[1] = .03 * randn();
    n.B[2] = .03 * randn();
    n.B[3] = .6 + .03 * randn();
    n.h[0] = std::tanh(.6 * x[0]);
    n.h[1] = std::tanh(.6 * x[1]);
    return n;
  }

  void spawn(const double x[2], const std::string &why) {
    Neuron nn = makeNeuron(x);
    if (w >= 0 &&
        w < (int)ns.size()) // warm-start controller from current winner
      for (int i = 0; i < 3; i++)
        nn.K[i] = ns[w].K[i];
    ns.push_back(nn);
    spawnC++;
    log("spawn", why);
  }

  void pushTrail(const double x[2], int reg) {
    trail.push_back({{x[0], x[1]}, reg});
    if (trail.size() > 150)
      trail.pop_front();
  }

  Visit newVisit() const {
    Visit v;
    v.i = schedI;
    v.reg = SEQ[schedI];
    return v;
  }

  static double mean(const std::vector<double> &a, size_t from, size_t to) {
    if (to <= from)
      return std::nan("");
    double s = 0;
    for (size_t i = from; i < to; i++)
      s += a[i];
    return s / double(to - from);
  }

  void closeVisit() {
    int dom = -1, best = -1;
    for (auto &kv : cv.wins)
      if (kv.second > best) {
        best = kv.second;
        dom = kv.first;
      }
    bool anyPrior = false, priorSame = false, sharedOther = false;
    for (auto &p : visits) {
      if (p.reg == cv.reg) {
        anyPrior = true;
        if (p.dom == dom)
          priorSame = true;
      } else if (p.dom == dom)
        sharedOther = true;
    }
    std::string verdict = "first visit";
    if (anyPrior)
      verdict = priorSame ? "REUSED" : "new neuron";
    if (sharedOther)
      verdict += " (shared w/ other morph)";
    const size_t n = cv.errs.size();
    visits.push_back({(int)visits.size() + 1, cv.reg,
                      mean(cv.errs, 0, std::min<size_t>(150, n)),
                      mean(cv.errs, n > 300 ? n - 300 : 0, n), cv.spawns, dom,
                      verdict});
  }

  void pruneAndMerge() {
    // pruning old and unused
    for (int i = (int)ns.size() - 1; i >= 0 && ns.size() > 1; i--) {
      if (ns[i].age > 3000 && ns[i].util < .0015) {
        pruneC++;
        log("prune", fmt("id %d usage %.4f", ns[i].id, ns[i].util));
        ns.erase(ns.begin() + i);
      }
    }
    // merging neurons whose predictors converged...
    for (int i = 0; i < (int)ns.size(); i++)
      for (int j = i + 1; j < (int)ns.size(); j++) {
        Neuron &a = ns[i], &o = ns[j];
        if (a.age < 2 * PROB || o.age < 2 * PROB)
          continue;
        const double d = std::sqrt((a.P[0] - o.P[0]) * (a.P[0] - o.P[0]) +
                                   (a.P[1] - o.P[1]) * (a.P[1] - o.P[1]) +
                                   (a.P[2] - o.P[2]) * (a.P[2] - o.P[2]) +
                                   (a.P[3] - o.P[3]) * (a.P[3] - o.P[3]));
        if (d < MERGE) {
          for (int k = 0; k < 4; k++) {
            a.P[k] = (a.P[k] + o.P[k]) / 2;
            a.wins[k] += o.wins[k];
          }
          for (int k = 0; k < 3; k++)
            a.K[k] = (a.K[k] + o.K[k]) / 2;
          const int oid = o.id, aid = a.id;
          ns.erase(ns.begin() + j);
          j--;
          mergeC++;
          log("merge", fmt("%d absorbed %d d=%.3f", aid, oid, d));
        }
      }
  }

  void physics(double u) {
    Robot &r = rob;
    if (r.fall > 0) {
      r.fall--;
      r.lu = r.ls = 0;
      for (int k = 0; k < SUB; k++) {
        r.v *= .99;
        r.s += r.v * DT;
        r.wa += r.v * DT / WR;
      }
      if (r.fall == 0) {
        r.th = r.om = r.v = 0;
        log("stand", "self-righted");
      }
      return;
    }
    for (int k = 0; k < SUB; k++) {
      r.t += DT;
      r.h += r.hd * DT;
      r.F += -r.F / .2 * DT + 3.5 * std::sqrt(DT) * randn();
      const double ad = u + (adapt ? .4 : 0) * (KT * (r.th - r.tr) + KW * r.om);
      r.lu = u;
      r.ls = ad - u;
      const double tau = clampd(ad, -TMAX, TMAX);
      r.sat = std::fabs(ad) >= TMAX;
      const double m = r.m, l = r.h, c = std::cos(r.th), s = std::sin(r.th);
      const double Mt = m + 1.5 * MW, J = m * l * l + IB * m;
      const double r1 =
          tau / WR + r.F + m * l * s * r.om * r.om - 2 * m * r.hd * c * r.om;
      const double r2 =
          -tau + r.F * l * c + m * G0 * l * s - 2 * m * l * r.hd * r.om;
      const double det = Mt * J - m * m * l * l * c * c;
      const double xa = (J * r1 - m * l * c * r2) / det;
      const double ta = (Mt * r2 - m * l * c * r1) / det;
      r.om += ta * DT;
      r.th += r.om * DT;
      r.v += xa * DT;
      r.s += r.v * DT;
      r.wa += r.v * DT / WR;
      if (std::fabs(r.s) > WALL) {
        r.s = sgn(r.s) * WALL;
        r.v = 0;
      }
      if (std::fabs(r.th) > 1.1) {
        falls++;
        log("FALL", fmt("morph %d h=%.2f m=%.1f", SEQ[schedI], r.h, r.m));
        r.th = sgn(r.th) * 1.25;
        r.om = 0;
        r.fall = 20;
        break;
      }
    }
  }
};

} // namespace nprn
