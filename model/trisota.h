// ============================================================================
// trisota.h — COMBINED-SOTA RL BYPASS MODEL
// ----------------------------------------------------------------------------
// A single model that fuses the THREE implemented SOTA techniques, using ONLY
// their features / algorithms / parameters (no DASCA, no our-own features), and
// arbitrates the final bypass with a reinforcement-learning Q-table whose state
// is built solely from the SOTA signals.
//
//   DENSITY    -> per-PC live_score (reuse FREQUENCY, 0..3) + WQ-pressure ladder
//                 (downward / writeback bypass).
//   NOVELLA    -> per-PC RPT (reuse counter, ==0 => no reuse) + per-set
//                 dead-probability + access-rate (AR) congestion gate + miss-rate
//                 band  (upward / fill bypass).
//   MOCKINGJAY -> per-PC RDP (reuse-DISTANCE, temporal-difference) + per-line ETR;
//                 bypass a fill whose predicted reuse is farther than every
//                 resident line  (upward / fill bypass).
//
// RL layer: a Q-table indexed by {live_score, rpt==0, mj_bypass, ar_congested,
// wq_band, mr_band}. Q>=0 => allow bypass. Trained by reuse feedback (a bypassed
// line that is later re-accessed => negative reward; a confirmed-dead bypass =>
// small positive). This is the "combined strength of all SOTA, decided by RL".
//
// All parameters are taken verbatim from the SOTA sources:
//   Density   : liveness bins 21/51/71 %, WQ bands 0.25/0.5/0.75, write_th 0.75
//   Novella   : mrthl=0.1 mrthh=0.8 dpth=0.3 ARth=70th-pctile RPT 3-bit
//   Mockingjay: HISTORY=8 GRANULARITY=8 INF_RD=LLC_WAY*8-1 MAX_RD=INF_RD-22
//               TEMP_DIFFERENCE=1/16
// ============================================================================
#ifndef TRISOTA_H
#define TRISOTA_H
#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>

namespace trisota {

// ---- common PC signature (CRC, 16-bit) ----
static const uint32_t SIG_N = 1024;   // NANO-max: 2048->1024
static inline uint32_t pcsig(uint64_t pc) {
    uint64_t h = pc;
    for (int i = 0; i < 3; i++) h = (h & 1) ? ((h >> 1) ^ 3988292384ULL) : (h >> 1);
    return (uint32_t)(h & (SIG_N - 1));   // COMPACT: mask to SIG_N
}

// =====================================================================
// DENSITY — per-PC live_score (reuse frequency, 0..3)
// hits/evicts binned: >=71%->3, >=51%->2, >=21%->1, else 0   (Density exact)
// =====================================================================
// COMPACT: hits/evicts int32->uint16. They feed only a ratio (ls_bin); on
// saturation both are halved, preserving the ratio.
struct LSEntry { uint16_t hits = 0, evicts = 0; uint8_t score = 0; };
static LSEntry ls_tab[SIG_N];
static inline uint8_t ls_bin(int32_t h, int32_t e) {
    if (e == 0) return 0;
    double p = 100.0 * (double)h / (double)e;
    if (p >= 71.0) return 3; if (p >= 51.0) return 2; if (p >= 21.0) return 1; return 0;
}
static inline void ls_recall(uint32_t s) {
    if (ls_tab[s].hits >= 0xFFFF) { ls_tab[s].hits >>= 1; ls_tab[s].evicts >>= 1; }
    ls_tab[s].hits++;   ls_tab[s].score = ls_bin(ls_tab[s].hits, ls_tab[s].evicts);
}
static inline void ls_evict (uint32_t s) {
    if (ls_tab[s].evicts >= 0xFFFF) { ls_tab[s].hits >>= 1; ls_tab[s].evicts >>= 1; }
    ls_tab[s].evicts++; ls_tab[s].score = ls_bin(ls_tab[s].hits, ls_tab[s].evicts);
}
static inline uint8_t ls_score(uint32_t s) { return ls_tab[s].score; }

// =====================================================================
// NOVELLA — per-PC RPT (3-bit reuse counter) + per-set dead-probability
// rpt[pc]==0 => no reuse observed => bypass-eligible.
// =====================================================================
static uint8_t rpt[SIG_N];                 // 0..7
static inline void rpt_reuse(uint32_t s) { if (rpt[s] < 7) rpt[s]++; }
static inline void rpt_new(uint32_t s)   { rpt[s] = 0; }
static inline bool rpt_no_reuse(uint32_t s) { return rpt[s] == 0; }
// per-set dead-probability  (past_dead/past_fill >= dpth=0.3)
static const uint32_t NSET = LLC_SET;
// COMPACT: per-set statistical state is SAMPLED to a fixed MJ_SETS pool so it
// does not scale with LLC size. Real sets fold into MJ_SETS slots; the global
// PC-sig predictors (rdp/ls_tab/rpt) stay full so prediction quality holds.
static const uint32_t MJ_SETS = 256;                // NANO: 512->256
static inline uint32_t mjset(uint32_t set) {         // fold real set -> sampled slot
    return (MJ_SETS >= NSET) ? set : (set & (MJ_SETS - 1));
}
static uint32_t past_fill[MJ_SETS];
static uint32_t past_dead[MJ_SETS];
static inline void dp_fill(uint32_t set) { uint32_t s=mjset(set); if (++past_fill[s] >= (1u<<20)) { past_fill[s]>>=1; past_dead[s]>>=1; } }
static inline void dp_dead(uint32_t set) { past_dead[mjset(set)]++; }
static inline bool dp_high(uint32_t set) { uint32_t s=mjset(set); if (past_fill[s] < 32) return false; return (double)past_dead[s]/(double)past_fill[s] >= 0.3; }

// access-rate (AR) congestion gate + miss-rate band (Novella exact: 0.1, 0.8)
static const int AR_HIST = 16;
static uint64_t ar_last_cycle = 0, ar_last_acc = 0, ar_acc = 0;
static double   ar_hist[AR_HIST]; static int ar_head = 0;
static double   ar_cur = 0.0, ar_th = 0.0;
static const uint64_t AR_WIN = 100000;
static inline void ar_tick(uint64_t cycle) {
    ar_acc++;
    if (cycle - ar_last_cycle < AR_WIN) return;
    double rate = (cycle > ar_last_cycle) ? (double)(ar_acc - ar_last_acc) / (double)(cycle - ar_last_cycle) : 0.0;
    ar_hist[ar_head] = rate; ar_head = (ar_head + 1) % AR_HIST; ar_cur = rate;
    double tmp[AR_HIST]; int n = 0;
    for (int i = 0; i < AR_HIST; i++) if (ar_hist[i] > 0) tmp[n++] = ar_hist[i];
    if (n) { std::sort(tmp, tmp + n); ar_th = tmp[(int)(0.7 * (n - 1))]; }
    ar_last_acc = ar_acc; ar_last_cycle = cycle;
}
static inline bool ar_congested() { return ar_th <= 0.0 ? true : ar_cur >= ar_th; }
// running LLC miss-rate for the band
static uint64_t llc_acc = 0, llc_mis = 0;
static inline void mr_acc(bool miss) { llc_acc++; if (miss) llc_mis++; }
static inline double mr() { return llc_acc ? (double)llc_mis / (double)llc_acc : 0.0; }
static inline bool mr_band() { double m = mr(); return m >= 0.1 && m <= 0.8; }

// =====================================================================
// MOCKINGJAY — per-PC RDP (reuse-distance, temporal-difference) + per-line ETR
// HISTORY=8 GRANULARITY=8 INF_RD=LLC_WAY*8-1 MAX_RD=INF_RD-22 TEMP_DIFFERENCE=1/16
// Faithful-core port: RDP trained by observed reuse distances measured in LLC
// accesses to the set; ETR aged per GRANULARITY accesses; victim = max|ETR|.
// =====================================================================
static const int MJ_HISTORY = 8;
static const int MJ_GRAN    = 8;
static const int MJ_INF_RD  = LLC_WAY * MJ_HISTORY - 1;       // 127 @ 16-way
static const int MJ_MAX_RD  = MJ_INF_RD - 22;                 // 105
static const int MJ_INF_ETR = (LLC_WAY * MJ_HISTORY) / MJ_GRAN - 1;
static int16_t rdp[SIG_N];                    // predicted reuse distance; -1 unseen (COMPACT: int->int16, range [-1,127])
static int8_t  etr[MJ_SETS][LLC_WAY];   // COMPACT: range [-15,+15] fits int8; sampled sets
static uint8_t etr_clock[MJ_SETS];      // COMPACT: 0..MJ_GRAN(8) fits uint8
static uint16_t set_acc[MJ_SETS];             // per-set access timestamp (COMPACT: u16, sampled)
static uint16_t line_ts[MJ_SETS][LLC_WAY];    // last-access timestamp (COMPACT: u16, sampled)
static uint16_t line_sig[MJ_SETS][LLC_WAY];   // pc sig that filled the line (COMPACT: u16, sampled)
// NANO: last_touch_pc (DASCA closing-write PC seed) moved OUT of per-line BLOCK
// metadata into a SAMPLED side table folded by mjset(). The per-line array was
// 512 KB @ 16 MB (it scaled with cache size); this fixes it at MJ_SETS*WAY*2B.
// Same rationale as MJ state: it only seeds a PC-indexed predictor, so a sampled
// pool aliasing a few sets together is sufficient.
static uint16_t ltp_tab[MJ_SETS][LLC_WAY];    // sampled last-touch PC (low 16b)
static inline void     ltp_set(uint32_t set, int way, uint64_t ip) { ltp_tab[mjset(set)][way] = (uint16_t)ip; }
static inline uint16_t ltp_get(uint32_t set, int way) { return ltp_tab[mjset(set)][way]; }
static bool trisota_inited = false;
static inline void init_once() {
    if (trisota_inited) return;
    for (uint32_t s = 0; s < SIG_N; s++) rdp[s] = -1;
    for (uint32_t st = 0; st < MJ_SETS; st++) { etr_clock[st] = MJ_GRAN; for (int w = 0; w < LLC_WAY; w++) etr[st][w] = 0; }
    trisota_inited = true;
}
static inline int temporal_difference(int init, int sample) {
    if (sample > init) { int d = (sample - init) / 16; if (d < 1) d = 1; return std::min(init + d, MJ_INF_RD); }
    if (sample < init) { int d = (init - sample) / 16; if (d < 1) d = 1; return std::max(init - d, 0); }
    return init;
}
// age ETR clock for a set on each access (Mockingjay GRANULARITY aging)
static inline void mj_age(uint32_t set, int filled_way) {
    if (set >= NSET) return;
    uint32_t s = mjset(set);
    if (etr_clock[s] == MJ_GRAN) {
        for (int w = 0; w < LLC_WAY; w++)
            if (w != filled_way && std::abs(etr[s][w]) < MJ_INF_ETR) etr[s][w]--;
        etr_clock[s] = 0;
    }
    etr_clock[s]++;
}
// Mockingjay upward bypass test: bypass fill if RDP says reuse is farther than
// every resident line (or effectively never).
static inline bool mj_bypass_fill(uint32_t set, uint32_t sig) {
    if (set >= NSET || rdp[sig] < 0) return false;
    uint32_t s = mjset(set);
    int max_etr = 0;
    for (int w = 0; w < LLC_WAY; w++) { int a = std::abs(etr[s][w]); if (a > max_etr) max_etr = a; }
    return (rdp[sig] > MJ_MAX_RD) || (rdp[sig] / MJ_GRAN > max_etr);
}
// Mockingjay says this PC's lines are reused SOON (small predicted reuse distance):
// such lines must NOT be downward-bypassed (invalidation would destroy the reuse).
static inline bool mj_soon_reuse(uint32_t sig) { return rdp[sig] >= 0 && rdp[sig] <= MJ_MAX_RD; }
// train RDP + ETR on an LLC access (hit or fill) to (set,way) by pc sig.
static inline void mj_on_access(uint32_t set, int way, uint32_t sig, bool hit) {
    if (set >= NSET) return;
    uint32_t s = mjset(set);
    set_acc[s]++;
    mj_age(set, way);
    if (hit) {
        // observed reuse distance = accesses since this line's last touch
        uint16_t dist = (uint16_t)(set_acc[s] - line_ts[s][way]);  // COMPACT: u16 wraparound
        int sample = (int)std::min<uint32_t>((uint32_t)dist, (uint32_t)MJ_INF_RD);
        uint32_t fsig = line_sig[s][way];
        rdp[fsig] = (rdp[fsig] < 0) ? sample : temporal_difference(rdp[fsig], sample);
        rpt_reuse(fsig);                              // Novella RPT reuse
        ls_recall(fsig);                              // Density recall
        // refresh ETR of the reused line toward its predicted distance
        etr[s][way] = (rdp[fsig] > MJ_MAX_RD) ? MJ_INF_ETR : rdp[fsig] / MJ_GRAN;
        line_ts[s][way] = set_acc[s];
    } else {
        // a fresh fill installs into the way
        line_sig[s][way] = sig;
        line_ts[s][way]  = set_acc[s];
        etr[s][way] = (rdp[sig] < 0) ? 0 : ((rdp[sig] > MJ_MAX_RD) ? MJ_INF_ETR : rdp[sig] / MJ_GRAN);
    }
}
// a line leaving the LLC without reuse: RDP -> INF, Density evict, dead-prob.
static inline void mj_on_evict(uint32_t set, int way, bool used) {
    if (set >= NSET) return;
    uint32_t fsig = line_sig[mjset(set)][way];
    if (!used) { rdp[fsig] = MJ_INF_RD; ls_evict(fsig); dp_dead(set); }
}

// =====================================================================
// RL Q-TABLE over the COMBINED SOTA feature vector
// state bits: live_score(2) | rpt0(1) | mj(1) | ar(1) | wq_band(2) | mr_band(1) = 8 -> 256
// =====================================================================
static const int QN = 256;
static int16_t Q[QN];
static const int16_t Q_MAX = 64, Q_MIN = -64;
static const int Q_POS = 1, Q_NEG = 2;            // gentle penalty so RL only trims persistently-bad contexts
// PERMISSIVE gate: the SOTA signals (mj/nov/density) drive the bypass; the RL Q
// only SUPPRESSES a context after it has been confidently wrong many times
// (Q < -24 ~= 12 consecutive reuse penalties). This lets TriSOTA bypass as
// aggressively as the pure SOTAs (which have no RL gate) while still learning to
// veto the few contexts that repeatedly mispredict. Q>=0-gating throttled us
// below Novella; -24 frees the SOTA aggressiveness.
static const int16_t Q_THRESH = -24;              // allow bypass unless Q < -24
static inline uint32_t wq_band(double f) { return f > 0.75 ? 3u : f > 0.5 ? 2u : f > 0.25 ? 1u : 0u; }
static inline uint32_t qidx(uint8_t live, bool rpt0, bool mj, bool ar, uint32_t wqb, bool mrb) {
    return ((uint32_t)(live & 3))
         | ((uint32_t)(rpt0 ? 1 : 0) << 2)
         | ((uint32_t)(mj   ? 1 : 0) << 3)
         | ((uint32_t)(ar   ? 1 : 0) << 4)
         | ((wqb & 3) << 5)
         | ((uint32_t)(mrb ? 1 : 0) << 7);
}
static inline bool q_allow(uint32_t i) { return Q[i] >= Q_THRESH; }
static inline void q_reward(uint32_t i, int d) { int v = Q[i] + d; if (v > Q_MAX) v = Q_MAX; if (v < Q_MIN) v = Q_MIN; Q[i] = (int16_t)v; }

// ====================================================================
// WRITE-DEAD ACCURACY (confusion matrix over the ACTUAL nano decisions)
//   "predicted dead" = the model BYPASSED a fill. Ground truth from outcome:
//     wd_byp : every bypass = a predicted-dead event
//     wd_fp  : a bypassed line later RE-ACCESSED  (predicted dead, was LIVE)
//     -> TP  = wd_byp - wd_fp                      (predicted dead, was dead)
//   INSERTED (kept) lines classified at eviction by the used bit:
//     wd_fn  : inserted line evicted used==0       (was dead, kept = missed)
//     wd_tn  : inserted line evicted used==1       (was live, correctly kept)
//   Precision=TP/(TP+FP)  Recall=TP/(TP+FN)  F1=2PR/(P+R)
// ====================================================================
static uint64_t wd_byp = 0, wd_fp = 0, wd_fn = 0, wd_tn = 0;
static inline void wd_on_bypass()                 { wd_byp++; }
static inline void wd_on_evict_insert(bool used)  { if (used) wd_tn++; else wd_fn++; }

// reuse-feedback breadcrumb: remember (addr-tag -> qidx) for bypassed lines so a
// later access applies the negative reward (bypass was wrong).
static const uint32_t BC_N = 512;   // NANO-max: 1024->512
struct BC { uint32_t tag; uint16_t qi; uint8_t valid; };
static BC bc[BC_N];
static inline uint32_t bc_h(uint64_t addr) { uint64_t a = addr >> 6; a ^= a >> 13; return (uint32_t)(a & (BC_N - 1)); }
static inline void bc_record(uint64_t addr, uint32_t qi) { uint32_t h = bc_h(addr); bc[h].tag = (uint32_t)(addr >> 6); bc[h].qi = (uint16_t)qi; bc[h].valid = 1; }
static inline void bc_check(uint64_t addr) {  // called on an LLC access; if it was bypassed -> wrong
    uint32_t h = bc_h(addr);
    if (bc[h].valid && bc[h].tag == (uint32_t)(addr >> 6)) {
        q_reward(bc[h].qi, -Q_NEG);
        wd_fp++;                 // ACCURACY: a bypassed line was re-accessed -> false positive
        bc[h].valid = 0;
    }
}

// stats
static uint64_t up_pred = 0, up_byp = 0, down_pred = 0, down_byp = 0, reuse_wrong = 0;

} // namespace trisota
#endif
