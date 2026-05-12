/*
 * ===========================================================================
 * File        : sa3x3_benchmark.c
 * Project     : CSNE-SoC – Configurable Systolic Neural Engine
 * Platform    : Terasic DE10-Nano  (Intel Cyclone V SoC  5CSEBA6U23I7)
 * OS          : Linux (Yocto / Angstrom / Ubuntu for DE10-Nano)
 *
 * Description :
 *   Benchmark suite for the IP_SA_3x3 hardware accelerator (3×3 MAC array).
 *   Mirrors pe_benchmark.c (1×1 PE) in structure so results are directly
 *   comparable for the paper's scaling analysis.
 *
 *   Key difference vs 1×1:
 *     One "operation" here is a full 3×3 matrix multiply C = A × B,
 *     which requires 3 sequential MAC k-slices (k = 0, 1, 2).
 *     Each k-slice writes 6 operands, pulses start, and waits for done.
 *     Total compute per operation: 9 MACs (vs 1 MAC in the 1×1).
 *
 *   Benchmarks:
 *   [1] LATENCY   – time for one full 3×3 matrix multiply (flush + 3 MACs)
 *   [2] THROUGHPUT – 3×3 matrix multiplies back-to-back (no flush between)
 *   [3] COMMS OVERHEAD – raw PIO round-trip without computation
 *   [4] SOFTWARE BASELINE – same 3×3 multiply in ARM software (reference)
 *   [5] FUNCTIONAL VALIDATION – verifies C = A×B before benchmarking
 *
 *   Output files:
 *     sa3x3_latency_samples.csv   – one row per matrix-multiply latency sample
 *     sa3x3_comms_overhead.csv    – one row per comms overhead sample
 *     sa3x3_poll_distribution.csv – histogram of polling iterations per k-slice
 *     sa3x3_summary.txt           – human-readable summary for paper
 *
 * Architecture (Platform Designer PIO bridge — same as 1×1):
 *   pio32_in_0   [LW+0x0000]  pe_readdata  (32-bit INPUT  from IP_SA_3x3)
 *   pio32_out_0  [LW+0x0004]  pe_address   (32-bit OUTPUT to  IP_SA_3x3)
 *   pio32_out_1  [LW+0x0008]  pe_writedata (32-bit OUTPUT to  IP_SA_3x3)
 *   pio8_out_0   [LW+0x000C]  pe_write     ( 8-bit OUTPUT to  IP_SA_3x3)
 *
 * IP_SA_3x3 register map (byte offsets decoded by PE_address[6:2]):
 *   0x00  W   a_row0[7:0]  — A[0,k]
 *   0x04  W   a_row1[7:0]  — A[1,k]
 *   0x08  W   a_row2[7:0]  — A[2,k]
 *   0x0C  W   b_col0[7:0]  — B[k,0]
 *   0x10  W   b_col1[7:0]  — B[k,1]
 *   0x14  W   b_col2[7:0]  — B[k,2]
 *   0x18  W   control (bit0 = flush)
 *   0x1C  W   start   (any write triggers one MAC operation)
 *   0x20  R   status  (bit0 = done)
 *   0x24  R   acc_00  — C[0][0]  (32-bit signed)
 *   0x28  R   acc_01  — C[0][1]
 *   0x2C  R   acc_02  — C[0][2]
 *   0x30  R   acc_10  — C[1][0]
 *   0x34  R   acc_11  — C[1][1]
 *   0x38  R   acc_12  — C[1][2]
 *   0x3C  R   acc_20  — C[2][0]
 *   0x40  R   acc_21  — C[2][1]
 *   0x44  R   acc_22  — C[2][2]
 *
 * Build:
 *   gcc -O1 -Wall -Wextra -lm -o sa3x3_benchmark sa3x3_benchmark.c
 *
 * Run:
 *   sudo ./sa3x3_benchmark
 *
 * Author  : Daniel G. Fajardo Lopez
 * Date    : 2026-05-11
 * Version : 1.0
 * ===========================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <sys/mman.h>

/* ===========================================================================
 * Hardware constants – Cyclone V 5CSEBA6U23I7
 * ===========================================================================*/
#define HPS_LW_BRIDGE_BASE    (0xFF200000UL)
#define HPS_LW_BRIDGE_SPAN    (0x00200000UL)
#define HPS_LW_BRIDGE_MASK    (HPS_LW_BRIDGE_SPAN - 1UL)

#define FPGA_CLOCK_HZ         (50000000UL)
#define FPGA_CLOCK_PERIOD_NS  (20.0)

/* ===========================================================================
 * PIO offsets – identical to 1×1 (same Platform Designer bridge structure)
 * ===========================================================================*/
#define PIO_PE_READDATA_OFFSET   (0x0000UL)  /* pe_readdata  (32-bit INPUT)  */
#define PIO_PE_ADDRESS_OFFSET    (0x0004UL)  /* pe_address   (32-bit OUTPUT) */
#define PIO_PE_WRITEDATA_OFFSET  (0x0008UL)  /* pe_writedata (32-bit OUTPUT) */
#define PIO_PE_WRITE_OFFSET      (0x000CUL)  /* pe_write     ( 8-bit OUTPUT) */

/* ===========================================================================
 * IP_SA_3x3 register byte offsets
 * ===========================================================================*/
#define SA_REG_A_ROW0   (0x00U)  /* W  A[0,k] — 8-bit signed operand        */
#define SA_REG_A_ROW1   (0x04U)  /* W  A[1,k]                                */
#define SA_REG_A_ROW2   (0x08U)  /* W  A[2,k]                                */
#define SA_REG_B_COL0   (0x0CU)  /* W  B[k,0] — 8-bit signed weight          */
#define SA_REG_B_COL1   (0x10U)  /* W  B[k,1]                                */
#define SA_REG_B_COL2   (0x14U)  /* W  B[k,2]                                */
#define SA_REG_CONTROL  (0x18U)  /* W  control: bit0 = flush                 */
#define SA_REG_START    (0x1CU)  /* W  start: any write triggers MAC         */
#define SA_REG_STATUS   (0x20U)  /* R  status: bit0 = done                   */
#define SA_REG_ACC_00   (0x24U)  /* R  C[0][0] (32-bit signed result)        */
#define SA_REG_ACC_01   (0x28U)  /* R  C[0][1]                               */
#define SA_REG_ACC_02   (0x2CU)  /* R  C[0][2]                               */
#define SA_REG_ACC_10   (0x30U)  /* R  C[1][0]                               */
#define SA_REG_ACC_11   (0x34U)  /* R  C[1][1]                               */
#define SA_REG_ACC_12   (0x38U)  /* R  C[1][2]                               */
#define SA_REG_ACC_20   (0x3CU)  /* R  C[2][0]                               */
#define SA_REG_ACC_21   (0x40U)  /* R  C[2][1]                               */
#define SA_REG_ACC_22   (0x44U)  /* R  C[2][2]                               */

/* Bit masks */
#define SA_CTRL_FLUSH_BIT    (0x00000001U)
#define SA_STATUS_DONE_BIT   (0x00000001U)
#define SA_WRITE_ASSERT      (0xFFU)
#define SA_WRITE_DEASSERT    (0x00U)

/* ===========================================================================
 * Benchmark configuration
 * ===========================================================================*/
#define BENCH_LATENCY_SAMPLES     (1000U)
#define BENCH_THROUGHPUT_SAMPLES  (1000U)
#define BENCH_COMMS_SAMPLES       (1000U)
#define BENCH_SOFTWARE_SAMPLES    (1000U)
#define BENCH_READY_POLL_LIMIT    (50000U)

/* Benchmark matrix operands (INT8 signed — representative values) */
static const int8_t k_bench_A[3][3] = {
    { 15, -7,  3 },
    {  4,  8, -2 },
    { -9,  5,  6 }
};

static const int8_t k_bench_B[3][3] = {
    {  7,  2, -5 },
    { -3, 10,  1 },
    {  6, -4,  8 }
};

/* Validation test matrices (same as tb_SA_TOP_3x3.vhd Test Case 1) */
static const int8_t k_val_A[3][3] = {
    { 1, 2, 3 },
    { 4, 5, 6 },
    { 7, 8, 9 }
};
static const int8_t k_val_B[3][3] = {
    { 7, 1, 4 },
    { 8, 2, 5 },
    { 9, 3, 6 }
};
static const int32_t k_val_expected[3][3] = {
    {  50,  14,  32 },
    { 122,  32,  77 },
    { 194,  50, 122 }
};

/* ===========================================================================
 * Low-level PIO access macros (identical to 1×1 driver)
 * ===========================================================================*/
#define PIO_WRITE(lw, off, val) \
    (*((volatile uint32_t *)((uint8_t *)(lw) + (off))) = (uint32_t)(val))

#define PIO_READ(lw, off) \
    (*((volatile uint32_t *)((uint8_t *)(lw) + (off))))

/* ===========================================================================
 * Timing helper
 * ===========================================================================*/
static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* ===========================================================================
 * Avalon transaction helpers (identical to 1×1 driver)
 * ===========================================================================*/
static inline void avalon_write(void *lw, uint32_t addr, uint32_t data)
{
    PIO_WRITE(lw, PIO_PE_ADDRESS_OFFSET,   addr);
    PIO_WRITE(lw, PIO_PE_WRITEDATA_OFFSET, data);
    PIO_WRITE(lw, PIO_PE_WRITE_OFFSET, SA_WRITE_ASSERT);
    PIO_WRITE(lw, PIO_PE_WRITE_OFFSET, SA_WRITE_DEASSERT);
}

static inline uint32_t avalon_read(void *lw, uint32_t addr)
{
    PIO_WRITE(lw, PIO_PE_ADDRESS_OFFSET, addr);
    return PIO_READ(lw, PIO_PE_READDATA_OFFSET);
}

/* ===========================================================================
 * sa_flush() – assert then de-assert flush, clearing all 9 accumulators.
 * ===========================================================================*/
static void sa_flush(void *lw)
{
    avalon_write(lw, SA_REG_CONTROL, SA_CTRL_FLUSH_BIT);  /* flush = 1 */
    avalon_write(lw, SA_REG_CONTROL, 0U);                  /* flush = 0 */
}

/* ===========================================================================
 * sa_mac_kslice() – send one k-slice to the array and wait for done.
 *
 *   Writes A[:,k] to a_row* and B[k,:] to b_col*, then pulses start.
 *   Polls status until done=1 or watchdog expires.
 *
 *   Returns number of poll iterations (useful for latency analysis).
 * ===========================================================================*/
static uint32_t sa_mac_kslice(void       *lw,
                               int8_t      a0, int8_t a1, int8_t a2,
                               int8_t      b0, int8_t b1, int8_t b2)
{
    /* Step 1 – Write all 6 operands */
    avalon_write(lw, SA_REG_A_ROW0, (uint32_t)(uint8_t)a0);
    avalon_write(lw, SA_REG_A_ROW1, (uint32_t)(uint8_t)a1);
    avalon_write(lw, SA_REG_A_ROW2, (uint32_t)(uint8_t)a2);
    avalon_write(lw, SA_REG_B_COL0, (uint32_t)(uint8_t)b0);
    avalon_write(lw, SA_REG_B_COL1, (uint32_t)(uint8_t)b1);
    avalon_write(lw, SA_REG_B_COL2, (uint32_t)(uint8_t)b2);

    /* Step 2 – Pulse start */
    avalon_write(lw, SA_REG_START, 1U);

    /* Step 3 – Poll done with watchdog */
    uint32_t polls  = 0U;
    uint32_t status;
    do {
        status = avalon_read(lw, SA_REG_STATUS);
        polls++;
    } while (((status & SA_STATUS_DONE_BIT) == 0U) &&
             (polls < BENCH_READY_POLL_LIMIT));

    return polls;
}

/* ===========================================================================
 * sa_matmul() – full 3×3 matrix multiply: C = A × B using 3 k-slices.
 *
 *   Caller must flush before calling if accumulator must start at zero.
 *   Reads results into out_C[3][3] after the third k-slice.
 * ===========================================================================*/
static void sa_matmul(void          *lw,
                      const int8_t   A[3][3],
                      const int8_t   B[3][3],
                      int32_t        out_C[3][3])
{
    /* k = 0: A[:,0] × B[0,:] */
    sa_mac_kslice(lw,
                  A[0][0], A[1][0], A[2][0],
                  B[0][0], B[0][1], B[0][2]);

    /* k = 1: A[:,1] × B[1,:] */
    sa_mac_kslice(lw,
                  A[0][1], A[1][1], A[2][1],
                  B[1][0], B[1][1], B[1][2]);

    /* k = 2: A[:,2] × B[2,:] */
    sa_mac_kslice(lw,
                  A[0][2], A[1][2], A[2][2],
                  B[2][0], B[2][1], B[2][2]);

    /* Read 9 accumulators */
    out_C[0][0] = (int32_t)avalon_read(lw, SA_REG_ACC_00);
    out_C[0][1] = (int32_t)avalon_read(lw, SA_REG_ACC_01);
    out_C[0][2] = (int32_t)avalon_read(lw, SA_REG_ACC_02);
    out_C[1][0] = (int32_t)avalon_read(lw, SA_REG_ACC_10);
    out_C[1][1] = (int32_t)avalon_read(lw, SA_REG_ACC_11);
    out_C[1][2] = (int32_t)avalon_read(lw, SA_REG_ACC_12);
    out_C[2][0] = (int32_t)avalon_read(lw, SA_REG_ACC_20);
    out_C[2][1] = (int32_t)avalon_read(lw, SA_REG_ACC_21);
    out_C[2][2] = (int32_t)avalon_read(lw, SA_REG_ACC_22);
}

/* ===========================================================================
 * Statistics (identical to 1×1 benchmark)
 * ===========================================================================*/
typedef struct {
    double min, max, mean, stddev, median;
} stats_t;

static int cmp_double(const void *a, const void *b)
{
    double da = *(const double *)a;
    double db = *(const double *)b;
    return (da > db) - (da < db);
}

static stats_t compute_stats(double *data, uint32_t n)
{
    stats_t s = {0};
    if (n == 0U) return s;

    s.min = 1e18; s.max = -1e18;
    double sum = 0.0;
    for (uint32_t i = 0U; i < n; i++) {
        if (data[i] < s.min) s.min = data[i];
        if (data[i] > s.max) s.max = data[i];
        sum += data[i];
    }
    s.mean = sum / (double)n;

    double sq = 0.0;
    for (uint32_t i = 0U; i < n; i++) {
        double d = data[i] - s.mean;
        sq += d * d;
    }
    s.stddev = sqrt(sq / (double)n);

    double *copy = malloc(n * sizeof(double));
    if (copy) {
        memcpy(copy, data, n * sizeof(double));
        qsort(copy, n, sizeof(double), cmp_double);
        s.median = (n % 2U == 0U)
                   ? (copy[n/2U-1U] + copy[n/2U]) / 2.0
                   : copy[n/2U];
        free(copy);
    }
    return s;
}

/* ===========================================================================
 * BENCHMARK 0 – Functional validation
 *   Runs Test Case 1 from tb_SA_TOP_3x3.vhd and verifies all 9 results.
 *   Must pass before running performance benchmarks.
 * ===========================================================================*/
static bool bench_functional_validation(void *lw)
{
    printf("  [0/4] Functional validation ... ");
    fflush(stdout);

    int32_t C[3][3];
    sa_flush(lw);
    sa_matmul(lw, k_val_A, k_val_B, C);

    bool all_pass = true;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            if (C[r][c] != k_val_expected[r][c]) {
                all_pass = false;
                printf("\n  [FAIL] C[%d][%d] = %d  (expected %d)",
                       r, c, C[r][c], k_val_expected[r][c]);
            }
        }
    }

    if (all_pass) {
        printf("PASS (9/9 correct)\n");
    } else {
        printf("\n  -> Validation FAILED — results may be unreliable\n");
    }
    return all_pass;
}

/* ===========================================================================
 * BENCHMARK 1 – Latency
 *   One full 3×3 matrix multiply per sample (flush + 3 k-slices + read).
 *   Records total wall-clock time and total poll iterations.
 * ===========================================================================*/
static void bench_latency(void     *lw,
                          double   *samples_ns,
                          uint32_t *poll_totals,
                          uint32_t  n)
{
    printf("  [1/4] Latency benchmark  (%u matrix multiplies) ... ", n);
    fflush(stdout);

    for (uint32_t i = 0U; i < n; i++) {
        sa_flush(lw);

        uint64_t t0 = now_ns();

        uint32_t polls = 0U;

        /* k=0 */
        polls += sa_mac_kslice(lw,
            k_bench_A[0][0], k_bench_A[1][0], k_bench_A[2][0],
            k_bench_B[0][0], k_bench_B[0][1], k_bench_B[0][2]);
        /* k=1 */
        polls += sa_mac_kslice(lw,
            k_bench_A[0][1], k_bench_A[1][1], k_bench_A[2][1],
            k_bench_B[1][0], k_bench_B[1][1], k_bench_B[1][2]);
        /* k=2 */
        polls += sa_mac_kslice(lw,
            k_bench_A[0][2], k_bench_A[1][2], k_bench_A[2][2],
            k_bench_B[2][0], k_bench_B[2][1], k_bench_B[2][2]);

        /* Read all 9 accumulators to complete the transaction */
        (void)avalon_read(lw, SA_REG_ACC_00);
        (void)avalon_read(lw, SA_REG_ACC_22);

        uint64_t t1 = now_ns();

        samples_ns[i]  = (double)(t1 - t0);
        poll_totals[i] = polls;   /* sum of polls across 3 k-slices */
    }

    printf("done.\n");
}

/* ===========================================================================
 * BENCHMARK 2 – Throughput
 *   N matrix multiplies back-to-back WITHOUT flush between them.
 *   The accumulators keep accumulating — measures sustained pipeline rate.
 * ===========================================================================*/
static double bench_throughput(void *lw, uint32_t n)
{
    printf("  [2/4] Throughput benchmark (%u matrix multiplies, no flush) ... ", n);
    fflush(stdout);

    sa_flush(lw);  /* single flush at start only */

    uint64_t t0 = now_ns();

    for (uint32_t i = 0U; i < n; i++) {
        sa_mac_kslice(lw,
            k_bench_A[0][0], k_bench_A[1][0], k_bench_A[2][0],
            k_bench_B[0][0], k_bench_B[0][1], k_bench_B[0][2]);
        sa_mac_kslice(lw,
            k_bench_A[0][1], k_bench_A[1][1], k_bench_A[2][1],
            k_bench_B[1][0], k_bench_B[1][1], k_bench_B[1][2]);
        sa_mac_kslice(lw,
            k_bench_A[0][2], k_bench_A[1][2], k_bench_A[2][2],
            k_bench_B[2][0], k_bench_B[2][1], k_bench_B[2][2]);
    }

    uint64_t t1 = now_ns();

    /* Each iteration = 9 MAC operations = 1 matrix multiply */
    double total_ns  = (double)(t1 - t0);
    double mat_per_s = (double)n / total_ns * 1e9;       /* matrix mults/s   */
    double mops      = mat_per_s * 9.0 / 1e6;            /* MOPS (9 MACs each)*/

    printf("done. (%.3f MOPS  |  %.1f matrix mults/s)\n", mops, mat_per_s);
    return mops;
}

/* ===========================================================================
 * BENCHMARK 3 – Communication overhead
 * ===========================================================================*/
static void bench_comms_overhead(void *lw, double *samples_ns, uint32_t n)
{
    printf("  [3/4] Comms overhead benchmark (%u samples) ... ", n);
    fflush(stdout);

    for (uint32_t i = 0U; i < n; i++) {
        uint64_t t0 = now_ns();

        PIO_WRITE(lw, PIO_PE_ADDRESS_OFFSET,   SA_REG_STATUS);
        PIO_WRITE(lw, PIO_PE_WRITEDATA_OFFSET, 0U);
        PIO_WRITE(lw, PIO_PE_WRITE_OFFSET, SA_WRITE_ASSERT);
        PIO_WRITE(lw, PIO_PE_WRITE_OFFSET, SA_WRITE_DEASSERT);
        (void)PIO_READ(lw, PIO_PE_READDATA_OFFSET);

        uint64_t t1 = now_ns();
        samples_ns[i] = (double)(t1 - t0);
    }

    printf("done.\n");
}

/* ===========================================================================
 * BENCHMARK 4 – Software baseline: 3×3 matrix multiply in ARM C
 * ===========================================================================*/
static void bench_software(double *samples_ns, uint32_t n)
{
    printf("  [4/4] Software baseline benchmark (%u matrix multiplies) ... ", n);
    fflush(stdout);

    volatile int32_t C[3][3];

    for (uint32_t i = 0U; i < n; i++) {
        /* Reset accumulator */
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                C[r][c] = 0;

        uint64_t t0 = now_ns();

        /* 3×3 matrix multiply: 27 multiply-accumulate operations */
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                for (int k = 0; k < 3; k++)
                    C[r][c] += (int32_t)k_bench_A[r][k] *
                                (int32_t)k_bench_B[k][c];

        uint64_t t1 = now_ns();
        samples_ns[i] = (double)(t1 - t0);

        (void)C[0][0];  /* prevent dead-code elimination */
    }

    printf("done.\n");
}

/* ===========================================================================
 * CSV and summary export
 * ===========================================================================*/
static void export_latency_csv(const char   *filename,
                                double       *lat_ns,
                                uint32_t     *polls,
                                uint32_t      n)
{
    FILE *f = fopen(filename, "w");
    if (!f) { fprintf(stderr, "[WARN] %s: %s\n", filename, strerror(errno)); return; }

    fprintf(f, "sample,latency_ns,latency_cycles,total_poll_iterations\n");
    for (uint32_t i = 0U; i < n; i++)
        fprintf(f, "%u,%.1f,%.2f,%u\n",
                i + 1U, lat_ns[i],
                lat_ns[i] / FPGA_CLOCK_PERIOD_NS,
                polls[i]);
    fclose(f);
    printf("  -> Saved: %s\n", filename);
}

static void export_comms_csv(const char *filename, double *comms_ns, uint32_t n)
{
    FILE *f = fopen(filename, "w");
    if (!f) { fprintf(stderr, "[WARN] %s: %s\n", filename, strerror(errno)); return; }

    fprintf(f, "sample,comms_overhead_ns,comms_overhead_cycles\n");
    for (uint32_t i = 0U; i < n; i++)
        fprintf(f, "%u,%.1f,%.2f\n",
                i + 1U, comms_ns[i],
                comms_ns[i] / FPGA_CLOCK_PERIOD_NS);
    fclose(f);
    printf("  -> Saved: %s\n", filename);
}

static void export_poll_histogram_csv(const char   *filename,
                                       uint32_t     *polls,
                                       uint32_t      n)
{
    uint32_t max_poll = 0U;
    for (uint32_t i = 0U; i < n; i++)
        if (polls[i] > max_poll) max_poll = polls[i];

    uint32_t *hist = calloc(max_poll + 1U, sizeof(uint32_t));
    if (!hist) return;
    for (uint32_t i = 0U; i < n; i++) hist[polls[i]]++;

    FILE *f = fopen(filename, "w");
    if (!f) { free(hist); return; }

    fprintf(f, "total_poll_count,frequency,percentage\n");
    for (uint32_t p = 0U; p <= max_poll; p++)
        if (hist[p] > 0U)
            fprintf(f, "%u,%u,%.2f\n",
                    p, hist[p], 100.0 * (double)hist[p] / (double)n);
    fclose(f);
    free(hist);
    printf("  -> Saved: %s\n", filename);
}

static void export_summary_txt(const char *filename,
                                stats_t    *lat,
                                stats_t    *comms,
                                stats_t    *sw,
                                double      mops,
                                uint32_t    n_lat,
                                uint32_t    n_thr)
{
    FILE *f = fopen(filename, "w");
    if (!f) { fprintf(stderr, "[WARN] %s\n", filename); return; }

    double net_ns   = lat->mean - comms->mean;
    double speedup  = sw->mean / lat->mean;
    /* Effective speedup per MAC: 3x3=9 MACs per operation vs 1 in SW loop */
    double mac_speedup = (sw->mean / 9.0) / (lat->mean / 9.0);

    fprintf(f, "=================================================================\n");
    fprintf(f, "  IP_SA_3x3 Hardware Benchmark Summary\n");
    fprintf(f, "  Platform : Terasic DE10-Nano  (Cyclone V SoC 5CSEBA6U23I7)\n");
    fprintf(f, "  FPGA Clock: %lu MHz\n", FPGA_CLOCK_HZ / 1000000UL);
    fprintf(f, "  Array    : 3x3 = 9 Processing Elements\n");
    fprintf(f, "  MACs/op  : 9  (one full 3x3 matrix multiply = 3 k-slices)\n");
    fprintf(f, "=================================================================\n\n");

    fprintf(f, "[1] LATENCY per 3x3 matrix multiply  (%u samples, flush each)\n", n_lat);
    fprintf(f, "    Mean     : %8.1f ns  (%6.1f cycles)\n",
            lat->mean,   lat->mean   / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Std-dev  : %8.1f ns  (%6.1f cycles)\n",
            lat->stddev, lat->stddev / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Min      : %8.1f ns  (%6.1f cycles)\n",
            lat->min,    lat->min    / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Max      : %8.1f ns  (%6.1f cycles)\n",
            lat->max,    lat->max    / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Median   : %8.1f ns  (%6.1f cycles)\n",
            lat->median, lat->median / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Per MAC  : %8.1f ns  (%6.1f cycles)\n",
            lat->mean / 9.0, lat->mean / 9.0 / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "\n");

    fprintf(f, "[2] THROUGHPUT  (%u matrix multiplies, no flush)\n", n_thr);
    fprintf(f, "    Throughput  : %.4f MOPS  (%.1f matrix mults/s)\n",
            mops, mops * 1e6 / 9.0);
    fprintf(f, "    Cycles/MAC  : %.1f  (at %lu MHz)\n",
            (1.0 / (mops * 1e6)) * (double)FPGA_CLOCK_HZ,
            FPGA_CLOCK_HZ / 1000000UL);
    fprintf(f, "\n");

    fprintf(f, "[3] COMMUNICATION OVERHEAD  (%u samples)\n", BENCH_COMMS_SAMPLES);
    fprintf(f, "    Mean     : %8.1f ns  (%6.1f cycles)\n",
            comms->mean,   comms->mean   / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Std-dev  : %8.1f ns  (%6.1f cycles)\n",
            comms->stddev, comms->stddev / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "\n");

    fprintf(f, "[4] NET COMPUTE TIME  (latency - comms overhead)\n");
    fprintf(f, "    Net total   : %8.1f ns  (%6.1f cycles)\n",
            net_ns, net_ns / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Net per MAC : %8.1f ns  (%6.1f cycles)\n",
            net_ns / 9.0, net_ns / 9.0 / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "\n");

    fprintf(f, "[5] SOFTWARE BASELINE  (%u matrix multiplies, ARM Cortex-A9)\n",
            BENCH_SOFTWARE_SAMPLES);
    fprintf(f, "    Mean     : %8.1f ns  (full 3x3 multiply)\n", sw->mean);
    fprintf(f, "    Std-dev  : %8.1f ns\n", sw->stddev);
    fprintf(f, "    Per MAC  : %8.1f ns\n", sw->mean / 9.0);
    fprintf(f, "\n");

    fprintf(f, "[6] SPEEDUP  (software / hardware)\n");
    fprintf(f, "    Matrix multiply speedup : %.2fx\n", speedup);
    fprintf(f, "    Per-MAC speedup         : %.2fx\n", mac_speedup);
    if (speedup >= 1.0)
        fprintf(f, "    -> Hardware is %.2fx FASTER than ARM software\n", speedup);
    else
        fprintf(f, "    -> Hardware is %.2fx SLOWER (bridge overhead dominates at 3x3)\n",
                1.0 / speedup);
    fprintf(f, "\n");

    fprintf(f, "[7] COMPARISON WITH 1x1 PE (from pe_benchmark results)\n");
    fprintf(f, "    1x1 latency ref  : ~7390 ns  (1 MAC)\n");
    fprintf(f, "    3x3 latency/MAC  : %.1f ns\n", lat->mean / 9.0);
    fprintf(f, "    Bridge overhead  : %.1f ns (amortised over 9 MACs vs 1)\n",
            comms->mean / 9.0);
    fprintf(f, "\n");

    fprintf(f, "=================================================================\n");
    fprintf(f, "  Bridge overhead is shared across the 3 k-slices within each\n");
    fprintf(f, "  matrix multiply. As array size grows to NxN, the fixed bridge\n");
    fprintf(f, "  cost is further amortised over N^2 MACs, increasing efficiency.\n");
    fprintf(f, "=================================================================\n");

    fclose(f);
    printf("  -> Saved: %s\n", filename);
}

/* ===========================================================================
 * main
 * ===========================================================================*/
int main(void)
{
    int       fd        = -1;
    void     *lw_bridge = NULL;
    int       rc        = EXIT_SUCCESS;

    double   *lat_ns    = malloc(BENCH_LATENCY_SAMPLES   * sizeof(double));
    uint32_t *lat_polls = malloc(BENCH_LATENCY_SAMPLES   * sizeof(uint32_t));
    double   *comms_ns  = malloc(BENCH_COMMS_SAMPLES     * sizeof(double));
    double   *sw_ns     = malloc(BENCH_SOFTWARE_SAMPLES  * sizeof(double));

    if (!lat_ns || !lat_polls || !comms_ns || !sw_ns) {
        fprintf(stderr, "[ERROR] malloc failed\n");
        return EXIT_FAILURE;
    }

    printf("\n");
    printf("================================================================\n");
    printf("  IP_SA_3x3 Benchmark Suite v1.0\n");
    printf("  DE10-Nano  |  Cyclone V 5CSEBA6U23I7  |  50 MHz\n");
    printf("  Array: 3x3 = 9 PEs  |  9 MACs per matrix multiply\n");
    printf("================================================================\n\n");

    /* Open /dev/mem */
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        fprintf(stderr, "[ERROR] open /dev/mem: %s  (try sudo)\n", strerror(errno));
        rc = EXIT_FAILURE; goto cleanup;
    }

    /* Map LW bridge */
    lw_bridge = mmap(NULL, HPS_LW_BRIDGE_SPAN,
                     PROT_READ | PROT_WRITE, MAP_SHARED,
                     fd, (off_t)HPS_LW_BRIDGE_BASE);
    if (lw_bridge == MAP_FAILED) {
        fprintf(stderr, "[ERROR] mmap: %s\n", strerror(errno));
        rc = EXIT_FAILURE; goto cleanup;
    }

    printf("  LW bridge mapped @ virtual %p\n\n", lw_bridge);
    printf("  Running benchmarks:\n");

    /* Functional validation first — abort if wrong */
    if (!bench_functional_validation(lw_bridge)) {
        fprintf(stderr, "\n[ERROR] Functional validation failed. "
                        "Check hardware before benchmarking.\n");
        rc = EXIT_FAILURE; goto cleanup;
    }

    /* Performance benchmarks */
    bench_latency       (lw_bridge, lat_ns, lat_polls, BENCH_LATENCY_SAMPLES);
    double mops = bench_throughput  (lw_bridge, BENCH_THROUGHPUT_SAMPLES);
    bench_comms_overhead(lw_bridge, comms_ns, BENCH_COMMS_SAMPLES);
    bench_software      (sw_ns, BENCH_SOFTWARE_SAMPLES);

    /* Statistics */
    stats_t lat_s   = compute_stats(lat_ns,   BENCH_LATENCY_SAMPLES);
    stats_t comms_s = compute_stats(comms_ns, BENCH_COMMS_SAMPLES);
    stats_t sw_s    = compute_stats(sw_ns,    BENCH_SOFTWARE_SAMPLES);

    /* Console summary */
    printf("\n  Quick summary:\n");
    printf("  %-32s %8.1f ns  (%5.1f cycles)\n",
           "Latency mean (3x3 matmul):",
           lat_s.mean, lat_s.mean / FPGA_CLOCK_PERIOD_NS);
    printf("  %-32s %8.1f ns  (%5.1f cycles)\n",
           "Latency per MAC:",
           lat_s.mean / 9.0, lat_s.mean / 9.0 / FPGA_CLOCK_PERIOD_NS);
    printf("  %-32s %8.1f ns\n",
           "Latency stddev:", lat_s.stddev);
    printf("  %-32s %8.4f MOPS\n",
           "Throughput:", mops);
    printf("  %-32s %8.1f ns\n",
           "Comms overhead mean:", comms_s.mean);
    printf("  %-32s %8.1f ns\n",
           "Net compute (lat-comms):", lat_s.mean - comms_s.mean);
    printf("  %-32s %8.1f ns\n",
           "SW baseline (3x3 matmul):", sw_s.mean);
    printf("  %-32s %8.2fx\n",
           "Speedup (SW/HW):", sw_s.mean / lat_s.mean);

    /* Export results */
    printf("\n  Exporting results:\n");
    export_latency_csv       ("sa3x3_latency_samples.csv",   lat_ns,   lat_polls, BENCH_LATENCY_SAMPLES);
    export_comms_csv         ("sa3x3_comms_overhead.csv",    comms_ns, BENCH_COMMS_SAMPLES);
    export_poll_histogram_csv("sa3x3_poll_distribution.csv", lat_polls, BENCH_LATENCY_SAMPLES);
    export_summary_txt       ("sa3x3_summary.txt",
                              &lat_s, &comms_s, &sw_s, mops,
                              BENCH_LATENCY_SAMPLES, BENCH_THROUGHPUT_SAMPLES);

    printf("\n  Done. Transfer files to host:\n");
    printf("    scp root@<board-ip>:~/soc_apps/sa3x3_*.csv .\n");
    printf("    scp root@<board-ip>:~/soc_apps/sa3x3_summary.txt .\n\n");

cleanup:
    free(lat_ns); free(lat_polls); free(comms_ns); free(sw_ns);
    if (lw_bridge && lw_bridge != MAP_FAILED) munmap(lw_bridge, HPS_LW_BRIDGE_SPAN);
    if (fd >= 0) close(fd);
    return rc;
}
