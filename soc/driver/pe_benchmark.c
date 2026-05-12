/*
 * ===========================================================================
 * File        : pe_benchmark.c
 * Project     : Processing Element (PE) – Hardware Benchmark Suite
 * Platform    : Terasic DE10-Nano  (Intel Cyclone V SoC  5CSEBA6U23I7)
 * OS          : Linux (Yocto / Angstrom / Ubuntu for DE10-Nano)
 *
 * Description :
 *   Comprehensive benchmark suite for the IP_PE hardware accelerator.
 *   Measures and exports the following metrics for academic publication:
 *
 *   [1] LATENCY BENCHMARK
 *       Each operation is isolated (flush before each).
 *       Measures: total round-trip time HPS→FPGA→HPS per MAC operation.
 *       Reports:  min, max, mean, std-dev in nanoseconds and clock cycles.
 *
 *   [2] THROUGHPUT BENCHMARK
 *       Operations run back-to-back WITHOUT flush (pipeline saturated).
 *       Measures: wall-clock time for N consecutive MAC operations.
 *       Reports:  MOPS (Mega Operations Per Second), cycles/op.
 *
 *   [3] COMMUNICATION OVERHEAD BENCHMARK
 *       Measures raw PIO read/write round-trip with no computation.
 *       Isolates bridge latency from compute latency.
 *
 *   [4] SOFTWARE BASELINE BENCHMARK
 *       Same MAC (multiply-accumulate) computed in ARM software.
 *       Reports:  ns/op in software for speedup comparison.
 *
 *   [5] POLLING CYCLE DISTRIBUTION
 *       Records how many poll iterations ip_ready takes per operation.
 *       Validates FSM pipeline depth (expected: 10-15 cycles typically).
 *
 * Output files:
 *   pe_latency_samples.csv     – one row per latency sample
 *   pe_throughput_samples.csv  – one row per throughput batch
 *   pe_poll_distribution.csv   – histogram of polling counts
 *   pe_summary.txt             – human-readable summary for paper
 *
 * Architecture (Platform Designer PIO bridge):
 *   pio32_in_0   [LW+0x0000]  pe_readdata  (32-bit INPUT)
 *   pio32_out_0  [LW+0x0004]  pe_address   (32-bit OUTPUT)
 *   pio32_out_1  [LW+0x0008]  pe_writedata (32-bit OUTPUT)
 *   pio8_out_0   [LW+0x000C]  pe_write     ( 8-bit OUTPUT)
 *
 * Build:
 *   gcc -O1 -Wall -Wextra -lm -o pe_benchmark pe_benchmark.c
 *
 * Run:
 *   sudo ./pe_benchmark
 *
 * Author  : Daniel Fajardo
 * Date    : 2026-05-07
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
#define HPS_LW_BRIDGE_BASE (0xFF200000UL)
#define HPS_LW_BRIDGE_SPAN (0x00200000UL)
#define HPS_LW_BRIDGE_MASK (HPS_LW_BRIDGE_SPAN - 1UL)

#define FPGA_CLOCK_HZ (50000000UL)  /* 50 MHz board oscillator   */
#define FPGA_CLOCK_PERIOD_NS (20.0) /* 20 ns per cycle           */

/* ===========================================================================
 * PIO offsets (from Platform Designer address map)
 * ===========================================================================*/
#define PIO_PE_READDATA_OFFSET (0x0000UL)
#define PIO_PE_ADDRESS_OFFSET (0x0004UL)
#define PIO_PE_WRITEDATA_OFFSET (0x0008UL)
#define PIO_PE_WRITE_OFFSET (0x000CUL)

/* ===========================================================================
 * IP_PE register offsets
 * ===========================================================================*/
#define PE_REG_OPERAND_A (0x00U)
#define PE_REG_OPERAND_B (0x02U)
#define PE_REG_CONTROL (0x04U)
#define PE_REG_START (0x06U)
#define PE_REG_STATUS (0x09U)
#define PE_REG_ACC_OUT (0x0AU)
#define PE_REG_A_OUT (0x0BU)
#define PE_REG_B_OUT (0x0CU)

#define PE_CTRL_FLUSH_BIT (0x00000001U)
#define PE_STATUS_READY_BIT (0x00000001U)
#define PE_WRITE_ASSERT (0xFFU)
#define PE_WRITE_DEASSERT (0x00U)

/* ===========================================================================
 * Benchmark configuration
 * ===========================================================================*/
#define BENCH_LATENCY_SAMPLES (1000U)    /* iterations for latency bench   */
#define BENCH_THROUGHPUT_SAMPLES (1000U) /* iterations for throughput bench */
#define BENCH_COMMS_SAMPLES (1000U)      /* iterations for comms overhead  */
#define BENCH_SOFTWARE_SAMPLES (1000U)   /* iterations for SW baseline     */
#define BENCH_READY_POLL_LIMIT (50000U)  /* watchdog                       */

/* Operands used for benchmarking (representative signed 8-bit values) */
#define BENCH_OPERAND_A ((int8_t)15)
#define BENCH_OPERAND_B ((int8_t)-7)

/* ===========================================================================
 * Low-level PIO macros
 * ===========================================================================*/
#define PIO_WRITE(lw, off, val) \
    (*((volatile uint32_t *)((uint8_t *)(lw) + (off))) = (uint32_t)(val))

#define PIO_READ(lw, off) \
    (*((volatile uint32_t *)((uint8_t *)(lw) + (off))))

/* ===========================================================================
 * Timing helper
 * Returns current monotonic time in nanoseconds.
 * Uses CLOCK_MONOTONIC_RAW to avoid NTP adjustments skewing measurements.
 * ===========================================================================*/
static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* ===========================================================================
 * Avalon transaction helpers
 * ===========================================================================*/
static inline void avalon_write(void *lw, uint32_t addr, uint32_t data)
{
    PIO_WRITE(lw, PIO_PE_ADDRESS_OFFSET, addr);
    PIO_WRITE(lw, PIO_PE_WRITEDATA_OFFSET, data);
    PIO_WRITE(lw, PIO_PE_WRITE_OFFSET, PE_WRITE_ASSERT);
    PIO_WRITE(lw, PIO_PE_WRITE_OFFSET, PE_WRITE_DEASSERT);
}

static inline uint32_t avalon_read(void *lw, uint32_t addr)
{
    PIO_WRITE(lw, PIO_PE_ADDRESS_OFFSET, addr);
    return PIO_READ(lw, PIO_PE_READDATA_OFFSET);
}

static inline void pe_flush(void *lw)
{
    avalon_write(lw, PE_REG_CONTROL, PE_CTRL_FLUSH_BIT);
    avalon_write(lw, PE_REG_CONTROL, 0U);
}

/* ===========================================================================
 * Statistics helper – computes min, max, mean, std-dev over a double array
 * ===========================================================================*/
typedef struct
{
    double min;
    double max;
    double mean;
    double stddev;
    double median; /* approximate: sorts a copy of the array */
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
    if (n == 0U)
        return s;

    /* min / max / mean */
    s.min = 1e18;
    s.max = -1e18;
    double sum = 0.0;
    for (uint32_t i = 0U; i < n; i++)
    {
        if (data[i] < s.min)
            s.min = data[i];
        if (data[i] > s.max)
            s.max = data[i];
        sum += data[i];
    }
    s.mean = sum / (double)n;

    /* std-dev (population) */
    double sq_sum = 0.0;
    for (uint32_t i = 0U; i < n; i++)
    {
        double diff = data[i] - s.mean;
        sq_sum += diff * diff;
    }
    s.stddev = sqrt(sq_sum / (double)n);

    /* median: sort a copy */
    double *copy = malloc(n * sizeof(double));
    if (copy)
    {
        memcpy(copy, data, n * sizeof(double));
        qsort(copy, n, sizeof(double), cmp_double);
        s.median = (n % 2U == 0U)
                       ? (copy[n / 2U - 1U] + copy[n / 2U]) / 2.0
                       : copy[n / 2U];
        free(copy);
    }

    return s;
}

/* ===========================================================================
 * BENCHMARK 1 – Latency
 *   Each sample: flush → write A → write B → start → poll ready → read acc
 *   Flush before each ensures independent measurements.
 * ===========================================================================*/
static void bench_latency(void *lw,
                          double *samples_ns,
                          uint32_t *poll_counts,
                          uint32_t n)
{
    printf("  [1/4] Latency benchmark  (%u samples) ... ", n);
    fflush(stdout);

    for (uint32_t i = 0U; i < n; i++)
    {

        pe_flush(lw);

        uint64_t t0 = now_ns();

        avalon_write(lw, PE_REG_OPERAND_A, (uint32_t)(uint8_t)BENCH_OPERAND_A);
        avalon_write(lw, PE_REG_OPERAND_B, (uint32_t)(uint8_t)BENCH_OPERAND_B);
        avalon_write(lw, PE_REG_START, 1U);

        uint32_t polls = 0U;
        uint32_t status;
        do
        {
            status = avalon_read(lw, PE_REG_STATUS);
            polls++;
        } while (((status & PE_STATUS_READY_BIT) == 0U) &&
                 (polls < BENCH_READY_POLL_LIMIT));

        /* read result to complete the transaction */
        (void)avalon_read(lw, PE_REG_ACC_OUT);

        uint64_t t1 = now_ns();

        samples_ns[i] = (double)(t1 - t0);
        poll_counts[i] = polls;
    }

    printf("done.\n");
}

/* ===========================================================================
 * BENCHMARK 2 – Throughput
 *   Runs N MAC operations back-to-back WITHOUT flush (accumulator runs free).
 *   Measures total wall-clock time then derives ops/second.
 * ===========================================================================*/
static double bench_throughput(void *lw, uint32_t n)
{
    printf("  [2/4] Throughput benchmark (%u ops, no flush) ... ", n);
    fflush(stdout);

    pe_flush(lw); /* single flush at start only */

    uint64_t t0 = now_ns();

    for (uint32_t i = 0U; i < n; i++)
    {
        avalon_write(lw, PE_REG_OPERAND_A, (uint32_t)(uint8_t)BENCH_OPERAND_A);
        avalon_write(lw, PE_REG_OPERAND_B, (uint32_t)(uint8_t)BENCH_OPERAND_B);
        avalon_write(lw, PE_REG_START, 1U);

        uint32_t polls = 0U;
        uint32_t status;
        do
        {
            status = avalon_read(lw, PE_REG_STATUS);
            polls++;
        } while (((status & PE_STATUS_READY_BIT) == 0U) &&
                 (polls < BENCH_READY_POLL_LIMIT));

        (void)avalon_read(lw, PE_REG_ACC_OUT);
    }

    uint64_t t1 = now_ns();

    double total_ns = (double)(t1 - t0);
    double mops = ((double)n / total_ns) * 1000.0; /* MOPS */

    printf("done. (%.3f MOPS)\n", mops);
    return mops;
}

/* ===========================================================================
 * BENCHMARK 3 – Communication overhead
 *   Measures raw PIO write + read without triggering computation.
 *   Sends a write to pe_address only (no start), then reads pe_readdata.
 *   This isolates LW bridge round-trip latency.
 * ===========================================================================*/
static void bench_comms_overhead(void *lw, double *samples_ns, uint32_t n)
{
    printf("  [3/4] Comms overhead benchmark (%u samples) ... ", n);
    fflush(stdout);

    for (uint32_t i = 0U; i < n; i++)
    {
        uint64_t t0 = now_ns();

        /* One write transaction + one read transaction, no compute */
        PIO_WRITE(lw, PIO_PE_ADDRESS_OFFSET, PE_REG_STATUS);
        PIO_WRITE(lw, PIO_PE_WRITEDATA_OFFSET, 0U);
        PIO_WRITE(lw, PIO_PE_WRITE_OFFSET, PE_WRITE_ASSERT);
        PIO_WRITE(lw, PIO_PE_WRITE_OFFSET, PE_WRITE_DEASSERT);
        (void)PIO_READ(lw, PIO_PE_READDATA_OFFSET);

        uint64_t t1 = now_ns();
        samples_ns[i] = (double)(t1 - t0);
    }

    printf("done.\n");
}

/* ===========================================================================
 * BENCHMARK 4 – Software baseline
 *   Same MAC operation (A * B + acc) computed entirely in ARM software.
 *   Uses volatile to prevent the compiler from optimising the loop away.
 * ===========================================================================*/
static void bench_software(double *samples_ns, uint32_t n)
{
    printf("  [4/4] Software baseline benchmark (%u samples) ... ", n);
    fflush(stdout);

    volatile int32_t acc = 0;

    for (uint32_t i = 0U; i < n; i++)
    {
        acc = 0; /* reset accumulator like a flush */

        uint64_t t0 = now_ns();

        acc += (int32_t)BENCH_OPERAND_A * (int32_t)BENCH_OPERAND_B;

        uint64_t t1 = now_ns();
        samples_ns[i] = (double)(t1 - t0);

        (void)acc; /* prevent dead-code elimination */
    }

    printf("done.\n");
}

/* ===========================================================================
 * CSV export helpers
 * ===========================================================================*/
static void export_latency_csv(const char *filename,
                               double *lat_ns,
                               uint32_t *polls,
                               uint32_t n)
{
    FILE *f = fopen(filename, "w");
    if (!f)
    {
        fprintf(stderr, "[WARN] Cannot open %s: %s\n", filename, strerror(errno));
        return;
    }

    fprintf(f, "sample,latency_ns,latency_cycles,poll_iterations\n");
    for (uint32_t i = 0U; i < n; i++)
    {
        fprintf(f, "%u,%.1f,%.2f,%u\n",
                i + 1U,
                lat_ns[i],
                lat_ns[i] / FPGA_CLOCK_PERIOD_NS,
                polls[i]);
    }
    fclose(f);
    printf("  -> Saved: %s\n", filename);
}

static void export_comms_csv(const char *filename, double *comms_ns, uint32_t n)
{
    FILE *f = fopen(filename, "w");
    if (!f)
    {
        fprintf(stderr, "[WARN] Cannot open %s: %s\n", filename, strerror(errno));
        return;
    }

    fprintf(f, "sample,comms_overhead_ns,comms_overhead_cycles\n");
    for (uint32_t i = 0U; i < n; i++)
    {
        fprintf(f, "%u,%.1f,%.2f\n",
                i + 1U,
                comms_ns[i],
                comms_ns[i] / FPGA_CLOCK_PERIOD_NS);
    }
    fclose(f);
    printf("  -> Saved: %s\n", filename);
}

static void export_poll_histogram_csv(const char *filename,
                                      uint32_t *polls,
                                      uint32_t n)
{
    /* find max poll count */
    uint32_t max_poll = 0U;
    for (uint32_t i = 0U; i < n; i++)
        if (polls[i] > max_poll)
            max_poll = polls[i];

    uint32_t *hist = calloc(max_poll + 1U, sizeof(uint32_t));
    if (!hist)
        return;
    for (uint32_t i = 0U; i < n; i++)
        hist[polls[i]]++;

    FILE *f = fopen(filename, "w");
    if (!f)
    {
        free(hist);
        return;
    }

    fprintf(f, "poll_count,frequency,percentage\n");
    for (uint32_t p = 0U; p <= max_poll; p++)
    {
        if (hist[p] > 0U)
            fprintf(f, "%u,%u,%.2f\n",
                    p, hist[p], 100.0 * (double)hist[p] / (double)n);
    }
    fclose(f);
    free(hist);
    printf("  -> Saved: %s\n", filename);
}

static void export_summary_txt(const char *filename,
                               stats_t *lat_stats,
                               stats_t *comms_stats,
                               stats_t *sw_stats,
                               double throughput_mops,
                               uint32_t lat_n,
                               uint32_t thr_n)
{
    FILE *f = fopen(filename, "w");
    if (!f)
    {
        fprintf(stderr, "[WARN] Cannot open %s\n", filename);
        return;
    }

    double net_compute_ns = lat_stats->mean - comms_stats->mean;
    double speedup = sw_stats->mean / lat_stats->mean;

    fprintf(f, "=================================================================\n");
    fprintf(f, "  IP_PE Hardware Benchmark Summary\n");
    fprintf(f, "  Platform : Terasic DE10-Nano  (Cyclone V SoC 5CSEBA6U23I7)\n");
    fprintf(f, "  FPGA Clock: %lu MHz\n", FPGA_CLOCK_HZ / 1000000UL);
    fprintf(f, "  Operands : A = %d,  B = %d\n",
            (int)BENCH_OPERAND_A, (int)BENCH_OPERAND_B);
    fprintf(f, "=================================================================\n\n");

    fprintf(f, "[1] LATENCY  (%u samples, flush before each operation)\n", lat_n);
    fprintf(f, "    Mean     : %8.1f ns  (%6.1f cycles)\n",
            lat_stats->mean, lat_stats->mean / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Std-dev  : %8.1f ns  (%6.1f cycles)\n",
            lat_stats->stddev, lat_stats->stddev / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Min      : %8.1f ns  (%6.1f cycles)\n",
            lat_stats->min, lat_stats->min / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Max      : %8.1f ns  (%6.1f cycles)\n",
            lat_stats->max, lat_stats->max / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Median   : %8.1f ns  (%6.1f cycles)\n",
            lat_stats->median, lat_stats->median / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "\n");

    fprintf(f, "[2] THROUGHPUT  (%u ops, pipeline saturated, no flush)\n", thr_n);
    fprintf(f, "    Throughput  : %.4f MOPS (Mega Operations Per Second)\n",
            throughput_mops);
    fprintf(f, "    Cycles/op   : %.1f  (at %lu MHz)\n",
            (1.0 / (throughput_mops * 1e6)) * (double)FPGA_CLOCK_HZ,
            FPGA_CLOCK_HZ / 1000000UL);
    fprintf(f, "\n");

    fprintf(f, "[3] COMMUNICATION OVERHEAD  (%u samples)\n", BENCH_COMMS_SAMPLES);
    fprintf(f, "    Mean     : %8.1f ns  (%6.1f cycles)\n",
            comms_stats->mean, comms_stats->mean / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Std-dev  : %8.1f ns  (%6.1f cycles)\n",
            comms_stats->stddev, comms_stats->stddev / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "    Min      : %8.1f ns  (%6.1f cycles)\n",
            comms_stats->min, comms_stats->min / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "\n");

    fprintf(f, "[4] NET COMPUTE TIME  (latency - comms overhead)\n");
    fprintf(f, "    Net compute : %8.1f ns  (%6.1f cycles)\n",
            net_compute_ns, net_compute_ns / FPGA_CLOCK_PERIOD_NS);
    fprintf(f, "\n");

    fprintf(f, "[5] SOFTWARE BASELINE  (%u samples, ARM Cortex-A9)\n",
            BENCH_SOFTWARE_SAMPLES);
    fprintf(f, "    Mean     : %8.1f ns\n", sw_stats->mean);
    fprintf(f, "    Std-dev  : %8.1f ns\n", sw_stats->stddev);
    fprintf(f, "    Min      : %8.1f ns\n", sw_stats->min);
    fprintf(f, "\n");

    fprintf(f, "[6] SPEEDUP  (software_mean / hardware_mean)\n");
    fprintf(f, "    Speedup  : %.2fx  ", speedup);
    if (speedup >= 1.0)
        fprintf(f, "(hardware is %.2fx FASTER than ARM software)\n", speedup);
    else
        fprintf(f, "(hardware is %.2fx SLOWER — dominated by bridge overhead)\n",
                1.0 / speedup);
    fprintf(f, "\n");

    fprintf(f, "=================================================================\n");
    fprintf(f, "  Note: latency includes full HPS->PIO->IP_PE->PIO->HPS round\n");
    fprintf(f, "  trip over the LW-HPS2FPGA AXI bridge. Net compute time\n");
    fprintf(f, "  isolates the FPGA pipeline contribution.\n");
    fprintf(f, "=================================================================\n");

    fclose(f);
    printf("  -> Saved: %s\n", filename);
}

/* ===========================================================================
 * main
 * ===========================================================================*/
int main(void)
{
    int fd = -1;
    void *lw_bridge = NULL;
    int rc = EXIT_SUCCESS;

    /* ------------------------------------------------------------------
     * Allocate sample buffers
     * ------------------------------------------------------------------ */
    double *lat_ns = malloc(BENCH_LATENCY_SAMPLES * sizeof(double));
    uint32_t *lat_polls = malloc(BENCH_LATENCY_SAMPLES * sizeof(uint32_t));
    double *comms_ns = malloc(BENCH_COMMS_SAMPLES * sizeof(double));
    double *sw_ns = malloc(BENCH_SOFTWARE_SAMPLES * sizeof(double));

    if (!lat_ns || !lat_polls || !comms_ns || !sw_ns)
    {
        fprintf(stderr, "[ERROR] malloc failed\n");
        return EXIT_FAILURE;
    }

    printf("\n");
    printf("================================================================\n");
    printf("  IP_PE Benchmark Suite v1.0\n");
    printf("  DE10-Nano  |  Cyclone V 5CSEBA6U23I7  |  50 MHz\n");
    printf("  Operands: A=%d  B=%d\n",
           (int)BENCH_OPERAND_A, (int)BENCH_OPERAND_B);
    printf("================================================================\n\n");

    /* ------------------------------------------------------------------
     * Open /dev/mem and map LW bridge
     * ------------------------------------------------------------------ */
    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0)
    {
        fprintf(stderr, "[ERROR] open /dev/mem: %s  (try sudo)\n", strerror(errno));
        rc = EXIT_FAILURE;
        goto cleanup;
    }

    lw_bridge = mmap(NULL, HPS_LW_BRIDGE_SPAN,
                     PROT_READ | PROT_WRITE, MAP_SHARED,
                     fd, (off_t)HPS_LW_BRIDGE_BASE);
    if (lw_bridge == MAP_FAILED)
    {
        fprintf(stderr, "[ERROR] mmap: %s\n", strerror(errno));
        rc = EXIT_FAILURE;
        goto cleanup;
    }

    printf("  LW bridge mapped @ virtual %p\n\n", lw_bridge);
    printf("  Running benchmarks:\n");

    /* ------------------------------------------------------------------
     * Run benchmarks
     * ------------------------------------------------------------------ */
    bench_latency(lw_bridge, lat_ns, lat_polls, BENCH_LATENCY_SAMPLES);
    double throughput_mops = bench_throughput(lw_bridge, BENCH_THROUGHPUT_SAMPLES);
    bench_comms_overhead(lw_bridge, comms_ns, BENCH_COMMS_SAMPLES);
    bench_software(sw_ns, BENCH_SOFTWARE_SAMPLES);

    /* ------------------------------------------------------------------
     * Compute statistics
     * ------------------------------------------------------------------ */
    stats_t lat_stats = compute_stats(lat_ns, BENCH_LATENCY_SAMPLES);
    stats_t comms_stats = compute_stats(comms_ns, BENCH_COMMS_SAMPLES);
    stats_t sw_stats = compute_stats(sw_ns, BENCH_SOFTWARE_SAMPLES);

    /* ------------------------------------------------------------------
     * Print quick summary to console
     * ------------------------------------------------------------------ */
    printf("\n  Quick summary:\n");
    printf("  %-28s %8.1f ns  (%5.1f cycles)\n",
           "Latency mean:",
           lat_stats.mean, lat_stats.mean / FPGA_CLOCK_PERIOD_NS);
    printf("  %-28s %8.1f ns  (%5.1f cycles)\n",
           "Latency stddev:",
           lat_stats.stddev, lat_stats.stddev / FPGA_CLOCK_PERIOD_NS);
    printf("  %-28s %8.4f MOPS\n",
           "Throughput:", throughput_mops);
    printf("  %-28s %8.1f ns\n",
           "Comms overhead mean:", comms_stats.mean);
    printf("  %-28s %8.1f ns\n",
           "Net compute (lat-comms):",
           lat_stats.mean - comms_stats.mean);
    printf("  %-28s %8.1f ns\n",
           "SW baseline mean:", sw_stats.mean);
    printf("  %-28s %8.2fx\n",
           "Speedup (SW/HW):",
           sw_stats.mean / lat_stats.mean);

    /* ------------------------------------------------------------------
     * Export CSV and summary files
     * ------------------------------------------------------------------ */
    printf("\n  Exporting results:\n");
    export_latency_csv("pe_latency_samples.csv", lat_ns, lat_polls, BENCH_LATENCY_SAMPLES);
    export_comms_csv("pe_comms_overhead.csv", comms_ns, BENCH_COMMS_SAMPLES);
    export_poll_histogram_csv("pe_poll_distribution.csv", lat_polls, BENCH_LATENCY_SAMPLES);
    export_summary_txt("pe_summary.txt",
                       &lat_stats, &comms_stats, &sw_stats,
                       throughput_mops,
                       BENCH_LATENCY_SAMPLES, BENCH_THROUGHPUT_SAMPLES);

    printf("\n  Done. Transfer files to host for plotting:\n");
    printf("    scp root@<board-ip>:~/soc_apps/pe_*.csv .\n");
    printf("    scp root@<board-ip>:~/soc_apps/pe_summary.txt .\n\n");

cleanup:
    free(lat_ns);
    free(lat_polls);
    free(comms_ns);
    free(sw_ns);
    if (lw_bridge && lw_bridge != MAP_FAILED)
        munmap(lw_bridge, HPS_LW_BRIDGE_SPAN);
    if (fd >= 0)
        close(fd);

    return rc;
}